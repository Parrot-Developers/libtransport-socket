/**
 * Copyright (c) 2020 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef _WIN32
#	include <winsock2.h>
#	include <ws2tcpip.h>
#	undef OPAQUE
#else /* !_WIN32 */
#	include <arpa/inet.h>
#	include <netdb.h>
#	include <netinet/in.h>
#endif /* !_WIN32 */

#define ULOG_TAG tskt_resolv_impl
#include <ulog.h>
ULOG_DECLARE_TAG(tskt_resolv_impl);

#include <futils/futils.h>
#include <libpomp.h>
#include <transport-socket/tskt_resolv_ops.h>


struct resolv_impl_request;


struct resolv_impl {
	struct tskt_resolv rslv;
	pthread_t thread;
	pthread_mutex_t lock;
	pthread_cond_t cond;
	int next_id;
	bool quit;
	struct list_node pending;
	struct resolv_impl_request *current;
	struct list_node done;
};


static void *resolv_impl_thread(void *userdata);
static void resolv_impl_destroy(struct tskt_resolv *self);
static int resolv_impl_getaddrinfo(struct tskt_resolv *self,
				   const char *hostname,
				   struct pomp_loop *loop,
				   tskt_resolv_cb_t cb,
				   void *userdata,
				   int *ret_id);
static int resolv_impl_cancel(struct tskt_resolv *self, int id);


static const struct tskt_resolv_ops resolv_impl_ops = {
	.destroy = resolv_impl_destroy,
	.getaddrinfo = resolv_impl_getaddrinfo,
	.cancel = resolv_impl_cancel,
};


int tskt_resolv_new(struct tskt_resolv **ret_obj)
{
	struct resolv_impl *self;
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		return -ENOMEM;
	}

	/* Initialize object */
	self->rslv.ops = &resolv_impl_ops;
	self->rslv.refcount = 1;
	list_init(&self->pending);
	list_init(&self->done);
	pthread_mutex_init(&self->lock, NULL);
	pthread_cond_init(&self->cond, NULL);

	/* Create async thread */
	res = pthread_create(&self->thread, NULL, resolv_impl_thread, self);
	if (res != 0) {
		ULOG_ERRNO("pthread_create", res);
		pthread_mutex_destroy(&self->lock);
		pthread_cond_destroy(&self->cond);
		free(self);
		return -res;
	}

	*ret_obj = (struct tskt_resolv *)self;
	return 0;
}


static void resolv_impl_destroy(struct tskt_resolv *rslv)
{
	struct resolv_impl *self = (struct resolv_impl *)rslv;

	/* Terminate async thread */
	pthread_mutex_lock(&self->lock);
	self->quit = true;
	pthread_cond_signal(&self->cond);
	pthread_mutex_unlock(&self->lock);
	pthread_join(self->thread, NULL);

	/* Release resources */
	pthread_mutex_destroy(&self->lock);
	pthread_cond_destroy(&self->cond);
	free(self);
}


struct resolv_impl_request {
	struct resolv_impl *resolv;
	int id;
	struct pomp_loop *loop;
	tskt_resolv_cb_t cb;
	void *userdata;
	char *hostname;
	enum tskt_resolv_error result;
	char addr[INET_ADDRSTRLEN];
	struct list_node node;
};


static enum tskt_resolv_error get_gai_result(int err)
{
	switch (err) {
	case 0:
		return TSKT_RESOLV_ERROR_OK;
	case EAI_AGAIN:
		return TSKT_RESOLV_ERROR_AGAIN;
	case EAI_FAIL:
		return TSKT_RESOLV_ERROR_FAIL;
	case EAI_MEMORY:
		return TSKT_RESOLV_ERROR_MEMORY;
	case EAI_NODATA:
		return TSKT_RESOLV_ERROR_NODATA;
	case EAI_NONAME:
		return TSKT_RESOLV_ERROR_NONAME;
#ifdef EAI_SYSTEM
	case EAI_SYSTEM:
#endif
	default:
		return TSKT_RESOLV_ERROR_SYTEM;
	}
}


static void resolv_idle_cb(void *userdata)
{
	struct resolv_impl_request *req = userdata;
	struct resolv_impl *self = req->resolv;
	const char *addrs[1];

	/* Detach request from done list */
	pthread_mutex_lock(&self->lock);
	list_del(&req->node);
	pthread_mutex_unlock(&self->lock);

	/* Notify result */
	addrs[0] = req->addr;
	req->cb(&self->rslv,
		req->id,
		req->result,
		req->addr[0] != 0 ? 1 : 0,
		addrs,
		req->userdata);

	/* Free request */
	tskt_resolv_unref(&self->rslv);
	free(req->hostname);
	free(req);
}


static void *resolv_impl_thread(void *userdata)
{
	struct resolv_impl *self = userdata;

	ULOGD("thread started");

	pthread_mutex_lock(&self->lock);
	while (true) {
		struct resolv_impl_request *req;
		int res;
		static const struct addrinfo hints = {
			.ai_family = AF_INET,
		};
		struct addrinfo *addr;

		/* Wait for quit signal or pending request */
		if (self->quit)
			break;
		if (list_is_empty(&self->pending)) {
			pthread_cond_wait(&self->cond, &self->lock);
			continue;
		}

		/* Get next request */
		req = list_entry(list_first(&self->pending),
				 struct resolv_impl_request,
				 node);
		list_del(&req->node);
		self->current = req;
		pthread_mutex_unlock(&self->lock);

		ULOGD("request id=%d hostname=\"%s\"\n",
		      req->id,
		      req->hostname);

		res = getaddrinfo(req->hostname, NULL, &hints, &addr);
		if (res == 0) {
			/* XXX TO DO: return more than one address */
			res = getnameinfo(addr->ai_addr,
					  addr->ai_addrlen,
					  req->addr,
					  sizeof(req->addr),
					  NULL,
					  0,
					  NI_NUMERICHOST);
			if (res != 0) {
				/* Invalid address */
				ULOGD("getaddrinfo: invalid address");
				req->result = TSKT_RESOLV_ERROR_NODATA;
			} else {
				ULOGD("addr=%s", req->addr);
			}
			freeaddrinfo(addr);
		} else {
			ULOGD("getaddrinfo: %s", gai_strerror(res));
			req->result = get_gai_result(res);
		}

		/* Request done */
		pthread_mutex_lock(&self->lock);
		self->current = NULL;
		if (req->loop == NULL) {
			/* Cancelled request */
			ULOGD("request cancelled");
			tskt_resolv_unref(&self->rslv);
			free(req->hostname);
			free(req);
		} else {
			/* Trigger async callback */
			res = pomp_loop_idle_add(
				req->loop, resolv_idle_cb, req);
			if (res < 0) {
				ULOG_ERRNO("pomp_loop_idle_add", -res);
				/* XXX no way to properly report this error */
				tskt_resolv_unref(&self->rslv);
				free(req->hostname);
				free(req);
			} else {
				list_add_before(&self->done, &req->node);
			}
		}
	}
	pthread_mutex_unlock(&self->lock);

	ULOGD("thread stopped");

	return NULL;
}


static int resolv_impl_getaddrinfo(struct tskt_resolv *rslv,
				   const char *hostname,
				   struct pomp_loop *loop,
				   tskt_resolv_cb_t cb,
				   void *userdata,
				   int *ret_id)
{
	struct resolv_impl *self = (struct resolv_impl *)rslv;
	struct resolv_impl_request *req;
	struct in_addr addr;
	int res = 0;

	/* Create request */
	req = calloc(1, sizeof(*req));
	if (req == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		return -ENOMEM;
	}
	req->resolv = self;
	req->loop = loop;
	req->cb = cb;
	req->userdata = userdata;
	pthread_mutex_lock(&self->lock);
	req->id = self->next_id++;
	if (req->id == TSKT_RESOLV_INVALID_ID) /* Just in case... */
		req->id = self->next_id++;
	pthread_mutex_unlock(&self->lock);

	/* Check for literal address */
	if (inet_pton(AF_INET, hostname, &addr) == 1) {
		/* Write back the address in the request */
		if (inet_ntop(AF_INET, &addr, req->addr, sizeof(req->addr)) ==
		    NULL) {
			res = -errno;
			ULOG_ERRNO("inet_ntop", -res);
			goto error;
		}
		/* Trigger async callback */
		res = pomp_loop_idle_add(loop, resolv_idle_cb, req);
		if (res < 0) {
			ULOG_ERRNO("pomp_loop_idle_add", -res);
			goto error;
		}
		pthread_mutex_lock(&self->lock);
		list_add_before(&self->done, &req->node);
		pthread_mutex_unlock(&self->lock);
	} else {
		/* Enqueue request for processing by async thread */
		req->hostname = strdup(hostname);
		if (req->hostname == NULL) {
			res = -ENOMEM;
			ULOG_ERRNO("strdup", -res);
			goto error;
		}
		pthread_mutex_lock(&self->lock);
		list_add_before(&self->pending, &req->node);
		pthread_cond_signal(&self->cond);
		pthread_mutex_unlock(&self->lock);
	}

	/* Success */
	tskt_resolv_ref(rslv);
	if (ret_id != NULL)
		*ret_id = req->id;
	return 0;

error:
	/* Failure */
	free(req);
	return res;
}


static int resolv_impl_cancel(struct tskt_resolv *rslv, int id)
{
	struct resolv_impl *self = (struct resolv_impl *)rslv;
	struct resolv_impl_request *req = NULL;
	struct resolv_impl_request *tmp_req;
	bool running = false;

	/* Find request */
	pthread_mutex_lock(&self->lock);
	if (self->current && self->current->id == id) {
		/* Currently running */
		req = self->current;
		running = true;
		/* Signal to async thread that request is cancelled */
		req->loop = NULL;
	} else {
		/* Detach from pending list */
		list_walk_entry_forward(&self->pending, tmp_req, node)
		{
			if (tmp_req->id == id) {
				list_del(&tmp_req->node);
				req = tmp_req;
				break;
			}
		}
		/* Otherwise detach from done list */
		if (req == NULL)
			list_walk_entry_forward(&self->done, tmp_req, node)
			{
				if (tmp_req->id == id) {
					list_del(&tmp_req->node);
					req = tmp_req;
					break;
				}
			}
	}
	pthread_mutex_unlock(&self->lock);

	if (req == NULL) {
		/* Invalid request Id */
		ULOGE("request %d not found", id);
		return -ENOENT;
	}

	if (running) {
		/* Currently running, cannot be cancelled here but
		 * async thread will handle the cancellation */
		return 0;
	}

	/* Free cancelled request */
	tskt_resolv_unref(&self->rslv);
	pomp_loop_idle_remove(req->loop, resolv_idle_cb, req);
	free(req->hostname);
	free(req);

	return 0;
}
