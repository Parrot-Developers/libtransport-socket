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
#include <libpomp.h>
#include <transport-packet/tpkt.h>
#include <transport-socket/tskt.h>
#include <transport-socket/tskt_client.h>

#define ULOG_TAG tskt_client
#include <ulog.h>
ULOG_DECLARE_TAG(tskt_client);

#define RECONNECT_DELAY_MS 2000

#ifndef EHOSTDOWN
#	define EHOSTDOWN WSAEHOSTDOWN
#endif

/** Check if a connection error should be logged, ignore common errors */
#define SHOULD_LOG_CONNECT_ERROR(_err)                                         \
	((_err) != ECONNREFUSED && (_err) != EHOSTUNREACH &&                   \
	 (_err) != EHOSTDOWN && (_err) != ENETUNREACH && (_err) != ENETDOWN && \
	 (_err) != ENOENT && (_err) != ETIMEDOUT)

/** Check if a connection error means it is in progress */
#define CONNECT_IN_PROGRESS(_err)                                              \
	((_err) == EINPROGRESS || (_err) == EWOULDBLOCK)


struct addr_port {
	char *addr;
	uint16_t port;
};

struct tskt_client {
	struct tskt_socket *sock;
	struct tskt_client_cbs cbs;
	void *cbs_userdata;

	struct pomp_loop *loop;
	struct pomp_timer *timer;
	struct addr_port local;
	struct addr_port remote;

	bool connected;
	bool started;
	bool stopping;
	int notifying;
};

static inline void xfree(void **ptr)
{
	if (ptr) {
		free(*ptr);
		*ptr = NULL;
	}
}

static inline char *xstrdup(const char *s)
{
	return s == NULL ? NULL : strdup(s);
}

static inline void notify_event(struct tskt_client *self,
				enum tskt_client_event event)
{
	self->notifying++;
	self->cbs.event_cb(self, event, self->sock, self->cbs_userdata);
	self->notifying--;
}

static inline void clear_addr_port(struct tskt_client *self)
{
	xfree((void **)&self->local.addr);
	self->local.port = 0;
	xfree((void **)&self->remote.addr);
	self->remote.port = 0;
}

static void
sock_event_cb(struct tskt_socket *sock, uint32_t revents, void *userdata)
{
	int err;
	struct tskt_client *self = userdata;
	struct tpkt_packet *pkt = NULL;

	if (!self)
		return;

	if (revents & POMP_FD_EVENT_ERR)
		goto error;

	if (revents & POMP_FD_EVENT_OUT) {
		/* remove write notifications, get read notifications */
		tskt_socket_update_events(
			sock, POMP_FD_EVENT_IN, POMP_FD_EVENT_OUT);
		if (!self->connected) {
			/* socket is connected, */
			self->connected = true;
			notify_event(self, TSKT_CLIENT_EVENT_CONNECTED);
			return;
		} else {
			notify_event(self, TSKT_CLIENT_EVENT_READY_TO_SEND);
		}
	}

	if (!(revents & POMP_FD_EVENT_IN))
		return;

	err = tskt_socket_read_pkt_alloc(sock, &pkt);
	if (err >= 0) {
		size_t len = 0;
		/* check for end of file */
		err = tpkt_get_cdata(pkt, NULL, &len, 0);
		if (err < 0) {
			ULOG_ERRNO("tpkt_get_cdata", -err);
			tpkt_unref(pkt);
			goto error;
		}
		if (len == 0) {
			/* connection closed, destroy socket to remove from
			 * epoll */
			tskt_socket_destroy(self->sock);
			self->sock = NULL;
			self->connected = false;
			notify_event(self, TSKT_CLIENT_EVENT_DISCONNECTED);

			/* reconnect if needed */
			if (!self->stopping) {
				err = pomp_timer_set(self->timer,
						     RECONNECT_DELAY_MS);
				if (err < 0)
					ULOG_ERRNO("pomp_timer_set", -err);
			}
		} else {
			/* process the packet */
			self->notifying++;
			self->cbs.data_cb(self, pkt, self->cbs_userdata);
			self->notifying--;
		}
		tpkt_unref(pkt);
	} else if (err != -EAGAIN) {
		ULOG_ERRNO("socket_read_pkt_alloc", -err);
		goto error;
	}

	return;

error:
	/* internal error, destroy socket to remove from epoll */
	tskt_socket_destroy(self->sock);
	self->sock = NULL;
	if (self->connected) {
		self->connected = false;
		notify_event(self, TSKT_CLIENT_EVENT_DISCONNECTED);
	}
	err = pomp_timer_set(self->timer, RECONNECT_DELAY_MS);
	if (err < 0)
		ULOG_ERRNO("pomp_timer_set", -err);
}

static int client_start(struct tskt_client *self)
{
	int res;
	int fd;
	if (self->sock != NULL) {
		tskt_socket_destroy(self->sock);
		self->sock = NULL;
	}
	res = tskt_socket_new_tcp(self->loop, &self->sock);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_new_tcp", -res);
		goto error;
	}
	res = tskt_socket_get_fd(self->sock);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_get_fd", -res);
		goto error;
	}
	fd = res;
	if (self->cbs.socket_created_cb != NULL) {
		self->notifying++;
		self->cbs.socket_created_cb(
			self, self->sock, fd, self->cbs_userdata);
		self->notifying--;
	}

	res = tskt_socket_connect(self->sock,
				  self->local.addr,
				  self->local.port,
				  self->remote.addr,
				  self->remote.port);
	if (res != 0 && !CONNECT_IN_PROGRESS(errno)) {
		if (SHOULD_LOG_CONNECT_ERROR(errno))
			ULOG_ERRNO("tskt_socket_connect", -res);
		res = pomp_timer_set(self->timer, RECONNECT_DELAY_MS);
		if (res < 0) {
			ULOG_ERRNO("pomp_timer_set", -res);
			goto error;
		}
		tskt_socket_destroy(self->sock);
		self->sock = NULL;
		return 0;
	}

	if (self->cbs.socket_upgrade_cb != NULL) {
		struct tskt_socket *tmp_sock = NULL;
		self->notifying++;
		tmp_sock = self->cbs.socket_upgrade_cb(
			self, self->sock, self->cbs_userdata);
		self->notifying--;
		if (tmp_sock != NULL) {
			self->sock = tmp_sock;
		} else {
			/* res still holds the (0 or in-progress) result of the
			 * preceding tskt_socket_connect() call: without this,
			 * client_start() (and thus tskt_client_connect())
			 * would report success even though the upgrade was
			 * rejected, leaving the client stuck with started
			 * latched true, sock NULL, and no reconnect timer
			 * armed. */
			res = -ECANCELED;
			goto error;
		}
	}

	res = tskt_socket_set_event_cb(
		self->sock, POMP_FD_EVENT_OUT, sock_event_cb, self);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_set_event_cb", -res);
		goto error;
	}

	/* Notification will be done later when the socket is writable
	 * (even if it succeeded now, notify asynchronously) */
	return 0;

error:
	tskt_socket_destroy(self->sock);
	self->sock = NULL;
	return res;
}

static int client_stop(struct tskt_client *self)
{
	self->connected = false;
	notify_event(self, TSKT_CLIENT_EVENT_DISCONNECTED);
	self->stopping = false;
	tskt_socket_destroy(self->sock);
	self->sock = NULL;

	pomp_timer_clear(self->timer);
	clear_addr_port(self);
	self->started = false;

	return 0;
}

static void tskt_client_stop_idle(void *userdata)
{
	struct tskt_client *self = userdata;
	client_stop(self);
}

static void timer_cb(struct pomp_timer *timer, void *userdata)
{
	struct tskt_client *self = userdata;
	pomp_timer_clear(self->timer);

	client_start(self);
}

int tskt_client_connect(struct tskt_client *self,
			const char *local_addr,
			uint16_t local_port,
			const char *remote_addr,
			uint16_t remote_port)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(remote_addr == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(remote_addr[0] == '\0', EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(remote_port == 0, EINVAL);

	if (self->started)
		return -EBUSY;

	if (local_addr != NULL) {
		self->local.addr = xstrdup(local_addr);
		ULOG_ERRNO_RETURN_ERR_IF(self->local.addr == NULL, ENOMEM);
	}
	self->local.port = local_port;
	self->remote.addr = xstrdup(remote_addr);
	ULOG_ERRNO_RETURN_ERR_IF(self->remote.addr == NULL, ENOMEM);
	self->remote.port = remote_port;

	res = client_start(self);
	if (res < 0) {
		clear_addr_port(self);
		self->started = false;
	} else {
		self->started = true;
	}
	return res;
}

int tskt_client_new(struct pomp_loop *loop,
		    struct tskt_client_cbs cbs,
		    void *userdata,
		    struct tskt_client **ret_obj)
{
	int res = 0;
	struct tskt_client *self;

	ULOG_ERRNO_RETURN_ERR_IF(loop == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs.event_cb == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs.data_cb == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		return -ENOMEM;
	}

	/* Initialize object */
	self->loop = loop;
	self->cbs = cbs;
	self->cbs_userdata = userdata;

	/* Allocate timer */
	self->timer = pomp_timer_new(self->loop, &timer_cb, self);
	if (self->timer == NULL) {
		res = -EPROTO;
		ULOG_ERRNO("pomp_timer_new", -res);
		goto error;
	}

	*ret_obj = self;
	return 0;

error:
	return res;
}

ssize_t tskt_client_send(struct tskt_client *self, const void *buf, size_t len)
{
	ssize_t res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->sock == NULL, ENOTCONN);

	res = tskt_socket_write(self->sock, buf, len);
	if (res == -EAGAIN) {
		int err = tskt_socket_update_events(
			self->sock, POMP_FD_EVENT_OUT, 0);
		if (err < 0)
			ULOG_ERRNO("tskt_socket_update_events", -err);
	} else if (res < 0) {
		ULOG_ERRNO("tskt_socket_write", -res);
	}

	return res;
}

int tskt_client_send_pkt(struct tskt_client *self, struct tpkt_packet *pkt)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->sock == NULL, ENOTCONN);

	res = tskt_socket_write_pkt(self->sock, pkt);
	if (res == -EAGAIN) {
		int err = tskt_socket_update_events(
			self->sock, POMP_FD_EVENT_OUT, 0);
		if (err < 0)
			ULOG_ERRNO("tskt_socket_update_events", -err);
	} else if (res < 0) {
		ULOG_ERRNO("tskt_socket_write_pkt", -res);
	}

	return res;
}

int tskt_client_stop(struct tskt_client *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (!self->started || self->stopping)
		return 0;

	self->stopping = true;

	/* If currently notifying events/messages, do the stop in idle,
	 * otherwise do it synchronously */
	if (self->notifying > 0) {
		return pomp_loop_idle_add_with_cookie(
			self->loop, &tskt_client_stop_idle, self, self);
	} else {
		tskt_client_stop_idle(self);
		return 0;
	}
}

int tskt_client_destroy(struct tskt_client *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (self->timer != NULL)
		(void)pomp_timer_destroy(self->timer);
	if (self->sock != NULL)
		(void)tskt_socket_destroy(self->sock);

	(void)pomp_loop_idle_remove_by_cookie(self->loop, self);

	free(self->local.addr);
	free(self->remote.addr);
	free(self);

	return 0;
}

struct tskt_socket *tskt_client_get_socket(struct tskt_client *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	return self->sock;
}
