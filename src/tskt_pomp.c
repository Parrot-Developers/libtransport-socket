/**
 * Copyright (c) 2021 Parrot Drones SAS
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
#include <transport-socket/tskt_pomp.h>
#include <stdbool.h>

#define ULOG_TAG tskt_pomp
#include <ulog.h>
ULOG_DECLARE_TAG(tskt_pomp);

#define RCV_BUF_MAX 4096

struct tskt_pomp {
	struct tskt_socket *sock;
	int sock_error;
	tskt_pomp_event_cb_t cb;
	void *userdata;
	bool connected;
	struct pomp_prot *prot;
	struct pomp_msg *wmsg;
	struct pomp_msg *rmsg;
	uint8_t rbuf[RCV_BUF_MAX];
	uint8_t *rptr;
	size_t rlen;
};

static void
sock_event_cb(struct tskt_socket *sock, uint32_t revents, void *userdata)
{
	struct tskt_pomp *self = userdata;

	if (!self)
		return;

	if (revents & POMP_FD_EVENT_ERR) {
		/* get socket error (returned as positive value) */
		self->sock_error = -tskt_socket_get_error(sock);
		goto error;
	}

	if (revents & POMP_FD_EVENT_OUT) {
		if (!self->connected) {
			/* socket is connected, */
			/* remove write notifications, get read notifications */
			tskt_socket_update_events(
				sock, POMP_FD_EVENT_IN, POMP_FD_EVENT_OUT);
			self->connected = true;
			self->cb(self,
				 TSKT_POMP_EVENT_CONNECTED,
				 NULL,
				 self->userdata);
			return;
		}
		/* write pending messages */
		/* XXX TO DO */
	}

	if (!(revents & POMP_FD_EVENT_IN))
		return;

	while (true) {
		ssize_t rlen;
		/* read next message */
		if (self->rlen == 0) {
			/* fill recv buffer */
			rlen = tskt_socket_read(
				sock, self->rbuf, sizeof(self->rbuf), NULL);
			if (rlen == 0) {
				/* connection closed */
				tskt_socket_set_event_cb(sock, 0, NULL, NULL);
				self->cb(self,
					 TSKT_POMP_EVENT_DISCONNECTED,
					 NULL,
					 self->userdata);
				return;
			}
			if (rlen == -EAGAIN)
				/* no data */
				return;
			if (rlen < 0) {
				/* socket error */
				self->sock_error = (int)rlen;
				goto error;
			}
			self->rptr = self->rbuf;
			self->rlen = (size_t)rlen;
		}
		/* decode message */
		if (self->rmsg != NULL) {
			(void)pomp_prot_release_msg(self->prot, self->rmsg);
			self->rmsg = NULL;
		}
		rlen = pomp_prot_decode_msg(
			self->prot, self->rptr, self->rlen, &self->rmsg);
		if (rlen <= 0) {
			/* decoder error */
			self->sock_error = (int)rlen;
			goto error;
		}
		self->rptr += rlen;
		self->rlen -= rlen;
		if (self->rmsg != NULL) {
			self->cb(self,
				 TSKT_POMP_EVENT_MSG,
				 self->rmsg,
				 self->userdata);
			return;
		}
	} /* while(1) */

	return;

error:
	/* internal error, remove socket notifications */
	tskt_socket_set_event_cb(sock, 0, NULL, NULL);
	self->cb(self, TSKT_POMP_EVENT_ERROR, NULL, self->userdata);
}

int tskt_pomp_new(struct tskt_socket *sock,
		  tskt_pomp_event_cb_t cb,
		  void *userdata,
		  struct tskt_pomp **ret_obj)
{
	struct tskt_pomp *self;
	int res = 0;

	ULOG_ERRNO_RETURN_ERR_IF(sock == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cb == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		return res;
	}

	/* init protocol parser */
	self->prot = pomp_prot_new();
	if (self->prot == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("pomp_prot_new", -res);
		goto error;
	}

	/* allocate write msg */
	self->wmsg = pomp_msg_new();
	if (self->wmsg == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("pomp_msg_new", -res);
		goto error;
	}

	/* set event handler and wait for connection (socket writable) */
	res = tskt_socket_set_event_cb(
		sock, POMP_FD_EVENT_OUT, sock_event_cb, self);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_set_event_cb", -res);
		goto error;
	}

	/* return object */
	self->sock = sock;
	self->cb = cb;
	self->userdata = userdata;
	self->rlen = 0;

	*ret_obj = self;
	return 0;

error:
	(void)tskt_pomp_destroy(self);
	return res;
}

int tskt_pomp_destroy(struct tskt_pomp *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (self->prot != NULL)
		(void)pomp_prot_destroy(self->prot);
	if (self->wmsg != NULL)
		(void)pomp_msg_destroy(self->wmsg);
	if (self->rmsg != NULL)
		(void)pomp_msg_destroy(self->rmsg);
	if (self->sock != NULL)
		(void)tskt_socket_destroy(self->sock);
	free(self);
	return 0;
}

int tskt_pomp_send_msg(struct tskt_pomp *self, const struct pomp_msg *msg)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(msg == NULL, EINVAL);

	if (self->sock_error)
		return self->sock_error;

	if (!self->connected)
		return -ENOTCONN;

	/* get message buffer */
	struct pomp_buffer *buf = pomp_msg_get_buffer(msg);
	if (!buf)
		return -EINVAL;

	/* check message */
	const void *ptr;
	size_t len;
	int res = pomp_buffer_get_cdata(buf, &ptr, &len, NULL);
	if (res < 0)
		return res;
	if (len == 0)
		return -EINVAL;

	/* XXX TO DO: check for pending buffers */

	/* send message */
	ssize_t wlen = tskt_socket_write(self->sock, ptr, len);
	if (wlen == -EAGAIN || (wlen > 0 && (size_t)wlen != len)) {
		/* partial write */
		/* XXX TO DO */
		return -EAGAIN;
	}
	if (wlen < 0) {
		/* socket error */
		self->sock_error = (int)wlen;
		return self->sock_error;
	}

	return 0;
}

int tskt_pomp_send(struct tskt_pomp *self, uint32_t msgid, const char *fmt, ...)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(fmt == NULL, EINVAL);

	int res = 0;
	va_list args;
	va_start(args, fmt);
	res = tskt_pomp_sendv(self, msgid, fmt, args);
	va_end(args);
	return res;
}

int tskt_pomp_sendv(struct tskt_pomp *self,
		    uint32_t msgid,
		    const char *fmt,
		    va_list args)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(fmt == NULL, EINVAL);

	/* Write message and send it*/
	int res = pomp_msg_writev(self->wmsg, msgid, fmt, args);
	if (res == 0)
		res = tskt_pomp_send_msg(self, self->wmsg);

	/* Always cleanup message */
	(void)pomp_msg_clear(self->wmsg);
	return res;
}

struct tskt_socket *tskt_pomp_get_socket(struct tskt_pomp *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);
	return self->sock;
}

int tskt_pomp_get_error(struct tskt_pomp *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->sock_error;
}
