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
#include <transport-socket/tskt_ops.h>

#define ULOG_TAG tskt
#include <ulog.h>
ULOG_DECLARE_TAG(tskt);


/* Set default packet size if not set */
#define CHECK_RXPKT_MAX_SIZE(self)                                             \
	do {                                                                   \
		if (self->rxpkt_max_size == 0)                                 \
			self->rxpkt_max_size =                                 \
				TSKT_SOCKET_RXPKT_MAX_SIZE_DEFAULT;            \
	} while (0)


struct tpkt_packet *tskt_socket_rxpkt_alloc(struct tskt_socket *self)
{
	struct pomp_buffer *buf;
	struct tpkt_packet *pkt;
	int res;

	/* Try to get a packet from free list */
	if (self->rxpkt_free != NULL) {
		pkt = tpkt_list_first(self->rxpkt_free);
		if (pkt != NULL && tpkt_list_remove(self->rxpkt_free, pkt) == 0)
			return pkt;
	}

	CHECK_RXPKT_MAX_SIZE(self);

	/* Allocate new packet */
	buf = pomp_buffer_new(self->rxpkt_max_size);
	if (buf == NULL) {
		ULOG_ERRNO("pomp_buffer_new", ENOMEM);
		return NULL;
	}
	res = tpkt_new_from_buffer(buf, &pkt);
	pomp_buffer_unref(buf);
	if (res < 0) {
		ULOG_ERRNO("tpkt_new_from_buffer", -res);
		return NULL;
	}
	return pkt;
}


void tskt_socket_rxpkt_free(struct tskt_socket *self, struct tpkt_packet *pkt)
{
	/* Allocate free list if it doesn't exist */
	if (self->rxpkt_free == NULL) {
		int res = tpkt_list_new(&self->rxpkt_free);
		if (res < 0) {
			ULOG_ERRNO("tpkt_list_new", -res);
			(void)tpkt_unref(pkt);
			return;
		}
	}

	/* Put packet in free list */
	(void)tpkt_list_add_first(self->rxpkt_free, pkt);
	(void)tpkt_unref(pkt);
}


int tskt_socket_destroy(struct tskt_socket *self)
{
	if (self == NULL)
		return 0;

	struct tpkt_list *rxpkt_free = self->rxpkt_free;

	int res = self->ops->destroy(self);

	if (rxpkt_free != NULL)
		(void)tpkt_list_destroy(rxpkt_free);

	return res;
}


int tskt_socket_get_fd(struct tskt_socket *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->get_fd == NULL, EOPNOTSUPP);
	return self->ops->get_fd(self);
}


int tskt_socket_set_fd_cb(struct tskt_socket *self,
			  pomp_fd_event_cb_t fd_cb,
			  void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->set_fd_cb == NULL, EOPNOTSUPP);
	return self->ops->set_fd_cb(self, fd_cb, userdata);
}


int tskt_socket_get_local_addr(struct tskt_socket *self, char *str, size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->get_local_addr == NULL, EOPNOTSUPP);
	return self->ops->get_local_addr(self, str, len, NULL);
}


uint16_t tskt_socket_get_local_port(struct tskt_socket *self)
{
	uint16_t port = 0;
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(
		self->ops->get_local_addr == NULL, EOPNOTSUPP, 0);
	self->ops->get_local_addr(self, NULL, 0, &port);
	return port;
}


int tskt_socket_get_remote_addr(struct tskt_socket *self, char *str, size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->get_remote_addr == NULL,
				 EOPNOTSUPP);
	return self->ops->get_remote_addr(self, str, len, NULL);
}


uint16_t tskt_socket_get_remote_port(struct tskt_socket *self)
{
	uint16_t port = 0;
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(
		self->ops->get_remote_addr == NULL, EOPNOTSUPP, 0);
	self->ops->get_remote_addr(self, NULL, 0, &port);
	return port;
}


int tskt_socket_set_remote(struct tskt_socket *self,
			   const char *addr,
			   uint16_t port)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->set_remote_addr == NULL,
				 EOPNOTSUPP);
	return self->ops->set_remote_addr(self, addr, port);
}


int tskt_socket_get_rxbuf_size(struct tskt_socket *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->get_option(self, TSKT_OPT_RXBUF_SIZE);
}


int tskt_socket_set_rxbuf_size(struct tskt_socket *self, size_t size)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->set_option(self, TSKT_OPT_RXBUF_SIZE, (int)size);
}


int tskt_socket_get_txbuf_size(struct tskt_socket *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->get_option(self, TSKT_OPT_TXBUF_SIZE);
}


int tskt_socket_set_txbuf_size(struct tskt_socket *self, size_t size)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->set_option(self, TSKT_OPT_TXBUF_SIZE, (int)size);
}


int tskt_socket_get_class_selector(struct tskt_socket *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->get_option(self, TSKT_OPT_CLASS_SELECTOR);
}


int tskt_socket_set_class_selector(struct tskt_socket *self, int cls)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->set_option(self, TSKT_OPT_CLASS_SELECTOR, cls);
}


int tskt_socket_get_nodelay(struct tskt_socket *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->get_option(self, TSKT_OPT_NODELAY);
}


int tskt_socket_set_nodelay(struct tskt_socket *self, int nodelay)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->set_option(self, TSKT_OPT_NODELAY, nodelay);
}


int tskt_socket_get_error(struct tskt_socket *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->get_option(self, TSKT_OPT_ERROR);
}


int tskt_socket_get_reset(struct tskt_socket *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->get_option(self, TSKT_OPT_RESET);
}


int tskt_socket_set_reset(struct tskt_socket *self, int reset)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->set_option(self, TSKT_OPT_RESET, reset);
}


int tskt_socket_get_rxpkt_max_size(struct tskt_socket *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	CHECK_RXPKT_MAX_SIZE(self);

	return (int)self->rxpkt_max_size;
}


int tskt_socket_set_rxpkt_max_size(struct tskt_socket *self, size_t max_size)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(max_size == 0, EINVAL);

	if (max_size == self->rxpkt_max_size)
		return 0;

	self->rxpkt_max_size = max_size;

	if (self->rxpkt_free)
		tpkt_list_flush(self->rxpkt_free);

	return 0;
}


ssize_t tskt_socket_read(struct tskt_socket *self,
			 void *buf,
			 size_t cap,
			 uint64_t *ts_us)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->read(self, buf, cap, ts_us);
}


ssize_t tskt_socket_write(struct tskt_socket *self, const void *buf, size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->write(self, buf, len);
}


ssize_t tskt_socket_readv(struct tskt_socket *self,
			  const struct iovec *iov,
			  size_t iov_len,
			  uint64_t *ts_us)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->readv == NULL, EOPNOTSUPP);
	return self->ops->readv(self, iov, iov_len, ts_us);
}


ssize_t tskt_socket_writev(struct tskt_socket *self,
			   const struct iovec *iov,
			   size_t iov_len)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->writev == NULL, EOPNOTSUPP);
	return self->ops->writev(self, iov, iov_len);
}


ssize_t tskt_socket_readmv(struct tskt_socket *self,
			   struct tskt_miovec *miov,
			   size_t mlen)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(miov == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mlen == 0, EINVAL);

	if (self->ops->readmv != NULL)
		return self->ops->readmv(self, miov, mlen);

	/* Just read one message if readmv is not implemented */
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->readv == NULL, EOPNOTSUPP);

	int res = self->ops->readv(
		self, miov[0].iov, miov[0].iovlen, &miov[0].ts_us);
	if (res < 0)
		return res;
	miov[0].len = res;
	return 1;
}


ssize_t tskt_socket_writemv(struct tskt_socket *self,
			    struct tskt_miovec *miov,
			    size_t mlen)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(miov == NULL, EINVAL);

	if (self->ops->writemv != NULL)
		return self->ops->writemv(self, miov, mlen);

	/* Just write one message if writemv is not supported */
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->writev == NULL, EOPNOTSUPP);

	if (mlen == 0)
		return 0;

	int res = self->ops->writev(self, miov[0].iov, miov[0].iovlen);
	if (res < 0)
		return res;
	miov[0].len = res;
	return 1;
}


int tskt_socket_read_pkt(struct tskt_socket *self, struct tpkt_packet *pkt)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->read_pkt(self, pkt);
}


int tskt_socket_read_pkt_alloc(struct tskt_socket *self,
			       struct tpkt_packet **ret_pkt)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_pkt == NULL, EINVAL);

	*ret_pkt = NULL;

	if (self->ops->read_pkt_alloc != NULL)
		return self->ops->read_pkt_alloc(self, ret_pkt);

	/* Emulate function if not implemented */
	struct tpkt_packet *pkt = tskt_socket_rxpkt_alloc(self);
	if (pkt == NULL)
		return -ENOMEM;
	int res = self->ops->read_pkt(self, pkt);
	if (res < 0)
		tskt_socket_rxpkt_free(self, pkt);
	else
		*ret_pkt = pkt;
	return res;
}


int tskt_socket_write_pkt(struct tskt_socket *self, struct tpkt_packet *pkt)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->write_pkt(self, pkt);
}


ssize_t tskt_socket_read_pkt_list(struct tskt_socket *self,
				  struct tpkt_list *list,
				  size_t max_pkts)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(list == NULL, EINVAL);

	if (self->ops->read_pkt_list != NULL)
		return self->ops->read_pkt_list(self, list, max_pkts);

	/* Emulate function if not implemented */
	if (max_pkts == 0)
		return 0;
	struct tpkt_packet *pkt = tskt_socket_rxpkt_alloc(self);
	if (pkt == NULL)
		return -ENOMEM;

	int res = self->ops->read_pkt(self, pkt);
	if (res < 0) {
		tskt_socket_rxpkt_free(self, pkt);
		return res;
	}
	(void)tpkt_list_add_last(list, pkt);
	(void)tpkt_unref(pkt);
	return 1;
}


ssize_t tskt_socket_write_pkt_list(struct tskt_socket *self,
				   struct tpkt_list *list)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(list == NULL, EINVAL);

	if (self->ops->write_pkt_list != NULL)
		return self->ops->write_pkt_list(self, list);

	/* Emulate function if not implemented */
	struct tpkt_packet *pkt = tpkt_list_first(list);
	if (pkt == NULL)
		return 0;
	int res = self->ops->write_pkt(self, pkt);
	if (res < 0)
		return res;
	(void)tpkt_list_remove(list, pkt);
	(void)tpkt_unref(pkt);
	return 1;
}


int tskt_socket_set_event_cb(struct tskt_socket *self,
			     uint32_t events,
			     tskt_socket_event_cb_t cb,
			     void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->set_event_cb(self, events, cb, userdata);
}


int tskt_socket_update_events(struct tskt_socket *self,
			      uint32_t events_to_add,
			      uint32_t events_to_remove)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	return self->ops->update_events(self, events_to_add, events_to_remove);
}


int tskt_socket_connect(struct tskt_socket *self,
			const char *local_addr,
			uint16_t local_port,
			const char *remote_addr,
			uint16_t remote_port)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->connect == NULL, EOPNOTSUPP);
	return self->ops->connect(
		self, local_addr, local_port, remote_addr, remote_port);
}


int tskt_socket_listen(struct tskt_socket *self,
		       const char *local_addr,
		       uint16_t local_port)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->listen == NULL, EOPNOTSUPP);
	return self->ops->listen(self, local_addr, local_port);
}


int tskt_socket_accept(struct tskt_socket *self,
		       char *remote_addr,
		       size_t remote_len,
		       uint16_t *remote_port,
		       struct tskt_socket **ret_obj)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->ops->accept == NULL, EOPNOTSUPP);
	return self->ops->accept(
		self, remote_addr, remote_len, remote_port, ret_obj);
}
