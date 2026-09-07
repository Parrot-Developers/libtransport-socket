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

#include "tskt_test.h"

#include <string.h>


static inline struct fake_sock *to_fake_sock(struct tskt_socket *self)
{
	return (struct fake_sock *)self;
}


static int fake_sock_destroy(struct tskt_socket *self)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->destroy_called = true;
	return fake->destroy_ret;
}


static struct pomp_loop *fake_sock_get_loop(struct tskt_socket *self)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->get_loop_called = true;
	return fake->get_loop_ret;
}


static int fake_sock_get_fd(struct tskt_socket *self)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->get_fd_called = true;
	return fake->get_fd_ret;
}


static int fake_sock_set_fd_cb(struct tskt_socket *self,
			       pomp_fd_event_cb_t fd_cb,
			       void *userdata)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->set_fd_cb_called = true;
	fake->set_fd_cb_cb = fd_cb;
	fake->set_fd_cb_userdata = userdata;
	return fake->set_fd_cb_ret;
}


static int fake_sock_get_local_addr(struct tskt_socket *self,
				    char *str,
				    size_t len,
				    uint16_t *port)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->get_local_addr_called = true;
	fake->get_local_addr_len = len;
	if (str != NULL && len > 0)
		snprintf(str,
			 len,
			 "%s",
			 fake->get_local_addr_str ? fake->get_local_addr_str
						  : "");
	if (port != NULL)
		*port = fake->get_local_addr_port;
	return fake->get_local_addr_ret;
}


static int fake_sock_get_remote_addr(struct tskt_socket *self,
				     char *str,
				     size_t len,
				     uint16_t *port)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->get_remote_addr_called = true;
	fake->get_remote_addr_len = len;
	if (str != NULL && len > 0)
		snprintf(str,
			 len,
			 "%s",
			 fake->get_remote_addr_str ? fake->get_remote_addr_str
						   : "");
	if (port != NULL)
		*port = fake->get_remote_addr_port;
	return fake->get_remote_addr_ret;
}


static int fake_sock_set_remote_addr(struct tskt_socket *self,
				     const char *addr,
				     uint16_t port)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->set_remote_addr_called = true;
	snprintf(fake->set_remote_addr_addr,
		 sizeof(fake->set_remote_addr_addr),
		 "%s",
		 addr ? addr : "");
	fake->set_remote_addr_port = port;
	return fake->set_remote_addr_ret;
}


static int fake_sock_get_option(struct tskt_socket *self,
				enum tskt_option option)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->get_option_called = true;
	fake->get_option_option = option;
	return fake->get_option_ret[option];
}


static int fake_sock_set_option(struct tskt_socket *self,
				enum tskt_option option,
				int value)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->set_option_called = true;
	fake->set_option_option = option;
	fake->set_option_value = value;
	return fake->set_option_ret;
}


static int fake_sock_get_rxpkt_max_size(struct tskt_socket *self)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->get_rxpkt_max_size_called = true;
	return fake->get_rxpkt_max_size_ret;
}


static int fake_sock_set_rxpkt_max_size(struct tskt_socket *self,
					size_t max_size)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->set_rxpkt_max_size_called = true;
	fake->set_rxpkt_max_size_value = max_size;
	return fake->set_rxpkt_max_size_ret;
}


static ssize_t
fake_sock_read(struct tskt_socket *self, void *buf, size_t cap, uint64_t *ts_us)
{
	struct fake_sock *fake = to_fake_sock(self);
	size_t n;
	ssize_t ret = fake->read_ret;
	const void *data = fake->read_data;
	size_t data_len = fake->read_data_len;

	fake->read_called = true;
	fake->read_call_count++;
	if (fake->read_call_count > 1 && fake->read_ret2_set) {
		ret = fake->read_ret2;
		data = fake->read_data2;
		data_len = fake->read_data2_len;
	}
	if (buf != NULL && data != NULL) {
		n = data_len < cap ? data_len : cap;
		memcpy(buf, data, n);
	}
	if (ts_us != NULL)
		*ts_us = fake->read_ts_us;
	return ret;
}


static ssize_t
fake_sock_write(struct tskt_socket *self, const void *buf, size_t len)
{
	struct fake_sock *fake = to_fake_sock(self);
	size_t n;
	fake->write_called = true;
	if (buf != NULL) {
		n = len < sizeof(fake->write_data) ? len
						   : sizeof(fake->write_data);
		memcpy(fake->write_data, buf, n);
		fake->write_data_len = n;
	}
	return fake->write_ret_all ? (ssize_t)len : fake->write_ret;
}


static ssize_t fake_sock_readv(struct tskt_socket *self,
			       const struct iovec *iov,
			       size_t iov_len,
			       uint64_t *ts_us)
{
	struct fake_sock *fake = to_fake_sock(self);
	(void)iov;
	(void)iov_len;
	fake->readv_called = true;
	if (ts_us != NULL)
		*ts_us = fake->read_ts_us;
	return fake->readv_ret;
}


static ssize_t fake_sock_writev(struct tskt_socket *self,
				const struct iovec *iov,
				size_t iov_len)
{
	struct fake_sock *fake = to_fake_sock(self);
	(void)iov;
	(void)iov_len;
	fake->writev_called = true;
	return fake->writev_ret;
}


static ssize_t fake_sock_readmv(struct tskt_socket *self,
				struct tskt_miovec *miov,
				size_t mlen)
{
	struct fake_sock *fake = to_fake_sock(self);
	(void)miov;
	(void)mlen;
	fake->readmv_called = true;
	return fake->readmv_ret;
}


static ssize_t fake_sock_writemv(struct tskt_socket *self,
				 struct tskt_miovec *miov,
				 size_t mlen)
{
	struct fake_sock *fake = to_fake_sock(self);
	(void)miov;
	(void)mlen;
	fake->writemv_called = true;
	return fake->writemv_ret;
}


static ssize_t fake_sock_write_cs(struct tskt_socket *self,
				  const void *buf,
				  size_t len,
				  int cs)
{
	struct fake_sock *fake = to_fake_sock(self);
	(void)buf;
	(void)len;
	fake->write_cs_called = true;
	fake->write_cs_cs = cs;
	return fake->write_cs_ret;
}


static ssize_t fake_sock_writev_cs(struct tskt_socket *self,
				   const struct iovec *iov,
				   size_t iov_len,
				   int cs)
{
	struct fake_sock *fake = to_fake_sock(self);
	(void)iov;
	(void)iov_len;
	fake->writev_cs_called = true;
	fake->writev_cs_cs = cs;
	return fake->writev_cs_ret;
}


static int fake_sock_read_pkt(struct tskt_socket *self, struct tpkt_packet *pkt)
{
	struct fake_sock *fake = to_fake_sock(self);
	(void)pkt;
	fake->read_pkt_called = true;
	return fake->read_pkt_ret;
}


static int fake_sock_read_pkt_alloc(struct tskt_socket *self,
				    struct tpkt_packet **ret_pkt)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->read_pkt_alloc_called = true;
	if (fake->read_pkt_alloc_ret == 0 && ret_pkt != NULL)
		*ret_pkt = fake->read_pkt_alloc_ret_obj;
	return fake->read_pkt_alloc_ret;
}


static int fake_sock_write_pkt(struct tskt_socket *self,
			       struct tpkt_packet *pkt)
{
	struct fake_sock *fake = to_fake_sock(self);
	(void)pkt;
	fake->write_pkt_called = true;
	return fake->write_pkt_ret;
}


static ssize_t fake_sock_read_pkt_list(struct tskt_socket *self,
				       struct tpkt_list *list,
				       size_t max_pkts)
{
	struct fake_sock *fake = to_fake_sock(self);
	(void)list;
	(void)max_pkts;
	fake->read_pkt_list_called = true;
	return fake->read_pkt_list_ret;
}


static ssize_t fake_sock_write_pkt_list(struct tskt_socket *self,
					struct tpkt_list *list)
{
	struct fake_sock *fake = to_fake_sock(self);
	(void)list;
	fake->write_pkt_list_called = true;
	return fake->write_pkt_list_ret;
}


static int fake_sock_set_event_cb(struct tskt_socket *self,
				  uint32_t events,
				  tskt_socket_event_cb_t cb,
				  void *userdata)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->set_event_cb_called = true;
	fake->set_event_cb_events = events;
	fake->event_cb = cb;
	fake->event_cb_userdata = userdata;
	return fake->set_event_cb_ret;
}


static int fake_sock_update_events(struct tskt_socket *self,
				   uint32_t events_to_add,
				   uint32_t events_to_remove)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->update_events_called = true;
	fake->update_events_add = events_to_add;
	fake->update_events_remove = events_to_remove;
	return fake->update_events_ret;
}


static int fake_sock_connect(struct tskt_socket *self,
			     const char *local_addr,
			     uint16_t local_port,
			     const char *remote_addr,
			     uint16_t remote_port)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->connect_called = true;
	snprintf(fake->connect_local_addr,
		 sizeof(fake->connect_local_addr),
		 "%s",
		 local_addr ? local_addr : "");
	fake->connect_local_port = local_port;
	snprintf(fake->connect_remote_addr,
		 sizeof(fake->connect_remote_addr),
		 "%s",
		 remote_addr ? remote_addr : "");
	fake->connect_remote_port = remote_port;
	return fake->connect_ret;
}


static int fake_sock_listen(struct tskt_socket *self,
			    const char *local_addr,
			    uint16_t local_port)
{
	struct fake_sock *fake = to_fake_sock(self);
	fake->listen_called = true;
	snprintf(fake->listen_local_addr,
		 sizeof(fake->listen_local_addr),
		 "%s",
		 local_addr ? local_addr : "");
	fake->listen_local_port = local_port;
	return fake->listen_ret;
}


static int fake_sock_accept(struct tskt_socket *self,
			    char *remote_addr,
			    size_t remote_len,
			    uint16_t *remote_port,
			    struct tskt_socket **ret_obj)
{
	struct fake_sock *fake = to_fake_sock(self);
	(void)remote_addr;
	(void)remote_len;
	(void)remote_port;
	fake->accept_called = true;
	if (fake->accept_ret == 0 && ret_obj != NULL)
		*ret_obj = fake->accept_ret_obj;
	return fake->accept_ret;
}


const struct tskt_socket_ops g_fake_sock_ops = {
	.destroy = fake_sock_destroy,
	.get_loop = fake_sock_get_loop,
	.get_fd = fake_sock_get_fd,
	.set_fd_cb = fake_sock_set_fd_cb,
	.get_local_addr = fake_sock_get_local_addr,
	.get_remote_addr = fake_sock_get_remote_addr,
	.set_remote_addr = fake_sock_set_remote_addr,
	.get_option = fake_sock_get_option,
	.set_option = fake_sock_set_option,
	.get_rxpkt_max_size = fake_sock_get_rxpkt_max_size,
	.set_rxpkt_max_size = fake_sock_set_rxpkt_max_size,
	.read = fake_sock_read,
	.write = fake_sock_write,
	.readv = fake_sock_readv,
	.writev = fake_sock_writev,
	.readmv = fake_sock_readmv,
	.writemv = fake_sock_writemv,
	.write_cs = fake_sock_write_cs,
	.writev_cs = fake_sock_writev_cs,
	.read_pkt = fake_sock_read_pkt,
	.read_pkt_alloc = fake_sock_read_pkt_alloc,
	.write_pkt = fake_sock_write_pkt,
	.read_pkt_list = fake_sock_read_pkt_list,
	.write_pkt_list = fake_sock_write_pkt_list,
	.set_event_cb = fake_sock_set_event_cb,
	.update_events = fake_sock_update_events,
	.connect = fake_sock_connect,
	.listen = fake_sock_listen,
	.accept = fake_sock_accept,
};


void fake_sock_init(struct fake_sock *fake)
{
	memset(fake, 0, sizeof(*fake));
	fake->base.ops = &g_fake_sock_ops;
}


static inline struct fake_resolv *to_fake_resolv(struct tskt_resolv *self)
{
	return (struct fake_resolv *)self;
}


static void fake_resolv_destroy(struct tskt_resolv *self)
{
	struct fake_resolv *fake = to_fake_resolv(self);
	fake->destroy_called = true;
}


static int fake_resolv_getaddrinfo(struct tskt_resolv *self,
				   const char *hostname,
				   struct pomp_loop *loop,
				   tskt_resolv_cb_t cb,
				   void *userdata,
				   int *ret_id)
{
	struct fake_resolv *fake = to_fake_resolv(self);
	fake->getaddrinfo_called = true;
	snprintf(fake->getaddrinfo_hostname,
		 sizeof(fake->getaddrinfo_hostname),
		 "%s",
		 hostname ? hostname : "");
	fake->getaddrinfo_loop = loop;
	fake->getaddrinfo_cb = cb;
	fake->getaddrinfo_userdata = userdata;
	if (ret_id != NULL)
		*ret_id = fake->getaddrinfo_ret_id;
	return fake->getaddrinfo_ret;
}


static int fake_resolv_getaddrinfo6(struct tskt_resolv *self,
				    const char *hostname,
				    struct pomp_loop *loop,
				    tskt_resolv_cb_t cb,
				    void *userdata,
				    int *ret_id)
{
	struct fake_resolv *fake = to_fake_resolv(self);
	fake->getaddrinfo6_called = true;
	snprintf(fake->getaddrinfo6_hostname,
		 sizeof(fake->getaddrinfo6_hostname),
		 "%s",
		 hostname ? hostname : "");
	fake->getaddrinfo6_loop = loop;
	fake->getaddrinfo6_cb = cb;
	fake->getaddrinfo6_userdata = userdata;
	if (ret_id != NULL)
		*ret_id = fake->getaddrinfo6_ret_id;
	return fake->getaddrinfo6_ret;
}


static int fake_resolv_cancel(struct tskt_resolv *self, int id)
{
	struct fake_resolv *fake = to_fake_resolv(self);
	fake->cancel_called = true;
	fake->cancel_id = id;
	return fake->cancel_ret;
}


const struct tskt_resolv_ops g_fake_resolv_ops = {
	.destroy = fake_resolv_destroy,
	.getaddrinfo = fake_resolv_getaddrinfo,
	.getaddrinfo6 = fake_resolv_getaddrinfo6,
	.cancel = fake_resolv_cancel,
};


void fake_resolv_init(struct fake_resolv *fake)
{
	memset(fake, 0, sizeof(*fake));
	fake->base.ops = &g_fake_resolv_ops;
	fake->base.refcount = 1;
}
