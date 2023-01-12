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

#ifndef _TSKT_OPS_H_
#define _TSKT_OPS_H_


#include <transport-socket/tskt.h>


struct tskt_socket_ops;


struct tskt_socket {
	const struct tskt_socket_ops *ops;
	size_t rxpkt_max_size;
	struct tpkt_list *rxpkt_free;
};


enum tskt_option {
	TSKT_OPT_RXBUF_SIZE,
	TSKT_OPT_TXBUF_SIZE,
	TSKT_OPT_CLASS_SELECTOR,
	TSKT_OPT_NODELAY,
	TSKT_OPT_ERROR,
	TSKT_OPT_RESET,
};


struct tskt_socket_ops {
	int (*destroy)(struct tskt_socket *self);
	struct pomp_loop *(*get_loop)(struct tskt_socket *self);
	int (*get_fd)(struct tskt_socket *self);
	int (*set_fd_cb)(struct tskt_socket *self,
			 pomp_fd_event_cb_t fd_cb,
			 void *userdata);
	int (*get_local_addr)(struct tskt_socket *self,
			      char *str,
			      size_t len,
			      uint16_t *port);
	int (*get_remote_addr)(struct tskt_socket *self,
			       char *str,
			       size_t len,
			       uint16_t *port);
	int (*set_remote_addr)(struct tskt_socket *self,
			       const char *addr,
			       uint16_t port);
	int (*get_option)(struct tskt_socket *self, enum tskt_option option);
	int (*set_option)(struct tskt_socket *self,
			  enum tskt_option option,
			  int value);
	int (*get_rxpkt_max_size)(struct tskt_socket *self);
	int (*set_rxpkt_max_size)(struct tskt_socket *self, size_t max_size);
	ssize_t (*read)(struct tskt_socket *self,
			void *buf,
			size_t cap,
			uint64_t *ts_us);
	ssize_t (*write)(struct tskt_socket *self, const void *buf, size_t len);
	ssize_t (*readv)(struct tskt_socket *self,
			 const struct iovec *iov,
			 size_t iov_len,
			 uint64_t *ts_us);
	ssize_t (*writev)(struct tskt_socket *self,
			  const struct iovec *iov,
			  size_t iov_len);
	ssize_t (*readmv)(struct tskt_socket *self,
			  struct tskt_miovec *miov,
			  size_t mlen);
	ssize_t (*writemv)(struct tskt_socket *self,
			   struct tskt_miovec *miov,
			   size_t mlen);
	ssize_t (*write_cs)(struct tskt_socket *self,
			    const void *buf,
			    size_t len,
			    int cs);
	ssize_t (*writev_cs)(struct tskt_socket *self,
			     const struct iovec *iov,
			     size_t iov_len,
			     int cs);
	int (*read_pkt)(struct tskt_socket *self, struct tpkt_packet *pkt);
	int (*read_pkt_alloc)(struct tskt_socket *self,
			      struct tpkt_packet **ret_pkt);
	int (*write_pkt)(struct tskt_socket *self, struct tpkt_packet *pkt);
	ssize_t (*read_pkt_list)(struct tskt_socket *self,
				 struct tpkt_list *list,
				 size_t max_pkts);
	ssize_t (*write_pkt_list)(struct tskt_socket *self,
				  struct tpkt_list *list);
	int (*set_event_cb)(struct tskt_socket *self,
			    uint32_t events,
			    tskt_socket_event_cb_t cb,
			    void *userdata);
	int (*update_events)(struct tskt_socket *self,
			     uint32_t events_to_add,
			     uint32_t events_to_remove);
	int (*connect)(struct tskt_socket *self,
		       const char *local_addr,
		       uint16_t local_port,
		       const char *remote_addr,
		       uint16_t remote_port);
	int (*listen)(struct tskt_socket *self,
		      const char *local_addr,
		      uint16_t local_port);
	int (*accept)(struct tskt_socket *self,
		      char *remote_addr,
		      size_t remote_len,
		      uint16_t *remote_port,
		      struct tskt_socket **ret_obj);
};


TSKT_API struct tpkt_packet *tskt_socket_rxpkt_alloc(struct tskt_socket *self);


TSKT_API void tskt_socket_rxpkt_free(struct tskt_socket *self,
				     struct tpkt_packet *pkt);


#endif /* !_TSKT_OPS_H_ */
