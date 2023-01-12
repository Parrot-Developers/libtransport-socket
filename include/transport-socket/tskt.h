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

#ifndef _TSKT_H_
#define _TSKT_H_

#include <inttypes.h>
#include <stdint.h>
#include <unistd.h>

#include <libpomp.h>
#include <transport-packet/tpkt.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef TSKT_API_EXPORTS
#	ifdef _WIN32
#		define TSKT_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define TSKT_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !TSKT_API_EXPORTS */
#	define TSKT_API
#endif /* !TSKT_API_EXPORTS */


/* Forward declarations */
struct tskt_socket;
struct iovec;


/** Default receive packet max size. */
#define TSKT_SOCKET_RXPKT_MAX_SIZE_DEFAULT 1500


/** Multiple messages data. */
struct tskt_miovec {
	struct iovec *iov; /**< array of iovecs */
	size_t iovlen; /**< number of iovecs in iov array */
	size_t len; /**< number of bytes received/sent */
	uint64_t ts_us; /**< timestamp of received message */
};


/**
 * Socket event callback prototype.
 * @param self : socket object handle.
 * @param revents : event that occurred. @see pomp_fd_event.
 * @param userdata : callback user data.
 */
typedef void (*tskt_socket_event_cb_t)(struct tskt_socket *self,
				       uint32_t revents,
				       void *userdata);


/**
 * Create an UDP socket object.
 * For receiving multicast, the mcast_addr parameter must be a valid multicast
 * address and the local_address should be the address of the local interface
 * to use (otherwise the system will choose the interface itself).
 * For sending multicast, the remote_addr parameter must be a valid multicast
 * address and the local_address should be the address of the local interface
 * to use (otherwise the system will choose the interface itself).
 * The fd_cb callback function is optional and can be NULL; the callback can
 * be set later using the tskt_socket_set_fd_cb() function.
 * The object handle is returned through the ret_obj parameter.
 * When no longer needed, the object must be freed using the
 * tskt_socket_destroy() function.
 * @param local_addr: local IP address (NULL, empty, and "0.0.0.0" mean any)
 * @param local_port: local port (0 means any), if not 0, the specified port
 *                    is used if available. If 0, or specified port is not
 *                    available, a port will be allocated and returned through
 *                    this parameter (input/output)
 * @param remote_addr: remote IP address (NULL, empty, and "0.0.0.0" mean
 *                     unknown)
 * @param remote_port: remote port (0 means unknown)
 * @param mcast_addr: multicat IP address (NULL, empty, and "0.0.0.0" mean
 *                    unknown)
 * @param loop: pomp loop to use
 * @param fd_cb: pomp file descriptor event callback function
 *               (optional, can be NULL)
 * @param userdata: fd event callback user data (optional, can be NULL)
 * @param ret_obj: socket object handle (output)
 * @return 0 on success, negative errno value in case of error
 * @note if successful, local_port should be read on return, to retrieve the
 *       allocated port, even if it was specified with non 0 value.
 */
TSKT_API int tskt_socket_new(const char *local_addr,
			     uint16_t *local_port,
			     const char *remote_addr,
			     uint16_t remote_port,
			     const char *mcast_addr,
			     struct pomp_loop *loop,
			     pomp_fd_event_cb_t fd_cb,
			     void *userdata,
			     struct tskt_socket **ret_obj);


/**
 * Create a TCP socket object.
 * The socket can be configured as a server by calling tskt_socket_listen,
 * or as a client by calling tskt_socket_connect.
 * The object handle is returned through the ret_obj parameter.
 * When no longer needed, the object must be freed using the
 * tskt_socket_destroy() function.
 * @param loop: pomp loop to use
 * @param ret_obj: socket object handle (output)
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_new_tcp(struct pomp_loop *loop,
				 struct tskt_socket **ret_obj);


/**
 * Free a socket object.
 * This function frees all resources associated with a socket object.
 * @param self: socket object handle
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_destroy(struct tskt_socket *self);


/**
 * Get the pomp loop used by the socket.
 * @param self: socket object handle
 * @return pomp loop on success, NULL in case of error
 */
TSKT_API struct pomp_loop *tskt_socket_get_loop(struct tskt_socket *self);


/**
 * Get the socket file descriptor.
 * @param self: socket object handle
 * @return file descriptor on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_get_fd(struct tskt_socket *self);


/**
 * Set the file descriptor callback function.
 * This function sets or updates the file descriptor callback function
 * for the socket, replacing any previously registered function. If fd_cb
 * is NULL, any previously registered callback function is unregistered.
 * @param self: socket object handle
 * @param fd_cb: pomp file descriptor event callback function
 *               (optional, can be NULL)
 * @param userdata: fd event callback user data (optional, can be NULL)
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_set_fd_cb(struct tskt_socket *self,
				   pomp_fd_event_cb_t fd_cb,
				   void *userdata);


/**
 * Get the local address.
 * This function fills the str array with the null-terminated local address.
 * The string must have been previously allocated. The function writes up to
 * len characters.
 * @param self: socket object handle
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int
tskt_socket_get_local_addr(struct tskt_socket *self, char *str, size_t len);


/**
 * Get the local port.
 * @param self: socket object handle
 * @return local port on success, 0 in case of error
 */
TSKT_API uint16_t tskt_socket_get_local_port(struct tskt_socket *self);


/**
 * Get the remote address.
 * This function fills the str array with the null-terminated remote address.
 * The string must have been previously allocated. The function writes up to
 * len characters.
 * @param self: socket object handle
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int
tskt_socket_get_remote_addr(struct tskt_socket *self, char *str, size_t len);


/**
 * Get the remote port.
 * @param self: socket object handle
 * @return remote port on success, 0 in case of error
 */
TSKT_API uint16_t tskt_socket_get_remote_port(struct tskt_socket *self);


/**
 * Set the remote address and port.
 * This function sets the remote address and port. It is useful for example
 * when the socket is created with unknown remote address and port values and
 * these are known later (for exemple through the RTSP protocol).
 * @param self: socket object handle
 * @param addr: remote IP address
 * @param port: remote port
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_set_remote(struct tskt_socket *self,
				    const char *addr,
				    uint16_t port);


/**
 * Get the socket receive buffer size.
 * @param self: socket object handle
 * @return socket buffer size in bytes on success, negative errno value
 *         in case of error
 */
TSKT_API int tskt_socket_get_rxbuf_size(struct tskt_socket *self);


/**
 * Set the socket receive buffer size.
 * @param self: socket object handle
 * @param size: socket buffer size in bytes
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_set_rxbuf_size(struct tskt_socket *self, size_t size);


/**
 * Get the socket send buffer size.
 * @param self: socket object handle
 * @return socket buffer size in bytes on success, negative errno value
 *         in case of error
 */
TSKT_API int tskt_socket_get_txbuf_size(struct tskt_socket *self);


/**
 * Set the socket send buffer size.
 * @param self: socket object handle
 * @param size: socket buffer size in bytes
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_set_txbuf_size(struct tskt_socket *self, size_t size);


/**
 * Get the socket class selector codepoint (RFC 2474).
 * @param self: socket object handle
 * @return class selector codepoint on success, negative errno value
 *         in case of error
 */
TSKT_API int tskt_socket_get_class_selector(struct tskt_socket *self);


/**
 * Set the socket class selector codepoint (RFC 2474).
 * @param self: socket object handle
 * @param cls: class selector codepoint
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_set_class_selector(struct tskt_socket *self, int cls);


/**
 * Get the socket no delay option.
 * @param self: socket object handle
 * @return no delay option value on success,
 *         negative errno value in case of error.
 */
TSKT_API int tskt_socket_get_nodelay(struct tskt_socket *self);


/**
 * Set the socket no delay option.
 * @param self: socket object handle
 * @param nodelay: no delay option value
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_set_nodelay(struct tskt_socket *self, int nodelay);


/**
 * Get and clear the socket pending error.
 * @param self: socket object handle
 * @return 0 if no error, a positive error code if socket error is pending,
 *         or a negative errno value if the function call has failed
 */
TSKT_API int tskt_socket_get_error(struct tskt_socket *self);


/**
 * Get the socket reset on close option.
 * @param self: socket object handle
 * @return reset on close option value on success,
 *         negative errno value in case of error.
 */
TSKT_API int tskt_socket_get_reset(struct tskt_socket *self);


/**
 * Set the socket reset on close option.
 * When this option is set, a RESET is sent to the peer when
 * the socket is closed by tskt_socket_destroy instead of
 * sending a normal close indication (connection-oriented sockets only).
 * @param self: socket object handle
 * @param reset: reset on close option value
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_set_reset(struct tskt_socket *self, int reset);


/**
 * Get the maximum size of packets received by tskt_socket_read_pkt_list
 * and tskt_socket_read_pkt_alloc.
 * @param self: socket object handle
 * @return maximum packet size value on success,
 *         negative errno value in case of error.
 */
TSKT_API int tskt_socket_get_rxpkt_max_size(struct tskt_socket *self);


/**
 * Set the maximum size of packets received by tskt_socket_read_pkt_list
 * and tskt_socket_read_pkt_alloc.
 * @param self: socket object handle
 * @param max_size: maximum packet size
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_set_rxpkt_max_size(struct tskt_socket *self,
					    size_t max_size);


/**
 * Read from the socket.
 * @param self: socket object handle
 * @param buf: pointer to a data buffer
 * @param cap: data buffer capacity in bytes
 * @param ts_us: optional reception timestamp in microseconds (output)
 * @return read bytes count on success, negative errno value in case of error
 */
TSKT_API ssize_t tskt_socket_read(struct tskt_socket *self,
				  void *buf,
				  size_t cap,
				  uint64_t *ts_us);


/**
 * Write into the socket.
 * @param self: socket object handle
 * @param buf: pointer to the data to write
 * @param len: size to write in bytes
 * @return written bytes count on success, negative errno value in case of error
 */
TSKT_API ssize_t tskt_socket_write(struct tskt_socket *self,
				   const void *buf,
				   size_t len);


/**
 * Read buffers from the socket.
 * @param self: socket object handle
 * @param iov: array of data buffers
 * @param iov_len: number of buffers
 * @param ts_us: optional reception timestamp in microseconds (output)
 * @return read bytes count on success, negative errno value in case of error
 */
TSKT_API ssize_t tskt_socket_readv(struct tskt_socket *self,
				   const struct iovec *iov,
				   size_t iov_len,
				   uint64_t *ts_us);


/**
 * Write buffers into the socket.
 * @param self: socket object handle
 * @param iov: array of buffers to write
 * @param iov_len: number of buffers
 * @return written bytes count on success, negative errno value in case of error
 */
TSKT_API ssize_t tskt_socket_writev(struct tskt_socket *self,
				    const struct iovec *iov,
				    size_t iov_len);


/**
 * Write into the socket and set class selector of packet.
 * @param self: socket object handle
 * @param buf: pointer to the data to write
 * @param len: size to write in bytes
 * @param cs: class selector, -1 to use default socket class selector
 * @return written bytes count on success, negative errno value in case of error
 */
TSKT_API ssize_t tskt_socket_write_cs(struct tskt_socket *self,
				      const void *buf,
				      size_t len,
				      int cs);


/**
 * Write buffers into the socket and set class selector of packet.
 * @param self: socket object handle
 * @param iov: array of buffers to write
 * @param iov_len: number of buffers
 * @param cs: class selector, -1 to use default socket class selector
 * @return written bytes count on success, negative errno value in case of error
 */
TSKT_API ssize_t tskt_socket_writev_cs(struct tskt_socket *self,
				       const struct iovec *iov,
				       size_t iov_len,
				       int cs);


/**
 * Read multiple messages from the socket.
 * @param self: socket object handle
 * @param miov: array of messages to store the read data
 * @param mlen: number of messages
 * @return number of messages read on success, negative errno value in case of
 * error
 */
TSKT_API ssize_t tskt_socket_readmv(struct tskt_socket *self,
				    struct tskt_miovec *miov,
				    size_t mlen);


/**
 * Write multiple messages into the socket.
 * @param self: socket object handle
 * @param miov: array of messages to write
 * @param mlen: number of messages
 * @return number of messages written on success, negative errno value in case
 * of error
 */
TSKT_API ssize_t tskt_socket_writemv(struct tskt_socket *self,
				     struct tskt_miovec *miov,
				     size_t mlen);


/**
 * Read a packet from the socket.
 * @param self: socket object handle
 * @param pkt: pointer to a packet object
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_read_pkt(struct tskt_socket *self,
				  struct tpkt_packet *pkt);


/**
 * Read a packet from the socket, the packet is allocated by the function.
 * The maximum size of received packet is set by
 * tskt_socket_set_rxpkt_max_size.
 * @param self: socket object handle
 * @param ret_pkt: read packet object (output)
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_read_pkt_alloc(struct tskt_socket *self,
					struct tpkt_packet **ret_pkt);


/**
 * Write a packet into the socket.
 * @param self: socket object handle
 * @param pkt: pointer to a packet object
 * @return 0 on success, negative errno value in case of error
 */
TSKT_API int tskt_socket_write_pkt(struct tskt_socket *self,
				   struct tpkt_packet *pkt);


/**
 * Read a list of packets from the socket.
 * The received packets are added at the end of the list.
 * The maximum size of received packets is set by
 * tskt_socket_set_rxpkt_max_size.
 * @param self: socket object handle
 * @param list: pointer to a list of packets
 * @param max_pkts: maximum number of packets to read
 * @return number of read packets on success,
 *         negative errno value in case of error
 */
TSKT_API ssize_t tskt_socket_read_pkt_list(struct tskt_socket *self,
					   struct tpkt_list *list,
					   size_t max_pkts);


/**
 * Write a list of packet into the socket.
 * The written packet are removed from the start of the list and are
 * unreferenced by this function
 * @param self: socket object handle
 * @param list: pointer to a list of packets
 * @return number of written packets on success,
 *         negative errno value in case of error
 */
TSKT_API ssize_t tskt_socket_write_pkt_list(struct tskt_socket *self,
					    struct tpkt_list *list);


/**
 * Set event notification callback
 * @param self: socket object handle
 * @param events: events to monitor. @see pomp_fd_event.
 * @param cb: callback for notifications, NULL to remove notifications.
 * @param userdata: user data to give to the callback.
 * @return 0 on success, negative errno value in case of error.
 */
TSKT_API int tskt_socket_set_event_cb(struct tskt_socket *self,
				      uint32_t events,
				      tskt_socket_event_cb_t cb,
				      void *userdata);


/**
 * Modify the set of events to monitor for a socket.
 * @param self: socket object handle
 * @param events_to_add: events to add. @see pomp_fd_event.
 * @param events_to_remove: events to remove. @see pomp_fd_event.
 * @return 0 on success, negative errno value in case of error.
 */
TSKT_API int tskt_socket_update_events(struct tskt_socket *self,
				       uint32_t events_to_add,
				       uint32_t events_to_remove);


/**
 * Connect a socket to a peer.
 * For a stream (TCP) socket, this initiates a connection to the peer.
 * For a datagram (UDP) socket, this sets the remote address like
 * tskt_socket_set_remote() but the address stored in packets to send
 * is ignored, this also sets the only address from which datagrams
 * are received. A datagram socket can be disconnected by specifying
 * a NULL remote address.
 * @param self: socket object handle
 * @param local_addr: local address, NULL or "0.0.0.0" to automatically
 *                    select a valid address.
 * @param local_port: local port (host byte order), 0 means any,
 *                    if 0 an available port is opened.
 * @param remote_addr: remote address of the peer to connect to,
 *                     NULL to disconnect a datagram socket.
 * @param remote_port: remote port of the peer (host byte order),
 *                     0 is invalid if remote address is not NULL.
 * @return 0 on success, negative errno value in case of error.
 */
TSKT_API int tskt_socket_connect(struct tskt_socket *self,
				 const char *local_addr,
				 uint16_t local_port,
				 const char *remote_addr,
				 uint16_t remote_port);


/**
 * Bind local address and port, and listen for connections on a socket.
 * @param self: socket object handle
 * @param local_addr: local IP address, NULL or "0.0.0.0" for any address.
 * @param local_port : local port (host byte order), if 0 an available
 *                     port is opened, call tskt_socket_get_local_port()
 *                     to retrieve the allocated port value.
 * @return 0 on success, negative errno value in case of error.
 */
TSKT_API int tskt_socket_listen(struct tskt_socket *self,
				const char *local_addr,
				uint16_t local_port);


/**
 * Accept a connection on a listening socket.
 * @param self: socket object handle
 * @param remote_addr: pointer to the remote address buffer to write to
 *                     (output), can be NULL.
 * @param remote_len: maximum length of the string
 * @param remote_port : the remote port of the peer (output), can be NULL.
 * @param ret_obj : new socket to use for the accepted connection (output).
 * @return 0 on success, negative errno value in case of error.
 */
TSKT_API int tskt_socket_accept(struct tskt_socket *self,
				char *remote_addr,
				size_t remote_len,
				uint16_t *remote_port,
				struct tskt_socket **ret_obj);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_TSKT_H_ */
