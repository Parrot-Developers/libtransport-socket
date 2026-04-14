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

#ifndef _TSKT_CLIENT_H_
#define _TSKT_CLIENT_H_

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
struct pomp_loop;
struct tskt_client;


/**
 * Client connection events.
 *
 * These events are delivered to the `event_cb` callback to notify the
 * application about changes in the client connection state. They are
 * invoked from the client’s event loop context.
 */
enum tskt_client_event {
	/** The client has successfully connected to the peer. */
	TSKT_CLIENT_EVENT_CONNECTED = 0,

	/** The client has been disconnected from the peer. */
	TSKT_CLIENT_EVENT_DISCONNECTED,

	/** The client is ready to send data. */
	TSKT_CLIENT_EVENT_READY_TO_SEND,
};


/**
 * Client context callbacks.
 */
struct tskt_client_cbs {
	/**
	 * Event callback (Required).
	 *
	 * This callback is called to notify the application of a change in the
	 * client connection state. Typical events include connection
	 * established or disconnected. The callback is called from the context
	 * of the client event loop.
	 *
	 * @param self     The client context that generated the event.
	 * @param event    The event that occurred. See enum tskt_client_event.
	 * @param sock     The socket currently used by the client, or NULL if
	 *                 no socket is active.
	 * @param userdata User data provided when the client was created
	 *                 (tskt_client_new).
	 */
	void (*event_cb)(struct tskt_client *self,
			 enum tskt_client_event event,
			 struct tskt_socket *sock,
			 void *userdata);

	/**
	 * Data callback (Required).
	 *
	 * Called when data has been received from the peer. The received packet
	 * is provided as a tpkt_packet object. The callback is responsible for
	 * processing the packet but should not destroy it unless documented
	 * otherwise.
	 *
	 * @param self     The client context that received the data.
	 * @param pkt      The received packet.
	 * @param userdata User data provided when the client was created
	 *                 (tskt_client_new).
	 */
	void (*data_cb)(struct tskt_client *self,
			struct tpkt_packet *pkt,
			void *userdata);

	/**
	 * Socket created callback (Optional).
	 *
	 * This callback, if set, is called right after a new socket has been
	 * successfully created but before any connection attempt is made. It
	 * allows the caller to perform additional configuration on the socket
	 * (e.g., set socket options or register it with an event loop).
	 *
	 * @param self     The client context that owns the socket.
	 * @param sock     The newly created socket object.
	 * @param fd       The underlying file descriptor of the socket.
	 * @param userdata User data provided when the client context was
	 *                 created (tskt_client_new).
	 */
	void (*socket_created_cb)(struct tskt_client *self,
				  struct tskt_socket *sock,
				  int fd,
				  void *userdata);

	/**
	 * Socket upgrade callback (Optional).
	 *
	 * Called right after a successful connect(), before any read/write
	 * occurs. The application may use this callback to wrap the socket
	 * (e.g. create a TLS socket on top of the connected socket).
	 *
	 * The callback must return:
	 *   - a valid socket to use (may be the same as the input),
	 *   - NULL on error (the connection will fail).
	 *
	 * @param self      Client instance.
	 * @param sock      Newly connected socket (non-NULL).
	 * @param userdata  User data provided when creating the client.
	 *
	 * @return The socket to use after upgrade, or NULL on error.
	 */
	struct tskt_socket *(*socket_upgrade_cb)(struct tskt_client *self,
						 struct tskt_socket *sock,
						 void *userdata);
};


/**
 * Create a new client context.
 *
 * Allocates and initializes a new client context. The returned context
 * must later be freed with tskt_client_destroy().
 *
 * @param loop      Pomp event loop to use for asynchronous operations.
 * @param cbs       Callbacks structure. Required callbacks must be set.
 * @param userdata  User-defined pointer that will be passed to all callbacks.
 * @param ret_obj   Output pointer to receive the allocated client context.
 *
 * @return 0 on success, negative errno value in case of error.
 */
TSKT_API int tskt_client_new(struct pomp_loop *loop,
			     struct tskt_client_cbs cbs,
			     void *userdata,
			     struct tskt_client **ret_obj);


/**
 * Start a client and initiate connection to a remote peer.
 *
 * This function triggers the connection process. If the connection cannot
 * be completed immediately, the client will automatically retry until
 * success or stop is called. Events and data are delivered via callbacks.
 *
 * @param self         Client context obtained from tskt_client_new().
 * @param local_addr   Local IP address to bind (NULL, empty, or "0.0.0.0" means
 *                     any).
 * @param local_port   Local port to bind (0 means any). If the requested port
 *                     is unavailable, a different port may be allocated.
 * @param remote_addr  Remote IP address (NULL, empty, or "0.0.0.0" means
 *                     unknown).
 * @param remote_port  Remote port (0 means unknown).
 *
 * @return 0 on success, negative errno value on failure.
 */
TSKT_API int tskt_client_connect(struct tskt_client *self,
				 const char *local_addr,
				 uint16_t local_port,
				 const char *remote_addr,
				 uint16_t remote_port);


/**
 * Send data through the client's socket.
 *
 * This function attempts to send data through the client's currently connected
 * socket. If the socket's transmit buffer is full, it will return -EAGAIN.
 * The client will automatically monitor for write readiness and notify the
 * application with a TSKT_CLIENT_EVENT_READY_TO_SEND event when the socket is
 * ready for writing again.
 *
 * @param self Client context obtained from tskt_client_new().
 * @param buf  Buffer containing the data to send.
 * @param len  Length of the data to send.
 *
 * @return number of bytes sent on success, -EAGAIN if the socket is not ready
 *         for writing, or another negative errno value on other failures.
 */
TSKT_API ssize_t tskt_client_send(struct tskt_client *self,
				  const void *buf,
				  size_t len);


/**
 * Send a packet through the client's socket.
 *
 * This function attempts to send a transport packet through the client's
 * currently connected socket. If the socket's transmit buffer is full,
 * it will return -EAGAIN. The client will automatically monitor for write
 * readiness and notify the application with a TSKT_CLIENT_EVENT_READY_TO_SEND
 * event when the socket is ready for writing again.
 *
 * @param self Client context obtained from tskt_client_new().
 * @param pkt  The transport packet to send. The caller is responsible for
 *             managing the packet's reference count.
 *
 * @return 0 on success, -EAGAIN if the socket is not ready for writing,
 *         or another negative errno value on other failures.
 */
TSKT_API int tskt_client_send_pkt(struct tskt_client *self,
				  struct tpkt_packet *pkt);


/**
 * Stop the client.
 *
 * Disconnects the remote peer and stops the client event loop. Any active
 * sockets are closed, and relevant events (TSKT_CLIENT_EVENT_DISCONNECTED) are
 * delivered to callbacks. The client context itself is not freed and can be
 * reused by calling tskt_client_connect() again.
 *
 * @param self  Client context.
 *
 * @return 0 on success, negative errno value on failure.
 */
TSKT_API int tskt_client_stop(struct tskt_client *self);


/**
 * Destroy a client context.
 *
 * Frees all resources associated with the client, including the transport
 * socket. After this call, the client context pointer becomes invalid and
 * must not be used again.
 *
 * @param self  Client context to destroy.
 *
 * @return 0 on success, negative errno value on failure.
 */
TSKT_API int tskt_client_destroy(struct tskt_client *self);


/**
 * Get the transport socket currently used by the client.
 *
 * Provides access to the underlying transport socket. The returned socket
 * must not be destroyed by the caller; the client context owns it.
 *
 * @param self  Client context.
 *
 * @return Pointer to the transport socket, or NULL if no socket is active
 *         or an error occurred.
 */
TSKT_API struct tskt_socket *tskt_client_get_socket(struct tskt_client *self);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_TSKT_CLIENT_H_ */
