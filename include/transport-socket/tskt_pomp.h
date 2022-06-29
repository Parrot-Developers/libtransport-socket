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

#ifndef _TSKT_POMP_H_
#define _TSKT_POMP_H_

#include <transport-socket/tskt.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/* Forward declarations */
struct tskt_pomp;


/** Pomp event. */
enum tskt_pomp_event {
	TSKT_POMP_EVENT_CONNECTED = 0, /**< Peer is connected */
	TSKT_POMP_EVENT_DISCONNECTED, /**< Peer is disconnected */
	TSKT_POMP_EVENT_MSG, /**< Message received from peer */
	TSKT_POMP_EVENT_ERROR, /**< Connection error */
};


/** Pomp event callback prototype.
 * @param self : pomp socket object handle.
 * @param event : event that occurred. @see tskt_pomp_event.
 * @param msg : received message when event is TSKT_POMP_EVENT_MSG.
 * @param userdata : callback user data.
 */
typedef void (*tskt_pomp_event_cb_t)(struct tskt_pomp *self,
				     enum tskt_pomp_event event,
				     const struct pomp_msg *msg,
				     void *userdata);


/**
 * Create a pomp socket object.
 * On success the transport sockets is owned by the pomp
 * socket object that is returned through the ret_obj parameter.
 * When no longer needed, the object must be freed using the
 * tskt_pomp_destroy() function, this will also free the
 * used transport sockets.
 * @param sock: transport socket to use.
 * @param cb : function to be called when events occur.
 * @param userdata : user data to give in cb.
 * @param ret_obj: pomp socket handle (output).
 * @return 0 on success, negative errno value in case of error.
 */
TSKT_API int tskt_pomp_new(struct tskt_socket *sock,
			   tskt_pomp_event_cb_t cb,
			   void *userdata,
			   struct tskt_pomp **ret_obj);


/**
 * Free a pomp socket object.
 * This function frees all resources associated with a pomp socket object,
 * including the transport socket.
 * @param self: pomp socket object handle.
 * @return 0 on success, negative errno value in case of error.
 */
TSKT_API int tskt_pomp_destroy(struct tskt_pomp *self);


/**
 * Send a message to the peer of the pomp socket.
 * @param self: pomp socket object handle.
 * @param msg : message to send.
 * @return 0 in case of success, negative errno value in case of error.
 */
TSKT_API int tskt_pomp_send_msg(struct tskt_pomp *self,
				const struct pomp_msg *msg);


/**
 * Format and send a message to the peer of the pomp socket.
 * @param self: pomp socket object handle.
 * @param msgid : message id.
 * @param fmt : format string. Can be NULL if no arguments given.
 * @param ... : message arguments.
 * @return 0 in case of success, negative errno value in case of error.
 */
TSKT_API int
tskt_pomp_send(struct tskt_pomp *self, uint32_t msgid, const char *fmt, ...);


/**
 * Format and send a message to the peer of the pomp socket.
 * @param self: pomp socket object handle.
 * @param msgid : message id.
 * @param fmt : format string. Can be NULL if no arguments given.
 * @param args : message arguments.
 * @return 0 in case of success, negative errno value in case of error.
 */
TSKT_API int tskt_pomp_sendv(struct tskt_pomp *self,
			     uint32_t msgid,
			     const char *fmt,
			     va_list args);


/**
 * Get the transport socket used by the pomp socket.
 * @param self: pomp socket object handle.
 * @return transport socket on success, NULL in case of error.
 */
TSKT_API struct tskt_socket *tskt_pomp_get_socket(struct tskt_pomp *self);


/**
 * Get the error code associated with TSKT_POMP_EVENT_ERROR event.
 * @param self: pomp socket object handle.
 * @return 0 if no error, a negative errno value if error has occured.
 */
TSKT_API int tskt_pomp_get_error(struct tskt_pomp *self);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_TSKT_POMP_H_ */
