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

#ifndef _TSKT_RESOLV_H_
#define _TSKT_RESOLV_H_

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
struct tskt_resolv;


/** Invalid request id. */
#define TSKT_RESOLV_INVALID_ID (-1)


/** Resolver request error code. */
enum tskt_resolv_error {
	TSKT_RESOLV_ERROR_OK = 0, /**< no error. */
	TSKT_RESOLV_ERROR_AGAIN, /**< name server temporary error, try again
				    later */
	TSKT_RESOLV_ERROR_FAIL, /**< name server permanent error. */
	TSKT_RESOLV_ERROR_MEMORY, /**< out of memory. */
	TSKT_RESOLV_ERROR_NODATA, /**< no address for this host. */
	TSKT_RESOLV_ERROR_NONAME, /**< unknown host. */
	TSKT_RESOLV_ERROR_SYTEM, /**< unspecified system error. */
};


/**
 * Resolver request callback prototype.
 * @param self : resolver object handle.
 * @param id : request id.
 * @param result : request error code.
 * @param naddrs : number of addresses in addrs array.
 * @param addrs : array of addresses.
 * @param userdata : callback user data.
 */
typedef void (*tskt_resolv_cb_t)(struct tskt_resolv *self,
				 int id,
				 enum tskt_resolv_error result,
				 int naddrs,
				 const char *const *addrs,
				 void *userdata);


/**
 * Create a new resolver object.
 * The returned object uses the default system resolver.
 * @param ret_obj : resolver object handle (output).
 * @return 0 on success, negative errno value in case of error.
 */
TSKT_API int tskt_resolv_new(struct tskt_resolv **ret_obj);


/**
 * Reference a resolver.
 * This function increments the reference counter of a resolver object.
 * @param self : resolver object handle.
 */
TSKT_API void tskt_resolv_ref(struct tskt_resolv *self);


/**
 * Unreference a resolver.
 * This function decrements the reference counter of a resolver object.
 * The resolver is destroyed if its reference counter reaches zero.
 * @param self : resolver object handle.
 */
TSKT_API void tskt_resolv_unref(struct tskt_resolv *self);


/**
 * Request a name to address resolution.
 * @param self : resolver object handle.
 * @param hostname : host name to resolv.
 * @param loop : pomp loop to use.
 * @param cb : result callback function.
 * @param userdata : callback user data.
 * @param ret_id : request id (output, optional, can be NULL).
 * @return 0 on success, negative errno value in case of error.
 */
TSKT_API int tskt_resolv_getaddrinfo(struct tskt_resolv *self,
				     const char *hostname,
				     struct pomp_loop *loop,
				     tskt_resolv_cb_t cb,
				     void *userdata,
				     int *ret_id);


/**
 * Cancel a resolver request. The callback will not be called
 * after the request has been cancelled.
 * @param self : resolver object handle.
 * @param id : id of the request to be cancelled.
 * @return 0 on success, negative errno value in case of error.
 */
TSKT_API int tskt_resolv_cancel(struct tskt_resolv *self, int id);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_TSKT_RESOLV_H_ */
