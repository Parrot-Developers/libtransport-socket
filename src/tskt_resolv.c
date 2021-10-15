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
#include <transport-socket/tskt_resolv_ops.h>

#define ULOG_TAG tskt_resolv
#include <ulog.h>
ULOG_DECLARE_TAG(tskt_resolv);


void tskt_resolv_ref(struct tskt_resolv *self)
{
	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

#if defined(__GNUC__)
	__sync_add_and_fetch(&self->refcount, 1);
#elif defined(_WIN32)
	/* codecheck_ignore[SPACING,VOLATILE] */
	InterlockedIncrement((long volatile *)&self->refcount);
#else
#	error No atomic increment function found on this platform
#endif
}


void tskt_resolv_unref(struct tskt_resolv *self)
{
	uint32_t res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

#if defined(__GNUC__)
	res = __sync_sub_and_fetch(&self->refcount, 1);
#elif defined(_WIN32)
	/* codecheck_ignore[SPACING,VOLATILE] */
	res = (uint32_t)InterlockedDecrement((long volatile *)&self->refcount);
#else
#	error No atomic decrement function found on this platform
#endif

	/* Free resource when ref count reaches 0 */
	if (res == 0)
		self->ops->destroy(self);
}


int tskt_resolv_getaddrinfo(struct tskt_resolv *self,
			    const char *hostname,
			    struct pomp_loop *loop,
			    tskt_resolv_cb_t cb,
			    void *userdata,
			    int *ret_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(hostname == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(loop == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cb == NULL, EINVAL);

	return self->ops->getaddrinfo(
		self, hostname, loop, cb, userdata, ret_id);
}


int tskt_resolv_cancel(struct tskt_resolv *self, int id)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->ops->cancel(self, id);
}
