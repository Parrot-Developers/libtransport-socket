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

/*
 * Tests for src/tskt_pomp.c, entirely driven by a struct fake_sock: no real
 * socket I/O. The connect handshake, message framing/decoding and
 * disconnection/error paths are all exercised by directly invoking the
 * tskt_socket_event_cb_t captured by the fake's set_event_cb op (the same
 * callback tskt_pomp_new() registers on the real socket), with scripted
 * read()/write() results standing in for the wire.
 */

#include "tskt_test.h"


struct pomp_test_ctx {
	int last_event;
	int event_count;
	bool had_msg;
	uint32_t last_msg_id;
};


static void pomp_test_ctx_reset(struct pomp_test_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->last_event = -1;
}


static void pomp_test_cb(struct tskt_pomp *self,
			 enum tskt_pomp_event event,
			 const struct pomp_msg *msg,
			 void *userdata)
{
	struct pomp_test_ctx *ctx = userdata;

	/* several tests pass NULL as userdata when they don't care about
	 * event tracking (e.g. send_msg tests only interested in the return
	 * value), but still exercise do_connect(), which fires this
	 * callback: stay a no-op instead of crashing. */
	if (ctx == NULL)
		return;

	ctx->last_event = (int)event;
	ctx->event_count++;
	ctx->had_msg = (msg != NULL);
	if (msg != NULL)
		ctx->last_msg_id = pomp_msg_get_id(msg);
}


/* Simulate the peer becoming writable: this is the handshake tskt_pomp_new()
 * arms via set_event_cb(POMP_FD_EVENT_OUT, ...), and drives self->connected
 * from false to true on the first occurrence. */
static void do_connect(struct fake_sock *fake)
{
	CU_ASSERT_PTR_NOT_NULL(fake->event_cb);
	if (fake->event_cb == NULL)
		return;
	fake->event_cb(&fake->base, POMP_FD_EVENT_OUT, fake->event_cb_userdata);
}


static struct pomp_msg *build_msg(uint32_t msgid, const char *fmt, ...)
{
	struct pomp_msg *msg = pomp_msg_new();
	va_list args;
	int res;

	va_start(args, fmt);
	res = pomp_msg_writev(msg, msgid, fmt, args);
	va_end(args);
	CU_ASSERT_EQUAL(res, 0);

	return msg;
}


static void test_new_guards(void)
{
	struct fake_sock fake;
	struct tskt_pomp *pomp = NULL;

	fake_sock_init(&fake);

	CU_ASSERT_EQUAL(tskt_pomp_new(NULL, &pomp_test_cb, NULL, &pomp),
			-EINVAL);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, NULL, NULL, &pomp), -EINVAL);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, NULL, NULL),
			-EINVAL);
}


static void test_new_success(void)
{
	struct fake_sock fake;
	struct pomp_test_ctx ctx;
	struct tskt_pomp *pomp = NULL;
	int res;

	fake_sock_init(&fake);
	pomp_test_ctx_reset(&ctx);

	res = tskt_pomp_new(&fake.base, &pomp_test_cb, &ctx, &pomp);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(pomp);

	/* tskt_pomp_new arms the connect handshake by watching OUT events */
	CU_ASSERT_TRUE(fake.set_event_cb_called);
	CU_ASSERT_EQUAL(fake.set_event_cb_events, (uint32_t)POMP_FD_EVENT_OUT);
	CU_ASSERT_PTR_NOT_NULL(fake.event_cb);

	CU_ASSERT_PTR_EQUAL(tskt_pomp_get_socket(pomp), &fake.base);
	CU_ASSERT_EQUAL(tskt_pomp_get_error(pomp), 0);

	/* the socket is now owned by pomp: destroying pomp destroys it too */
	CU_ASSERT_EQUAL(tskt_pomp_destroy(pomp), 0);
	CU_ASSERT_TRUE(fake.destroy_called);
}


static void test_new_set_event_cb_failure(void)
{
	struct fake_sock fake;
	struct tskt_pomp *pomp = NULL;
	int res;

	fake_sock_init(&fake);
	fake.set_event_cb_ret = -EIO;

	res = tskt_pomp_new(&fake.base, &pomp_test_cb, NULL, &pomp);
	CU_ASSERT_EQUAL(res, -EIO);

	/* the socket is only owned by pomp on success: on failure here,
	 * self->sock was never assigned, so tskt_pomp_new's internal cleanup
	 * (tskt_pomp_destroy) must not touch the caller's socket */
	CU_ASSERT_FALSE(fake.destroy_called);
}


static void test_destroy_guard(void)
{
	CU_ASSERT_EQUAL(tskt_pomp_destroy(NULL), -EINVAL);
}


static void test_get_socket_get_error_guards(void)
{
	CU_ASSERT_PTR_NULL(tskt_pomp_get_socket(NULL));
	CU_ASSERT_EQUAL(tskt_pomp_get_error(NULL), -EINVAL);
}


static void test_connect_handshake(void)
{
	struct fake_sock fake;
	struct pomp_test_ctx ctx;
	struct tskt_pomp *pomp = NULL;

	fake_sock_init(&fake);
	pomp_test_ctx_reset(&ctx);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, &ctx, &pomp),
			0);

	do_connect(&fake);
	CU_ASSERT_EQUAL(ctx.event_count, 1);
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_POMP_EVENT_CONNECTED);
	CU_ASSERT_TRUE(fake.update_events_called);
	CU_ASSERT_EQUAL(fake.update_events_add, (uint32_t)POMP_FD_EVENT_IN);
	CU_ASSERT_EQUAL(fake.update_events_remove, (uint32_t)POMP_FD_EVENT_OUT);

	/* a second OUT event once already connected is a documented no-op
	 * (write-pending-data is an unimplemented "XXX TO DO" in the current
	 * code, see CLAUDE.md); pin the current behaviour: no extra event */
	fake.event_cb(&fake.base, POMP_FD_EVENT_OUT, fake.event_cb_userdata);
	CU_ASSERT_EQUAL(ctx.event_count, 1);

	tskt_pomp_destroy(pomp);
}


static void test_send_msg_guards(void)
{
	struct fake_sock fake;
	struct tskt_pomp *pomp = NULL;
	struct pomp_msg *msg;

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, NULL, &pomp),
			0);
	msg = build_msg(1, NULL);

	CU_ASSERT_EQUAL(tskt_pomp_send_msg(NULL, msg), -EINVAL);
	CU_ASSERT_EQUAL(tskt_pomp_send_msg(pomp, NULL), -EINVAL);

	pomp_msg_destroy(msg);
	tskt_pomp_destroy(pomp);
}


static void test_send_msg_not_connected(void)
{
	struct fake_sock fake;
	struct tskt_pomp *pomp = NULL;
	struct pomp_msg *msg;

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, NULL, &pomp),
			0);
	msg = build_msg(1, NULL);

	/* connect handshake not done yet: not connected */
	CU_ASSERT_EQUAL(tskt_pomp_send_msg(pomp, msg), -ENOTCONN);
	CU_ASSERT_FALSE(fake.write_called);

	pomp_msg_destroy(msg);
	tskt_pomp_destroy(pomp);
}


static void test_send_msg_unwritten_buffer(void)
{
	struct fake_sock fake;
	struct tskt_pomp *pomp = NULL;
	struct pomp_msg *msg;

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, NULL, &pomp),
			0);
	do_connect(&fake);

	/* a freshly allocated message that was never written has no buffer */
	msg = pomp_msg_new();
	CU_ASSERT_EQUAL(tskt_pomp_send_msg(pomp, msg), -EINVAL);
	CU_ASSERT_FALSE(fake.write_called);

	pomp_msg_destroy(msg);
	tskt_pomp_destroy(pomp);
}


static void test_send_msg_success(void)
{
	struct fake_sock fake;
	struct tskt_pomp *pomp = NULL;
	struct pomp_msg *msg;
	struct pomp_buffer *buf;
	const void *ptr;
	size_t len;
	int res;

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, NULL, &pomp),
			0);
	do_connect(&fake);

	msg = build_msg(42, "%u", 1234u);
	buf = pomp_msg_get_buffer(msg);
	CU_ASSERT_PTR_NOT_NULL(buf);
	CU_ASSERT_EQUAL(pomp_buffer_get_cdata(buf, &ptr, &len, NULL), 0);
	CU_ASSERT_TRUE(len > 0);

	fake.write_ret_all = true;
	res = tskt_pomp_send_msg(pomp, msg);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(fake.write_called);
	CU_ASSERT_EQUAL(fake.write_data_len, len);
	CU_ASSERT_NSTRING_EQUAL(fake.write_data, ptr, len);

	pomp_msg_destroy(msg);
	tskt_pomp_destroy(pomp);
}


static void test_send_msg_partial_write_is_eagain(void)
{
	struct fake_sock fake;
	struct tskt_pomp *pomp = NULL;
	struct pomp_msg *msg;
	struct pomp_buffer *buf;
	const void *ptr;
	size_t len;

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, NULL, &pomp),
			0);
	do_connect(&fake);

	msg = build_msg(1, NULL);
	buf = pomp_msg_get_buffer(msg);
	CU_ASSERT_EQUAL(pomp_buffer_get_cdata(buf, &ptr, &len, NULL), 0);
	CU_ASSERT_TRUE(len > 1);

	/* Pin the CURRENT behaviour: tskt_pomp_send_msg() has no write
	 * buffering (see the "XXX TO DO: check for pending buffers" /
	 * "XXX TO DO" markers in src/tskt_pomp.c, kept as-is, see
	 * CLAUDE.md). A partial write, just like -EAGAIN, is reported to the
	 * caller as -EAGAIN and the unsent bytes are silently dropped: the
	 * caller must resend the whole message. */
	fake.write_ret = (ssize_t)(len - 1);
	CU_ASSERT_EQUAL(tskt_pomp_send_msg(pomp, msg), -EAGAIN);
	CU_ASSERT_EQUAL(tskt_pomp_get_error(pomp), 0);

	fake.write_ret = -EAGAIN;
	CU_ASSERT_EQUAL(tskt_pomp_send_msg(pomp, msg), -EAGAIN);
	CU_ASSERT_EQUAL(tskt_pomp_get_error(pomp), 0);

	pomp_msg_destroy(msg);
	tskt_pomp_destroy(pomp);
}


static void test_send_msg_write_error_is_sticky(void)
{
	struct fake_sock fake;
	struct tskt_pomp *pomp = NULL;
	struct pomp_msg *msg;

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, NULL, &pomp),
			0);
	do_connect(&fake);
	msg = build_msg(1, NULL);

	fake.write_ret = -EPIPE;
	CU_ASSERT_EQUAL(tskt_pomp_send_msg(pomp, msg), -EPIPE);
	CU_ASSERT_EQUAL(tskt_pomp_get_error(pomp), -EPIPE);

	/* once a socket error is latched, further sends short-circuit
	 * without even calling into the socket again */
	fake.write_called = false;
	CU_ASSERT_EQUAL(tskt_pomp_send_msg(pomp, msg), -EPIPE);
	CU_ASSERT_FALSE(fake.write_called);

	pomp_msg_destroy(msg);
	tskt_pomp_destroy(pomp);
}


/*
 * Regression test: tskt_pomp_send()/tskt_pomp_sendv() used to reject a NULL
 * fmt with -EINVAL, contradicting their own doc comment ("fmt : format
 * string. Can be NULL if no arguments given.") and the underlying
 * pomp_encoder_writev(), which explicitly documents and implements NULL fmt
 * as a valid "no arguments" no-op. Fixed in src/tskt_pomp.c by dropping the
 * spurious guard; this pins fmt == NULL as accepted going forward.
 */
static void test_send_fmt_null_allowed(void)
{
	struct fake_sock fake;
	struct tskt_pomp *pomp = NULL;

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, NULL, &pomp),
			0);
	do_connect(&fake);

	fake.write_ret_all = true;
	CU_ASSERT_EQUAL(tskt_pomp_send(pomp, 7, NULL), 0);
	CU_ASSERT_TRUE(fake.write_called);

	tskt_pomp_destroy(pomp);
}


static int send_va(struct tskt_pomp *pomp, uint32_t msgid, const char *fmt, ...)
{
	va_list args;
	int res;

	va_start(args, fmt);
	res = tskt_pomp_sendv(pomp, msgid, fmt, args);
	va_end(args);
	return res;
}


static void test_send_sendv_guards(void)
{
	struct fake_sock fake;
	struct tskt_pomp *pomp = NULL;

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, NULL, &pomp),
			0);

	CU_ASSERT_EQUAL(tskt_pomp_send(NULL, 1, NULL), -EINVAL);
	CU_ASSERT_EQUAL(send_va(NULL, 1, NULL), -EINVAL);

	/* fmt == NULL is valid (see test_send_fmt_null_allowed), it must not
	 * be rejected here */
	fake.write_ret_all = true;
	do_connect(&fake);
	CU_ASSERT_EQUAL(send_va(pomp, 1, NULL), 0);

	tskt_pomp_destroy(pomp);
}


static void test_msg_received(void)
{
	struct fake_sock fake;
	struct pomp_test_ctx ctx;
	struct tskt_pomp *pomp = NULL;
	struct pomp_msg *msg;
	struct pomp_buffer *buf;
	const void *ptr;
	size_t len;

	fake_sock_init(&fake);
	pomp_test_ctx_reset(&ctx);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, &ctx, &pomp),
			0);
	do_connect(&fake);
	pomp_test_ctx_reset(&ctx);

	msg = build_msg(99, "%s", "hello");
	buf = pomp_msg_get_buffer(msg);
	CU_ASSERT_EQUAL(pomp_buffer_get_cdata(buf, &ptr, &len, NULL), 0);

	/* whole encoded message arrives in a single read() */
	fake.read_data = ptr;
	fake.read_data_len = len;
	fake.read_ret = (ssize_t)len;
	fake.event_cb(&fake.base, POMP_FD_EVENT_IN, fake.event_cb_userdata);

	CU_ASSERT_EQUAL(ctx.event_count, 1);
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_POMP_EVENT_MSG);
	CU_ASSERT_TRUE(ctx.had_msg);
	CU_ASSERT_EQUAL(ctx.last_msg_id, 99u);

	pomp_msg_destroy(msg);
	tskt_pomp_destroy(pomp);
}


static void test_msg_split_across_reads(void)
{
	struct fake_sock fake;
	struct pomp_test_ctx ctx;
	struct tskt_pomp *pomp = NULL;
	struct pomp_msg *msg;
	struct pomp_buffer *buf;
	const uint8_t *ptr;
	size_t len, split;

	fake_sock_init(&fake);
	pomp_test_ctx_reset(&ctx);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, &ctx, &pomp),
			0);
	do_connect(&fake);
	pomp_test_ctx_reset(&ctx);

	msg = build_msg(7, "%u", 123456u);
	buf = pomp_msg_get_buffer(msg);
	CU_ASSERT_EQUAL(
		pomp_buffer_get_cdata(buf, (const void **)&ptr, &len, NULL), 0);
	CU_ASSERT_TRUE(len > 4);
	split = 3;

	/* first read only delivers the first few bytes; the 2nd read
	 * attempted within the same event (the decoder consumed everything
	 * given but the message is still incomplete) must see -EAGAIN, i.e.
	 * "no more data has arrived yet" */
	fake.read_data = ptr;
	fake.read_data_len = split;
	fake.read_ret = (ssize_t)split;
	fake.read_ret2_set = true;
	fake.read_ret2 = -EAGAIN;
	fake.event_cb(&fake.base, POMP_FD_EVENT_IN, fake.event_cb_userdata);

	CU_ASSERT_EQUAL(ctx.event_count, 0);

	/* remainder arrives in a second, separate read event */
	fake.read_call_count = 0;
	fake.read_ret2_set = false;
	fake.read_data = ptr + split;
	fake.read_data_len = len - split;
	fake.read_ret = (ssize_t)(len - split);
	fake.event_cb(&fake.base, POMP_FD_EVENT_IN, fake.event_cb_userdata);

	CU_ASSERT_EQUAL(ctx.event_count, 1);
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_POMP_EVENT_MSG);
	CU_ASSERT_EQUAL(ctx.last_msg_id, 7u);

	pomp_msg_destroy(msg);
	tskt_pomp_destroy(pomp);
}


static void test_disconnected(void)
{
	struct fake_sock fake;
	struct pomp_test_ctx ctx;
	struct tskt_pomp *pomp = NULL;

	fake_sock_init(&fake);
	pomp_test_ctx_reset(&ctx);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, &ctx, &pomp),
			0);
	do_connect(&fake);
	pomp_test_ctx_reset(&ctx);

	fake.read_ret = 0; /* peer closed the connection */
	fake.event_cb(&fake.base, POMP_FD_EVENT_IN, fake.event_cb_userdata);

	CU_ASSERT_EQUAL(ctx.event_count, 1);
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_POMP_EVENT_DISCONNECTED);
	/* notifications are disabled once disconnected */
	CU_ASSERT_EQUAL(fake.set_event_cb_events, 0u);
	CU_ASSERT_PTR_NULL(fake.event_cb);

	tskt_pomp_destroy(pomp);
}


static void test_error_event(void)
{
	struct fake_sock fake;
	struct pomp_test_ctx ctx;
	struct tskt_pomp *pomp = NULL;
	tskt_socket_event_cb_t cb;
	void *cb_userdata;

	fake_sock_init(&fake);
	pomp_test_ctx_reset(&ctx);
	CU_ASSERT_EQUAL(tskt_pomp_new(&fake.base, &pomp_test_cb, &ctx, &pomp),
			0);
	do_connect(&fake);
	pomp_test_ctx_reset(&ctx);
	cb = fake.event_cb;
	cb_userdata = fake.event_cb_userdata;

	/* get_option(TSKT_OPT_ERROR) stands in for tskt_socket_get_error(),
	 * which is documented to return the error as a positive value */
	fake.get_option_ret[TSKT_OPT_ERROR] = EPIPE;
	cb(&fake.base, POMP_FD_EVENT_ERR, cb_userdata);

	CU_ASSERT_EQUAL(ctx.event_count, 1);
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_POMP_EVENT_ERROR);
	CU_ASSERT_EQUAL(tskt_pomp_get_error(pomp), -EPIPE);
	CU_ASSERT_PTR_NULL(fake.event_cb);

	tskt_pomp_destroy(pomp);
}


CU_TestInfo g_tskt_test_pomp[] = {
	{FN("new-guards"), &test_new_guards},
	{FN("new-success"), &test_new_success},
	{FN("new-set-event-cb-failure"), &test_new_set_event_cb_failure},
	{FN("destroy-guard"), &test_destroy_guard},
	{FN("get-socket-get-error-guards"), &test_get_socket_get_error_guards},
	{FN("connect-handshake"), &test_connect_handshake},
	{FN("send-msg-guards"), &test_send_msg_guards},
	{FN("send-msg-not-connected"), &test_send_msg_not_connected},
	{FN("send-msg-unwritten-buffer"), &test_send_msg_unwritten_buffer},
	{FN("send-msg-success"), &test_send_msg_success},
	{FN("send-msg-partial-write-is-eagain"),
	 &test_send_msg_partial_write_is_eagain},
	{FN("send-msg-write-error-is-sticky"),
	 &test_send_msg_write_error_is_sticky},
	{FN("send-fmt-null-allowed"), &test_send_fmt_null_allowed},
	{FN("send-sendv-guards"), &test_send_sendv_guards},
	{FN("msg-received"), &test_msg_received},
	{FN("msg-split-across-reads"), &test_msg_split_across_reads},
	{FN("disconnected"), &test_disconnected},
	{FN("error-event"), &test_error_event},

	CU_TEST_INFO_NULL,
};
