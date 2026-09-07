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
 * Real tests for src/tskt_resolv_impl.c, restricted to literal IP addresses
 * ("127.0.0.1", "::1") and "localhost" so no actual DNS resolution or
 * network access is required: resolv_impl_getaddrinfo_gen() special-cases
 * inet_pton()-parsable hostnames and resolves them synchronously via a
 * pomp_loop_idle_add(), without ever touching the internal async resolver
 * thread (resolv_impl_thread()). "localhost" is the one non-literal
 * hostname used here specifically to exercise that thread's real
 * getaddrinfo()/getnameinfo() path (see test_getaddrinfo_hostname_localhost
 * below) while staying network-independent: it resolves via /etc/hosts (NSS
 * "files" source) on every mainstream system, never via an actual DNS
 * query. Resolution of any other (real, network-dependent) hostname is out
 * of scope here, per project decision (see CLAUDE.md).
 */

#include "tskt_test.h"


#define TEST_TIMEOUT_MS 2000
#define POLL_STEP_MS 20


struct resolv_test_ctx {
	int call_count;
	int id;
	enum tskt_resolv_error result;
	int naddrs;
	char addr[64];
};


static void reset_ctx(struct resolv_test_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
}


static void resolv_test_cb(struct tskt_resolv *self,
			   int id,
			   enum tskt_resolv_error result,
			   int naddrs,
			   const char *const *addrs,
			   void *userdata)
{
	struct resolv_test_ctx *ctx = userdata;

	ctx->call_count++;
	ctx->id = id;
	ctx->result = result;
	ctx->naddrs = naddrs;
	ctx->addr[0] = '\0';
	if (naddrs > 0 && addrs != NULL && addrs[0] != NULL)
		snprintf(ctx->addr, sizeof(ctx->addr), "%s", addrs[0]);
}


static bool spin_until_called(struct pomp_loop *loop,
			      struct resolv_test_ctx *ctx,
			      int count,
			      int timeout_ms)
{
	int elapsed = 0;

	while (ctx->call_count < count) {
		if (elapsed >= timeout_ms)
			return false;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	}
	return true;
}


static void test_new_guard(void)
{
	CU_ASSERT_EQUAL(tskt_resolv_new(NULL), -EINVAL);
}


static void test_getaddrinfo_literal_ipv4(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_resolv *resolv = NULL;
	struct resolv_test_ctx ctx;
	int id = TSKT_RESOLV_INVALID_ID;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx);
	CU_ASSERT_EQUAL(tskt_resolv_new(&resolv), 0);
	CU_ASSERT_EQUAL(resolv->refcount, 1);

	res = tskt_resolv_getaddrinfo(
		resolv, "127.0.0.1", loop, &resolv_test_cb, &ctx, &id);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(id != TSKT_RESOLV_INVALID_ID);
	/* the pending request holds an extra reference until it completes */
	CU_ASSERT_EQUAL(resolv->refcount, 2);

	CU_ASSERT_TRUE(spin_until_called(loop, &ctx, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx.id, id);
	CU_ASSERT_EQUAL(ctx.result, TSKT_RESOLV_ERROR_OK);
	CU_ASSERT_EQUAL(ctx.naddrs, 1);
	CU_ASSERT_STRING_EQUAL(ctx.addr, "127.0.0.1");
	CU_ASSERT_EQUAL(resolv->refcount, 1);

	tskt_resolv_unref(resolv);
	pomp_loop_destroy(loop);
}


static void test_getaddrinfo6_literal_ipv6(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_resolv *resolv = NULL;
	struct resolv_test_ctx ctx;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx);
	CU_ASSERT_EQUAL(tskt_resolv_new(&resolv), 0);

	res = tskt_resolv_getaddrinfo6(
		resolv, "::1", loop, &resolv_test_cb, &ctx, NULL);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(spin_until_called(loop, &ctx, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx.result, TSKT_RESOLV_ERROR_OK);
	CU_ASSERT_EQUAL(ctx.naddrs, 1);
	CU_ASSERT_STRING_EQUAL(ctx.addr, "::1");

	tskt_resolv_unref(resolv);
	pomp_loop_destroy(loop);
}


static void test_multiple_concurrent_requests(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_resolv *resolv = NULL;
	struct resolv_test_ctx ctx4, ctx6;
	int id4 = TSKT_RESOLV_INVALID_ID, id6 = TSKT_RESOLV_INVALID_ID;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx4);
	reset_ctx(&ctx6);
	CU_ASSERT_EQUAL(tskt_resolv_new(&resolv), 0);

	CU_ASSERT_EQUAL(tskt_resolv_getaddrinfo(resolv,
						"127.0.0.1",
						loop,
						&resolv_test_cb,
						&ctx4,
						&id4),
			0);
	CU_ASSERT_EQUAL(
		tskt_resolv_getaddrinfo6(
			resolv, "::1", loop, &resolv_test_cb, &ctx6, &id6),
		0);
	CU_ASSERT_TRUE(id4 != id6);

	CU_ASSERT_TRUE(spin_until_called(loop, &ctx4, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_TRUE(spin_until_called(loop, &ctx6, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx4.id, id4);
	CU_ASSERT_STRING_EQUAL(ctx4.addr, "127.0.0.1");
	CU_ASSERT_EQUAL(ctx6.id, id6);
	CU_ASSERT_STRING_EQUAL(ctx6.addr, "::1");

	tskt_resolv_unref(resolv);
	pomp_loop_destroy(loop);
}


static void test_cancel_before_completion(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_resolv *resolv = NULL;
	struct resolv_test_ctx ctx;
	int id = TSKT_RESOLV_INVALID_ID;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx);
	CU_ASSERT_EQUAL(tskt_resolv_new(&resolv), 0);

	/* literal-address requests are queued (via pomp_loop_idle_add) but
	 * not yet fired since the loop hasn't been pumped: cancel must find
	 * it on the "done" list and remove the idle callback before it ever
	 * runs */
	CU_ASSERT_EQUAL(
		tskt_resolv_getaddrinfo(
			resolv, "127.0.0.1", loop, &resolv_test_cb, &ctx, &id),
		0);
	CU_ASSERT_EQUAL(tskt_resolv_cancel(resolv, id), 0);

	CU_ASSERT_FALSE(spin_until_called(loop, &ctx, 1, 200));
	CU_ASSERT_EQUAL(ctx.call_count, 0);
	CU_ASSERT_EQUAL(resolv->refcount, 1);

	tskt_resolv_unref(resolv);
	pomp_loop_destroy(loop);
}


/*
 * Coverage: unlike the literal-address tests above, "localhost" is not
 * inet_pton()-parsable, so this drives the real resolv_impl_thread() body -
 * the pending queue, the pthread_cond_wait()/pthread_cond_signal() handoff,
 * and the actual getaddrinfo()/getnameinfo() calls - instead of the
 * synchronous literal-address fast path.
 *
 * Not covered here (left undocumented-but-untested, deliberately): the
 * "request cancelled while running" branch in resolv_impl_thread() (needs
 * to race tskt_resolv_cancel() against an in-flight getaddrinfo() call,
 * which is inherently timing-dependent against a real, if local, DNS/NSS
 * lookup) and the pomp_loop_idle_add() failure branch (OOM-only, needs
 * fault injection). Both fall in the same "not reliably testable without
 * fault injection or flaky timing" category as similar gaps documented in
 * tests/tskt_test_client.c.
 */
static void test_getaddrinfo_hostname_localhost(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_resolv *resolv = NULL;
	struct resolv_test_ctx ctx4, ctx6;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx4);
	reset_ctx(&ctx6);
	CU_ASSERT_EQUAL(tskt_resolv_new(&resolv), 0);

	CU_ASSERT_EQUAL(tskt_resolv_getaddrinfo(resolv,
						"localhost",
						loop,
						&resolv_test_cb,
						&ctx4,
						NULL),
			0);
	CU_ASSERT_EQUAL(tskt_resolv_getaddrinfo6(resolv,
						 "localhost",
						 loop,
						 &resolv_test_cb,
						 &ctx6,
						 NULL),
			0);

	CU_ASSERT_TRUE(spin_until_called(loop, &ctx4, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx4.result, TSKT_RESOLV_ERROR_OK);
	CU_ASSERT_EQUAL(ctx4.naddrs, 1);
	CU_ASSERT_STRING_EQUAL(ctx4.addr, "127.0.0.1");

	CU_ASSERT_TRUE(spin_until_called(loop, &ctx6, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx6.result, TSKT_RESOLV_ERROR_OK);
	CU_ASSERT_EQUAL(ctx6.naddrs, 1);
	CU_ASSERT_STRING_EQUAL(ctx6.addr, "::1");

	tskt_resolv_unref(resolv);
	pomp_loop_destroy(loop);
}


static void test_cancel_invalid_id(void)
{
	struct tskt_resolv *resolv = NULL;

	CU_ASSERT_EQUAL(tskt_resolv_new(&resolv), 0);
	CU_ASSERT_EQUAL(tskt_resolv_cancel(resolv, TSKT_RESOLV_INVALID_ID),
			-ENOENT);
	tskt_resolv_unref(resolv);
}


CU_TestInfo g_tskt_test_resolv_impl[] = {
	{FN("new-guard"), &test_new_guard},
	{FN("getaddrinfo-literal-ipv4"), &test_getaddrinfo_literal_ipv4},
	{FN("getaddrinfo6-literal-ipv6"), &test_getaddrinfo6_literal_ipv6},
	{FN("multiple-concurrent-requests"),
	 &test_multiple_concurrent_requests},
	{FN("cancel-before-completion"), &test_cancel_before_completion},
	{FN("getaddrinfo-hostname-localhost"),
	 &test_getaddrinfo_hostname_localhost},
	{FN("cancel-invalid-id"), &test_cancel_invalid_id},

	CU_TEST_INFO_NULL,
};
