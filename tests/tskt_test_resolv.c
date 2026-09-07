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

/*
 * Tests for src/tskt_resolv.c, the tskt_resolv_* dispatcher: argument
 * validation, atomic refcounting/destroy, and the getaddrinfo6-specific
 * -ENOSYS guard. Fully driven by struct fake_resolv, no real DNS
 * resolution.
 */

#include "tskt_test.h"


static void dummy_resolv_cb(struct tskt_resolv *self,
			    int id,
			    enum tskt_resolv_error result,
			    int naddrs,
			    const char *const *addrs,
			    void *userdata)
{
}


static void test_ref_unref(void)
{
	struct fake_resolv fake;

	/* NULL is a documented no-op, must not crash */
	tskt_resolv_ref(NULL);
	tskt_resolv_unref(NULL);

	/* refcount starts at 1 (fake_resolv_init): one unref destroys it */
	fake_resolv_init(&fake);
	tskt_resolv_unref(&fake.base);
	CU_ASSERT_TRUE(fake.destroy_called);

	/* refcount raised to 2: first unref must not destroy, second must */
	fake_resolv_init(&fake);
	tskt_resolv_ref(&fake.base);
	CU_ASSERT_EQUAL(fake.base.refcount, 2);
	tskt_resolv_unref(&fake.base);
	CU_ASSERT_FALSE(fake.destroy_called);
	CU_ASSERT_EQUAL(fake.base.refcount, 1);
	tskt_resolv_unref(&fake.base);
	CU_ASSERT_TRUE(fake.destroy_called);
	CU_ASSERT_EQUAL(fake.base.refcount, 0);
}


static void test_getaddrinfo(void)
{
	struct fake_resolv fake;
	struct pomp_loop *loop = (struct pomp_loop *)0x1234;
	int id;

	fake_resolv_init(&fake);
	CU_ASSERT_EQUAL(
		tskt_resolv_getaddrinfo(
			NULL, "host", loop, &dummy_resolv_cb, NULL, &id),
		-EINVAL);
	CU_ASSERT_EQUAL(
		tskt_resolv_getaddrinfo(
			&fake.base, NULL, loop, &dummy_resolv_cb, NULL, &id),
		-EINVAL);
	CU_ASSERT_EQUAL(
		tskt_resolv_getaddrinfo(
			&fake.base, "host", NULL, &dummy_resolv_cb, NULL, &id),
		-EINVAL);
	CU_ASSERT_EQUAL(tskt_resolv_getaddrinfo(
				&fake.base, "host", loop, NULL, NULL, &id),
			-EINVAL);

	/* valid call: pure passthrough, including ret_id being optional */
	fake.getaddrinfo_ret = 0;
	fake.getaddrinfo_ret_id = 7;
	id = -1;
	CU_ASSERT_EQUAL(tskt_resolv_getaddrinfo(&fake.base,
						"example.invalid",
						loop,
						&dummy_resolv_cb,
						&fake,
						&id),
			0);
	CU_ASSERT_TRUE(fake.getaddrinfo_called);
	CU_ASSERT_STRING_EQUAL(fake.getaddrinfo_hostname, "example.invalid");
	CU_ASSERT_PTR_EQUAL(fake.getaddrinfo_loop, loop);
	CU_ASSERT_PTR_EQUAL(fake.getaddrinfo_cb, &dummy_resolv_cb);
	CU_ASSERT_PTR_EQUAL(fake.getaddrinfo_userdata, &fake);
	CU_ASSERT_EQUAL(id, 7);

	CU_ASSERT_EQUAL(tskt_resolv_getaddrinfo(&fake.base,
						"example.invalid",
						loop,
						&dummy_resolv_cb,
						NULL,
						NULL),
			0);
}


static void test_getaddrinfo6(void)
{
	struct fake_resolv fake;
	struct tskt_resolv_ops ops;
	struct pomp_loop *loop = (struct pomp_loop *)0x1234;

	fake_resolv_init(&fake);
	CU_ASSERT_EQUAL(
		tskt_resolv_getaddrinfo6(
			NULL, "host", loop, &dummy_resolv_cb, NULL, NULL),
		-EINVAL);
	CU_ASSERT_EQUAL(
		tskt_resolv_getaddrinfo6(
			&fake.base, NULL, loop, &dummy_resolv_cb, NULL, NULL),
		-EINVAL);
	CU_ASSERT_EQUAL(
		tskt_resolv_getaddrinfo6(
			&fake.base, "host", NULL, &dummy_resolv_cb, NULL, NULL),
		-EINVAL);
	CU_ASSERT_EQUAL(tskt_resolv_getaddrinfo6(
				&fake.base, "host", loop, NULL, NULL, NULL),
			-EINVAL);

	/* getaddrinfo6 unimplemented: -ENOSYS, unlike getaddrinfo which has
	 * no equivalent guard and relies on the op always being present */
	fake_resolv_init(&fake);
	ops = g_fake_resolv_ops;
	ops.getaddrinfo6 = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(
		tskt_resolv_getaddrinfo6(
			&fake.base, "host", loop, &dummy_resolv_cb, NULL, NULL),
		-ENOSYS);

	/* valid call: pure passthrough */
	fake_resolv_init(&fake);
	fake.getaddrinfo6_ret = 0;
	CU_ASSERT_EQUAL(
		tskt_resolv_getaddrinfo6(
			&fake.base, "::1", loop, &dummy_resolv_cb, &fake, NULL),
		0);
	CU_ASSERT_TRUE(fake.getaddrinfo6_called);
	CU_ASSERT_STRING_EQUAL(fake.getaddrinfo6_hostname, "::1");
	CU_ASSERT_PTR_EQUAL(fake.getaddrinfo6_loop, loop);
}


static void test_cancel(void)
{
	struct fake_resolv fake;

	CU_ASSERT_EQUAL(tskt_resolv_cancel(NULL, 1), -EINVAL);

	fake_resolv_init(&fake);
	fake.cancel_ret = 0;
	CU_ASSERT_EQUAL(tskt_resolv_cancel(&fake.base, 42), 0);
	CU_ASSERT_TRUE(fake.cancel_called);
	CU_ASSERT_EQUAL(fake.cancel_id, 42);
}


CU_TestInfo g_tskt_test_resolv[] = {
	{FN("ref-unref"), &test_ref_unref},
	{FN("getaddrinfo"), &test_getaddrinfo},
	{FN("getaddrinfo6"), &test_getaddrinfo6},
	{FN("cancel"), &test_cancel},

	CU_TEST_INFO_NULL,
};
