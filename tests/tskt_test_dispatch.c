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
 * Tests for src/tskt.c, the tskt_socket_* dispatcher: argument validation,
 * -EOPNOTSUPP when an optional vtable op is missing, passthrough of return
 * values, and the emulation fallbacks used when a "batch" op (readmv,
 * write_cs, read_pkt_alloc, read_pkt_list, write_pkt_list, ...) is not
 * implemented by the backing tskt_socket_ops. Fully driven by struct
 * fake_sock, no real socket I/O.
 */

#include "tskt_test.h"

#include <sys/uio.h>


static void test_destroy(void)
{
	struct fake_sock fake;
	struct tpkt_list *rxpkt_free = NULL;
	int res;

	CU_ASSERT_EQUAL(tskt_socket_destroy(NULL), 0);

	fake_sock_init(&fake);
	fake.destroy_ret = -EIO;
	res = tskt_socket_destroy(&fake.base);
	CU_ASSERT_TRUE(fake.destroy_called);
	CU_ASSERT_EQUAL(res, -EIO);

	/* a non-NULL rxpkt_free is destroyed too, on top of ops->destroy() */
	fake_sock_init(&fake);
	fake.destroy_ret = 0;
	CU_ASSERT_EQUAL(tpkt_list_new(&rxpkt_free), 0);
	fake.base.rxpkt_free = rxpkt_free;
	res = tskt_socket_destroy(&fake.base);
	CU_ASSERT_TRUE(fake.destroy_called);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_get_loop(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;
	struct pomp_loop *loop = (struct pomp_loop *)0x1234;

	CU_ASSERT_PTR_NULL(tskt_socket_get_loop(NULL));

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.get_loop = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_PTR_NULL(tskt_socket_get_loop(&fake.base));

	fake_sock_init(&fake);
	fake.get_loop_ret = loop;
	CU_ASSERT_PTR_EQUAL(tskt_socket_get_loop(&fake.base), loop);
	CU_ASSERT_TRUE(fake.get_loop_called);
}


static void test_get_fd(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;

	CU_ASSERT_EQUAL(tskt_socket_get_fd(NULL), -EINVAL);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.get_fd = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_get_fd(&fake.base), -EOPNOTSUPP);

	fake_sock_init(&fake);
	fake.get_fd_ret = 42;
	CU_ASSERT_EQUAL(tskt_socket_get_fd(&fake.base), 42);
	CU_ASSERT_TRUE(fake.get_fd_called);
}


static void dummy_fd_cb(int fd, uint32_t revents, void *userdata) {}


static void test_set_fd_cb(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;
	int userdata;

	CU_ASSERT_EQUAL(tskt_socket_set_fd_cb(NULL, dummy_fd_cb, &userdata),
			-EINVAL);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.set_fd_cb = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(
		tskt_socket_set_fd_cb(&fake.base, dummy_fd_cb, &userdata),
		-EOPNOTSUPP);

	fake_sock_init(&fake);
	fake.set_fd_cb_ret = 0;
	CU_ASSERT_EQUAL(
		tskt_socket_set_fd_cb(&fake.base, dummy_fd_cb, &userdata), 0);
	CU_ASSERT_TRUE(fake.set_fd_cb_called);
	CU_ASSERT_PTR_EQUAL(fake.set_fd_cb_cb, dummy_fd_cb);
	CU_ASSERT_PTR_EQUAL(fake.set_fd_cb_userdata, &userdata);
}


static void test_get_local_addr(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;
	char buf[64];

	CU_ASSERT_EQUAL(tskt_socket_get_local_addr(NULL, buf, sizeof(buf)),
			-EINVAL);

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_socket_get_local_addr(&fake.base, NULL, 16),
			-EINVAL);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.get_local_addr = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(
		tskt_socket_get_local_addr(&fake.base, buf, sizeof(buf)),
		-EOPNOTSUPP);

	fake_sock_init(&fake);
	fake.get_local_addr_str = "1.2.3.4";
	fake.get_local_addr_ret = 0;
	CU_ASSERT_EQUAL(
		tskt_socket_get_local_addr(&fake.base, buf, sizeof(buf)), 0);
	CU_ASSERT_STRING_EQUAL(buf, "1.2.3.4");
	CU_ASSERT_EQUAL(fake.get_local_addr_len, sizeof(buf));
}


static void test_get_local_port(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;

	CU_ASSERT_EQUAL(tskt_socket_get_local_port(NULL), 0);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.get_local_addr = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_get_local_port(&fake.base), 0);

	fake_sock_init(&fake);
	fake.get_local_addr_port = 4242;
	CU_ASSERT_EQUAL(tskt_socket_get_local_port(&fake.base), 4242);
	CU_ASSERT_TRUE(fake.get_local_addr_called);
	CU_ASSERT_EQUAL(fake.get_local_addr_len, 0);
}


static void test_get_remote_addr(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;
	char buf[64];

	CU_ASSERT_EQUAL(tskt_socket_get_remote_addr(NULL, buf, sizeof(buf)),
			-EINVAL);

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_socket_get_remote_addr(&fake.base, NULL, 16),
			-EINVAL);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.get_remote_addr = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(
		tskt_socket_get_remote_addr(&fake.base, buf, sizeof(buf)),
		-EOPNOTSUPP);

	fake_sock_init(&fake);
	fake.get_remote_addr_str = "5.6.7.8";
	fake.get_remote_addr_ret = 0;
	CU_ASSERT_EQUAL(
		tskt_socket_get_remote_addr(&fake.base, buf, sizeof(buf)), 0);
	CU_ASSERT_STRING_EQUAL(buf, "5.6.7.8");
}


static void test_get_remote_port(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;

	CU_ASSERT_EQUAL(tskt_socket_get_remote_port(NULL), 0);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.get_remote_addr = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_get_remote_port(&fake.base), 0);

	fake_sock_init(&fake);
	fake.get_remote_addr_port = 5353;
	CU_ASSERT_EQUAL(tskt_socket_get_remote_port(&fake.base), 5353);
	CU_ASSERT_TRUE(fake.get_remote_addr_called);
}


static void test_set_remote(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;

	CU_ASSERT_EQUAL(tskt_socket_set_remote(NULL, "1.2.3.4", 80), -EINVAL);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.set_remote_addr = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_set_remote(&fake.base, "1.2.3.4", 80),
			-EOPNOTSUPP);

	fake_sock_init(&fake);
	fake.set_remote_addr_ret = 0;
	CU_ASSERT_EQUAL(tskt_socket_set_remote(&fake.base, "1.2.3.4", 80), 0);
	CU_ASSERT_STRING_EQUAL(fake.set_remote_addr_addr, "1.2.3.4");
	CU_ASSERT_EQUAL(fake.set_remote_addr_port, 80);
}


static void test_option_getters_and_setters(void)
{
	struct fake_sock fake;

	/* NULL self guard, one representative per accessor */
	CU_ASSERT_EQUAL(tskt_socket_get_rxbuf_size(NULL), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_set_rxbuf_size(NULL, 1024), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_get_txbuf_size(NULL), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_set_txbuf_size(NULL, 1024), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_get_class_selector(NULL), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_set_class_selector(NULL, 1), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_get_nodelay(NULL), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_set_nodelay(NULL, 1), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_get_error(NULL), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_get_reset(NULL), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_set_reset(NULL, 1), -EINVAL);

	/* every getter/setter passes straight through to get_option/set_option
	 * with the matching enum tskt_option value, no EOPNOTSUPP guard */
	fake_sock_init(&fake);
	fake.get_option_ret[TSKT_OPT_RXBUF_SIZE] = 111;
	CU_ASSERT_EQUAL(tskt_socket_get_rxbuf_size(&fake.base), 111);
	CU_ASSERT_EQUAL(fake.get_option_option, TSKT_OPT_RXBUF_SIZE);

	fake.set_option_ret = 0;
	CU_ASSERT_EQUAL(tskt_socket_set_rxbuf_size(&fake.base, 2048), 0);
	CU_ASSERT_EQUAL(fake.set_option_option, TSKT_OPT_RXBUF_SIZE);
	CU_ASSERT_EQUAL(fake.set_option_value, 2048);

	fake.get_option_ret[TSKT_OPT_TXBUF_SIZE] = 222;
	CU_ASSERT_EQUAL(tskt_socket_get_txbuf_size(&fake.base), 222);
	CU_ASSERT_EQUAL(fake.get_option_option, TSKT_OPT_TXBUF_SIZE);

	CU_ASSERT_EQUAL(tskt_socket_set_txbuf_size(&fake.base, 4096), 0);
	CU_ASSERT_EQUAL(fake.set_option_option, TSKT_OPT_TXBUF_SIZE);
	CU_ASSERT_EQUAL(fake.set_option_value, 4096);

	fake.get_option_ret[TSKT_OPT_CLASS_SELECTOR] = 3;
	CU_ASSERT_EQUAL(tskt_socket_get_class_selector(&fake.base), 3);
	CU_ASSERT_EQUAL(tskt_socket_set_class_selector(&fake.base, 5), 0);
	CU_ASSERT_EQUAL(fake.set_option_option, TSKT_OPT_CLASS_SELECTOR);
	CU_ASSERT_EQUAL(fake.set_option_value, 5);

	fake.get_option_ret[TSKT_OPT_NODELAY] = 1;
	CU_ASSERT_EQUAL(tskt_socket_get_nodelay(&fake.base), 1);
	CU_ASSERT_EQUAL(tskt_socket_set_nodelay(&fake.base, 1), 0);
	CU_ASSERT_EQUAL(fake.set_option_option, TSKT_OPT_NODELAY);

	fake.get_option_ret[TSKT_OPT_ERROR] = 7;
	CU_ASSERT_EQUAL(tskt_socket_get_error(&fake.base), 7);
	CU_ASSERT_EQUAL(fake.get_option_option, TSKT_OPT_ERROR);

	fake.get_option_ret[TSKT_OPT_RESET] = 1;
	CU_ASSERT_EQUAL(tskt_socket_get_reset(&fake.base), 1);
	CU_ASSERT_EQUAL(tskt_socket_set_reset(&fake.base, 0), 0);
	CU_ASSERT_EQUAL(fake.set_option_option, TSKT_OPT_RESET);
	CU_ASSERT_EQUAL(fake.set_option_value, 0);
}


static void test_get_rxpkt_max_size(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;

	CU_ASSERT_EQUAL(tskt_socket_get_rxpkt_max_size(NULL), -EINVAL);

	/* op present: pure passthrough */
	fake_sock_init(&fake);
	fake.get_rxpkt_max_size_ret = 9000;
	CU_ASSERT_EQUAL(tskt_socket_get_rxpkt_max_size(&fake.base), 9000);
	CU_ASSERT_TRUE(fake.get_rxpkt_max_size_called);

	/* op absent: falls back to self->rxpkt_max_size, defaulted to
	 * TSKT_SOCKET_RXPKT_MAX_SIZE_DEFAULT the first time it is read */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.get_rxpkt_max_size = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_get_rxpkt_max_size(&fake.base),
			TSKT_SOCKET_RXPKT_MAX_SIZE_DEFAULT);
	CU_ASSERT_EQUAL(fake.base.rxpkt_max_size,
			TSKT_SOCKET_RXPKT_MAX_SIZE_DEFAULT);

	fake.base.rxpkt_max_size = 2000;
	CU_ASSERT_EQUAL(tskt_socket_get_rxpkt_max_size(&fake.base), 2000);
}


static void test_set_rxpkt_max_size(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;
	struct tpkt_list *rxpkt_free = NULL;
	struct tpkt_packet *pkt;

	CU_ASSERT_EQUAL(tskt_socket_set_rxpkt_max_size(NULL, 100), -EINVAL);

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_socket_set_rxpkt_max_size(&fake.base, 0), -EINVAL);

	/* op present: pure passthrough */
	fake_sock_init(&fake);
	fake.set_rxpkt_max_size_ret = 0;
	CU_ASSERT_EQUAL(tskt_socket_set_rxpkt_max_size(&fake.base, 2000), 0);
	CU_ASSERT_TRUE(fake.set_rxpkt_max_size_called);
	CU_ASSERT_EQUAL(fake.set_rxpkt_max_size_value, 2000);

	/* op absent, same value: no-op, no flush */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.set_rxpkt_max_size = NULL;
	fake.base.ops = &ops;
	fake.base.rxpkt_max_size = 1500;
	CU_ASSERT_EQUAL(tskt_socket_set_rxpkt_max_size(&fake.base, 1500), 0);
	CU_ASSERT_FALSE(fake.set_rxpkt_max_size_called);

	/* op absent, changed value: updates rxpkt_max_size and flushes any
	 * pending free-list entries */
	CU_ASSERT_EQUAL(tpkt_list_new(&rxpkt_free), 0);
	pkt = tskt_socket_rxpkt_alloc(&fake.base);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	CU_ASSERT_EQUAL(tpkt_list_add_last(rxpkt_free, pkt), 0);
	tpkt_unref(pkt);
	fake.base.rxpkt_free = rxpkt_free;
	CU_ASSERT_EQUAL(tpkt_list_get_count(rxpkt_free), 1);

	CU_ASSERT_EQUAL(tskt_socket_set_rxpkt_max_size(&fake.base, 3000), 0);
	CU_ASSERT_EQUAL(fake.base.rxpkt_max_size, 3000);
	CU_ASSERT_EQUAL(tpkt_list_get_count(rxpkt_free), 0);

	tpkt_list_destroy(rxpkt_free);
}


static void test_read(void)
{
	struct fake_sock fake;
	char buf[16];
	uint64_t ts_us;

	CU_ASSERT_EQUAL(tskt_socket_read(NULL, buf, sizeof(buf), &ts_us),
			-EINVAL);

	fake_sock_init(&fake);
	fake.read_ret = -EAGAIN;
	CU_ASSERT_EQUAL(tskt_socket_read(&fake.base, buf, sizeof(buf), &ts_us),
			-EAGAIN);
	CU_ASSERT_TRUE(fake.read_called);

	fake_sock_init(&fake);
	fake.read_data = "hello";
	fake.read_data_len = 5;
	fake.read_ret = 5;
	fake.read_ts_us = 1000;
	memset(buf, 0, sizeof(buf));
	CU_ASSERT_EQUAL(tskt_socket_read(&fake.base, buf, sizeof(buf), &ts_us),
			5);
	CU_ASSERT_NSTRING_EQUAL(buf, "hello", 5);
	CU_ASSERT_EQUAL(ts_us, 1000);
}


static void test_write(void)
{
	struct fake_sock fake;

	CU_ASSERT_EQUAL(tskt_socket_write(NULL, "x", 1), -EINVAL);

	fake_sock_init(&fake);
	fake.write_ret = 3;
	CU_ASSERT_EQUAL(tskt_socket_write(&fake.base, "abc", 3), 3);
	CU_ASSERT_TRUE(fake.write_called);
	CU_ASSERT_EQUAL(fake.write_data_len, 3);
	CU_ASSERT_NSTRING_EQUAL(fake.write_data, "abc", 3);
}


static void test_readv(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;
	uint64_t ts_us;

	CU_ASSERT_EQUAL(tskt_socket_readv(NULL, NULL, 0, &ts_us), -EINVAL);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.readv = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_readv(&fake.base, NULL, 0, &ts_us),
			-EOPNOTSUPP);

	fake_sock_init(&fake);
	fake.readv_ret = 42;
	CU_ASSERT_EQUAL(tskt_socket_readv(&fake.base, NULL, 2, &ts_us), 42);
	CU_ASSERT_TRUE(fake.readv_called);
}


static void test_writev(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;

	CU_ASSERT_EQUAL(tskt_socket_writev(NULL, NULL, 0), -EINVAL);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.writev = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_writev(&fake.base, NULL, 0), -EOPNOTSUPP);

	fake_sock_init(&fake);
	fake.writev_ret = 24;
	CU_ASSERT_EQUAL(tskt_socket_writev(&fake.base, NULL, 2), 24);
	CU_ASSERT_TRUE(fake.writev_called);
}


static void test_write_cs(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;

	CU_ASSERT_EQUAL(tskt_socket_write_cs(NULL, "x", 1, -1), -EINVAL);

	/* write_cs not implemented: falls back to write(), class selector
	 * silently ignored */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.write_cs = NULL;
	fake.base.ops = &ops;
	fake.write_ret = 5;
	CU_ASSERT_EQUAL(tskt_socket_write_cs(&fake.base, "abcde", 5, 3), 5);
	CU_ASSERT_TRUE(fake.write_called);
	CU_ASSERT_FALSE(fake.write_cs_called);

	/* write_cs implemented: used directly with the given class selector */
	fake_sock_init(&fake);
	fake.write_cs_ret = 5;
	CU_ASSERT_EQUAL(tskt_socket_write_cs(&fake.base, "abcde", 5, 7), 5);
	CU_ASSERT_TRUE(fake.write_cs_called);
	CU_ASSERT_FALSE(fake.write_called);
	CU_ASSERT_EQUAL(fake.write_cs_cs, 7);
}


static void test_writev_cs(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;

	CU_ASSERT_EQUAL(tskt_socket_writev_cs(NULL, NULL, 0, -1), -EINVAL);

	/* writev missing is an -EOPNOTSUPP guard on its own, even if
	 * writev_cs happens to be implemented: the dispatcher never even
	 * looks at writev_cs in that case */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.writev = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_writev_cs(&fake.base, NULL, 0, 1),
			-EOPNOTSUPP);
	CU_ASSERT_FALSE(fake.writev_cs_called);

	/* writev present, writev_cs absent: falls back to writev(), class
	 * selector silently ignored */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.writev_cs = NULL;
	fake.base.ops = &ops;
	fake.writev_ret = 8;
	CU_ASSERT_EQUAL(tskt_socket_writev_cs(&fake.base, NULL, 2, 3), 8);
	CU_ASSERT_TRUE(fake.writev_called);
	CU_ASSERT_FALSE(fake.writev_cs_called);

	/* both present: writev_cs used directly */
	fake_sock_init(&fake);
	fake.writev_cs_ret = 8;
	CU_ASSERT_EQUAL(tskt_socket_writev_cs(&fake.base, NULL, 2, 9), 8);
	CU_ASSERT_TRUE(fake.writev_cs_called);
	CU_ASSERT_FALSE(fake.writev_called);
	CU_ASSERT_EQUAL(fake.writev_cs_cs, 9);
}


static void test_readmv(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;
	struct tskt_miovec miov[1];
	struct iovec iov;

	memset(&miov, 0, sizeof(miov));
	miov[0].iov = &iov;
	miov[0].iovlen = 1;

	CU_ASSERT_EQUAL(tskt_socket_readmv(NULL, miov, 1), -EINVAL);

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_socket_readmv(&fake.base, NULL, 1), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_readmv(&fake.base, miov, 0), -EINVAL);

	/* readmv implemented: pure passthrough */
	fake_sock_init(&fake);
	fake.readmv_ret = 3;
	CU_ASSERT_EQUAL(tskt_socket_readmv(&fake.base, miov, 3), 3);
	CU_ASSERT_TRUE(fake.readmv_called);

	/* readmv AND readv missing: -EOPNOTSUPP */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.readmv = NULL;
	ops.readv = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_readmv(&fake.base, miov, 1), -EOPNOTSUPP);

	/* readmv missing, readv present: emulate a single-message read */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.readmv = NULL;
	fake.base.ops = &ops;
	fake.readv_ret = 12;
	memset(&miov, 0, sizeof(miov));
	miov[0].iov = &iov;
	miov[0].iovlen = 1;
	CU_ASSERT_EQUAL(tskt_socket_readmv(&fake.base, miov, 1), 1);
	CU_ASSERT_TRUE(fake.readv_called);
	CU_ASSERT_EQUAL(miov[0].len, 12);

	/* readv error is propagated as-is (not wrapped to 1) */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.readmv = NULL;
	fake.base.ops = &ops;
	fake.readv_ret = -EIO;
	CU_ASSERT_EQUAL(tskt_socket_readmv(&fake.base, miov, 1), -EIO);
}


static void test_writemv(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;
	struct tskt_miovec miov[1];
	struct iovec iov;

	memset(&miov, 0, sizeof(miov));
	miov[0].iov = &iov;
	miov[0].iovlen = 1;

	CU_ASSERT_EQUAL(tskt_socket_writemv(NULL, miov, 1), -EINVAL);

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_socket_writemv(&fake.base, NULL, 1), -EINVAL);

	/* writemv implemented: pure passthrough, mlen == 0 included */
	fake_sock_init(&fake);
	fake.writemv_ret = 0;
	CU_ASSERT_EQUAL(tskt_socket_writemv(&fake.base, miov, 0), 0);
	CU_ASSERT_TRUE(fake.writemv_called);

	/* writemv AND writev missing: -EOPNOTSUPP */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.writemv = NULL;
	ops.writev = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_writemv(&fake.base, miov, 1), -EOPNOTSUPP);

	/* writemv missing, writev present, mlen == 0: no-op */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.writemv = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_writemv(&fake.base, miov, 0), 0);
	CU_ASSERT_FALSE(fake.writev_called);

	/* writemv missing, writev present, mlen > 0: emulate a single-message
	 * write */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.writemv = NULL;
	fake.base.ops = &ops;
	fake.writev_ret = 9;
	memset(&miov, 0, sizeof(miov));
	miov[0].iov = &iov;
	miov[0].iovlen = 1;
	CU_ASSERT_EQUAL(tskt_socket_writemv(&fake.base, miov, 1), 1);
	CU_ASSERT_TRUE(fake.writev_called);
	CU_ASSERT_EQUAL(miov[0].len, 9);

	/* writev error propagated as-is */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.writemv = NULL;
	fake.base.ops = &ops;
	fake.writev_ret = -EIO;
	CU_ASSERT_EQUAL(tskt_socket_writemv(&fake.base, miov, 1), -EIO);
}


static void test_read_pkt(void)
{
	struct fake_sock fake;

	CU_ASSERT_EQUAL(tskt_socket_read_pkt(NULL, NULL), -EINVAL);

	fake_sock_init(&fake);
	fake.read_pkt_ret = 0;
	CU_ASSERT_EQUAL(tskt_socket_read_pkt(&fake.base, NULL), 0);
	CU_ASSERT_TRUE(fake.read_pkt_called);
}


static void test_write_pkt(void)
{
	struct fake_sock fake;

	CU_ASSERT_EQUAL(tskt_socket_write_pkt(NULL, NULL), -EINVAL);

	fake_sock_init(&fake);
	fake.write_pkt_ret = 0;
	CU_ASSERT_EQUAL(tskt_socket_write_pkt(&fake.base, NULL), 0);
	CU_ASSERT_TRUE(fake.write_pkt_called);
}


static void test_read_pkt_alloc(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;
	struct tpkt_packet *ret_pkt;
	struct tpkt_packet *scripted;

	CU_ASSERT_EQUAL(tskt_socket_read_pkt_alloc(NULL, &ret_pkt), -EINVAL);

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_socket_read_pkt_alloc(&fake.base, NULL), -EINVAL);

	/* read_pkt_alloc implemented: pure passthrough */
	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tpkt_new(4, &scripted), 0);
	fake.read_pkt_alloc_ret = 0;
	fake.read_pkt_alloc_ret_obj = scripted;
	ret_pkt = NULL;
	CU_ASSERT_EQUAL(tskt_socket_read_pkt_alloc(&fake.base, &ret_pkt), 0);
	CU_ASSERT_PTR_EQUAL(ret_pkt, scripted);
	CU_ASSERT_TRUE(fake.read_pkt_alloc_called);
	tpkt_unref(scripted);

	/* read_pkt_alloc not implemented, read_pkt succeeds: emulated via
	 * tskt_socket_rxpkt_alloc() + ops->read_pkt() */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.read_pkt_alloc = NULL;
	fake.base.ops = &ops;
	fake.read_pkt_ret = 0;
	ret_pkt = NULL;
	CU_ASSERT_EQUAL(tskt_socket_read_pkt_alloc(&fake.base, &ret_pkt), 0);
	CU_ASSERT_TRUE(fake.read_pkt_called);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ret_pkt);
	tpkt_unref(ret_pkt);

	/* read_pkt_alloc not implemented, read_pkt fails: allocated packet is
	 * released back to the free list, *ret_pkt stays NULL */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.read_pkt_alloc = NULL;
	fake.base.ops = &ops;
	fake.read_pkt_ret = -EIO;
	ret_pkt = (struct tpkt_packet *)0x1; /* poison to check it's reset */
	CU_ASSERT_EQUAL(tskt_socket_read_pkt_alloc(&fake.base, &ret_pkt), -EIO);
	CU_ASSERT_PTR_NULL(ret_pkt);
	CU_ASSERT_PTR_NOT_NULL(fake.base.rxpkt_free);
	CU_ASSERT_EQUAL(tpkt_list_get_count(fake.base.rxpkt_free), 1);
	tpkt_list_destroy(fake.base.rxpkt_free);
}


static void test_read_pkt_list(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;
	struct tpkt_list *list = NULL;

	CU_ASSERT_EQUAL(tskt_socket_read_pkt_list(NULL, NULL, 1), -EINVAL);

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_socket_read_pkt_list(&fake.base, NULL, 1),
			-EINVAL);

	/* read_pkt_list implemented: pure passthrough */
	fake_sock_init(&fake);
	fake.read_pkt_list_ret = 3;
	CU_ASSERT_EQUAL(tpkt_list_new(&list), 0);
	CU_ASSERT_EQUAL(tskt_socket_read_pkt_list(&fake.base, list, 5), 3);
	CU_ASSERT_TRUE(fake.read_pkt_list_called);
	tpkt_list_destroy(list);

	/* not implemented, max_pkts == 0: no-op, nothing allocated */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.read_pkt_list = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tpkt_list_new(&list), 0);
	CU_ASSERT_EQUAL(tskt_socket_read_pkt_list(&fake.base, list, 0), 0);
	CU_ASSERT_FALSE(fake.read_pkt_called);
	tpkt_list_destroy(list);

	/* not implemented, read_pkt succeeds: one packet allocated and
	 * appended to the caller's list */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.read_pkt_list = NULL;
	fake.base.ops = &ops;
	fake.read_pkt_ret = 0;
	CU_ASSERT_EQUAL(tpkt_list_new(&list), 0);
	CU_ASSERT_EQUAL(tskt_socket_read_pkt_list(&fake.base, list, 5), 1);
	CU_ASSERT_EQUAL(tpkt_list_get_count(list), 1);
	tpkt_list_destroy(list);

	/* not implemented, read_pkt fails: allocated packet is released, not
	 * appended */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.read_pkt_list = NULL;
	fake.base.ops = &ops;
	fake.read_pkt_ret = -EIO;
	CU_ASSERT_EQUAL(tpkt_list_new(&list), 0);
	CU_ASSERT_EQUAL(tskt_socket_read_pkt_list(&fake.base, list, 5), -EIO);
	CU_ASSERT_EQUAL(tpkt_list_get_count(list), 0);
	tpkt_list_destroy(list);
	if (fake.base.rxpkt_free != NULL)
		tpkt_list_destroy(fake.base.rxpkt_free);
}


static void test_write_pkt_list(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;
	struct tpkt_list *list = NULL;
	struct tpkt_packet *pkt;

	CU_ASSERT_EQUAL(tskt_socket_write_pkt_list(NULL, NULL), -EINVAL);

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_socket_write_pkt_list(&fake.base, NULL), -EINVAL);

	/* write_pkt_list implemented: pure passthrough */
	fake_sock_init(&fake);
	fake.write_pkt_list_ret = 2;
	CU_ASSERT_EQUAL(tpkt_list_new(&list), 0);
	CU_ASSERT_EQUAL(tskt_socket_write_pkt_list(&fake.base, list), 2);
	CU_ASSERT_TRUE(fake.write_pkt_list_called);
	tpkt_list_destroy(list);

	/* not implemented, empty list: no-op */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.write_pkt_list = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tpkt_list_new(&list), 0);
	CU_ASSERT_EQUAL(tskt_socket_write_pkt_list(&fake.base, list), 0);
	CU_ASSERT_FALSE(fake.write_pkt_called);
	tpkt_list_destroy(list);

	/* not implemented, write_pkt succeeds: first packet removed from the
	 * list */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.write_pkt_list = NULL;
	fake.base.ops = &ops;
	fake.write_pkt_ret = 0;
	CU_ASSERT_EQUAL(tpkt_list_new(&list), 0);
	CU_ASSERT_EQUAL(tpkt_new(4, &pkt), 0);
	CU_ASSERT_EQUAL(tpkt_list_add_last(list, pkt), 0);
	CU_ASSERT_EQUAL(tskt_socket_write_pkt_list(&fake.base, list), 1);
	CU_ASSERT_EQUAL(tpkt_list_get_count(list), 0);
	CU_ASSERT_EQUAL(tpkt_get_ref_count(pkt), 1);
	tpkt_unref(pkt);
	tpkt_list_destroy(list);

	/* not implemented, write_pkt fails: packet stays linked in the list */
	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.write_pkt_list = NULL;
	fake.base.ops = &ops;
	fake.write_pkt_ret = -EIO;
	CU_ASSERT_EQUAL(tpkt_list_new(&list), 0);
	CU_ASSERT_EQUAL(tpkt_new(4, &pkt), 0);
	CU_ASSERT_EQUAL(tpkt_list_add_last(list, pkt), 0);
	CU_ASSERT_EQUAL(tskt_socket_write_pkt_list(&fake.base, list), -EIO);
	CU_ASSERT_EQUAL(tpkt_list_get_count(list), 1);
	tpkt_list_destroy(list);
	tpkt_unref(pkt);
}


static void test_set_event_cb(void)
{
	struct fake_sock fake;
	int userdata;

	CU_ASSERT_EQUAL(
		tskt_socket_set_event_cb(NULL, POMP_FD_EVENT_IN, NULL, NULL),
		-EINVAL);

	fake_sock_init(&fake);
	fake.set_event_cb_ret = 0;
	CU_ASSERT_EQUAL(tskt_socket_set_event_cb(
				&fake.base, POMP_FD_EVENT_IN, NULL, &userdata),
			0);
	CU_ASSERT_TRUE(fake.set_event_cb_called);
	CU_ASSERT_EQUAL(fake.set_event_cb_events, POMP_FD_EVENT_IN);
	CU_ASSERT_PTR_EQUAL(fake.event_cb_userdata, &userdata);
}


static void test_update_events(void)
{
	struct fake_sock fake;

	CU_ASSERT_EQUAL(tskt_socket_update_events(NULL, POMP_FD_EVENT_IN, 0),
			-EINVAL);

	fake_sock_init(&fake);
	fake.update_events_ret = 0;
	CU_ASSERT_EQUAL(tskt_socket_update_events(&fake.base,
						  POMP_FD_EVENT_OUT,
						  POMP_FD_EVENT_IN),
			0);
	CU_ASSERT_TRUE(fake.update_events_called);
	CU_ASSERT_EQUAL(fake.update_events_add, POMP_FD_EVENT_OUT);
	CU_ASSERT_EQUAL(fake.update_events_remove, POMP_FD_EVENT_IN);
}


static void test_connect(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;

	CU_ASSERT_EQUAL(tskt_socket_connect(NULL, NULL, 0, "1.2.3.4", 80),
			-EINVAL);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.connect = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_connect(&fake.base, NULL, 0, "1.2.3.4", 80),
			-EOPNOTSUPP);

	fake_sock_init(&fake);
	fake.connect_ret = 0;
	CU_ASSERT_EQUAL(
		tskt_socket_connect(&fake.base, "0.0.0.0", 0, "1.2.3.4", 80),
		0);
	CU_ASSERT_TRUE(fake.connect_called);
	CU_ASSERT_STRING_EQUAL(fake.connect_local_addr, "0.0.0.0");
	CU_ASSERT_STRING_EQUAL(fake.connect_remote_addr, "1.2.3.4");
	CU_ASSERT_EQUAL(fake.connect_remote_port, 80);
}


static void test_listen(void)
{
	struct fake_sock fake;
	struct tskt_socket_ops ops;

	CU_ASSERT_EQUAL(tskt_socket_listen(NULL, NULL, 0), -EINVAL);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.listen = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_listen(&fake.base, NULL, 1234),
			-EOPNOTSUPP);

	fake_sock_init(&fake);
	fake.listen_ret = 0;
	CU_ASSERT_EQUAL(tskt_socket_listen(&fake.base, "0.0.0.0", 1234), 0);
	CU_ASSERT_TRUE(fake.listen_called);
	CU_ASSERT_EQUAL(fake.listen_local_port, 1234);
}


static void test_accept(void)
{
	struct fake_sock fake;
	struct fake_sock accepted;
	struct tskt_socket_ops ops;
	struct tskt_socket *ret_obj;

	CU_ASSERT_EQUAL(tskt_socket_accept(NULL, NULL, 0, NULL, &ret_obj),
			-EINVAL);

	fake_sock_init(&fake);
	CU_ASSERT_EQUAL(tskt_socket_accept(&fake.base, NULL, 0, NULL, NULL),
			-EINVAL);

	fake_sock_init(&fake);
	ops = g_fake_sock_ops;
	ops.accept = NULL;
	fake.base.ops = &ops;
	CU_ASSERT_EQUAL(tskt_socket_accept(&fake.base, NULL, 0, NULL, &ret_obj),
			-EOPNOTSUPP);

	fake_sock_init(&fake);
	fake_sock_init(&accepted);
	fake.accept_ret = 0;
	fake.accept_ret_obj = &accepted.base;
	ret_obj = NULL;
	CU_ASSERT_EQUAL(tskt_socket_accept(&fake.base, NULL, 0, NULL, &ret_obj),
			0);
	CU_ASSERT_TRUE(fake.accept_called);
	CU_ASSERT_PTR_EQUAL(ret_obj, &accepted.base);
}


static void test_rxpkt_alloc_free(void)
{
	struct fake_sock fake;
	struct tpkt_packet *pkt1, *pkt2;

	fake_sock_init(&fake);

	/* free-list miss: fresh allocation, default size applied */
	pkt1 = tskt_socket_rxpkt_alloc(&fake.base);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt1);
	CU_ASSERT_EQUAL(fake.base.rxpkt_max_size,
			TSKT_SOCKET_RXPKT_MAX_SIZE_DEFAULT);

	/* free() puts it on the free list, lazily created */
	tskt_socket_rxpkt_free(&fake.base, pkt1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fake.base.rxpkt_free);
	CU_ASSERT_EQUAL(tpkt_list_get_count(fake.base.rxpkt_free), 1);

	/* free-list hit: alloc() reuses the freed packet, list drains */
	pkt2 = tskt_socket_rxpkt_alloc(&fake.base);
	CU_ASSERT_PTR_EQUAL(pkt2, pkt1);
	CU_ASSERT_EQUAL(tpkt_list_get_count(fake.base.rxpkt_free), 0);

	tskt_socket_rxpkt_free(&fake.base, pkt2);
	tpkt_list_destroy(fake.base.rxpkt_free);
}


CU_TestInfo g_tskt_test_dispatch[] = {
	{FN("destroy"), &test_destroy},
	{FN("get-loop"), &test_get_loop},
	{FN("get-fd"), &test_get_fd},
	{FN("set-fd-cb"), &test_set_fd_cb},
	{FN("get-local-addr"), &test_get_local_addr},
	{FN("get-local-port"), &test_get_local_port},
	{FN("get-remote-addr"), &test_get_remote_addr},
	{FN("get-remote-port"), &test_get_remote_port},
	{FN("set-remote"), &test_set_remote},
	{FN("option-getters-and-setters"), &test_option_getters_and_setters},
	{FN("get-rxpkt-max-size"), &test_get_rxpkt_max_size},
	{FN("set-rxpkt-max-size"), &test_set_rxpkt_max_size},
	{FN("read"), &test_read},
	{FN("write"), &test_write},
	{FN("readv"), &test_readv},
	{FN("writev"), &test_writev},
	{FN("write-cs"), &test_write_cs},
	{FN("writev-cs"), &test_writev_cs},
	{FN("readmv"), &test_readmv},
	{FN("writemv"), &test_writemv},
	{FN("read-pkt"), &test_read_pkt},
	{FN("write-pkt"), &test_write_pkt},
	{FN("read-pkt-alloc"), &test_read_pkt_alloc},
	{FN("read-pkt-list"), &test_read_pkt_list},
	{FN("write-pkt-list"), &test_write_pkt_list},
	{FN("set-event-cb"), &test_set_event_cb},
	{FN("update-events"), &test_update_events},
	{FN("connect"), &test_connect},
	{FN("listen"), &test_listen},
	{FN("accept"), &test_accept},
	{FN("rxpkt-alloc-free"), &test_rxpkt_alloc_free},

	CU_TEST_INFO_NULL,
};
