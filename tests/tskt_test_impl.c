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
 * Functional tests for src/tskt_impl.c, using real UDP/TCP sockets on the
 * loopback interface (127.0.0.1 / ::1) with a real pomp_loop, mirroring the
 * flows exercised by tools/tskt_tcp_client.c and tools/tskt_tcp_server.c.
 * All sockets are non-blocking, so every blocking-looking operation below is
 * a short bounded poll loop (pomp_loop_wait_and_process() is used purely as
 * a cross-platform sleep here; no fd_cb is registered on the loop).
 */

#include "tskt_test.h"

#include <netinet/in.h>
#include <sys/uio.h>


#define ADDR4 "127.0.0.1"
#define ADDR6 "::1"
#define TEST_TIMEOUT_MS 2000
#define POLL_STEP_MS 20


static int wait_read(struct pomp_loop *loop,
		     struct tskt_socket *sock,
		     void *buf,
		     size_t cap,
		     uint64_t *ts_us)
{
	int elapsed = 0;
	ssize_t res;

	do {
		res = tskt_socket_read(sock, buf, cap, ts_us);
		if (res != -EAGAIN)
			return (int)res;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	} while (elapsed < TEST_TIMEOUT_MS);

	return -EAGAIN;
}


static int wait_write(struct pomp_loop *loop,
		      struct tskt_socket *sock,
		      const void *buf,
		      size_t len)
{
	int elapsed = 0;
	ssize_t res;

	do {
		res = tskt_socket_write(sock, buf, len);
		if (res != -EAGAIN)
			return (int)res;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	} while (elapsed < TEST_TIMEOUT_MS);

	return -EAGAIN;
}


static int wait_accept(struct pomp_loop *loop,
		       struct tskt_socket *server,
		       char *addr,
		       size_t addr_len,
		       uint16_t *port,
		       struct tskt_socket **ret_obj)
{
	int elapsed = 0;
	int res;

	do {
		res = tskt_socket_accept(server, addr, addr_len, port, ret_obj);
		if (res != -EAGAIN)
			return res;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	} while (elapsed < TEST_TIMEOUT_MS);

	return -EAGAIN;
}


static int wait_read_pkt(struct pomp_loop *loop,
			 struct tskt_socket *sock,
			 struct tpkt_packet *pkt)
{
	int elapsed = 0;
	int res;

	do {
		res = tskt_socket_read_pkt(sock, pkt);
		if (res != -EAGAIN)
			return res;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	} while (elapsed < TEST_TIMEOUT_MS);

	return -EAGAIN;
}


static int wait_write_pkt(struct pomp_loop *loop,
			  struct tskt_socket *sock,
			  struct tpkt_packet *pkt)
{
	int elapsed = 0;
	int res;

	do {
		res = tskt_socket_write_pkt(sock, pkt);
		if (res != -EAGAIN)
			return res;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	} while (elapsed < TEST_TIMEOUT_MS);

	return -EAGAIN;
}


static void make_udp_pair(struct pomp_loop *loop,
			  bool is_v6,
			  struct tskt_socket **ret_a,
			  struct tskt_socket **ret_b)
{
	const char *addr = is_v6 ? ADDR6 : ADDR4;
	uint16_t port_a = 0, port_b = 0;
	int res;

	/* A: bind to an ephemeral port, remote unknown yet */
	if (is_v6)
		res = tskt_socket_new_udp6(
			addr, &port_a, NULL, 0, loop, NULL, NULL, ret_a);
	else
		res = tskt_socket_new(
			addr, &port_a, NULL, 0, NULL, loop, NULL, NULL, ret_a);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(port_a != 0);

	/* B: bind to an ephemeral port, connected to A */
	if (is_v6)
		res = tskt_socket_new_udp6(
			addr, &port_b, addr, port_a, loop, NULL, NULL, ret_b);
	else
		res = tskt_socket_new(addr,
				      &port_b,
				      addr,
				      port_a,
				      NULL,
				      loop,
				      NULL,
				      NULL,
				      ret_b);
	CU_ASSERT_EQUAL(res, 0);

	/* connect A back to B, now that B's port is known */
	res = tskt_socket_connect(*ret_a, NULL, 0, addr, port_b);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_udp_new_guards(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	CU_ASSERT_EQUAL(
		tskt_socket_new(
			NULL, NULL, NULL, 0, NULL, loop, NULL, NULL, NULL),
		-EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_new_udp6(
				NULL, NULL, NULL, 0, loop, NULL, NULL, NULL),
			-EINVAL);

	pomp_loop_destroy(loop);
}


static void test_tcp_new_guards(void)
{
	struct tskt_socket *sock = NULL;
	struct pomp_loop *loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	CU_ASSERT_EQUAL(tskt_socket_new_tcp(NULL, &sock), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_new_tcp(loop, NULL), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_new_tcp6(NULL, &sock), -EINVAL);
	CU_ASSERT_EQUAL(tskt_socket_new_tcp6(loop, NULL), -EINVAL);

	pomp_loop_destroy(loop);
}


static void udp_send_recv(bool is_v6)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *a = NULL, *b = NULL;
	char sbuf[] = "hello udp";
	char rbuf[64];
	uint64_t ts_us = 0;
	char addr_str[INET6_ADDRSTRLEN];
	uint16_t port;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	make_udp_pair(loop, is_v6, &a, &b);

	/* A -> B */
	res = wait_write(loop, a, sbuf, sizeof(sbuf));
	CU_ASSERT_EQUAL(res, (int)sizeof(sbuf));

	memset(rbuf, 0, sizeof(rbuf));
	res = wait_read(loop, b, rbuf, sizeof(rbuf), &ts_us);
	CU_ASSERT_EQUAL(res, (int)sizeof(sbuf));
	CU_ASSERT_NSTRING_EQUAL(rbuf, sbuf, sizeof(sbuf));
	CU_ASSERT_TRUE(ts_us > 0);

	/* local/remote addr and port reporting */
	CU_ASSERT_EQUAL(
		tskt_socket_get_local_addr(a, addr_str, sizeof(addr_str)), 0);
	CU_ASSERT_STRING_EQUAL(addr_str, is_v6 ? ADDR6 : ADDR4);
	port = tskt_socket_get_remote_port(a);
	CU_ASSERT_EQUAL(port, tskt_socket_get_local_port(b));

	tskt_socket_destroy(a);
	tskt_socket_destroy(b);
	pomp_loop_destroy(loop);
}


static void test_udp_ipv4_send_recv(void)
{
	udp_send_recv(false);
}


static void test_udp_ipv6_send_recv(void)
{
	udp_send_recv(true);
}


static void test_udp_ipv4_addrinuse_fallback(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *a = NULL, *b = NULL;
	uint16_t port_a = 0, port_b;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	/* A grabs an ephemeral port and keeps it bound */
	res = tskt_socket_new(
		ADDR4, &port_a, NULL, 0, NULL, loop, NULL, NULL, &a);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(port_a != 0);

	/* B explicitly asks for the same (still bound) port: documented
	 * IPv4 behaviour is to silently retry with an OS-assigned port
	 * instead of failing (see the "XXX this is crappy behaviour" comment
	 * in tskt_socket_new_udp() / do_bind() retry_bind loop). This is a
	 * known quirk, kept as-is (see CLAUDE.md), not fixed here. */
	port_b = port_a;
	res = tskt_socket_new(
		ADDR4, &port_b, NULL, 0, NULL, loop, NULL, NULL, &b);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(port_b != port_a);

	tskt_socket_destroy(a);
	tskt_socket_destroy(b);
	pomp_loop_destroy(loop);
}


static void test_udp_ipv6_addrinuse_no_fallback(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *a = NULL, *b = NULL;
	uint16_t port_a = 0, port_b;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	res = tskt_socket_new_udp6(
		ADDR6, &port_a, NULL, 0, loop, NULL, NULL, &a);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(port_a != 0);

	/* Known asymmetry with IPv4 (documented, not fixed per project
	 * decision): tskt_socket_new_udp6() has no EADDRINUSE retry, so
	 * requesting an already-bound port fails outright. */
	port_b = port_a;
	res = tskt_socket_new_udp6(
		ADDR6, &port_b, NULL, 0, loop, NULL, NULL, &b);
	CU_ASSERT_EQUAL(res, -EADDRINUSE);
	CU_ASSERT_PTR_NULL(b);

	tskt_socket_destroy(a);
	pomp_loop_destroy(loop);
}


static void test_udp_reset_option_unsupported(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *a = NULL;
	uint16_t port = 0;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	res = tskt_socket_new(
		ADDR4, &port, NULL, 0, NULL, loop, NULL, NULL, &a);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(tskt_socket_get_reset(a), -EOPNOTSUPP);
	CU_ASSERT_EQUAL(tskt_socket_set_reset(a, 1), -EOPNOTSUPP);

	tskt_socket_destroy(a);
	pomp_loop_destroy(loop);
}


static void test_udp_class_selector_option(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *a = NULL;
	uint16_t port = 0;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	res = tskt_socket_new(
		ADDR4, &port, NULL, 0, NULL, loop, NULL, NULL, &a);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(tskt_socket_set_class_selector(a, 0x10), 0);
	CU_ASSERT_TRUE(tskt_socket_get_class_selector(a) >= 0);

	tskt_socket_destroy(a);
	pomp_loop_destroy(loop);
}


static void test_rxbuf_txbuf_size_options(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *a = NULL;
	uint16_t port = 0;
	int res, size;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	res = tskt_socket_new(
		ADDR4, &port, NULL, 0, NULL, loop, NULL, NULL, &a);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(tskt_socket_set_rxbuf_size(a, 65536), 0);
	size = tskt_socket_get_rxbuf_size(a);
	CU_ASSERT_TRUE(size > 0);

	CU_ASSERT_EQUAL(tskt_socket_set_txbuf_size(a, 65536), 0);
	size = tskt_socket_get_txbuf_size(a);
	CU_ASSERT_TRUE(size > 0);

	tskt_socket_destroy(a);
	pomp_loop_destroy(loop);
}


static void tcp_connect_accept(struct pomp_loop *loop,
			       bool is_v6,
			       struct tskt_socket **ret_client,
			       struct tskt_socket **ret_accepted)
{
	const char *addr = is_v6 ? ADDR6 : ADDR4;
	struct tskt_socket *server = NULL;
	char remote_addr[INET6_ADDRSTRLEN];
	uint16_t server_port = 0, remote_port = 0;
	int res;

	if (is_v6)
		res = tskt_socket_new_tcp6(loop, &server);
	else
		res = tskt_socket_new_tcp(loop, &server);
	CU_ASSERT_EQUAL(res, 0);

	res = tskt_socket_listen(server, addr, 0);
	CU_ASSERT_EQUAL(res, 0);
	server_port = tskt_socket_get_local_port(server);
	CU_ASSERT_TRUE(server_port != 0);

	if (is_v6)
		res = tskt_socket_new_tcp6(loop, ret_client);
	else
		res = tskt_socket_new_tcp(loop, ret_client);
	CU_ASSERT_EQUAL(res, 0);

	res = tskt_socket_connect(*ret_client, NULL, 0, addr, server_port);
	CU_ASSERT_EQUAL(res, 0);

	res = wait_accept(loop,
			  server,
			  remote_addr,
			  sizeof(remote_addr),
			  &remote_port,
			  ret_accepted);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(*ret_accepted);
	CU_ASSERT_STRING_EQUAL(remote_addr, addr);

	tskt_socket_destroy(server);
}


static void tcp_data_roundtrip(bool is_v6)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *client = NULL, *accepted = NULL;
	char sbuf[] = "hello tcp";
	char rbuf[64];
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	tcp_connect_accept(loop, is_v6, &client, &accepted);

	res = wait_write(loop, client, sbuf, sizeof(sbuf));
	CU_ASSERT_EQUAL(res, (int)sizeof(sbuf));

	memset(rbuf, 0, sizeof(rbuf));
	res = wait_read(loop, accepted, rbuf, sizeof(rbuf), NULL);
	CU_ASSERT_EQUAL(res, (int)sizeof(sbuf));
	CU_ASSERT_NSTRING_EQUAL(rbuf, sbuf, sizeof(sbuf));

	/* echo back */
	res = wait_write(loop, accepted, rbuf, res);
	CU_ASSERT_TRUE(res > 0);
	memset(rbuf, 0, sizeof(rbuf));
	res = wait_read(loop, client, rbuf, sizeof(rbuf), NULL);
	CU_ASSERT_EQUAL(res, (int)sizeof(sbuf));
	CU_ASSERT_NSTRING_EQUAL(rbuf, sbuf, sizeof(sbuf));

	/* half-close: destroying the peer must eventually surface as EOF */
	tskt_socket_destroy(accepted);
	res = wait_read(loop, client, rbuf, sizeof(rbuf), NULL);
	CU_ASSERT_EQUAL(res, 0);

	tskt_socket_destroy(client);
	pomp_loop_destroy(loop);
}


static void test_tcp_ipv4_data_roundtrip(void)
{
	tcp_data_roundtrip(false);
}


static void test_tcp_ipv6_data_roundtrip(void)
{
	tcp_data_roundtrip(true);
}


static void test_tcp_get_remote_addr_unconnected(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *sock = NULL;
	char addr_str[INET6_ADDRSTRLEN];
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	res = tskt_socket_new_tcp(loop, &sock);
	CU_ASSERT_EQUAL(res, 0);

	res = tskt_socket_get_remote_addr(sock, addr_str, sizeof(addr_str));
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_STRING_EQUAL(addr_str, "0.0.0.0");
	CU_ASSERT_EQUAL(tskt_socket_get_remote_port(sock), 0);

	tskt_socket_destroy(sock);
	pomp_loop_destroy(loop);
}


static void test_tcp_reset_option(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *client = NULL, *accepted = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	tcp_connect_accept(loop, false, &client, &accepted);

	CU_ASSERT_EQUAL(tskt_socket_get_reset(client), 0);
	CU_ASSERT_EQUAL(tskt_socket_set_reset(client, 1), 0);
	CU_ASSERT_EQUAL(tskt_socket_get_reset(client), 1);

	tskt_socket_destroy(client);
	tskt_socket_destroy(accepted);
	pomp_loop_destroy(loop);
}


static void test_tcp_nodelay_option(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *client = NULL, *accepted = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	tcp_connect_accept(loop, false, &client, &accepted);

	CU_ASSERT_EQUAL(tskt_socket_set_nodelay(client, 1), 0);
	CU_ASSERT_TRUE(tskt_socket_get_nodelay(client) != 0);
	CU_ASSERT_EQUAL(tskt_socket_set_nodelay(client, 0), 0);
	CU_ASSERT_EQUAL(tskt_socket_get_nodelay(client), 0);

	tskt_socket_destroy(client);
	tskt_socket_destroy(accepted);
	pomp_loop_destroy(loop);
}


static void test_tcp_read_write_pkt(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *client = NULL, *accepted = NULL;
	struct tpkt_packet *wpkt = NULL, *rpkt = NULL;
	const char payload[] = "hello pkt";
	const void *rdata;
	size_t rlen;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	tcp_connect_accept(loop, false, &client, &accepted);

	CU_ASSERT_EQUAL(tpkt_new_with_data(payload, sizeof(payload), &wpkt), 0);
	res = wait_write_pkt(loop, client, wpkt);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(tpkt_new(128, &rpkt), 0);
	res = wait_read_pkt(loop, accepted, rpkt);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(tpkt_get_cdata(rpkt, &rdata, &rlen, NULL), 0);
	CU_ASSERT_EQUAL(rlen, sizeof(payload));
	CU_ASSERT_NSTRING_EQUAL(rdata, payload, sizeof(payload));

	tpkt_unref(wpkt);
	tpkt_unref(rpkt);
	tskt_socket_destroy(client);
	tskt_socket_destroy(accepted);
	pomp_loop_destroy(loop);
}


static void test_tcp_writev_readv(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *client = NULL, *accepted = NULL;
	char part1[] = "hello ";
	char part2[] = "iovec";
	char rbuf[64];
	struct iovec wiov[2];
	struct iovec riov[2];
	ssize_t res;
	int elapsed;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	tcp_connect_accept(loop, false, &client, &accepted);

	wiov[0].iov_base = part1;
	wiov[0].iov_len = strlen(part1);
	wiov[1].iov_base = part2;
	wiov[1].iov_len = strlen(part2) + 1; /* include NUL */

	elapsed = 0;
	do {
		res = tskt_socket_writev(client, wiov, 2);
		if (res != -EAGAIN)
			break;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	} while (elapsed < TEST_TIMEOUT_MS);
	CU_ASSERT_EQUAL(res, (ssize_t)(wiov[0].iov_len + wiov[1].iov_len));

	memset(rbuf, 0, sizeof(rbuf));
	riov[0].iov_base = rbuf;
	riov[0].iov_len = sizeof(rbuf);
	elapsed = 0;
	do {
		res = tskt_socket_readv(accepted, riov, 1, NULL);
		if (res != -EAGAIN)
			break;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	} while (elapsed < TEST_TIMEOUT_MS);
	CU_ASSERT_EQUAL(res, (ssize_t)(wiov[0].iov_len + wiov[1].iov_len));
	CU_ASSERT_STRING_EQUAL(rbuf, "hello iovec");

	tskt_socket_destroy(client);
	tskt_socket_destroy(accepted);
	pomp_loop_destroy(loop);
}


#ifdef __linux__

static void test_udp_read_pkt_list(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *a = NULL, *b = NULL;
	struct tpkt_list *list = NULL;
	const char msg1[] = "pkt1";
	const char msg2[] = "pkt2";
	ssize_t count;
	int elapsed, res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	make_udp_pair(loop, false, &a, &b);

	CU_ASSERT_EQUAL(wait_write(loop, a, msg1, sizeof(msg1)),
			(int)sizeof(msg1));
	CU_ASSERT_EQUAL(wait_write(loop, a, msg2, sizeof(msg2)),
			(int)sizeof(msg2));

	CU_ASSERT_EQUAL(tpkt_list_new(&list), 0);
	elapsed = 0;
	do {
		count = tskt_socket_read_pkt_list(b, list, 8);
		if (count != -EAGAIN)
			break;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	} while (elapsed < TEST_TIMEOUT_MS);

	CU_ASSERT_TRUE(count >= 1);
	CU_ASSERT_EQUAL(tpkt_list_get_count(list), (int)count);

	res = tpkt_list_flush(list);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_list_destroy(list);
	tskt_socket_destroy(a);
	tskt_socket_destroy(b);
	pomp_loop_destroy(loop);
}


static void test_udp_write_cs(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *a = NULL, *b = NULL;
	const char msg[] = "cs";
	char rbuf[64];
	ssize_t res;
	int elapsed;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	make_udp_pair(loop, false, &a, &b);

	elapsed = 0;
	do {
		res = tskt_socket_write_cs(a, msg, sizeof(msg), 0x10);
		if (res != -EAGAIN)
			break;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	} while (elapsed < TEST_TIMEOUT_MS);
	CU_ASSERT_EQUAL(res, (ssize_t)sizeof(msg));

	memset(rbuf, 0, sizeof(rbuf));
	CU_ASSERT_EQUAL(wait_read(loop, b, rbuf, sizeof(rbuf), NULL),
			(int)sizeof(msg));
	CU_ASSERT_NSTRING_EQUAL(rbuf, msg, sizeof(msg));

	tskt_socket_destroy(a);
	tskt_socket_destroy(b);
	pomp_loop_destroy(loop);
}

#endif /* __linux__ */


CU_TestInfo g_tskt_test_impl[] = {
	{FN("udp-new-guards"), &test_udp_new_guards},
	{FN("tcp-new-guards"), &test_tcp_new_guards},
	{FN("udp-ipv4-send-recv"), &test_udp_ipv4_send_recv},
	{FN("udp-ipv6-send-recv"), &test_udp_ipv6_send_recv},
	{FN("udp-ipv4-addrinuse-fallback"), &test_udp_ipv4_addrinuse_fallback},
	{FN("udp-ipv6-addrinuse-no-fallback"),
	 &test_udp_ipv6_addrinuse_no_fallback},
	{FN("udp-reset-option-unsupported"),
	 &test_udp_reset_option_unsupported},
	{FN("udp-class-selector-option"), &test_udp_class_selector_option},
	{FN("rxbuf-txbuf-size-options"), &test_rxbuf_txbuf_size_options},
	{FN("tcp-ipv4-data-roundtrip"), &test_tcp_ipv4_data_roundtrip},
	{FN("tcp-ipv6-data-roundtrip"), &test_tcp_ipv6_data_roundtrip},
	{FN("tcp-get-remote-addr-unconnected"),
	 &test_tcp_get_remote_addr_unconnected},
	{FN("tcp-reset-option"), &test_tcp_reset_option},
	{FN("tcp-nodelay-option"), &test_tcp_nodelay_option},
	{FN("tcp-read-write-pkt"), &test_tcp_read_write_pkt},
	{FN("tcp-writev-readv"), &test_tcp_writev_readv},
#ifdef __linux__
	{FN("udp-read-pkt-list"), &test_udp_read_pkt_list},
	{FN("udp-write-cs"), &test_udp_write_cs},
#endif

	CU_TEST_INFO_NULL,
};
