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
 * Tests for src/tskt_client.c: parameter-validation guards need no real I/O
 * (a client that never connects always has a NULL internal socket), but the
 * reconnect state machine itself (connect / disconnect / auto-reconnect /
 * stop) is only meaningfully exercised with a real TCP server on the
 * loopback interface, since client_start() always creates a real
 * tskt_socket_new_tcp() internally (it is not injectable via the fakes).
 */

#include "tskt_test.h"

#include <netinet/in.h>


#define ADDR4 "127.0.0.1"
#define TEST_TIMEOUT_MS 2000
#define POLL_STEP_MS 20


struct client_test_ctx {
	int event_count;
	int last_event;
	struct tskt_socket *last_sock;
	int data_count;
	char last_data[256];
	size_t last_data_len;

	bool socket_created_called;
	int socket_created_fd;
	bool socket_upgrade_called;
	bool reject_upgrade; /* read by ctx_socket_upgrade_cb */
};


static void reset_ctx(struct client_test_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->last_event = -1;
}


static void ctx_event_cb(struct tskt_client *self,
			 enum tskt_client_event event,
			 struct tskt_socket *sock,
			 void *userdata)
{
	struct client_test_ctx *ctx = userdata;

	ctx->event_count++;
	ctx->last_event = (int)event;
	ctx->last_sock = sock;
}


static void
ctx_data_cb(struct tskt_client *self, struct tpkt_packet *pkt, void *userdata)
{
	struct client_test_ctx *ctx = userdata;
	const void *data;
	size_t len;

	if (tpkt_get_cdata(pkt, &data, &len, NULL) == 0) {
		size_t n = len < sizeof(ctx->last_data)
				   ? len
				   : sizeof(ctx->last_data);
		memcpy(ctx->last_data, data, n);
		ctx->last_data_len = len;
	}
	ctx->data_count++;
}


static struct tskt_client_cbs s_cbs = {
	.event_cb = ctx_event_cb,
	.data_cb = ctx_data_cb,
};


static void ctx_socket_created_cb(struct tskt_client *self,
				  struct tskt_socket *sock,
				  int fd,
				  void *userdata)
{
	struct client_test_ctx *ctx = userdata;

	ctx->socket_created_called = true;
	ctx->socket_created_fd = fd;
}


static struct tskt_socket *ctx_socket_upgrade_cb(struct tskt_client *self,
						 struct tskt_socket *sock,
						 void *userdata)
{
	struct client_test_ctx *ctx = userdata;

	ctx->socket_upgrade_called = true;
	return ctx->reject_upgrade ? NULL : sock;
}


static struct tskt_client_cbs s_cbs_full = {
	.event_cb = ctx_event_cb,
	.data_cb = ctx_data_cb,
	.socket_created_cb = ctx_socket_created_cb,
	.socket_upgrade_cb = ctx_socket_upgrade_cb,
};


static bool spin_until_event_count(struct pomp_loop *loop,
				   struct client_test_ctx *ctx,
				   int count,
				   int timeout_ms)
{
	int elapsed = 0;

	while (ctx->event_count < count) {
		if (elapsed >= timeout_ms)
			return false;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	}
	return true;
}


static bool spin_until_data_count(struct pomp_loop *loop,
				  struct client_test_ctx *ctx,
				  int count,
				  int timeout_ms)
{
	int elapsed = 0;

	while (ctx->data_count < count) {
		if (elapsed >= timeout_ms)
			return false;
		pomp_loop_wait_and_process(loop, POLL_STEP_MS);
		elapsed += POLL_STEP_MS;
	}
	return true;
}


static int wait_read(struct pomp_loop *loop,
		     struct tskt_socket *sock,
		     void *buf,
		     size_t cap)
{
	int elapsed = 0;
	ssize_t res;

	do {
		res = tskt_socket_read(sock, buf, cap, NULL);
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


static struct tskt_socket *make_listen_server(struct pomp_loop *loop,
					      uint16_t *port)
{
	struct tskt_socket *server = NULL;
	int res;

	res = tskt_socket_new_tcp(loop, &server);
	CU_ASSERT_EQUAL(res, 0);
	res = tskt_socket_listen(server, ADDR4, 0);
	CU_ASSERT_EQUAL(res, 0);
	*port = tskt_socket_get_local_port(server);
	CU_ASSERT_TRUE(*port != 0);

	return server;
}


static void test_new_guards(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_client *client = NULL;
	struct tskt_client_cbs bad_cbs;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	CU_ASSERT_EQUAL(tskt_client_new(NULL, s_cbs, NULL, &client), -EINVAL);
	CU_ASSERT_EQUAL(tskt_client_new(loop, s_cbs, NULL, NULL), -EINVAL);

	bad_cbs = s_cbs;
	bad_cbs.event_cb = NULL;
	CU_ASSERT_EQUAL(tskt_client_new(loop, bad_cbs, NULL, &client), -EINVAL);

	bad_cbs = s_cbs;
	bad_cbs.data_cb = NULL;
	CU_ASSERT_EQUAL(tskt_client_new(loop, bad_cbs, NULL, &client), -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_connect_guards(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_client *client = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	CU_ASSERT_EQUAL(tskt_client_new(loop, s_cbs, NULL, &client), 0);

	CU_ASSERT_EQUAL(tskt_client_connect(NULL, NULL, 0, ADDR4, 1), -EINVAL);
	CU_ASSERT_EQUAL(tskt_client_connect(client, NULL, 0, NULL, 1), -EINVAL);
	CU_ASSERT_EQUAL(tskt_client_connect(client, NULL, 0, "", 1), -EINVAL);
	CU_ASSERT_EQUAL(tskt_client_connect(client, NULL, 0, ADDR4, 0),
			-EINVAL);

	tskt_client_destroy(client);
	pomp_loop_destroy(loop);
}


static void test_send_and_accessor_guards(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_client *client = NULL;
	struct tpkt_packet *pkt = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	CU_ASSERT_EQUAL(tskt_client_new(loop, s_cbs, NULL, &client), 0);

	CU_ASSERT_EQUAL(tskt_client_send(NULL, "x", 1), -EINVAL);
	CU_ASSERT_EQUAL(tskt_client_send(client, NULL, 1), -EINVAL);
	CU_ASSERT_EQUAL(tskt_client_send(client, "x", 0), -EINVAL);
	/* never connected: internal socket is NULL */
	CU_ASSERT_EQUAL(tskt_client_send(client, "x", 1), -ENOTCONN);

	CU_ASSERT_EQUAL(tpkt_new_with_data("x", 1, &pkt), 0);
	CU_ASSERT_EQUAL(tskt_client_send_pkt(NULL, pkt), -EINVAL);
	CU_ASSERT_EQUAL(tskt_client_send_pkt(client, NULL), -EINVAL);
	CU_ASSERT_EQUAL(tskt_client_send_pkt(client, pkt), -ENOTCONN);
	tpkt_unref(pkt);

	CU_ASSERT_PTR_NULL(tskt_client_get_socket(NULL));
	CU_ASSERT_PTR_NULL(tskt_client_get_socket(client));

	tskt_client_destroy(client);
	pomp_loop_destroy(loop);
}


static void test_stop_destroy_guards(void)
{
	CU_ASSERT_EQUAL(tskt_client_stop(NULL), -EINVAL);
	CU_ASSERT_EQUAL(tskt_client_destroy(NULL), -EINVAL);
}


static void test_connect_and_data_roundtrip(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *server, *peer = NULL;
	struct tskt_client *client = NULL;
	struct client_test_ctx ctx;
	char remote_addr[INET6_ADDRSTRLEN];
	uint16_t port, remote_port;
	char rbuf[64];
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx);
	server = make_listen_server(loop, &port);

	CU_ASSERT_EQUAL(tskt_client_new(loop, s_cbs, &ctx, &client), 0);
	res = tskt_client_connect(client, NULL, 0, ADDR4, port);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(spin_until_event_count(loop, &ctx, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_CLIENT_EVENT_CONNECTED);
	CU_ASSERT_PTR_EQUAL(ctx.last_sock, tskt_client_get_socket(client));

	res = wait_accept(loop,
			  server,
			  remote_addr,
			  sizeof(remote_addr),
			  &remote_port,
			  &peer);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(peer);

	/* client -> peer */
	res = (int)tskt_client_send(client, "ping", 5);
	CU_ASSERT_EQUAL(res, 5);
	memset(rbuf, 0, sizeof(rbuf));
	res = wait_read(loop, peer, rbuf, sizeof(rbuf));
	CU_ASSERT_EQUAL(res, 5);
	CU_ASSERT_STRING_EQUAL(rbuf, "ping");

	/* peer -> client, delivered through data_cb as a tpkt_packet */
	res = wait_write(loop, peer, "pong", 5);
	CU_ASSERT_EQUAL(res, 5);
	CU_ASSERT_TRUE(spin_until_data_count(loop, &ctx, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx.last_data_len, (size_t)5);
	CU_ASSERT_STRING_EQUAL(ctx.last_data, "pong");

	tskt_socket_destroy(peer);
	tskt_socket_destroy(server);
	tskt_client_destroy(client);
	pomp_loop_destroy(loop);
}


/*
 * Slow test (~2s): drives the reconnect state machine end-to-end, including
 * the RECONNECT_DELAY_MS timer. Real-time loopback tests of this kind were
 * explicitly approved for this package (see CLAUDE.md).
 */
static void test_disconnect_and_reconnect(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *server, *peer = NULL;
	struct tskt_client *client = NULL;
	struct client_test_ctx ctx;
	char remote_addr[INET6_ADDRSTRLEN];
	uint16_t port, remote_port;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx);
	server = make_listen_server(loop, &port);
	CU_ASSERT_EQUAL(tskt_client_new(loop, s_cbs, &ctx, &client), 0);
	CU_ASSERT_EQUAL(tskt_client_connect(client, NULL, 0, ADDR4, port), 0);
	CU_ASSERT_TRUE(spin_until_event_count(loop, &ctx, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_CLIENT_EVENT_CONNECTED);

	res = wait_accept(loop,
			  server,
			  remote_addr,
			  sizeof(remote_addr),
			  &remote_port,
			  &peer);
	CU_ASSERT_EQUAL(res, 0);

	/* simulate the peer disconnecting */
	tskt_socket_destroy(peer);
	peer = NULL;
	CU_ASSERT_TRUE(spin_until_event_count(loop, &ctx, 2, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_CLIENT_EVENT_DISCONNECTED);
	CU_ASSERT_PTR_NULL(tskt_client_get_socket(client));

	/* auto-reconnect after the ~2s RECONNECT_DELAY_MS timer */
	CU_ASSERT_TRUE(spin_until_event_count(loop, &ctx, 3, 4000));
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_CLIENT_EVENT_CONNECTED);

	res = wait_accept(loop,
			  server,
			  remote_addr,
			  sizeof(remote_addr),
			  &remote_port,
			  &peer);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(peer);

	tskt_socket_destroy(peer);
	tskt_socket_destroy(server);
	tskt_client_destroy(client);
	pomp_loop_destroy(loop);
}


static void test_stop(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *server, *peer = NULL;
	struct tskt_client *client = NULL;
	struct client_test_ctx ctx;
	char remote_addr[INET6_ADDRSTRLEN];
	uint16_t port, remote_port;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx);
	server = make_listen_server(loop, &port);
	CU_ASSERT_EQUAL(tskt_client_new(loop, s_cbs, &ctx, &client), 0);
	CU_ASSERT_EQUAL(tskt_client_connect(client, NULL, 0, ADDR4, port), 0);
	CU_ASSERT_TRUE(spin_until_event_count(loop, &ctx, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(wait_accept(loop,
				    server,
				    remote_addr,
				    sizeof(remote_addr),
				    &remote_port,
				    &peer),
			0);

	CU_ASSERT_EQUAL(tskt_client_stop(client), 0);
	CU_ASSERT_TRUE(spin_until_event_count(loop, &ctx, 2, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_CLIENT_EVENT_DISCONNECTED);
	CU_ASSERT_PTR_NULL(tskt_client_get_socket(client));

	/* stopping an already-stopped client is a documented no-op */
	CU_ASSERT_EQUAL(tskt_client_stop(client), 0);
	CU_ASSERT_EQUAL(ctx.event_count, 2);

	/* the client context is reusable after stop */
	CU_ASSERT_EQUAL(tskt_client_connect(client, NULL, 0, ADDR4, port), 0);
	CU_ASSERT_TRUE(spin_until_event_count(loop, &ctx, 3, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_CLIENT_EVENT_CONNECTED);

	tskt_socket_destroy(peer);
	tskt_socket_destroy(server);
	tskt_client_destroy(client);
	pomp_loop_destroy(loop);
}


/*
 * Regression/documentation test: a connection attempt that fails before
 * ever reaching CONNECTED (here, refused because nothing listens on the
 * port anymore) is silently retried by the internal timer without ever
 * notifying the application - see the "error:" label in sock_event_cb()
 * (src/tskt_client.c), which only calls notify_event() if self->connected
 * was already true. This pins the current behaviour; the retry itself
 * (after RECONNECT_DELAY_MS) is already covered by
 * test_disconnect_and_reconnect().
 */
static void test_connect_refused_no_event(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *tmp = NULL;
	struct tskt_client *client = NULL;
	struct client_test_ctx ctx;
	uint16_t port = 0;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx);

	/* grab a port and free it right away: nothing listens on it anymore,
	 * so a connect attempt to it is refused */
	tmp = make_listen_server(loop, &port);
	tskt_socket_destroy(tmp);

	CU_ASSERT_EQUAL(tskt_client_new(loop, s_cbs, &ctx, &client), 0);
	res = tskt_client_connect(client, NULL, 0, ADDR4, port);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_FALSE(spin_until_event_count(loop, &ctx, 1, 300));

	tskt_client_destroy(client);
	pomp_loop_destroy(loop);
}


static void test_socket_created_and_upgrade_callbacks(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *server, *peer = NULL;
	struct tskt_client *client = NULL;
	struct client_test_ctx ctx;
	char remote_addr[INET6_ADDRSTRLEN];
	uint16_t port, remote_port;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx);
	server = make_listen_server(loop, &port);

	CU_ASSERT_EQUAL(tskt_client_new(loop, s_cbs_full, &ctx, &client), 0);
	res = tskt_client_connect(client, NULL, 0, ADDR4, port);
	CU_ASSERT_EQUAL(res, 0);

	/* both optional callbacks are invoked synchronously from
	 * client_start(), before tskt_client_connect() even returns */
	CU_ASSERT_TRUE(ctx.socket_created_called);
	CU_ASSERT_TRUE(ctx.socket_created_fd >= 0);
	CU_ASSERT_TRUE(ctx.socket_upgrade_called);

	CU_ASSERT_TRUE(spin_until_event_count(loop, &ctx, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_CLIENT_EVENT_CONNECTED);

	res = wait_accept(loop,
			  server,
			  remote_addr,
			  sizeof(remote_addr),
			  &remote_port,
			  &peer);
	CU_ASSERT_EQUAL(res, 0);

	tskt_socket_destroy(peer);
	tskt_socket_destroy(server);
	tskt_client_destroy(client);
	pomp_loop_destroy(loop);
}


/*
 * Regression test: socket_upgrade_cb() rejecting the connection (returning
 * NULL) used to make client_start() - and thus tskt_client_connect() -
 * report success (0) anyway, because the "res" local variable still held
 * the leftover 0 result of the preceding tskt_socket_connect() call at the
 * "goto error" site. This left the client stuck: "started" latched true
 * (rejecting any further tskt_client_connect() with -EBUSY) while sock was
 * NULL and no reconnect timer had been armed. Fixed in src/tskt_client.c by
 * setting res = -ECANCELED before the jump.
 */
static void test_socket_upgrade_reject(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *server, *peer = NULL;
	struct tskt_client *client = NULL;
	struct client_test_ctx ctx;
	char remote_addr[INET6_ADDRSTRLEN];
	uint16_t port, remote_port;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx);
	ctx.reject_upgrade = true;
	server = make_listen_server(loop, &port);

	CU_ASSERT_EQUAL(tskt_client_new(loop, s_cbs_full, &ctx, &client), 0);
	res = tskt_client_connect(client, NULL, 0, ADDR4, port);
	CU_ASSERT_TRUE(res < 0);
	CU_ASSERT_TRUE(ctx.socket_upgrade_called);
	CU_ASSERT_PTR_NULL(tskt_client_get_socket(client));
	CU_ASSERT_EQUAL(ctx.event_count, 0);

	/* "started" must have been reset to false: a further connect() must
	 * not be rejected with -EBUSY, and must succeed normally */
	ctx.reject_upgrade = false;
	res = tskt_client_connect(client, NULL, 0, ADDR4, port);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(spin_until_event_count(loop, &ctx, 1, TEST_TIMEOUT_MS));
	CU_ASSERT_EQUAL(ctx.last_event, (int)TSKT_CLIENT_EVENT_CONNECTED);

	res = wait_accept(loop,
			  server,
			  remote_addr,
			  sizeof(remote_addr),
			  &remote_port,
			  &peer);
	CU_ASSERT_EQUAL(res, 0);

	tskt_socket_destroy(peer);
	tskt_socket_destroy(server);
	tskt_client_destroy(client);
	pomp_loop_destroy(loop);
}


/*
 * Coverage/documentation test for client_start()'s *synchronous*
 * connect-failure branch, as opposed to test_connect_refused_no_event()
 * which exercises the *asynchronous* refusal detected later via
 * POMP_FD_EVENT_ERR: here tskt_socket_connect() fails synchronously because
 * do_bind() cannot bind the requested local port (already in use by
 * "occupier"), well before any connect() syscall is even attempted.
 */
static void test_client_start_local_bind_conflict(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *server, *occupier;
	struct tskt_client *client = NULL;
	struct client_test_ctx ctx;
	uint16_t port, occupied_port = 0;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx);
	server = make_listen_server(loop, &port);
	occupier = make_listen_server(loop, &occupied_port);

	CU_ASSERT_EQUAL(tskt_client_new(loop, s_cbs, &ctx, &client), 0);
	/* force the client's local bind to collide with "occupier": use the
	 * same exact address (not the wildcard) so the EADDRINUSE outcome
	 * doesn't depend on wildcard-vs-specific SO_REUSEADDR overlap rules
	 * (two sockets cannot both listen on the exact same address:port
	 * regardless of SO_REUSEADDR) */
	res = tskt_client_connect(client, ADDR4, occupied_port, ADDR4, port);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_FALSE(spin_until_event_count(loop, &ctx, 1, 300));
	CU_ASSERT_PTR_NULL(tskt_client_get_socket(client));

	tskt_client_destroy(client);
	tskt_socket_destroy(occupier);
	tskt_socket_destroy(server);
	pomp_loop_destroy(loop);
}


static void test_send_pkt_success_and_eagain(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct tskt_socket *server, *peer = NULL, *sock;
	struct tskt_client *client = NULL;
	struct client_test_ctx ctx;
	char remote_addr[INET6_ADDRSTRLEN];
	uint16_t port, remote_port;
	struct tpkt_packet *pkt, *pkt2, *big_pkt;
	char rbuf[64];
	char *big;
	size_t big_len = 4 * 1024 * 1024;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	reset_ctx(&ctx);
	server = make_listen_server(loop, &port);
	CU_ASSERT_EQUAL(tskt_client_new(loop, s_cbs, &ctx, &client), 0);
	CU_ASSERT_EQUAL(tskt_client_connect(client, NULL, 0, ADDR4, port), 0);
	CU_ASSERT_TRUE(spin_until_event_count(loop, &ctx, 1, TEST_TIMEOUT_MS));
	res = wait_accept(loop,
			  server,
			  remote_addr,
			  sizeof(remote_addr),
			  &remote_port,
			  &peer);
	CU_ASSERT_EQUAL(res, 0);

	/* success path */
	CU_ASSERT_EQUAL(tpkt_new_with_data("hi", 3, &pkt), 0);
	res = tskt_client_send_pkt(client, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);
	memset(rbuf, 0, sizeof(rbuf));
	res = wait_read(loop, peer, rbuf, sizeof(rbuf));
	CU_ASSERT_EQUAL(res, 3);
	CU_ASSERT_STRING_EQUAL(rbuf, "hi");

	/* force -EAGAIN: shrink the client socket's send buffer, then queue a
	 * packet too large to be written in one go (the peer never reads it
	 * back), leaving a write pending internally; a second send while
	 * that one is still pending must see -EAGAIN immediately (see the
	 * "if (self->tcp.wpkt != NULL) return -EAGAIN;" guard at the top of
	 * socket_impl_write_pkt() in src/tskt_impl.c) */
	sock = tskt_client_get_socket(client);
	CU_ASSERT_PTR_NOT_NULL(sock);
	CU_ASSERT_EQUAL(tskt_socket_set_txbuf_size(sock, 4096), 0);

	big = calloc(1, big_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(big);
	CU_ASSERT_EQUAL(tpkt_new_with_data(big, big_len, &big_pkt), 0);
	res = tskt_client_send_pkt(client, big_pkt);
	CU_ASSERT_EQUAL(res, 0); /* partial write queued internally */

	CU_ASSERT_EQUAL(tpkt_new_with_data("x", 1, &pkt2), 0);
	res = tskt_client_send_pkt(client, pkt2);
	CU_ASSERT_EQUAL(res, -EAGAIN);

	tpkt_unref(pkt2);
	tpkt_unref(big_pkt);
	free(big);

	tskt_socket_destroy(peer);
	tskt_socket_destroy(server);
	tskt_client_destroy(client);
	pomp_loop_destroy(loop);
}


CU_TestInfo g_tskt_test_client[] = {
	{FN("new-guards"), &test_new_guards},
	{FN("connect-guards"), &test_connect_guards},
	{FN("send-and-accessor-guards"), &test_send_and_accessor_guards},
	{FN("stop-destroy-guards"), &test_stop_destroy_guards},
	{FN("connect-and-data-roundtrip"), &test_connect_and_data_roundtrip},
	{FN("disconnect-and-reconnect"), &test_disconnect_and_reconnect},
	{FN("stop"), &test_stop},
	{FN("connect-refused-no-event"), &test_connect_refused_no_event},
	{FN("socket-created-and-upgrade-callbacks"),
	 &test_socket_created_and_upgrade_callbacks},
	{FN("socket-upgrade-reject"), &test_socket_upgrade_reject},
	{FN("client-start-local-bind-conflict"),
	 &test_client_start_local_bind_conflict},
	{FN("send-pkt-success-and-eagain"), &test_send_pkt_success_and_eagain},

	CU_TEST_INFO_NULL,
};
