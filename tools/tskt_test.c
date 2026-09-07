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
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>

#include <libpomp.h>
#include <transport-packet/tpkt.h>
#include <transport-socket/tskt.h>

#define ULOG_TAG tskt_test
#include <ulog.h>
ULOG_DECLARE_TAG(tskt_test);


#define ADDR_LOCAL "127.0.0.1"
#define ADDR6_LOCAL "::1"
#define ADDR_MCAST "239.255.42.1"
#define BUFFER_SIZE 65536


struct tskt_test {
	struct pomp_loop *loop;
	int thread_should_stop;
	struct tskt_socket *sender;
	struct tskt_socket *receiver;
	uint8_t *sender_buf;
	size_t sender_buf_len;
	uint8_t *receiver_buf;
	size_t receiver_buf_len;
	unsigned int count;
	unsigned int max_count;
	bool use_pkt;
	bool use_ipv6;
};


static struct tskt_test *s_self;


static const char short_options[] = "ha:m:pc6";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"addr", required_argument, NULL, 'a'},
	{"mcast", required_argument, NULL, 'm'},
	{"pkt", no_argument, NULL, 'p'},
	{"connect", no_argument, NULL, 'c'},
	{0, 0, 0, 0},
};


static void welcome(char *prog_name)
{
	printf("\n%s - "
	       "Transport socket library test program\n"
	       "Copyright (c) 2020 Parrot Drones SAS\n\n",
	       prog_name);
}


static void usage(char *prog_name)
{
	printf("Usage: %s [options]\n\n"
	       "Options:\n"
	       "  -h | --help                        "
	       "Print this message\n"
	       "  -a | --addr <address>              "
	       "Interface address (IPv4 \"x.y.z.t\", defaults to localhost)\n"
	       "  -m | --mcast <address>             "
	       "Multicast to the provided address (eg. \"239.255.42.1\")\n"
	       "  -p | --pkt                         "
	       "Use packet functions\n"
	       "  -6                                 "
	       "Use IPv6\n"
	       "  -c | --connect                     "
	       "Test connect/disconnect functions\n"
	       "\n",
	       prog_name);
}


static struct tpkt_packet *new_packet(size_t len)
{
	struct pomp_buffer *buf = pomp_buffer_new(len);
	if (!buf)
		return NULL;

	struct tpkt_packet *pkt;
	int res = tpkt_new_from_buffer(buf, &pkt);
	pomp_buffer_unref(buf);
	if (res < 0)
		return NULL;

	return pkt;
}


static void sender_socket_cb(int fd, uint32_t events, void *userdata)
{
	int res;
	struct tskt_test *self = userdata;
	ssize_t readlen = 0;
	uint64_t ts = 0;
	struct tpkt_packet *pkt = NULL;

	if (self->use_pkt) {
		pkt = new_packet(self->sender_buf_len);
		if (!pkt) {
			ULOGE("%s: failed to allocate packet", __func__);
			self->thread_should_stop = 1;
			return;
		}
	}

	if ((events & POMP_FD_EVENT_OUT) != 0)
		ULOGI("%s: ready to send", __func__);

	if ((events & POMP_FD_EVENT_IN) != 0) {
		do {
			char *buf;
			/* Read data */
			if (self->use_pkt) {
				res = tskt_socket_read_pkt(self->sender, pkt);
				if (res == 0)
					res = tpkt_get_data(pkt,
							    (void **)&buf,
							    (size_t *)&readlen,
							    NULL);
				if (res < 0)
					readlen = res;
				ts = tpkt_get_timestamp(pkt);
			} else {
				readlen = tskt_socket_read(self->sender,
							   self->sender_buf,
							   self->sender_buf_len,
							   &ts);
				buf = (char *)self->sender_buf;
			}
			if (readlen > 0) {
				size_t len;
				ULOGI("%s: %zi bytes received: %s (TS=%" PRIu64
				      ")",
				      __func__,
				      readlen,
				      buf,
				      ts);
				if (self->count < self->max_count) {
					len = snprintf(buf,
						       self->sender_buf_len,
						       "PING %d",
						       ++self->count);
					if (self->use_pkt) {
						tpkt_set_len(pkt, len + 1);
						res = tskt_socket_write_pkt(
							self->sender, pkt);
					} else {
						res = tskt_socket_write(
							self->sender,
							buf,
							len + 1);
					}
					if (res < 0)
						ULOG_ERRNO(
							"tskt_socket_write:"
							"receiver",
							-res);
				} else {
					self->thread_should_stop = 1;
				}
			}
		} while (readlen > 0);
	}

	tpkt_unref(pkt);
}


static void receiver_socket_cb(int fd, uint32_t events, void *userdata)
{
	int res;
	struct tskt_test *self = userdata;
	ssize_t readlen = 0;
	uint64_t ts = 0;
	struct tpkt_packet *pkt = NULL;

	if (self->use_pkt) {
		pkt = new_packet(self->receiver_buf_len);
		if (!pkt) {
			ULOGE("%s: failed to allocate packet", __func__);
			self->thread_should_stop = 1;
			return;
		}
	}

	if ((events & POMP_FD_EVENT_OUT) != 0)
		ULOGI("%s: ready to send", __func__);

	if ((events & POMP_FD_EVENT_IN) != 0) {
		do {
			char *buf;
			/* Read data */
			if (self->use_pkt) {
				res = tskt_socket_read_pkt(self->receiver, pkt);
				if (res == 0)
					res = tpkt_get_data(pkt,
							    (void **)&buf,
							    (size_t *)&readlen,
							    NULL);
				if (res < 0)
					readlen = res;
				ts = tpkt_get_timestamp(pkt);
			} else {
				readlen =
					tskt_socket_read(self->receiver,
							 self->receiver_buf,
							 self->receiver_buf_len,
							 &ts);
				buf = (char *)self->receiver_buf;
			}
			if (readlen > 0) {
				size_t len;
				ULOGI("%s: %zi bytes received: %s (TS=%" PRIu64
				      ")",
				      __func__,
				      readlen,
				      buf,
				      ts);
				len = snprintf(buf,
					       self->receiver_buf_len,
					       "PONG %d",
					       self->count);
				if (self->use_pkt) {
					tpkt_set_len(pkt, len + 1);
					res = tskt_socket_write_pkt(
						self->receiver, pkt);
				} else {
					res = tskt_socket_write(
						self->receiver, buf, len + 1);
				}
				if (res < 0)
					ULOG_ERRNO("tskt_socket_write:receiver",
						   -res);
			}
		} while (readlen > 0);
	}

	tpkt_unref(pkt);
}


static void send_idle(void *userdata)
{
	int res;
	struct tskt_test *self = userdata;
	char buf[10];
	size_t len;

	len = snprintf(buf, sizeof(buf), "PING %d", ++self->count);

	res = tskt_socket_write(self->sender, buf, len + 1);
	if (res < 0)
		ULOG_ERRNO("tskt_socket_write:sender", -res);
}


/* Win32 stubs */
#ifdef _WIN32
static inline const char *strsignal(int signum)
{
	return "??";
}
#endif /* _WIN32 */


static void sig_handler(int signum)
{
	ULOGI("signal %d(%s) received", signum, strsignal(signum));

	if (s_self == NULL)
		return;

	s_self->thread_should_stop = 1;
	if (s_self->loop != NULL)
		pomp_loop_wakeup(s_self->loop);
}


static void test_connect(bool use_ipv6)
{
	int res;
	struct tskt_socket *sock = NULL;
	uint16_t port = 0;
	char addr[INET6_ADDRSTRLEN];
	const char *caddr = use_ipv6 ? ADDR6_LOCAL : ADDR_LOCAL;

	if (use_ipv6)
		res = tskt_socket_new_udp6(
			caddr, &port, NULL, 0, s_self->loop, NULL, NULL, &sock);
	else
		res = tskt_socket_new(caddr,
				      &port,
				      NULL,
				      0,
				      NULL,
				      s_self->loop,
				      NULL,
				      NULL,
				      &sock);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_new", -res);
		goto done;
	}
	ULOGI("socket created, port=%u", port);

	port = 9999;
	res = tskt_socket_connect(sock, NULL, 0, caddr, port);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_connect", -res);
		goto done;
	}
	ULOGI("socket connected to %s,%u", caddr, port);

	/* disconnect */
	ULOGI("disconnect socket:");
	res = tskt_socket_connect(sock, NULL, 0, NULL, 0);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_disconnect", -res);
		goto done;
	}

	res = tskt_socket_get_local_addr(sock, addr, sizeof(addr));
	if (res < 0) {
		ULOG_ERRNO("tskt_get_local_addr", -res);
		goto done;
	}
	ULOGI("local address: %s:%u", addr, tskt_socket_get_local_port(sock));
	res = tskt_socket_get_remote_addr(sock, addr, sizeof(addr));
	if (res < 0) {
		ULOG_ERRNO("tskt_get_remote_addr", -res);
		goto done;
	}
	ULOGI("remote address: %s:%u", addr, tskt_socket_get_remote_port(sock));

done:
	if (sock != NULL)
		tskt_socket_destroy(sock);
}


int main(int argc, char **argv)
{
	int res, status = EXIT_SUCCESS;
	char *addr = NULL, *maddr = NULL;
	int mcast = 0;
	uint16_t receiver_port = 0;
	s_self = NULL;
	bool use_pkt = false;
	bool use_ipv6 = false;
	bool tstconn = false;

	welcome(argv[0]);

	/* Command-line parameters */
	int idx, c;
	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {
		case 0:
			break;

		case 'h':
			usage(argv[0]);
			exit(EXIT_SUCCESS);
			break;

		case 'a':
			addr = optarg;
			break;

		case 'm':
			mcast = 1;
			maddr = optarg;
			break;

		case 'p':
			use_pkt = true;
			break;

		case '6':
			use_ipv6 = true;
			break;

		case 'c':
			tstconn = true;
			break;

		default:
			usage(argv[0]);
			exit(EXIT_FAILURE);
			break;
		}
	}

	if (addr == NULL)
		addr = use_ipv6 ? ADDR6_LOCAL : ADDR_LOCAL;
	if (maddr == NULL)
		maddr = ADDR_MCAST;

	/* Setup signal handlers */
	signal(SIGINT, &sig_handler);
	signal(SIGTERM, &sig_handler);
#ifndef _WIN32
	signal(SIGPIPE, SIG_IGN);
#endif

	/* Create the context */
	s_self = calloc(1, sizeof(*s_self));
	if (s_self == NULL) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	s_self->max_count = 10;

	s_self->sender_buf = malloc(BUFFER_SIZE);
	if (s_self->sender_buf == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	s_self->sender_buf_len = BUFFER_SIZE;

	s_self->receiver_buf = malloc(BUFFER_SIZE);
	if (s_self->receiver_buf == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	s_self->receiver_buf_len = BUFFER_SIZE;
	s_self->use_pkt = use_pkt;
	s_self->use_ipv6 = use_ipv6;

	/* Create the loop */
	s_self->loop = pomp_loop_new();
	if (s_self->loop == NULL) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}

	if (s_self->use_pkt)
		printf("Use packet interface\n\n");

	if (s_self->use_ipv6)
		printf("Use IPv6\n\n");

	if (tstconn) {
		test_connect(use_ipv6);
		goto out;
	}

	/* Receiver */
	if (s_self->use_ipv6)
		res = tskt_socket_new_udp6(addr,
					   &receiver_port,
					   NULL,
					   0,
					   s_self->loop,
					   &receiver_socket_cb,
					   s_self,
					   &s_self->receiver);
	else
		res = tskt_socket_new(addr,
				      &receiver_port,
				      NULL,
				      0,
				      mcast ? maddr : NULL,
				      s_self->loop,
				      &receiver_socket_cb,
				      s_self,
				      &s_self->receiver);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_new:receiver", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Sender */
	if (s_self->use_ipv6)
		res = tskt_socket_new_udp6(NULL,
					   NULL,
					   addr,
					   receiver_port,
					   s_self->loop,
					   &sender_socket_cb,
					   s_self,
					   &s_self->sender);
	else
		res = tskt_socket_new(mcast ? addr : NULL,
				      NULL,
				      mcast ? maddr : addr,
				      receiver_port,
				      NULL,
				      s_self->loop,
				      &sender_socket_cb,
				      s_self,
				      &s_self->sender);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_new:sender", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	res = pomp_loop_idle_add(s_self->loop, &send_idle, s_self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_idle_add", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	while (!s_self->thread_should_stop)
		pomp_loop_wait_and_process(s_self->loop, -1);

out:
	if (s_self != NULL) {
		res = tskt_socket_destroy(s_self->sender);
		if (res < 0)
			ULOG_ERRNO("tskt_socket_destroy:sender", -res);
		res = tskt_socket_destroy(s_self->receiver);
		if (res < 0)
			ULOG_ERRNO("tskt_socket_destroy:receiver", -res);
		if (s_self->loop != NULL) {
			res = pomp_loop_destroy(s_self->loop);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_destroy", -res);
		}
		free(s_self->sender_buf);
		free(s_self->receiver_buf);
		free(s_self);
	}

	printf("\n%s\n", (status == EXIT_SUCCESS) ? "Finished!" : "Failed!");
	exit(status);
}
