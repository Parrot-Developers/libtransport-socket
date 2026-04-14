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

#ifdef _WIN32
#	include <winsock2.h>
#	include <ws2tcpip.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>

#include <libpomp.h>
#include <transport-packet/tpkt.h>
#include <transport-socket/tskt.h>
#include <transport-socket/tskt_client.h>

#define ULOG_TAG tskt_test
#include <ulog.h>
ULOG_DECLARE_TAG(tskt_test);


#define ADDR_LOCAL "127.0.0.1"
#define PORT 11111


struct tskt_test {
	struct pomp_loop *loop;
	int thread_should_stop;
	bool use_pomp;

	struct tskt_client *client;
	struct tskt_socket *sock;
	struct pomp_ctx *pomp_ctx;
};


static struct tskt_test *s_self;


static const char short_options[] = "hp";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"pomp", no_argument, NULL, 'p'},
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
	       "  -p | --pomp                        "
	       "Run pomp_ctx at the same time to test consistency\n"
	       "\n",
	       prog_name);
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


static void pomp_event_cb(struct pomp_ctx *ctx,
			  enum pomp_event event,
			  struct pomp_conn *conn,
			  const struct pomp_msg *msg,
			  void *userdata)
{
	switch (event) {
	case POMP_EVENT_CONNECTED:
		ULOGI("%s: CONNECTED", __func__);
		break;
	case POMP_EVENT_DISCONNECTED:
		ULOGI("%s: DISCONNECTED", __func__);
		break;
	case POMP_EVENT_MSG:
		ULOGI("%s: MSG", __func__);
		break;
	default:
		ULOGE("unknown event");
		break;
	}
}


static void pomp_socket_cb(struct pomp_ctx *ctx,
			   int fd,
			   enum pomp_socket_kind kind,
			   void *userdata)
{
	ULOGI("%s: fd=%d", __func__, fd);
}


static void ctx_event_cb(struct tskt_client *client,
			 enum tskt_client_event event,
			 struct tskt_socket *sock,
			 void *userdata)
{
	struct tskt_test *self = (struct tskt_test *)userdata;

	switch (event) {
	case TSKT_CLIENT_EVENT_CONNECTED: {
		ULOGI("%s: CONNECTED", __func__);
		self->sock = sock;

		char *buf = "HELLO WORLD!";
		size_t len = strlen(buf) + 1;
		ssize_t ret = tskt_client_send(self->client, buf, len);
		if (ret >= 0)
			ULOGI("%s: sent '%s'", __func__, buf);
		else if (ret != -EAGAIN)
			ULOG_ERRNO("tskt_client_send", -ret);
	} break;
	case TSKT_CLIENT_EVENT_DISCONNECTED: {
		ULOGI("%s: DISCONNECTED", __func__);
		self->sock = NULL;

	} break;
	case TSKT_CLIENT_EVENT_READY_TO_SEND: {
		ULOGI("%s: READY_TO_SEND", __func__);

	} break;
	default: {
		ULOGE("unknown event: %d", event);

	} break;
	}
}


static void
ctx_data_cb(struct tskt_client *client, struct tpkt_packet *pkt, void *userdata)
{
	int err;

	const void *data = NULL;
	size_t len = 0;

	err = tpkt_get_cdata(pkt, &data, &len, NULL);
	if (err < 0)
		ULOG_ERRNO("tpkt_get_cdata", -err);

	ULOGI("%s: received '%s'", __func__, (const char *)data);
}


static void ctx_socket_created_cb(struct tskt_client *client,
				  struct tskt_socket *sock,
				  int fd,
				  void *userdata)
{
	ULOGI("%s: fd=%d", __func__, fd);
}


static struct tskt_socket *ctx_socket_upgrade_cb(struct tskt_client *client,
						 struct tskt_socket *sock,
						 void *userdata)
{
	ULOGI("%s", __func__);
	return sock;
}


static struct tskt_client_cbs s_cbs = {
	.event_cb = ctx_event_cb,
	.data_cb = ctx_data_cb,
	.socket_created_cb = ctx_socket_created_cb,
	.socket_upgrade_cb = ctx_socket_upgrade_cb,
};


int main(int argc, char **argv)
{
	int res;
	int status = EXIT_SUCCESS;
	const char *addr = ADDR_LOCAL;
	bool use_pomp = false;
	struct sockaddr_in remote_addr_in;
	uint16_t port = PORT;

	s_self = NULL;

	welcome(argv[0]);

	/* Command-line parameters */
	int idx;
	int c;
	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {
		case 0:
			break;
		case 'h':
			usage(argv[0]);
			exit(EXIT_SUCCESS);
			break;
		case 'p':
			use_pomp = true;
			break;
		default:
			usage(argv[0]);
			exit(EXIT_FAILURE);
			break;
		}
	}

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

	s_self->use_pomp = use_pomp;

	/* Create the loop */
	s_self->loop = pomp_loop_new();
	if (s_self->loop == NULL) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}

	if (s_self->use_pomp) {
		s_self->pomp_ctx = pomp_ctx_new_with_loop(
			pomp_event_cb, s_self, s_self->loop);
		if (s_self->pomp_ctx == NULL) {
			res = -EPROTO;
			ULOG_ERRNO("pomp_ctx_new", -res);
			status = EXIT_FAILURE;
			goto out;
		}

		res = pomp_ctx_set_socket_cb(s_self->pomp_ctx, &pomp_socket_cb);
		if (res < 0) {
			ULOG_ERRNO("pomp_ctx_set_socket_cb", -res);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	res = tskt_client_new(s_self->loop, s_cbs, s_self, &s_self->client);
	if (res < 0) {
		ULOG_ERRNO("tskt_client_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	if (s_self->use_pomp) {
		/* Set address */
		res = inet_pton(AF_INET, addr, &remote_addr_in.sin_addr);
		if (res <= 0) {
			res = -errno;
			ULOG_ERRNO("inet_pton", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		remote_addr_in.sin_family = AF_INET;
		remote_addr_in.sin_port = htons(port);

		res = pomp_ctx_connect(s_self->pomp_ctx,
				       (const struct sockaddr *)&remote_addr_in,
				       sizeof(remote_addr_in));
		if (res < 0) {
			ULOG_ERRNO("pomp_ctx_connect", -res);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	res = tskt_client_connect(s_self->client, NULL, 0, addr, port);
	if (res < 0) {
		ULOG_ERRNO("tskt_client_connect", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	while (!s_self->thread_should_stop)
		pomp_loop_wait_and_process(s_self->loop, -1);

out:
	if (s_self != NULL) {
		if (s_self->client != NULL) {
			tskt_client_stop(s_self->client);
			tskt_client_destroy(s_self->client);
		}
		if (s_self->pomp_ctx != NULL) {
			pomp_ctx_stop(s_self->pomp_ctx);
			pomp_ctx_destroy(s_self->pomp_ctx);
		}
		pomp_loop_destroy(s_self->loop);
		free(s_self);
	}

	printf("\n%s\n", (status == EXIT_SUCCESS) ? "Finished!" : "Failed!");
	exit(status);
}
