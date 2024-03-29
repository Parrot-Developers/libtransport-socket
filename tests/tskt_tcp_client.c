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
#include <fcntl.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <futils/futils.h>
#include <libpomp.h>
#include <openssl/evp.h>
#include <transport-packet/tpkt.h>
#include <transport-socket/tskt.h>

#define SERVER_TCP_ADDR "127.0.0.1"
#define SERVER_TCP_ADDR6 "::1"
#define SERVER_TCP_PORT 11111

#define BYTES_TO_SEND 50000000

struct client {
	struct tskt_socket *sock;
	size_t to_send;
	size_t total;
	EVP_MD_CTX *evp_out;
	EVP_MD_CTX *evp_in;
};

static bool app_stop;

/* tcp client i/o events handler */
static void
tcp_client_cb(struct tskt_socket *sock, uint32_t revents, void *userdata)
{
	struct client *client = (struct client *)userdata;
	static char buf[2048];

	if (revents & POMP_FD_EVENT_ERR) {
		/* print socket error */
		printf("socket error: %s",
		       strerror(tskt_socket_get_error(client->sock)));
		/* close socket */
		goto close_socket;
	}

	if (revents & POMP_FD_EVENT_OUT) {
		/* write remaining data */
		size_t written_bytes = 0;
		while (client->to_send) {
			size_t len = MIN(client->to_send, sizeof(buf));
			futils_random_bytes(buf, len);
			ssize_t n = tskt_socket_write(client->sock, buf, len);
			if (n < 0 && n != -EAGAIN) {
				printf("socket_write: %s\n", strerror(-n));
				goto close_socket;
			}
			if (n <= 0)
				break;
			EVP_DigestUpdate(client->evp_out, buf, n);
			client->to_send -= n;
			written_bytes += n;
		}
		printf("written %u bytes\n", (unsigned)written_bytes);
		if (!client->to_send) {
			/* no more data to send */
			tskt_socket_update_events(
				client->sock, 0, POMP_FD_EVENT_OUT);
		}
	}

	if (revents & POMP_FD_EVENT_IN) {
		size_t read_bytes = 0;
		ssize_t n;
		while (1) {
			n = tskt_socket_read(
				client->sock, buf, sizeof(buf), NULL);
			if (n < 0 && n != -EAGAIN) {
				printf("tskt_sock_read: %s\n", strerror(-n));
				goto close_socket;
			}
			if (n <= 0)
				break;
			EVP_DigestUpdate(client->evp_in, buf, n);
			client->total += n;
			read_bytes += n;
		}
		printf("read %u bytes\n", (unsigned)read_bytes);
		if (n == 0 || client->total >= BYTES_TO_SEND)
			goto close_socket;
	}

	return;

close_socket:
	/* mask all events and terminate application */
	tskt_socket_update_events(client->sock, 0, ~0);
	app_stop = true;
}

/* main function */
int main(int argc, char **argv)
{
	int res;
	struct client client;
	bool is_v6;

	is_v6 = argc > 1 && strcmp(argv[1], "-6") == 0;

	printf("start TCP test client%s\n", is_v6 ? " (IPv6)" : "");

	/* pomp loop */
	struct pomp_loop *loop = pomp_loop_new();

	/* create ctp socket */
	if (is_v6)
		res = tskt_socket_new_tcp6(loop, &client.sock);
	else
		res = tskt_socket_new_tcp(loop, &client.sock);
	if (res < 0) {
		printf("tskt_socket_new_tcp: %s\n", strerror(-res));
		return 1;
	}
	client.to_send = BYTES_TO_SEND;
	client.total = 0;
	client.evp_out = EVP_MD_CTX_new();
	client.evp_in = EVP_MD_CTX_new();
	EVP_DigestInit(client.evp_out, EVP_md5());
	EVP_DigestInit(client.evp_in, EVP_md5());

	/* connect to server */
	res = tskt_socket_connect(client.sock,
				  NULL,
				  0,
				  is_v6 ? SERVER_TCP_ADDR6 : SERVER_TCP_ADDR,
				  SERVER_TCP_PORT);
	if (res < 0) {
		printf("tskt_socket_connect: %s\n", strerror(-res));
		return 1;
	}

	/* monitor i/o events */
	res = tskt_socket_set_event_cb(client.sock,
				       POMP_FD_EVENT_IN | POMP_FD_EVENT_OUT,
				       tcp_client_cb,
				       &client);
	if (res < 0) {
		printf("tskt_socket_set_event_cb: %s\n", strerror(-res));
		return 1;
	}

	/* main loop */
	while (!app_stop)
		pomp_loop_wait_and_process(loop, -1);

	/* check data */
	unsigned char digest_out[EVP_MAX_MD_SIZE];
	unsigned int md_out_len;
	unsigned char digest_in[EVP_MAX_MD_SIZE];
	unsigned int md_in_len;
	EVP_DigestFinal(client.evp_in, digest_in, &md_in_len);
	EVP_DigestFinal(client.evp_out, digest_out, &md_out_len);
	printf("sent: %zu bytes, received: %zu bytes\n",
	       BYTES_TO_SEND - client.to_send,
	       client.total);
	if (client.to_send != 0 || client.total != BYTES_TO_SEND)
		printf("FAILURE! invalid length!\n");
	else if (md_in_len != md_out_len ||
		 memcmp(digest_in, digest_out, md_in_len) != 0)
		printf("FAILURE! invalid MD5 signature!\n");
	else
		printf("SUCCESS!\n");

	/* release resources */
	tskt_socket_destroy(client.sock);
	pomp_loop_destroy(loop);
	EVP_MD_CTX_free(client.evp_in);
	EVP_MD_CTX_free(client.evp_out);

	return 0;
}
