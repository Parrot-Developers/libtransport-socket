/**
 * Copyright (c) 2020 Parrot Drones SAS
 *
 * DNS resolver test
 */

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libpomp.h>
#include <transport-socket/tskt_resolv.h>


static const struct {
	const char *hostname;
	bool ipv6;
	bool cancel;
} requests[] = {
	{"10.1.0.1", false, false},
	{"10.1.0.2", false, true},
	{"www.google.com", false, false},
	{"truc.pouet", false, false},
	{"www.google.fr", false, true},
	{"www.parrot.com", false, false},
	{"www.google.fr", false, false},
	{"2a00:1450:4007:81a::2001", true, false},
	{"www.google.com", true, false},
	{"truc.pouet", true, false},
	{"www.google.fr", true, false},
	{"www.parrot.com", true, false},
	{"drones.starfish.parrot.com", true, false},
};


/* codecheck_ignore[VOLATILE] */
static volatile sig_atomic_t app_stop;
static struct pomp_loop *loop;
static struct tskt_resolv *rslv;


static void sig_handler(int signum)
{
	printf("Stop!\n");

	app_stop = 1;
	if (loop != NULL)
		pomp_loop_wakeup(loop);
}


static void rslv_cb(struct tskt_resolv *self,
		    int id,
		    enum tskt_resolv_error result,
		    int naddrs,
		    const char *const *addrs,
		    void *userdata)
{
	printf("result: id=%d, hostname=\"%s\", error=%d\n",
	       id,
	       (char *)userdata,
	       (int)result);
	for (int i = 0; i < naddrs; i++)
		printf("  -> %s\n", addrs[i]);
	printf("\n");
}


static void tmr_cb(struct pomp_timer *tmr, void *userdata)
{
	app_stop = true;
	printf("Done\n");
}


/* main function */
int main(int argc, char **argv)
{
	int ret, id;
	struct pomp_timer *tmr;

	printf("DNS resolver test\n");

	ret = tskt_resolv_new(&rslv);
	if (ret < 0) {
		fprintf(stderr, "tskt_resolv_new: %s\n", strerror(-ret));
		exit(1);
	}

	/* pomp loop */
	loop = pomp_loop_new();

	/* Start requests */
	for (unsigned i = 0; i < sizeof(requests) / sizeof(requests[0]); i++) {
		if (requests[i].ipv6)
			ret = tskt_resolv_getaddrinfo6(
				rslv,
				requests[i].hostname,
				loop,
				rslv_cb,
				(void *)requests[i].hostname,
				&id);
		else
			ret = tskt_resolv_getaddrinfo(
				rslv,
				requests[i].hostname,
				loop,
				rslv_cb,
				(void *)requests[i].hostname,
				&id);
		printf("request: hostname=\"%s\"%s, ret=%d, id=%d\n",
		       requests[i].hostname,
		       requests[i].ipv6 ? " (IPv6)" : "",
		       ret,
		       id);
		if (ret >= 0 && requests[i].cancel) {
			printf("  -> cancelled!\n");
			ret = tskt_resolv_cancel(rslv, id);
			if (ret < 0)
				printf("cancel failed: %s\n", strerror(-ret));
		}
	}
	printf("\n");

	/* termination timer */
	tmr = pomp_timer_new(loop, tmr_cb, NULL);
	if (tmr == NULL) {
		fprintf(stderr, "failed to create timer\n");
		exit(1);
	}
	pomp_timer_set(tmr, 60 * 1000);

	/* Setup signal handlers */
	signal(SIGINT, &sig_handler);
	signal(SIGTERM, &sig_handler);
#ifndef _WIN32
	signal(SIGPIPE, SIG_IGN);
#endif

	/* main loop */
	while (!app_stop)
		pomp_loop_wait_and_process(loop, -1);

	pomp_timer_destroy(tmr);

	pomp_loop_destroy(loop);

	tskt_resolv_unref(rslv);

	return 0;
}
