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
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef _WIN32
#	include <winsock2.h>
#	include <ws2tcpip.h>
#	include <ws2ipdef.h>
#	undef OPAQUE
#	define SOCKERR_ENOTCONN WSAENOTCONN
/* Use same values as Linux/Posix */
#	undef INET_ADDRSTRLEN
#	define INET_ADDRSTRLEN 16
#	undef INET6_ADDRSTRLEN
#	define INET6_ADDRSTRLEN 46
#else /* !_WIN32 */
#	include <arpa/inet.h>
#	include <netinet/in.h>
#	include <netinet/tcp.h>
#	include <sys/mman.h>
#	include <sys/socket.h>
#	define SOCKERR_ENOTCONN ENOTCONN
#endif /* !_WIN32 */

#ifdef __linux__
#	include <linux/net_tstamp.h>
#endif /* __linux__ */

#define ULOG_TAG tskt_impl
#include <ulog.h>
ULOG_DECLARE_TAG(tskt_impl);

#include <futils/futils.h>
#include <transport-socket/tskt_ops.h>


#define ADDR_ANY_STR "0.0.0.0"
#define ADDR6_ANY_STR "::"


#define LISTEN_BACKLOG 8


/* Define the maximum number of messages that can be passed
 * to sendmmsg/recvmmsg in a single call */
#define MMSG_MAX 64


#ifdef _WIN32

#	define IOV_MAX 8

static inline int win32_socket(int domain, int type, int protocol)
{
	SOCKET s;
	s = socket(domain, type, protocol);
	if (s == INVALID_SOCKET)
		return -1;
	if (s > (SOCKET)0x7FFFFFFF) {
		/* Winsock2's socket() returns the unsigned type SOCKET,
		 * which is a 32-bit type for WIN32 and a 64-bit type for WIN64;
		 * as we cast the result to an int, return an error if the
		 * returned value does not fit into 31 bits. */
		ULOGE("%s: avoiding truncated socket handle", __func__);
		closesocket(s);
		return -1;
	}
	return (int)s;
}

static inline int win32_getsockopt(int sockfd,
				   int level,
				   int optname,
				   void *optval,
				   socklen_t *optlen)
{
	return getsockopt(
		(SOCKET)sockfd, level, optname, (char *)optval, optlen);
}

static inline int win32_setsockopt(int sockfd,
				   int level,
				   int optname,
				   const void *optval,
				   socklen_t optlen)
{
	return setsockopt(
		(SOCKET)sockfd, level, optname, (const char *)optval, optlen);
}

#	undef errno
#	undef socket
#	undef getsockopt
#	undef setsockopt

#	define errno ((int)WSAGetLastError())

#	define socket(_domain, _type, _protocol)                              \
		win32_socket((_domain), (_type), (_protocol))

#	define getsockopt(_sockfd, _level, _optname, _optval, _optlen)        \
		win32_getsockopt(                                              \
			(_sockfd), (_level), (_optname), (_optval), (_optlen))

#	define setsockopt(_sockfd, _level, _optname, _optval, _optlen)        \
		win32_setsockopt(                                              \
			(_sockfd), (_level), (_optname), (_optval), (_optlen))

#endif /* _WIN32 */


union socket_addr {
	struct sockaddr_in in;
	struct sockaddr_in6 in6;
};

struct socket_impl {
	struct tskt_socket sock;
	struct pomp_loop *loop;
	int fd;
	bool is_tcp;
	bool is_v6;
	tskt_socket_event_cb_t cb;
	void *userdata;
	union {
		struct {
			union socket_addr remote_addr;
			struct ip_mreq mcast_mreq;
			struct timespec monotonic_to_real_time_diff;
			bool ts_failed;
			bool is_connected;
		} udp;
		struct {
			struct tpkt_packet *wpkt;
			size_t wlen;
			int werror;
			bool wnotify;
			uint32_t revents;
		} tcp;
	};
#ifdef __linux__
	void *mmsg;
#endif
	bool netdown_logged;
};


static void get_monotonic_to_real_time_diff(struct socket_impl *self)
{
	int res;
	struct timespec ts1 = {0, 0}, ts2 = {0, 0}, ts3 = {0, 0}, ts4 = {0, 0};

	/* Monotonic timestamp 1 */
	res = time_get_monotonic(&ts1);
	if (res < 0) {
		ULOG_ERRNO("time_get_monotonic", errno);
		return;
	}

	/* Realtime timestamp */
	res = clock_gettime(CLOCK_REALTIME, &ts2);
	if (res < 0) {
		ULOG_ERRNO("clock_gettime", errno);
		return;
	}

	/* Monotonic timestamp 2 */
	res = time_get_monotonic(&ts3);
	if (res < 0) {
		ULOG_ERRNO("time_get_monotonic", errno);
		return;
	}

	/* Average of monotonic timestamps: ts4 = (ts1 + ts3) / 2 */
	ts4.tv_nsec = (ts1.tv_nsec + ts3.tv_nsec) / 2;
	ts4.tv_sec = ts1.tv_sec + ts3.tv_sec;
	if (ts4.tv_sec & 1)
		ts4.tv_nsec += 500000000;
	ts4.tv_sec /= 2;
	while (ts4.tv_nsec >= 1000000000) {
		ts4.tv_sec++;
		ts4.tv_nsec -= 1000000000;
	}

	/* Time difference: diff = ts2 - ts4 */
	res = time_timespec_diff(
		&ts4, &ts2, &self->udp.monotonic_to_real_time_diff);
	if (res < 0) {
		res = time_timespec_diff(
			&ts1, &ts2, &self->udp.monotonic_to_real_time_diff);
		if (res < 0) {
			ULOG_ERRNO("time_timespec_diff", -res);
			return;
		}
	}
}


#ifdef __linux__

union cmsg_buffer {
	uint8_t buf[CMSG_SPACE(sizeof(struct timespec) * 3)];
	struct cmsghdr cmsghdr[0];
	struct timespec ts[0];
};

static void get_timestamp_cmsg(struct socket_impl *self,
			       const struct timespec *ts_cur,
			       struct msghdr *msg,
			       uint64_t *ts_us)
{
	struct cmsghdr *cmsg;
	struct timespec *ts = NULL;
	struct timespec ts_mono = {0, 0};

	/* Get the kernel's software timestamp */
	for (cmsg = CMSG_FIRSTHDR(msg); cmsg != NULL;
	     cmsg = CMSG_NXTHDR(msg, cmsg)) {
		if ((cmsg->cmsg_level == SOL_SOCKET) &&
		    (cmsg->cmsg_type == SO_TIMESTAMPING)) {
			ts = (struct timespec *)CMSG_DATA(cmsg);
			break;
		}
	}

	/* Get the packet's monotonic timestamp */
	if (ts != NULL) {
		int res;
		int retried = 0;
		/* clang-format off */
retry:
		/* clang-format on */
		res = time_timespec_diff(
			&self->udp.monotonic_to_real_time_diff, ts, &ts_mono);
		if (res < 0) {
			if (!self->udp.ts_failed)
				ULOG_ERRNO("time_timespec_diff", -res);

			if (!retried) {
				/* Retry after reseting the monotonic to
				 * realtime diff, in case the system time
				 * has been updated backward
				 * (monotonic_to_real_time_diff > ts) */
				get_monotonic_to_real_time_diff(self);
				retried = 1;
				goto retry;
			}

			time_timespec_to_us(ts_cur, ts_us);
			goto ts_failed;
		}

		uint64_t diff = 0;

		res = time_timespec_diff_in_range(
			&ts_mono, ts_cur, 2000000, &diff);
		if (res == 1) {
			time_timespec_to_us(&ts_mono, ts_us);
			if (self->udp.ts_failed) {
				ULOGI("packet timestamping succeeded");
				self->udp.ts_failed = false;
			}
		} else if (!retried) {
			/* Retry after reseting the monotonic to
			 * realtime diff, in case the system time
			 * has been updated */
			get_monotonic_to_real_time_diff(self);
			retried = 1;
			goto retry;
		} else {
			time_timespec_to_us(ts_cur, ts_us);
			goto ts_failed;
		}
	} else {
		time_timespec_to_us(ts_cur, ts_us);
		goto ts_failed;
	}

	return;

ts_failed:
	if (!self->udp.ts_failed) {
		ULOGW("packet timestamping failed, fall back to current ts");
		self->udp.ts_failed = true;
	}
}


static int alloc_mmsg(struct socket_impl *self,
		      struct mmsghdr **ret_mmsg,
		      struct tpkt_packet ***ret_pkts,
		      union cmsg_buffer **ret_ctrl)
{
	if (self->mmsg == NULL) {
		/* Allocate buffer */
		self->mmsg = calloc(1,
				    MMSG_MAX * (sizeof(struct mmsghdr) +
						sizeof(struct tpkt_packet *) +
						sizeof(union cmsg_buffer)));
		if (self->mmsg == NULL)
			return -ENOMEM;
	}

	struct mmsghdr *mmsg = (struct mmsghdr *)self->mmsg;
	struct tpkt_packet **pkts = (struct tpkt_packet **)(mmsg + MMSG_MAX);
	union cmsg_buffer *ctrl = (union cmsg_buffer *)(pkts + MMSG_MAX);

	*ret_mmsg = mmsg;
	if (ret_pkts)
		*ret_pkts = pkts;
	if (ret_ctrl)
		*ret_ctrl = ctrl;

	return 0;
}

#endif /*__linux__ */


static int set_addr(struct sockaddr_in *saddr, const char *addr, uint16_t port)
{
	int res;

	memset(saddr, 0, sizeof(*saddr));
	saddr->sin_family = AF_INET;
	saddr->sin_port = htons(port);

	if (!addr || addr[0] == 0)
		return 0; /* any address 0.0.0.0 */

	res = inet_pton(AF_INET, addr, &saddr->sin_addr);
	if (res <= 0) {
		res = -errno;
		ULOG_ERRNO("inet_pton", -res);
		return res;
	}
	return 0;
}


static int
set_addr6(struct sockaddr_in6 *saddr, const char *addr, uint16_t port)
{
	int res;

	memset(saddr, 0, sizeof(*saddr));
	saddr->sin6_family = AF_INET6;
	saddr->sin6_port = htons(port);

	if (!addr || addr[0] == 0)
		return 0; /* any address :: */

	res = inet_pton(AF_INET6, addr, &saddr->sin6_addr);
	if (res <= 0) {
		res = -errno;
		ULOG_ERRNO("inet_pton", -res);
		return res;
	}
	return 0;
}


static void socket_impl_fd_cb(int fd, uint32_t revents, void *userdata)
{
	struct socket_impl *self = (struct socket_impl *)userdata;

	if (!self || !self->cb)
		return;

	self->cb((struct tskt_socket *)self, revents, self->userdata);
}


static int socket_impl_send_pending_tcp(struct socket_impl *self)
{
	const uint8_t *buf;
	size_t len;
	int res;

	if (self->tcp.wpkt == NULL)
		return 0;

	res = tpkt_get_cdata(self->tcp.wpkt, (const void **)&buf, &len, NULL);
	if (res < 0) {
		/* something wrong happened... */
		ULOG_ERRNO("tpkt_get_cdata(fd=%d)", -res, self->fd);
		tpkt_unref(self->tcp.wpkt);
		self->tcp.wpkt = NULL;
		/* clear OUT event so we won't loop forever in the handler */
		int res2 = pomp_loop_update2(
			self->loop, self->fd, 0, POMP_FD_EVENT_OUT);
		if (res2 < 0)
			ULOG_ERRNO("pomp_loop_update2(fd=%d)", -res2, self->fd);
		return res;
	} else {
		if (self->tcp.wlen >= len) {
			/* something wrong happened... */
			ULOGE("bad buffer(fd=%d)", self->fd);
			res = -EINVAL;
		} else {
			/* write remaining data in packet */
			buf += self->tcp.wlen;
			len -= self->tcp.wlen;
#ifdef _WIN32
			res = send((SOCKET)self->fd, (const char *)buf, len, 0);
			if (res == SOCKET_ERROR) {
				res = -errno;
				if (res == -WSAEWOULDBLOCK)
					res = -EAGAIN;
			}
#else
			do {
				res = (int)send(self->fd, buf, len, 0);
			} while (res < 0 && errno == EINTR);
			if (res < 0)
				res = -errno;
#endif
			if (res < 0) {
				if (res != -EAGAIN)
					ULOG_ERRNO(
						"send(fd=%d)", -res, self->fd);
				else
					res = 0;
			} else if (res > 0) {
				self->tcp.wlen += res;
				len -= res;
				ULOGD("async partial write on fd=%d (%u/%u)",
				      self->fd,
				      (unsigned)res,
				      (unsigned)len);
			}
		}
	}
	if (res < 0 || len == 0) {
		/* release packet in case of error or
		 * if all data has been sent */
		tpkt_unref(self->tcp.wpkt);
		self->tcp.wpkt = NULL;
		/* remove write notifications */
		if (res >= 0 && !self->tcp.wnotify) {
			res = pomp_loop_update2(
				self->loop, self->fd, 0, POMP_FD_EVENT_OUT);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_update2(fd=%d)",
					   -res,
					   self->fd);
		}
	}
	return res;
}


static void socket_impl_fd_cb_tcp(int fd, uint32_t revents, void *userdata)
{
	struct socket_impl *self = (struct socket_impl *)userdata;

	if (!self)
		return;

	if (revents & POMP_FD_EVENT_OUT) {
		/* write pending data if any */
		int res;
		res = socket_impl_send_pending_tcp(self);
		if (res < 0) {
			/* report write error to application */
			self->tcp.werror = res;
			revents |= POMP_FD_EVENT_ERR;
		}
		if (self->tcp.wpkt != NULL || !self->tcp.wnotify) {
			/* do not notify application if
			 * data to write is still pending or
			 * if application didn't set notify flag */
			revents &= ~POMP_FD_EVENT_OUT;
			if (revents == 0)
				return;
		}
	}

	if (self->cb == NULL)
		return;

	self->cb((struct tskt_socket *)self, revents, self->userdata);
}


static int socket_impl_destroy(struct tskt_socket *sock);
static struct pomp_loop *socket_impl_get_loop(struct tskt_socket *sock);
static int socket_impl_get_fd(struct tskt_socket *sock);
static int socket_impl_set_fd_cb(struct tskt_socket *sock,
				 pomp_fd_event_cb_t fd_cb,
				 void *userdata);
static int socket_impl_get_local_addr(struct tskt_socket *sock,
				      char *str,
				      size_t len,
				      uint16_t *port);
static int socket_impl_get_remote_addr(struct tskt_socket *sock,
				       char *str,
				       size_t len,
				       uint16_t *port);
static int socket_impl_set_remote_addr(struct tskt_socket *sock,
				       const char *addr,
				       uint16_t port);
static int socket_impl_get_option(struct tskt_socket *sock,
				  enum tskt_option option);
static int socket_impl_set_option(struct tskt_socket *sock,
				  enum tskt_option option,
				  int value);
static ssize_t socket_impl_read(struct tskt_socket *sock,
				void *buf,
				size_t cap,
				uint64_t *ts_us);
static ssize_t
socket_impl_write(struct tskt_socket *sock, const void *buf, size_t len);
#ifndef _WIN32
static ssize_t socket_impl_readv(struct tskt_socket *sock,
				 const struct iovec *iov,
				 size_t iov_len,
				 uint64_t *ts_us);
#endif
static ssize_t socket_impl_writev(struct tskt_socket *sock,
				  const struct iovec *iov,
				  size_t iov_len);
#ifdef __linux__
static ssize_t socket_impl_write_cs(struct tskt_socket *sock,
				    const void *buf,
				    size_t len,
				    int cs);
static ssize_t socket_impl_writev_cs(struct tskt_socket *sock,
				     const struct iovec *iov,
				     size_t iov_len,
				     int cs);
static ssize_t socket_impl_writemv(struct tskt_socket *sock,
				   struct tskt_miovec *miov,
				   size_t mlen);
#endif
static int socket_impl_read_pkt(struct tskt_socket *sock,
				struct tpkt_packet *pkt);
static int socket_impl_write_pkt(struct tskt_socket *sock,
				 struct tpkt_packet *pkt);
#ifdef __linux__
static ssize_t socket_impl_read_pkt_list(struct tskt_socket *sock,
					 struct tpkt_list *list,
					 size_t max_pkts);
#endif
static int socket_impl_set_event_cb(struct tskt_socket *sock,
				    uint32_t events,
				    tskt_socket_event_cb_t cb,
				    void *userdata);
static int socket_impl_update_events(struct tskt_socket *sock,
				     uint32_t events_to_add,
				     uint32_t events_to_remove);
static int socket_impl_connect(struct tskt_socket *sock,
			       const char *local_addr,
			       uint16_t local_port,
			       const char *remote_addr,
			       uint16_t remote_port);
static int socket_impl_listen(struct tskt_socket *sock,
			      const char *local_addr,
			      uint16_t local_port);
static int socket_impl_accept(struct tskt_socket *sock,
			      char *remote_addr,
			      size_t remote_len,
			      uint16_t *remote_port,
			      struct tskt_socket **ret_obj);


static const struct tskt_socket_ops socket_impl_ops = {
	.destroy = socket_impl_destroy,
	.get_loop = socket_impl_get_loop,
	.get_fd = socket_impl_get_fd,
	.set_fd_cb = socket_impl_set_fd_cb,
	.get_local_addr = socket_impl_get_local_addr,
	.get_remote_addr = socket_impl_get_remote_addr,
	.set_remote_addr = socket_impl_set_remote_addr,
	.get_option = socket_impl_get_option,
	.set_option = socket_impl_set_option,
	.read = socket_impl_read,
	.write = socket_impl_write,
#ifndef _WIN32
	.readv = socket_impl_readv,
#endif
	.writev = socket_impl_writev,
#ifdef __linux__
	.write_cs = socket_impl_write_cs,
	.writev_cs = socket_impl_writev_cs,
	.writemv = socket_impl_writemv,
#endif
	.read_pkt = socket_impl_read_pkt,
	.write_pkt = socket_impl_write_pkt,
#ifdef __linux__
	.read_pkt_list = socket_impl_read_pkt_list,
#endif
	.set_event_cb = socket_impl_set_event_cb,
	.update_events = socket_impl_update_events,
	.connect = socket_impl_connect,
	.listen = socket_impl_listen,
	.accept = socket_impl_accept,
};


static void get_addr(struct socket_impl *self,
		     const union socket_addr *addr,
		     char *str,
		     size_t len,
		     uint16_t *port)
{
	char buf[INET6_ADDRSTRLEN];
	const char *ret;

	if (port != NULL)
		*port = ntohs(self->is_v6 ? addr->in6.sin6_port
					  : addr->in.sin_port);

	if (str == NULL || len == 0)
		return;

	if (self->is_v6)
		ret = inet_ntop(
			AF_INET6, &addr->in6.sin6_addr, buf, sizeof(buf));
	else
		ret = inet_ntop(AF_INET, &addr->in.sin_addr, buf, sizeof(buf));
	if (ret == NULL) {
		ULOG_ERRNO("inet_ntop", errno);
		str[0] = '\0';
		return;
	}
	snprintf(str, len, "%s", buf);
}


static void log_addrs(struct socket_impl *self)
{
	char local_addr_str[INET6_ADDRSTRLEN];
	char remote_addr_str[INET6_ADDRSTRLEN];
	uint16_t local_port = 0;
	uint16_t remote_port = 0;

	local_addr_str[0] = '\0';
	remote_addr_str[0] = '\0';

	(void)socket_impl_get_local_addr((struct tskt_socket *)self,
					 local_addr_str,
					 sizeof(local_addr_str),
					 &local_port);
	(void)socket_impl_get_remote_addr((struct tskt_socket *)self,
					  remote_addr_str,
					  sizeof(remote_addr_str),
					  &remote_port);
	ULOGD("fd=%d local %s:%d remote %s:%d",
	      self->fd,
	      local_addr_str,
	      local_port,
	      remote_addr_str,
	      remote_port);
}


static void set_remote(struct socket_impl *self, union socket_addr *remote_addr)
{
	if (self->is_v6)
		self->udp.remote_addr.in6 = remote_addr->in6;
	else
		self->udp.remote_addr.in = remote_addr->in;

	/* Log the addresses and ports for debugging */
	log_addrs(self);
}


static bool has_remote(struct socket_impl *self)
{
	return (self->is_v6 && self->udp.remote_addr.in6.sin6_port != 0) ||
	       (!self->is_v6 && self->udp.remote_addr.in.sin_port != 0);
}


static bool is_addr_unspecified(struct socket_impl *self,
				const union socket_addr *addr)
{
	return addr == NULL ||
	       (self->is_v6 &&
		(addr->in6.sin6_port == 0 ||
		 IN6_IS_ADDR_UNSPECIFIED(&addr->in6.sin6_addr))) ||
	       (!self->is_v6 && (addr->in.sin_port == 0 ||
				 addr->in.sin_addr.s_addr == INADDR_ANY));
}


static int
do_bind(struct socket_impl *self, const char *local_addr, uint16_t local_port)
{
	int res;
	union socket_addr local;

	if (self->is_v6)
		res = set_addr6(&local.in6, local_addr, local_port);
	else
		res = set_addr(&local.in, local_addr, local_port);
	if (res < 0)
		return res;

	res = bind(self->fd,
		   (const struct sockaddr *)&local,
		   self->is_v6 ? sizeof(local.in6) : sizeof(local.in));
	if (res < 0) {
		res = -errno;
		ULOG_ERRNO("bind(fd=%d)", -res, self->fd);
		return res;
	}
	return 0;
}


static int tskt_socket_new_udp(bool is_v6,
			       const char *local_addr,
			       uint16_t *local_port,
			       const char *remote_addr,
			       uint16_t remote_port,
			       const char *mcast_addr,
			       struct pomp_loop *loop,
			       pomp_fd_event_cb_t fd_cb,
			       void *userdata,
			       struct tskt_socket **ret_obj)
{
	int res = 0, mcast_addr_first, remote_addr_first, val = 0;
	struct socket_impl *self;
	union socket_addr addr;
	socklen_t addrlen = is_v6 ? sizeof(addr.in6) : sizeof(addr.in);
	uint16_t l_port = 0;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(is_v6 && mcast_addr != NULL, EINVAL);

	if (local_addr == NULL || *local_addr == '\0')
		local_addr = is_v6 ? ADDR6_ANY_STR : ADDR_ANY_STR;
	if (local_port != NULL)
		l_port = *local_port;
	if (remote_addr == NULL || *remote_addr == '\0')
		remote_addr = is_v6 ? ADDR6_ANY_STR : ADDR_ANY_STR;
	if (!is_v6 && (mcast_addr == NULL || *mcast_addr == '\0'))
		mcast_addr = ADDR_ANY_STR;

	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		return res;
	}
	self->sock.ops = &socket_impl_ops;
	self->loop = loop;
	self->is_v6 = is_v6;

	/* Create socket */
	self->fd = socket(is_v6 ? AF_INET6 : AF_INET, SOCK_DGRAM, 0);
	if (self->fd < 0) {
		res = -errno;
		ULOG_ERRNO("socket", -res);
		goto error;
	}

	/* Setup flags and options */
#ifndef _WIN32
	res = fd_set_close_on_exec(self->fd);
	if (res < 0) {
		ULOG_ERRNO("fd_set_close_on_exec(fd=%d)", -res, self->fd);
		goto error;
	}
	res = fd_add_flags(self->fd, O_NONBLOCK);
	if (res < 0) {
		ULOG_ERRNO("fd_add_flags(fd=%d)", -res, self->fd);
		goto error;
	}
#endif /* !_WIN32 */
#ifdef __linux__
	val = SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
	res = setsockopt(
		self->fd, SOL_SOCKET, SO_TIMESTAMPING, &val, sizeof(val));
	if (res < 0) {
		res = -errno;
		ULOG_ERRNO("setsockopt:SO_TIMESTAMPING(fd=%d)", -res, self->fd);
		goto error;
	}
#endif /*__linux__ */

	/* Setup local and remote addresses */
	if (is_v6) {
		/* IPv6 */
		res = set_addr6(&addr.in6, local_addr, l_port);
		if (res < 0)
			goto error;
		res = set_addr6(
			&self->udp.remote_addr.in6, remote_addr, remote_port);
		if (res < 0)
			goto error;
		/* IPv6 multicast not supported */
		goto retry_bind;
	}

	/* IPv4 */
	res = set_addr(&addr.in, local_addr, l_port);
	if (res < 0)
		goto error;
	res = set_addr(&self->udp.remote_addr.in, remote_addr, remote_port);
	if (res < 0)
		goto error;
#ifdef _WIN32
	if (remote_port != 0 && self->udp.remote_addr.in.sin_addr.s_addr == 0) {
		/* Linux treats 0.0.0.0 as local address but not Windows,
		 * replace with loopback */
		self->udp.remote_addr.in.sin_addr.s_addr =
			htonl(INADDR_LOOPBACK);
	}
#endif

	mcast_addr_first = atoi(mcast_addr);
	if ((mcast_addr_first >= 224) && (mcast_addr_first <= 239)) {
		/* Multicast address is valid, join the multicast group
		 * for reception */
		res = inet_pton(AF_INET,
				mcast_addr,
				&self->udp.mcast_mreq.imr_multiaddr.s_addr);
		if (res <= 0) {
			res = -errno;
			ULOG_ERRNO("inet_pton", -res);
			goto error;
		}
		self->udp.mcast_mreq.imr_interface = addr.in.sin_addr;
		addr.in.sin_addr = self->udp.mcast_mreq.imr_multiaddr;
		if (setsockopt(self->fd,
			       IPPROTO_IP,
			       IP_ADD_MEMBERSHIP,
			       &self->udp.mcast_mreq,
			       sizeof(self->udp.mcast_mreq)) < 0) {
			res = -errno;
			ULOG_ERRNO("setsockopt:IP_ADD_MEMBERSHIP(fd=%d)",
				   -res,
				   self->fd);
			goto error;
		}
		ULOGD("joined multicast group on address %s", mcast_addr);
		val = 1;
		if (setsockopt(self->fd,
			       SOL_SOCKET,
			       SO_REUSEADDR,
			       &val,
			       sizeof(val)) < 0) {
			res = -errno;
			ULOG_ERRNO("setsockopt:IP_MULTICAST_LOOP(fd=%d)",
				   -res,
				   self->fd);
			goto error;
		}
	}

	remote_addr_first = atoi(remote_addr);
	if ((remote_addr_first >= 224) && (remote_addr_first <= 239) &&
	    (addr.in.sin_addr.s_addr == htonl(INADDR_LOOPBACK))) {
		/* Sending to a multicast address on localhost,
		 * set the local loop option */
		val = 1;
		if (setsockopt(self->fd,
			       IPPROTO_IP,
			       IP_MULTICAST_LOOP,
			       &val,
			       sizeof(val)) < 0) {
			res = -errno;
			ULOG_ERRNO("setsockopt:IP_MULTICAST_LOOP(fd=%d)",
				   -res,
				   self->fd);
			goto error;
		}
		ULOGD("multicast loop option set");
	}

	/* Bind to local address */
retry_bind:
	if (bind(self->fd, (const struct sockaddr *)&addr, addrlen) < 0) {
		res = -errno;
		if ((res == -EADDRINUSE) && !is_v6 && (addr.in.sin_port != 0)) {
			/* XXX this is crappy behaviour */
			addr.in.sin_port = 0;
			goto retry_bind;
		}
		ULOG_ERRNO("bind(fd=%d)", -res, self->fd);
		goto error;
	}

	if (local_port) {
		/* Get the real bound address and port */
		res = getsockname(self->fd, (struct sockaddr *)&addr, &addrlen);
		if (res < 0) {
			ULOG_ERRNO("getsockname(fd=%d)", -res, self->fd);
			goto error;
		}
		*local_port =
			ntohs(is_v6 ? addr.in6.sin6_port : addr.in.sin_port);
	}

	/* Log the addresses and ports for debugging */
	log_addrs(self);

	if ((self->loop != NULL) && (fd_cb != NULL)) {
		/* Start monitoring */
		res = pomp_loop_add(self->loop,
				    self->fd,
				    POMP_FD_EVENT_IN,
				    fd_cb,
				    userdata);
		if (res < 0) {
			ULOG_ERRNO("pomp_loop_add(fd=%d)", -res, self->fd);
			goto error;
		}
	}

	get_monotonic_to_real_time_diff(self);

	/* Success */
	*ret_obj = (struct tskt_socket *)self;
	return 0;

error:
	/* Cleanup in case of error */
	socket_impl_destroy((struct tskt_socket *)self);
	*ret_obj = NULL;
	return res;
}


int tskt_socket_new(const char *local_addr,
		    uint16_t *local_port,
		    const char *remote_addr,
		    uint16_t remote_port,
		    const char *mcast_addr,
		    struct pomp_loop *loop,
		    pomp_fd_event_cb_t fd_cb,
		    void *userdata,
		    struct tskt_socket **ret_obj)
{
	return tskt_socket_new_udp(false,
				   local_addr,
				   local_port,
				   remote_addr,
				   remote_port,
				   mcast_addr,
				   loop,
				   fd_cb,
				   userdata,
				   ret_obj);
}


int tskt_socket_new_udp6(const char *local_addr,
			 uint16_t *local_port,
			 const char *remote_addr,
			 uint16_t remote_port,
			 struct pomp_loop *loop,
			 pomp_fd_event_cb_t fd_cb,
			 void *userdata,
			 struct tskt_socket **ret_obj)
{
	return tskt_socket_new_udp(true,
				   local_addr,
				   local_port,
				   remote_addr,
				   remote_port,
				   NULL,
				   loop,
				   fd_cb,
				   userdata,
				   ret_obj);
}


static int tskt_socket_new_tcp0(int fd,
				bool is_v6,
				struct pomp_loop *loop,
				struct tskt_socket **ret_obj)
{
	int res = 0;
	int val = 0;
	struct socket_impl *self;

	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		return res;
	}
	self->sock.ops = &socket_impl_ops;
	self->loop = loop;
	self->is_tcp = true;
	self->is_v6 = is_v6;

	if (fd >= 0) {
		/* Use existing socket */
		self->fd = fd;
	} else {
		/* Create new socket */
		self->fd = socket(is_v6 ? AF_INET6 : AF_INET, SOCK_STREAM, 0);
		if (self->fd < 0) {
			res = -errno;
			ULOG_ERRNO("socket", -res);
			goto error;
		}
	}

	/* Setup flags and options */
#ifndef _WIN32
	res = fd_set_close_on_exec(self->fd);
	if (res < 0) {
		ULOG_ERRNO("fd_set_close_on_exec(fd=%d)", -res, self->fd);
		goto error;
	}
	res = fd_add_flags(self->fd, O_NONBLOCK);
	if (res < 0) {
		ULOG_ERRNO("fd_add_flags(fd=%d)", -res, self->fd);
		goto error;
	}
#endif /* !_WIN32 */

	/* Avoid 'address already in use' error when binding a port */
	val = 1;
	res = setsockopt(self->fd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));
	if (res < 0) {
		res = -errno;
		ULOG_ERRNO("setsockopt:SO_REUSEADDR(fd=%d)", -res, self->fd);
		goto error;
	}

	/* Set event handler if socket is from accept() */
	if (fd >= 0) {
		res = pomp_loop_add(self->loop,
				    self->fd,
				    POMP_FD_EVENT_ERR,
				    socket_impl_fd_cb_tcp,
				    self);
		if (res < 0) {
			ULOG_ERRNO("pomp_loop_add(fd=%d)", -res, self->fd);
			goto error;
		}
	}

	/* Success */
	*ret_obj = (struct tskt_socket *)self;
	return 0;

error:
	/* Cleanup in case of error */
	if (fd >= 0) {
		/* do not close file descriptor two times */
		if (pomp_loop_has_fd(loop, fd))
			pomp_loop_remove(loop, fd);
		self->fd = -1;
	}
	socket_impl_destroy((struct tskt_socket *)self);
	*ret_obj = NULL;
	return res;
}


int tskt_socket_new_tcp(struct pomp_loop *loop, struct tskt_socket **ret_obj)
{
	ULOG_ERRNO_RETURN_ERR_IF(loop == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	/* create new TCP socket object */
	return tskt_socket_new_tcp0(-1, false, loop, ret_obj);
}


int tskt_socket_new_tcp6(struct pomp_loop *loop, struct tskt_socket **ret_obj)
{
	ULOG_ERRNO_RETURN_ERR_IF(loop == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	/* create new TCP socket object */
	return tskt_socket_new_tcp0(-1, true, loop, ret_obj);
}


static int socket_impl_destroy(struct tskt_socket *sock)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res = 0;

	if (self->fd >= 0) {
		if (self->loop != NULL) {
			if (pomp_loop_has_fd(self->loop, self->fd)) {
				res = pomp_loop_remove(self->loop, self->fd);
				if (res < 0)
					ULOG_ERRNO("pomp_loop_remove", -res);
			}
		}
		if (!self->is_tcp &&
		    self->udp.mcast_mreq.imr_multiaddr.s_addr != INADDR_ANY) {
			/* Leave the multicast group */
			if (setsockopt(self->fd,
				       IPPROTO_IP,
				       IP_DROP_MEMBERSHIP,
				       &self->udp.mcast_mreq,
				       sizeof(self->udp.mcast_mreq)) < 0) {
				res = -errno;
				ULOG_ERRNO(
					"setsockopt:IP_DROP_MEMBERSHIP(fd=%d)",
					-res,
					self->fd);
			}
		}
		close(self->fd);
	}

	if (self->is_tcp)
		tpkt_unref(self->tcp.wpkt);

#ifdef __linux__
	if (self->mmsg != NULL)
		free(self->mmsg);
#endif

	free(self);

	return 0;
}


static struct pomp_loop *socket_impl_get_loop(struct tskt_socket *sock)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	return self->loop;
}


static int socket_impl_get_fd(struct tskt_socket *sock)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	return self->fd;
}


static int socket_impl_set_fd_cb(struct tskt_socket *sock,
				 pomp_fd_event_cb_t fd_cb,
				 void *userdata)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self->fd < 0, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(self->loop == NULL, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(self->is_tcp, EOPNOTSUPP);

	if (pomp_loop_has_fd(self->loop, self->fd)) {
		res = pomp_loop_remove(self->loop, self->fd);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_remove", -res);
	}

	if (fd_cb != NULL) {
		/* Start monitoring */
		res = pomp_loop_add(self->loop,
				    self->fd,
				    POMP_FD_EVENT_IN,
				    fd_cb,
				    userdata);
		if (res < 0) {
			ULOG_ERRNO("pomp_loop_add(fd=%d)", -res, self->fd);
			return res;
		}
	}

	return 0;
}


static int socket_impl_get_local_addr(struct tskt_socket *sock,
				      char *str,
				      size_t len,
				      uint16_t *port)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;
	union socket_addr addr;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		str != NULL && ((self->is_v6 && len < INET6_ADDRSTRLEN) ||
				(!self->is_v6 && len < INET_ADDRSTRLEN)),
		EINVAL);

	socklen_t slen = sizeof(addr);
	res = getsockname(self->fd, (struct sockaddr *)&addr, &slen);
	if (res < 0) {
		res = -errno;
		ULOG_ERRNO("getsockname", -res);
		return res;
	}

	get_addr(self, &addr, str, len, port);

	return 0;
}


static int socket_impl_get_remote_addr(struct tskt_socket *sock,
				       char *str,
				       size_t len,
				       uint16_t *port)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;
	union socket_addr addr;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		str != NULL && ((self->is_v6 && len < INET6_ADDRSTRLEN) ||
				(!self->is_v6 && len < INET_ADDRSTRLEN)),
		EINVAL);

	if (self->is_tcp) {
		socklen_t slen = sizeof(addr);
		res = getpeername(self->fd, (struct sockaddr *)&addr, &slen);
		if (res < 0) {
			if (errno != SOCKERR_ENOTCONN) {
				res = -errno;
				ULOG_ERRNO("getpeername", -res);
				return res;
			}
			/* Socket is not connected,
			 * return 0.0.0.0:0 address */
			if (self->is_v6) {
				memset(&addr.in6, 0, sizeof(addr.in6));
				addr.in6.sin6_family = AF_INET6;
			} else {
				addr.in.sin_family = AF_INET;
				addr.in.sin_port = 0;
				addr.in.sin_addr.s_addr = 0;
			}
		}
	} else if (self->is_v6) {
		addr.in6 = self->udp.remote_addr.in6;
	} else {
		addr.in = self->udp.remote_addr.in;
	}

	get_addr(self, &addr, str, len, port);

	return 0;
}


static int socket_impl_set_remote_addr(struct tskt_socket *sock,
				       const char *addr,
				       uint16_t port)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;
	union socket_addr remote_addr;

	ULOG_ERRNO_RETURN_ERR_IF(self->is_tcp, EOPNOTSUPP);
	ULOG_ERRNO_RETURN_ERR_IF(addr == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(addr[0] == '\0', EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(port == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->udp.is_connected, EISCONN);

	if (self->is_v6)
		res = set_addr6(&remote_addr.in6, addr, port);
	else
		res = set_addr(&remote_addr.in, addr, port);
	if (res < 0)
		return res;

	set_remote(self, &remote_addr);

	return 0;
}


static int get_sockopt(enum tskt_option option, int *level, int *sockopt)
{
	switch (option) {
	case TSKT_OPT_RXBUF_SIZE:
		*level = SOL_SOCKET;
		*sockopt = SO_RCVBUF;
		break;
	case TSKT_OPT_TXBUF_SIZE:
		*level = SOL_SOCKET;
		*sockopt = SO_SNDBUF;
		break;
	case TSKT_OPT_CLASS_SELECTOR:
		*level = IPPROTO_IP;
		*sockopt = IP_TOS;
		break;
	case TSKT_OPT_NODELAY:
		*level = IPPROTO_TCP;
		*sockopt = TCP_NODELAY;
		break;
	case TSKT_OPT_ERROR:
		*level = SOL_SOCKET;
		*sockopt = SO_ERROR;
		break;
	case TSKT_OPT_RESET:
		return -EOPNOTSUPP;
	default:
		return -EINVAL;
	}
	return 0;
}


static const char *get_optname(enum tskt_option option)
{
	switch (option) {
	case TSKT_OPT_RXBUF_SIZE:
		return "SO_RCVBUF";
	case TSKT_OPT_TXBUF_SIZE:
		return "SO_SNDBUF";
	case TSKT_OPT_CLASS_SELECTOR:
		return "IP_TOS";
	case TSKT_OPT_NODELAY:
		return "TCP_NODELAY";
	case TSKT_OPT_ERROR:
		return "SO_ERROR";
	case TSKT_OPT_RESET:
		return "SO_LINGER";
	default:
		return "???";
	}
}


static int socket_impl_get_option(struct tskt_socket *sock,
				  enum tskt_option option)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res, level, sockopt;
	int value = 0;
	socklen_t len = sizeof(value);

	if (option == TSKT_OPT_RESET && self->is_tcp) {
		struct linger linger_opt;
		len = sizeof(linger_opt);
		if (getsockopt(self->fd,
			       SOL_SOCKET,
			       SO_LINGER,
			       &linger_opt,
			       &len) < 0)
			goto error;

		return linger_opt.l_onoff;
	}

	res = get_sockopt(option, &level, &sockopt);
	if (res < 0) {
		ULOG_ERRNO(" invalid option:%d(fd=%d)",
			   -res,
			   (int)option,
			   self->fd);
		return res;
	}

	if (getsockopt(self->fd, level, sockopt, &value, &len) < 0)
		goto error;

	return value;

error:
	res = -errno;
	ULOG_ERRNO("getsockopt:%s(fd=%d)", -res, get_optname(option), self->fd);
	return res;
}


static int socket_impl_set_option(struct tskt_socket *sock,
				  enum tskt_option option,
				  int value)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res, level, sockopt;

	if (option == TSKT_OPT_RESET && self->is_tcp) {
		struct linger linger_opt = {
			.l_onoff = value,
			.l_linger = 0,
		};
		if (setsockopt(self->fd,
			       SOL_SOCKET,
			       SO_LINGER,
			       &linger_opt,
			       sizeof(linger_opt)) < 0)
			goto error;

		return 0;
	}

	res = get_sockopt(option, &level, &sockopt);
	if (res < 0) {
		ULOG_ERRNO(" invalid option:%d(fd=%d)",
			   -res,
			   (int)option,
			   self->fd);
		return res;
	}

	if (setsockopt(self->fd, level, sockopt, &value, sizeof(value)) < 0)
		goto error;

	return 0;

error:
	res = -errno;
	ULOG_ERRNO("setsockopt:%s(fd=%d)", -res, get_optname(option), self->fd);
	return res;
}


#ifdef _WIN32

static ssize_t socket_impl_read(struct tskt_socket *sock,
				void *buf,
				size_t cap,
				uint64_t *ts_us)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;
	DWORD readlen = 0, flags = 0;
	WSABUF wsabuf;
	union socket_addr remote_addr;
	int addr_len = sizeof(remote_addr);
	struct timespec ts_cur = {0, 0};

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cap == 0, EINVAL);

	if (ts_us != NULL && !self->is_tcp) {
		res = time_get_monotonic(&ts_cur);
		if (res < 0) {
			ULOG_ERRNO("time_get_monotonic", -res);
			return res;
		}
	}

	/* Read data */
	wsabuf.buf = buf;
	wsabuf.len = (ULONG)cap;
	if (self->is_tcp)
		res = WSARecv((SOCKET)self->fd,
			      &wsabuf,
			      1,
			      &readlen,
			      &flags,
			      NULL,
			      NULL);
	else
		res = WSARecvFrom((SOCKET)self->fd,
				  &wsabuf,
				  1,
				  &readlen,
				  &flags,
				  (struct sockaddr *)&remote_addr,
				  &addr_len,
				  NULL,
				  NULL);
	if (res == SOCKET_ERROR) {
		res = -errno;
		if (res == -WSAEWOULDBLOCK)
			res = -EAGAIN;
		if (res != -EAGAIN)
			ULOG_ERRNO("WSARecvFrom(fd=%d)", -res, self->fd);
		return res;
	}

	if (self->is_tcp) {
		if (ts_us)
			*ts_us = 0;
		return readlen;
	}

	if (ts_us == NULL)
		goto done;

	time_timespec_to_us(&ts_cur, ts_us);

done:
	if (!has_remote(self))
		set_remote(self, &remote_addr);

	return readlen;
}


#else /* _WIN32 */


static ssize_t socket_impl_read(struct tskt_socket *sock,
				void *buf,
				size_t cap,
				uint64_t *ts_us)
{
	struct iovec iov;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cap == 0, EINVAL);

	iov.iov_base = buf;
	iov.iov_len = cap;
	return socket_impl_readv(sock, &iov, 1, ts_us);
}


static ssize_t socket_impl_readv(struct tskt_socket *sock,
				 const struct iovec *iov,
				 size_t iov_len,
				 uint64_t *ts_us)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;
	ssize_t readlen = 0;
	struct msghdr msg;
	struct timespec ts_cur = {0, 0};
#	ifdef __linux__
	union cmsg_buffer control;
#	endif /*__linux__ */
	union socket_addr remote_addr;
	memset(&msg, 0, sizeof(msg));

	ULOG_ERRNO_RETURN_ERR_IF(iov == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(iov_len == 0, EINVAL);

	if (ts_us != NULL && !self->is_tcp) {
		res = time_get_monotonic(&ts_cur);
		if (res < 0) {
			ULOG_ERRNO("time_get_monotonic", -res);
			return res;
		}
	}

	msg.msg_iov = (struct iovec *)iov;
	msg.msg_iovlen = iov_len;
	if (!self->is_tcp) {
#	ifdef __linux__
		msg.msg_control = &control;
		msg.msg_controllen = sizeof(control);
#	endif /*__linux__ */
		msg.msg_name = &remote_addr;
		msg.msg_namelen = sizeof(remote_addr);
	}
	/* Read data, ignoring interrupts */
	do {
		readlen = recvmsg(self->fd, &msg, 0);
	} while (readlen < 0 && errno == EINTR);

	if (readlen < 0) {
		res = -errno;
		if (res != -EAGAIN)
			ULOG_ERRNO("recvmsg(fd=%d)", -res, self->fd);
		return res;
	}

	if (self->is_tcp) {
		if (ts_us)
			*ts_us = 0;
		return readlen;
	}

#	ifdef __linux__
	if ((msg.msg_flags & MSG_CTRUNC) == MSG_CTRUNC) {
		ULOGE("ancillary data truncated");
		return -EIO;
	}
#	endif

	if (ts_us == NULL)
		goto done;

#	ifdef __linux__
	get_timestamp_cmsg(self, &ts_cur, &msg, ts_us);
#	else /*__linux__ */
	time_timespec_to_us(&ts_cur, ts_us);
#	endif /*__linux__ */

done:
	if (!has_remote(self))
		set_remote(self, &remote_addr);

	return readlen;
}

#endif /* _WIN32 */


static ssize_t
socket_impl_write(struct tskt_socket *sock, const void *buf, size_t len)
{
	struct iovec iov;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == 0, EINVAL);

	iov.iov_base = (void *)buf;
	iov.iov_len = len;
	return socket_impl_writev(sock, &iov, 1);
}


#ifdef _WIN32

static ssize_t socket_impl_writev(struct tskt_socket *sock,
				  const struct iovec *iov,
				  size_t iov_len)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;
	DWORD writelen = 0;
	WSABUF wsabuf[IOV_MAX];

	ULOG_ERRNO_RETURN_ERR_IF(iov == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(iov_len == 0 || iov_len > IOV_MAX, EINVAL);
	if (self->is_tcp) {
		ULOG_ERRNO_RETURN_ERR_IF(self->tcp.werror != 0,
					 -self->tcp.werror);
		if (self->tcp.wpkt != NULL)
			return -EAGAIN;
	}

	/* Write data */
	for (size_t i = 0; i < iov_len; i++) {
		wsabuf[i].buf = iov[i].iov_base;
		wsabuf[i].len = (ULONG)iov[i].iov_len;
	}
	if (self->is_tcp || self->udp.is_connected)
		res = WSASend((SOCKET)self->fd,
			      wsabuf,
			      iov_len,
			      &writelen,
			      0,
			      NULL,
			      NULL);
	else
		res = WSASendTo((SOCKET)self->fd,
				wsabuf,
				iov_len,
				&writelen,
				0,
				(const struct sockaddr *)&self->udp.remote_addr,
				sizeof(self->udp.remote_addr),
				NULL,
				NULL);
	if (res == SOCKET_ERROR) {
		res = -errno;
		if (res == -WSAEWOULDBLOCK)
			res = -EAGAIN;
		if (res != -EAGAIN)
			ULOG_ERRNO("WSASendTo(fd=%d)", -res, self->fd);
		return res;
	}

	return writelen;
}


#else /* _WIN32 */


static ssize_t socket_impl_writev(struct tskt_socket *sock,
				  const struct iovec *iov,
				  size_t iov_len)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	ssize_t writelen = 0;
	struct msghdr msg;
	memset(&msg, 0, sizeof(msg));

	ULOG_ERRNO_RETURN_ERR_IF(iov == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(iov_len == 0, EINVAL);
	if (self->is_tcp) {
		ULOG_ERRNO_RETURN_ERR_IF(self->tcp.werror != 0,
					 -self->tcp.werror);
		if (self->tcp.wpkt != NULL)
			return -EAGAIN;
	}

	msg.msg_iov = (struct iovec *)iov;
	msg.msg_iovlen = iov_len;
	if (!self->is_tcp && !self->udp.is_connected) {
		msg.msg_name = &self->udp.remote_addr;
		msg.msg_namelen = sizeof(self->udp.remote_addr);
	}

	/* Write ignoring interrupts */
	do {
		writelen = sendmsg(self->fd, &msg, 0);
	} while (writelen < 0 && errno == EINTR);

	if (writelen < 0) {
		int res = -errno;
		if (res != -EAGAIN)
			ULOG_ERRNO("sendmsg(fd=%d)", -res, self->fd);
		return res;
	}

	return writelen;
}

#endif /* _WIN32 */


#ifdef __linux__

static ssize_t socket_impl_write_cs(struct tskt_socket *sock,
				    const void *buf,
				    size_t len,
				    int cs)
{
	struct iovec iov;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == 0, EINVAL);

	iov.iov_base = (void *)buf;
	iov.iov_len = len;
	return socket_impl_writev_cs(sock, &iov, 1, cs);
}


static ssize_t socket_impl_writev_cs(struct tskt_socket *sock,
				     const struct iovec *iov,
				     size_t iov_len,
				     int cs)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	ssize_t writelen = 0;
	int *tos;
	struct msghdr msg;
	memset(&msg, 0, sizeof(msg));
	char buf[CMSG_SPACE(sizeof(*tos))];
	struct cmsghdr *chdr;

	ULOG_ERRNO_RETURN_ERR_IF(iov == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(iov_len == 0, EINVAL);
	/* tcp doesn't support per packet class selector */
	if (self->is_tcp || cs < 0)
		return socket_impl_writev(sock, iov, iov_len);

	msg.msg_iov = (struct iovec *)iov;
	msg.msg_iovlen = iov_len;
	if (!self->udp.is_connected) {
		msg.msg_name = &self->udp.remote_addr;
		msg.msg_namelen = sizeof(self->udp.remote_addr);
	}

	/* Set class selector in tos field of packet */
	chdr = (struct cmsghdr *)buf;
	chdr->cmsg_len = CMSG_LEN(sizeof(*tos));
	chdr->cmsg_level = IPPROTO_IP;
	chdr->cmsg_type = IP_TOS;
	tos = (int *)CMSG_DATA(chdr);
	*tos = cs;
	msg.msg_control = chdr;
	msg.msg_controllen = CMSG_LEN(sizeof(*tos));

	/* Write ignoring interrupts */
	do {
		writelen = sendmsg(self->fd, &msg, 0);
	} while (writelen < 0 && errno == EINTR);

	if (writelen < 0) {
		int res = -errno;
		if (res != -EAGAIN)
			ULOG_ERRNO("sendmsg(fd=%d)", -res, self->fd);
		return res;
	}

	return writelen;
}


static ssize_t socket_impl_writemv(struct tskt_socket *sock,
				   struct tskt_miovec *miov,
				   size_t mlen)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;
	unsigned int sent;
	struct mmsghdr *mmsg;

	if (self->is_tcp) {
		ULOG_ERRNO_RETURN_ERR_IF(self->tcp.werror != 0,
					 -self->tcp.werror);
		if (self->tcp.wpkt != NULL)
			return -EAGAIN;
	}

	res = alloc_mmsg(self, &mmsg, NULL, NULL);
	if (res < 0) {
		ULOG_ERRNO("alloc_mmsg", -res);
		return res;
	}

	sent = 0;
	while (mlen != 0) {
		unsigned int i, len;

		if (mlen > MMSG_MAX)
			len = MMSG_MAX;
		else
			len = (unsigned int)mlen;
		memset(mmsg, 0, len * sizeof(mmsg[0]));

		/* Prepare messages to send */
		for (i = 0; i < len; i++) {
			mmsg[i].msg_hdr.msg_iov = miov[i].iov;
			mmsg[i].msg_hdr.msg_iovlen = miov[i].iovlen;
			if (!self->is_tcp && !self->udp.is_connected) {
				mmsg[i].msg_hdr.msg_name =
					&self->udp.remote_addr;
				mmsg[i].msg_hdr.msg_namelen =
					sizeof(self->udp.remote_addr);
			}
		}

		/* Write ignoring interrupts */
		do {
			res = sendmmsg(self->fd, mmsg, len, 0);
		} while (res < 0 && errno == EINTR);

		if (res < 0) {
			res = -errno;
			if (res != -EAGAIN)
				ULOG_ERRNO("sendmmsg(fd=%d)", -res, self->fd);
			/* Return error only if no message was sent */
			return sent == 0 ? (ssize_t)res : (ssize_t)sent;
		}

		/* Return transmitted bytes */
		for (i = 0; i < (unsigned int)res; i++)
			miov[i].len = mmsg[i].msg_len;

		sent += res;
		miov += res;
		mlen -= res;
	}

	return (ssize_t)sent;
}

#endif /* __linux__ */


#ifdef _WIN32

static int socket_impl_read_pkt(struct tskt_socket *sock,
				struct tpkt_packet *pkt)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;
	DWORD readlen = 0, flags = 0;
	LPWSABUF wsabufs = NULL;
	size_t wsabuf_count = 0;
	void *remote_addr = NULL;
	int addr_len = self->is_v6 ? sizeof(struct sockaddr_in6)
				   : sizeof(struct sockaddr_in);
	struct timespec ts_cur = {0, 0};
	uint64_t timestamp = 0;

	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);

	if (!self->is_tcp) {
		res = time_get_monotonic(&ts_cur);
		if (res < 0) {
			ULOG_ERRNO("time_get_monotonic", -res);
			return res;
		}
	}

	res = tpkt_get_wsabufs_read(pkt, &wsabufs, &wsabuf_count);
	if (res < 0) {
		ULOG_ERRNO("tpkt_get_wsabufs_read", -res);
		return res;
	}
	if (self->is_tcp) {
		/* Read data */
		res = WSARecv((SOCKET)self->fd,
			      wsabufs,
			      wsabuf_count,
			      &readlen,
			      &flags,
			      NULL,
			      NULL);
	} else {
		remote_addr = self->is_v6 ? (void *)tpkt_get_addr6(pkt)
					  : (void *)tpkt_get_addr(pkt);
		if (remote_addr == NULL) {
			res = -EPROTO;
			ULOG_ERRNO("tpkt_get_addr", -res);
			return res;
		}

		/* Read data */
		res = WSARecvFrom((SOCKET)self->fd,
				  wsabufs,
				  wsabuf_count,
				  &readlen,
				  &flags,
				  (struct sockaddr *)remote_addr,
				  &addr_len,
				  NULL,
				  NULL);
	}
	if (res == SOCKET_ERROR) {
		res = -errno;
		if (res == -WSAEWOULDBLOCK)
			res = -EAGAIN;
		if (res != -EAGAIN)
			ULOG_ERRNO("WSARecvFrom(fd=%d)", -res, self->fd);
		return res;
	}

	res = tpkt_set_len(pkt, readlen);
	if (res < 0) {
		ULOG_ERRNO("tpkt_set_len", -res);
		return res;
	}

	if (self->is_tcp)
		goto done;

	time_timespec_to_us(&ts_cur, &timestamp);
	res = tpkt_set_timestamp(pkt, timestamp);
	if (res < 0)
		ULOG_ERRNO("tpkt_set_timestamp", -res);

	if (!has_remote(self) && (remote_addr != NULL))
		set_remote(self, remote_addr);

done:
	return 0;
}


#else /* _WIN32 */


static int socket_impl_read_pkt(struct tskt_socket *sock,
				struct tpkt_packet *pkt)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;
	ssize_t readlen = 0;
	struct msghdr msg;
	void *remote_addr = NULL;
#	ifdef __linux__
	union cmsg_buffer control;
#	endif /*__linux__ */
	struct timespec ts_cur = {0, 0};
	uint64_t timestamp = 0;
	memset(&msg, 0, sizeof(msg));

	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);

	if (!self->is_tcp) {
		res = time_get_monotonic(&ts_cur);
		if (res < 0) {
			ULOG_ERRNO("time_get_monotonic", -res);
			return res;
		}
	}

	res = tpkt_get_iov_read(pkt, &msg.msg_iov, (size_t *)&msg.msg_iovlen);
	if (res < 0) {
		ULOG_ERRNO("tpkt_get_iov_read", -res);
		return res;
	}
	if (!self->is_tcp) {
#	ifdef __linux__
		msg.msg_control = &control;
		msg.msg_controllen = sizeof(control);
#	endif /*__linux__ */
		remote_addr = self->is_v6 ? (void *)tpkt_get_addr6(pkt)
					  : (void *)tpkt_get_addr(pkt);
		if (remote_addr == NULL) {
			res = -EPROTO;
			ULOG_ERRNO("tpkt_get_addr", -res);
			return res;
		}
		msg.msg_name = remote_addr;
		msg.msg_namelen = self->is_v6 ? sizeof(struct sockaddr_in6)
					      : sizeof(struct sockaddr_in);
	}

	/* Read data, ignoring interrupts */
	do {
		readlen = recvmsg(self->fd, &msg, 0);
	} while (readlen < 0 && errno == EINTR);

	if (readlen < 0) {
		res = -errno;
		if (res != -EAGAIN)
			ULOG_ERRNO("recvmsg(fd=%d)", -res, self->fd);
		return res;
	}

	res = tpkt_set_len(pkt, readlen);
	if (res < 0) {
		ULOG_ERRNO("tpkt_set_len", -res);
		return res;
	}

	if (self->is_tcp)
		goto done;

#	ifdef __linux__
	if ((msg.msg_flags & MSG_CTRUNC) == MSG_CTRUNC) {
		ULOGE("ancillary data truncated");
		return -EIO;
	}

	get_timestamp_cmsg(self, &ts_cur, &msg, &timestamp);
#	else /*__linux__ */
	time_timespec_to_us(&ts_cur, &timestamp);
#	endif /*__linux__ */
	res = tpkt_set_timestamp(pkt, timestamp);
	if (res < 0)
		ULOG_ERRNO("tpkt_set_timestamp", -res);

	if (!has_remote(self) && (remote_addr != NULL))
		set_remote(self, (union socket_addr *)remote_addr);

done:
	return 0;
}

#endif /* _WIN32 */


#ifdef _WIN32

static int socket_impl_write_pkt(struct tskt_socket *sock,
				 struct tpkt_packet *pkt)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;
	DWORD writelen = 0;
	size_t len = 0;
	LPWSABUF wsabufs = NULL;
	size_t wsabuf_count = 0;

	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);
	if (self->is_tcp) {
		ULOG_ERRNO_RETURN_ERR_IF(self->tcp.werror != 0,
					 -self->tcp.werror);
		if (self->tcp.wpkt != NULL)
			return -EAGAIN;
	}

	res = tpkt_get_wsabufs_write(pkt, &wsabufs, &wsabuf_count);
	if (res < 0) {
		ULOG_ERRNO("tpkt_get_wsabufs_write", -res);
		return res;
	}
	if (self->is_tcp || self->udp.is_connected) {
		/* Write data */
		res = WSASend((SOCKET)self->fd,
			      wsabufs,
			      wsabuf_count,
			      &writelen,
			      0,
			      NULL,
			      NULL);
	} else {
		void *pkt_addr = self->is_v6 ? (void *)tpkt_get_addr6(pkt)
					     : (void *)tpkt_get_addr(pkt);
		void *addr = (void *)&self->udp.remote_addr;
		if (!is_addr_unspecified(self, pkt_addr)) {
			/* Send to the address associated with the packet if
			 * specified, otherwise the socket remote address is
			 * used */
			addr = pkt_addr;
		}

		/* Write data */
		res = WSASendTo((SOCKET)self->fd,
				wsabufs,
				wsabuf_count,
				&writelen,
				0,
				(const struct sockaddr *)addr,
				self->is_v6 ? sizeof(struct sockaddr_in6)
					    : sizeof(struct sockaddr_in),
				NULL,
				NULL);
	}
	if (res == SOCKET_ERROR) {
		res = -errno;
		if (res == -WSAEWOULDBLOCK)
			res = -EAGAIN;
		if (res != -EAGAIN)
			ULOG_ERRNO("WSASendTo(fd=%d)", -res, self->fd);
		return res;
	}

	/* Check the written size */
	res = tpkt_get_cdata(pkt, NULL, &len, NULL);
	if (res < 0) {
		ULOG_ERRNO("tpkt_get_cdata", -res);
	} else if ((size_t)writelen != len) {
		/* Partial write */
		if (self->is_tcp) {
			/* keep reference to packet to write pending data
			 * later when socket is ready */
			ULOGD("partial write on fd=%d (%u/%u)",
			      self->fd,
			      (unsigned int)writelen,
			      (unsigned int)len);
			tpkt_ref(pkt);
			self->tcp.wpkt = pkt;
			self->tcp.wlen = (size_t)writelen;
			res = pomp_loop_update2(
				self->loop, self->fd, POMP_FD_EVENT_OUT, 0);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_update2(fd=%d)",
					   -res,
					   self->fd);
			return res;
		}
		ULOGW("partial write on fd=%d (%u/%u)",
		      self->fd,
		      (unsigned int)writelen,
		      (unsigned int)len);
	}

	return 0;
}


#else /* _WIN32 */

static int socket_impl_write_pkt(struct tskt_socket *sock,
				 struct tpkt_packet *pkt)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;
	ssize_t writelen = 0;
	size_t len = 0;
	struct msghdr msg;
	memset(&msg, 0, sizeof(msg));

	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);
	if (self->is_tcp) {
		ULOG_ERRNO_RETURN_ERR_IF(self->tcp.werror != 0,
					 -self->tcp.werror);
		if (self->tcp.wpkt != NULL)
			return -EAGAIN;
	}

	res = tpkt_get_iov_write(pkt, &msg.msg_iov, (size_t *)&msg.msg_iovlen);
	if (res < 0) {
		ULOG_ERRNO("tpkt_get_iov_write", -res);
		return res;
	}
	if (!self->is_tcp && !self->udp.is_connected) {
		void *pkt_addr = self->is_v6 ? (void *)tpkt_get_addr6(pkt)
					     : (void *)tpkt_get_addr(pkt);
		if (!is_addr_unspecified(self, pkt_addr)) {
			/* Send to the address associated with the packet
			 * if specified */
			msg.msg_name = pkt_addr;
		} else {
			/* Otherwise send to the socket remote address */
			msg.msg_name = &self->udp.remote_addr;
		}
		msg.msg_namelen = self->is_v6 ? sizeof(struct sockaddr_in6)
					      : sizeof(struct sockaddr_in);
	}

	/* Write ignoring interrupts */
	do {
		writelen = sendmsg(self->fd, &msg, 0);
	} while (writelen < 0 && errno == EINTR);

	if (writelen < 0) {
		res = -errno;
		if (res != -EAGAIN) {
			if (res == -ENETUNREACH || res == -ENETDOWN) {
				if (!self->netdown_logged) {
					ULOG_ERRNO(
						"sendmsg(fd=%d) "
						"(logged only once)",
						-res,
						self->fd);
					self->netdown_logged = true;
				}
			} else {
				ULOG_ERRNO("sendmsg(fd=%d)", -res, self->fd);
				self->netdown_logged = false;
			}
		} else {
			self->netdown_logged = false;
		}
		return res;
	}
	self->netdown_logged = false;

	/* Check the written size */
	res = tpkt_get_cdata(pkt, NULL, &len, NULL);
	if (res < 0) {
		ULOG_ERRNO("tpkt_get_cdata", -res);
	} else if ((size_t)writelen != len) {
		/* Partial write */
		if (self->is_tcp) {
			/* keep reference to packet to write pending data
			 * later when socket is ready */
			ULOGD("partial write on fd=%d (%u/%u)",
			      self->fd,
			      (unsigned int)writelen,
			      (unsigned int)len);
			tpkt_ref(pkt);
			self->tcp.wpkt = pkt;
			self->tcp.wlen = (size_t)writelen;
			res = pomp_loop_update2(
				self->loop, self->fd, POMP_FD_EVENT_OUT, 0);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_update2(fd=%d)",
					   -res,
					   self->fd);
			return res;
		}
		ULOGW("partial write on fd=%d (%u/%u)",
		      self->fd,
		      (unsigned int)writelen,
		      (unsigned int)len);
	}

	return 0;
}

#endif /* _WIN32 */


#ifdef __linux__

static ssize_t socket_impl_read_pkt_list(struct tskt_socket *sock,
					 struct tpkt_list *list,
					 size_t max_pkts)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int num, i, res = 0;
	struct tpkt_packet **pkt;
	struct mmsghdr *mmsg;
	union cmsg_buffer *control;
	struct timespec ts_cur = {0, 0};

	if (max_pkts == 0)
		return 0;

	res = alloc_mmsg(self, &mmsg, &pkt, &control);
	if (res < 0) {
		ULOG_ERRNO("alloc_mmsg", -res);
		return res;
	}

	if (max_pkts > MMSG_MAX)
		max_pkts = MMSG_MAX;
	memset(mmsg, 0, max_pkts * sizeof(mmsg[0]));

	if (!self->is_tcp) {
		res = time_get_monotonic(&ts_cur);
		if (res < 0) {
			ULOG_ERRNO("time_get_monotonic", -res);
			return res;
		}
	}

	/* Prepare array of messages */
	for (i = 0; i < (int)max_pkts; i++) {

		pkt[i] = tskt_socket_rxpkt_alloc(sock);
		if (pkt[i] == NULL)
			break;
		res = tpkt_get_iov_read(pkt[i],
					&mmsg[i].msg_hdr.msg_iov,
					(size_t *)&mmsg[i].msg_hdr.msg_iovlen);
		if (res < 0) {
			ULOG_ERRNO("tpkt_get_iov_read", -res);
			break;
		}
		if (!self->is_tcp) {
			mmsg[i].msg_hdr.msg_control = &control[i];
			mmsg[i].msg_hdr.msg_controllen = sizeof(control[i]);
			struct sockaddr_in *remote_addr = tpkt_get_addr(pkt[i]);
			if (remote_addr == NULL) {
				res = -EPROTO;
				ULOG_ERRNO("tpkt_get_addr", -res);
				return res;
			}
			mmsg[i].msg_hdr.msg_name = remote_addr;
			mmsg[i].msg_hdr.msg_namelen = sizeof(*remote_addr);
		}
	}
	max_pkts = i;
	if (max_pkts == 0) {
		/* Failed to allocate packets */
		return -ENOMEM;
	}

	/* Read data, ignoring interrupts */
	do {
		num = recvmmsg(self->fd, mmsg, max_pkts, 0, NULL);
	} while (num < 0 && errno == EINTR);

	if (num < 0) {
		res = -errno;
		if (res != -EAGAIN)
			ULOG_ERRNO("recvmmsg(fd=%d)", -res, self->fd);
		num = 0;
	}

	for (i = 0; i < num; i++) {
		uint64_t timestamp;

		/* Set packet's length */
		res = tpkt_set_len(pkt[i], mmsg[i].msg_len);
		if (res < 0) {
			ULOG_ERRNO("tpkt_set_len", -res);
			num = i;
			break;
		}

		/* Set timestamp and remote address */
		if (!self->is_tcp) {
			if ((mmsg[i].msg_hdr.msg_flags & MSG_CTRUNC) ==
			    MSG_CTRUNC) {
				ULOGE("ancillary data truncated");
				res = -EIO;
				num = i;
				break;
			}

			get_timestamp_cmsg(
				self, &ts_cur, &mmsg[i].msg_hdr, &timestamp);
			res = tpkt_set_timestamp(pkt[i], timestamp);
			if (res < 0)
				ULOG_ERRNO("tpkt_set_timestamp", -res);

			if (!has_remote(self) &&
			    (mmsg[i].msg_hdr.msg_name != NULL))
				set_remote(self, mmsg[i].msg_hdr.msg_name);
		}

		/* Add packet to receive list */
		(void)tpkt_list_add_last(list, pkt[i]);
		(void)tpkt_unref(pkt[i]);
	}

	/* Free unused packets */
	for (i = num; i < (int)max_pkts; i++)
		tskt_socket_rxpkt_free(sock, pkt[i]);

	return num == 0 ? res : num;
}

#endif /* __linux__ */


static int socket_impl_set_event_cb(struct tskt_socket *sock,
				    uint32_t events,
				    tskt_socket_event_cb_t cb,
				    void *userdata)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;

	self->cb = cb;
	self->userdata = userdata;

	if (self->is_tcp) {
		if (cb != NULL)
			events &= POMP_FD_EVENT_IN | POMP_FD_EVENT_OUT;
		else
			events = 0;
		self->tcp.wnotify = events & POMP_FD_EVENT_OUT;
		if (self->tcp.wpkt != NULL)
			events |= POMP_FD_EVENT_OUT;
		/* Do not update events yet if socket is not setup */
		if (!pomp_loop_has_fd(self->loop, self->fd)) {
			self->tcp.revents = events;
			return 0;
		}
		res = pomp_loop_update2(
			self->loop,
			self->fd,
			events,
			~events & (POMP_FD_EVENT_IN | POMP_FD_EVENT_OUT));
		if (res < 0)
			ULOG_ERRNO("pomp_loop_update2", -res);
		return res;
	}

	/* Remove current handler */
	if (pomp_loop_has_fd(self->loop, self->fd)) {
		res = pomp_loop_remove(self->loop, self->fd);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_remove", -res);
	}

	if (cb != NULL) {
		/* Start monitoring */
		events &= POMP_FD_EVENT_IN | POMP_FD_EVENT_OUT;
		res = pomp_loop_add(self->loop,
				    self->fd,
				    POMP_FD_EVENT_ERR | events,
				    socket_impl_fd_cb,
				    self);
		if (res < 0) {
			ULOG_ERRNO("pomp_loop_add(fd=%d)", -res, self->fd);
			return res;
		}
	}

	return 0;
}


static int socket_impl_update_events(struct tskt_socket *sock,
				     uint32_t events_to_add,
				     uint32_t events_to_remove)
{
	struct socket_impl *self = (struct socket_impl *)sock;

	events_to_add &= POMP_FD_EVENT_IN | POMP_FD_EVENT_OUT;
	events_to_remove &= POMP_FD_EVENT_IN | POMP_FD_EVENT_OUT;

	if (self->is_tcp) {
		/* save OUT flag status */
		if (events_to_add & POMP_FD_EVENT_OUT)
			self->tcp.wnotify = true;
		if (events_to_remove & POMP_FD_EVENT_OUT)
			self->tcp.wnotify = false;
		/* do not clear OUT event if data to write is pending */
		if (self->tcp.wpkt != NULL)
			events_to_remove &= ~POMP_FD_EVENT_OUT;
		/* do not update events yet if socket is not setup */
		if (!pomp_loop_has_fd(self->loop, self->fd)) {
			self->tcp.revents |= events_to_add;
			self->tcp.revents &= ~events_to_remove;
			return 0;
		}
	}

	return pomp_loop_update2(
		self->loop, self->fd, events_to_add, events_to_remove);
}


static int socket_impl_connect(struct tskt_socket *sock,
			       const char *local_addr,
			       uint16_t local_port,
			       const char *remote_addr,
			       uint16_t remote_port)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	union socket_addr remote;
	int res;

	if (self->is_tcp) {
		ULOG_ERRNO_RETURN_ERR_IF(remote_addr == NULL, EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(remote_addr[0] == '\0', EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(remote_port == 0, EINVAL);
	} else {
		ULOG_ERRNO_RETURN_ERR_IF(remote_addr != NULL &&
						 remote_addr[0] != '\0' &&
						 remote_port == 0,
					 EINVAL);
	}

	/* bind local address if specified */
	if (local_port != 0 || (local_addr != NULL && local_addr[0] != '\0')) {
		res = do_bind(self, local_addr, local_port);
		if (res < 0)
			return res;
	}

	/* connect to remote */
	if (remote_addr != NULL && remote_addr[0] != '\0') {
		if (self->is_v6)
			res = set_addr6(&remote.in6, remote_addr, remote_port);
		else
			res = set_addr(&remote.in, remote_addr, remote_port);
		if (res < 0)
			return res;
	} else {
		/* disconnect socket */
		if (!self->udp.is_connected)
			return 0;
		/* On Linux we have to set an unspecified address
		 * to remove the association */
		if (self->is_v6) {
			memset(&remote.in6, 0, sizeof(remote.in6));
#ifdef __linux__
			remote.in6.sin6_family = AF_UNSPEC;
#else
			remote.in6.sin6_family = AF_INET6;
#endif
		} else {
#ifdef __linux__
			remote.in.sin_family = AF_UNSPEC;
#else
			remote.in.sin_family = AF_INET;
#endif
			remote.in.sin_addr.s_addr = 0;
			remote.in.sin_port = 0;
		}
	}
	res = connect(self->fd,
		      (const struct sockaddr *)&remote,
		      self->is_v6 ? sizeof(remote.in6) : sizeof(remote.in));
	if (res < 0) {
		res = -errno;
#ifdef _WIN32
		if (res == -WSAEWOULDBLOCK)
			goto done_tcp;
#else
		if (res == -EINPROGRESS)
			goto done_tcp;
#endif
		ULOG_ERRNO("connect(fd=%d)", -res, self->fd);
		return res;
	}
	if (!self->is_tcp) {
		self->udp.is_connected = !is_addr_unspecified(self, &remote);
		set_remote(self, &remote);
		return 0;
	}

done_tcp:
	/* set event handler */
	res = pomp_loop_add(self->loop,
			    self->fd,
			    POMP_FD_EVENT_ERR | self->tcp.revents,
			    socket_impl_fd_cb_tcp,
			    self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_add(fd=%d)", -res, self->fd);
		return res;
	}

	log_addrs(self);

	return 0;
}


static int socket_impl_listen(struct tskt_socket *sock,
			      const char *local_addr,
			      uint16_t local_port)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(!self->is_tcp, EOPNOTSUPP);

	/* bind local address and port */
	res = do_bind(self, local_addr, local_port);
	if (res < 0)
		return res;
	log_addrs(self);

	/* listen for connections */
	res = listen(self->fd, LISTEN_BACKLOG);
	if (res < 0) {
		res = -errno;
		ULOG_ERRNO("listen(fd=%d)", -res, self->fd);
		return res;
	}

	/* set event handler */
	res = pomp_loop_add(self->loop,
			    self->fd,
			    POMP_FD_EVENT_ERR | self->tcp.revents,
			    socket_impl_fd_cb_tcp,
			    self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_add(fd=%d)", -res, self->fd);
		return res;
	}

	return 0;
}


static int socket_impl_accept(struct tskt_socket *sock,
			      char *remote_addr,
			      size_t remote_len,
			      uint16_t *remote_port,
			      struct tskt_socket **ret_obj)
{
	struct socket_impl *self = (struct socket_impl *)sock;
	union socket_addr remote;
	socklen_t slen = sizeof(remote);
	int res, fd;

	ULOG_ERRNO_RETURN_ERR_IF(!self->is_tcp, EOPNOTSUPP);
	ULOG_ERRNO_RETURN_ERR_IF(
		remote_addr != NULL &&
			((self->is_v6 && remote_len < INET6_ADDRSTRLEN) ||
			 (!self->is_v6 && remote_len < INET_ADDRSTRLEN)),
		EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	/* accept connect */
	fd = accept(self->fd, (struct sockaddr *)&remote, &slen);
	if (fd < 0) {
		res = -errno;
#ifdef _WIN32
		if (res == -WSAEWOULDBLOCK)
			res = -EAGAIN;
#endif
		if (res != -EAGAIN)
			ULOG_ERRNO("accept(fd=%d)", -res, self->fd);
		return res;
	}

	/* create new socket object */
	res = tskt_socket_new_tcp0(fd, self->is_v6, self->loop, ret_obj);
	if (res < 0) {
		/* failed to create socket object, send TCP RESET
		 * to peer to report the error */
		static const struct linger linger_opt = {
			.l_onoff = 1,
			.l_linger = 0,
		};
		if (setsockopt(fd,
			       SOL_SOCKET,
			       SO_LINGER,
			       &linger_opt,
			       sizeof(linger_opt)) < 0)
			ULOG_ERRNO("setsockopt", errno);
		/* close socket and return error */
		close(fd);
		return res;
	}
	log_addrs((struct socket_impl *)(*ret_obj));

	/* return remote address */
	get_addr(self, &remote, remote_addr, remote_len, remote_port);

	return 0;
}
