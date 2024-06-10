/*
 * Copyright (c) 2019 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_POSIX_SYS_SELECT_H_
#define ZEPHYR_INCLUDE_POSIX_SYS_SELECT_H_

#include <signal.h>

#include <zephyr/posix/sys/time.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct zvfs_fd_set fd_set;

int pselect(int nfds, fd_set *ZRESTRICT readfds, fd_set *ZRESTRICT writefds,
	    fd_set *ZRESTRICT errorfds, const struct timespec *ZRESTRICT timeout,
	    const sigset_t *ZRESTRICT sigmask);
int select(int nfds, fd_set *ZRESTRICT readfds, fd_set *ZRESTRICT writefds,
	   fd_set *ZRESTRICT errorfds, struct timeval *ZRESTRICT timeout);
void FD_CLR(int fd, fd_set *fdset);
int FD_ISSET(int fd, fd_set *fdset);
void FD_SET(int fd, fd_set *fdset);
void FD_ZERO(fd_set *fdset);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_POSIX_SYS_SELECT_H_ */
