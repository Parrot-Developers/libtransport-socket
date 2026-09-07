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

#ifndef _TSKT_TEST_H_
#define _TSKT_TEST_H_

#include <CUnit/Automated.h>
#include <CUnit/Basic.h>
#include <CUnit/CUnit.h>

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libpomp.h>
#include <transport-socket/tskt.h>
#include <transport-socket/tskt_client.h>
#include <transport-socket/tskt_ops.h>
#include <transport-socket/tskt_pomp.h>
#include <transport-socket/tskt_resolv.h>
#include <transport-socket/tskt_resolv_ops.h>

#include "tskt_test_fakes.h"

/* codecheck_ignore[COMPLEX_MACRO] */
#define FN(_name) (char *)_name


/* Test suites, one CU_TestInfo array per source file under test, each
 * implemented in its own tests/tskt_test_*.c file and registered in
 * tskt_test.c. Declared here as they get implemented. */
extern CU_TestInfo g_tskt_test_dispatch[];
extern CU_TestInfo g_tskt_test_resolv[];
extern CU_TestInfo g_tskt_test_pomp[];
extern CU_TestInfo g_tskt_test_impl[];
extern CU_TestInfo g_tskt_test_client[];
extern CU_TestInfo g_tskt_test_resolv_impl[];


#endif /* !_TSKT_TEST_H_ */
