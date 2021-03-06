/* SPDX-License-Identifier: BSD-2-Clause */

/*! @file
 * @brief Implementation of auxiliary functions of bdbuf tests.
 *
 * Copyright (C) 2010 OKTET Labs, St.-Petersburg, Russia
 * Author: Oleg Kravtsov <Oleg.Kravtsov@oktetlabs.ru>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <rtems.h>
#include <rtems/io.h>
#include <rtems/bdbuf.h>
#include <rtems/inttypes.h>
#include "bdbuf_tests.h"

#include <tmacros.h>

struct bdbuf_test_descr {
    void (* main)(void);
} bdbuf_tests[] = {
    { bdbuf_test1_1_main },
    { bdbuf_test1_2_main },
    { bdbuf_test1_3_main },
    { bdbuf_test1_4_main },
    { bdbuf_test1_5_main },

    { bdbuf_test2_1_main },
    { bdbuf_test2_2_main },

    { bdbuf_test3_1_main },
    { bdbuf_test3_2_main },
    { bdbuf_test3_3_main },

    { bdbuf_test4_1_main },
    { bdbuf_test4_2_main },
    { bdbuf_test4_3_main },
};

#define TEST_SEM_ATTRIBS RTEMS_DEFAULT_ATTRIBUTES

/** Device ID used for testing */
rtems_disk_device *test_dd = NULL;

/** Test result variable */
bool       good_test_result = true;

test_ctx g_test_ctx;

const char *test_name = "NO TEST";

bdbuf_test_msg test_drv_msg;

/** Count of messages in RX message queue used on disk driver side. */
#define TEST_DRV_RX_MQUEUE_COUNT 10
/** Name of disk driver RX message queue. */
#define TEST_DRV_RX_MQUEUE_NAME  (rtems_build_name( 'M', 'Q', 'D', ' ' ))

/** Count of messages in Test task RX message queue */
#define TEST_TASK_RX_MQUEUE_COUNT 10
/** Name of Test task RX message queue */
#define TEST_TASK_RX_MQUEUE_NAME  (rtems_build_name( 'M', 'Q', 'T', ' ' ))

rtems_status_code
bdbuf_test_start_aux_task(rtems_name name,
                          rtems_task_entry entry_point,
                          rtems_task_argument arg,
                          Objects_Id *id)
{
    rtems_status_code rc;
    Objects_Id        task_id;

    rc = rtems_task_create(name, BDBUF_TEST_THREAD_PRIO_NORMAL, 1024 * 2,
                           RTEMS_PREEMPT | RTEMS_NO_TIMESLICE | RTEMS_NO_ASR,
                           RTEMS_LOCAL | RTEMS_NO_FLOATING_POINT,
                           &task_id);
    if (rc != RTEMS_SUCCESSFUL)
    {
        printf("Failed to create task\n");
        return rc;
    }

    rc = rtems_task_start(task_id, entry_point, arg);
    if (rc != RTEMS_SUCCESSFUL)
    {
        printf("Failed to start task\n");
    }
    else
    {
        if (id != NULL)
            *id = task_id;
    }
    return rc;
}

void
run_bdbuf_tests()
{
    rtems_status_code sc;
    unsigned int      i;
    int               fd;
    int               rv;

    /* Create a message queue to get events from disk driver. */
    sc = rtems_message_queue_create(TEST_TASK_RX_MQUEUE_NAME,
                                    TEST_TASK_RX_MQUEUE_COUNT,
                                    sizeof(bdbuf_test_msg),
                                    RTEMS_DEFAULT_ATTRIBUTES,
                                    &g_test_ctx.test_qid);

    if (sc != RTEMS_SUCCESSFUL)
    {
        printf("Failed to create message queue for test task: %u\n", sc);
        return;
    }

    sc = test_disk_initialize();
    rtems_test_assert(sc == RTEMS_SUCCESSFUL);

    fd = open(TEST_DISK_NAME, O_RDWR);
    rtems_test_assert(fd >= 0);

    rv = rtems_disk_fd_get_disk_device(fd, &test_dd);
    rtems_test_assert(rv == 0);

    rv = close(fd);
    rtems_test_assert(rv == 0);

    /*
     * On initialization test disk device driver registers
     * its RX message queue, so we just need to locate it.
     */
    sc = rtems_message_queue_ident(TEST_DRV_RX_MQUEUE_NAME,
                                   RTEMS_SEARCH_ALL_NODES,
                                   &g_test_ctx.test_drv_qid);
    if (sc != RTEMS_SUCCESSFUL)
    {
        printf("Failed to find Test Driver Queue: %u\n", sc);
        return;
    }

    for (i = 0; i < ARRAY_NUM(g_test_ctx.test_sync_main); i++)
    {
        sc = rtems_semaphore_create(rtems_build_name('T', 'S', 'M', '0' + i),
                                    0, TEST_SEM_ATTRIBS, 0,
                                    &g_test_ctx.test_sync_main[i]);
        if (sc != RTEMS_SUCCESSFUL)
        {
            printf("Failed to create sync sem for test task: %u\n", sc);
            return;
        }
    }

    for (i = 0; i < ARRAY_NUM(g_test_ctx.test_sync); i++)
    {
        sc = rtems_semaphore_create(rtems_build_name('T', 'S', 'T', '0' + i),
                                    0, TEST_SEM_ATTRIBS, 0,
                                    &g_test_ctx.test_sync[i]);
        if (sc != RTEMS_SUCCESSFUL)
        {
            printf("Failed to create sync sem for test task #%d: %u\n", i + 1, sc);
            return;
        }
    }

    sc = rtems_semaphore_create(rtems_build_name('T', 'S', 'M', 'E'),
                                0, TEST_SEM_ATTRIBS, 0,
                                &g_test_ctx.test_end_main);
    if (sc != RTEMS_SUCCESSFUL)
    {
        printf("Failed to create end sync sem for test task: %u\n", sc);
        return;
    }

    for (i = 0; i < ARRAY_NUM(g_test_ctx.test_task); i++)
        g_test_ctx.test_task[i] = OBJECTS_ID_NONE;

    for (i = 0; i < sizeof(bdbuf_tests) / sizeof(bdbuf_tests[0]); i++)
    {
        bdbuf_tests[i].main();
    }
}


rtems_status_code
bdbuf_test_create_drv_rx_queue(Objects_Id *id)
{
    return rtems_message_queue_create(TEST_DRV_RX_MQUEUE_NAME,
                                      TEST_DRV_RX_MQUEUE_COUNT,
                                      sizeof(bdbuf_test_msg),
                                      RTEMS_DEFAULT_ATTRIBUTES,
                                      id);
}

rtems_status_code
bdbuf_test_create_drv_tx_queue(Objects_Id *id)
{
    return  rtems_message_queue_ident(TEST_TASK_RX_MQUEUE_NAME,
                                      RTEMS_SEARCH_ALL_NODES,
                                      id);
}

rtems_status_code
bdbuf_test_start_thread(unsigned int idx, rtems_task_entry func)
{
    rtems_status_code sc;

    if (g_test_ctx.test_task[idx] != OBJECTS_ID_NONE)
    {
        sc = rtems_task_delete(g_test_ctx.test_task[idx]);
        if (sc != RTEMS_SUCCESSFUL)
        {
            printf("Failed to delete test thread %u in test %s\n",
                   idx + 1, g_test_ctx.test_name);
            return sc;
        }
    }
    sc = bdbuf_test_start_aux_task(
            rtems_build_name('T', 'S', '.', '0' + idx),
            func, (rtems_task_argument)(NULL),
            &g_test_ctx.test_task[idx]);
    return sc;
}

rtems_status_code
bdbuf_test_end()
{
    rtems_status_code sc;
    unsigned int      i;

    for (i = 0; i < ARRAY_NUM(g_test_ctx.test_task); i++)
    {
        if (g_test_ctx.test_task[i] != OBJECTS_ID_NONE)
        {
            sc = rtems_semaphore_obtain(g_test_ctx.test_end_main,
                                        RTEMS_WAIT, RTEMS_NO_TIMEOUT);
            if (sc != RTEMS_SUCCESSFUL)
            {
                printf("Failed to get a thread stopped\n");
            }
            g_test_ctx.test_task[i] = OBJECTS_ID_NONE;
        }
    }
    return RTEMS_SUCCESSFUL;
}
