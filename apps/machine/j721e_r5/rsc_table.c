/*
 * Copyright (C) 2023 Texas Instruments Incorporated - https://www.ti.com/
 *	Andrew Davis <afd@ti.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * This file populates resource table for the remote core
 * for use by the Linux host
 */

#include <openamp/open_amp.h>

#include <unistd.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <metal/sys.h>
#include <metal/device.h>
#include <metal/io.h>
#include <metal/alloc.h>

#include "rsc_table.h"
#include "helper.h"

/* Place resource table in special ELF section */
#define __section_t(S)          __attribute__((__section__(#S)))
#define __resource              __section_t(.resource_table)

struct remote_resource_table __resource resource_table =
{
    {
        1U,         /* we're the first version that implements this */
        2U,         /* number of entries, MUST be 2 */
        { 0U, 0U, } /* reserved, must be zero */
    },
    /* offsets to the entries */
    {
        offsetof(RPMessage_ResourceTable, vdev),
        offsetof(RPMessage_ResourceTable, trace),
    },
    /* vdev entry */
    {
        RPMESSAGE_RSC_TYPE_VDEV, RPMESSAGE_RSC_VIRTIO_ID_RPMSG,
        0U, 1U, 0U, 0U, 0U, 2U, { 0U, 0U },
    },
    /* the two vrings */
    { RPMESSAGE_RSC_VRING_ADDR_ANY, 4096U, 256U, 1U, 0U },
    { RPMESSAGE_RSC_VRING_ADDR_ANY, 4096U, 256U, 2U, 0U },
    {
        (RPMESSAGE_RSC_TRACE_INTS_VER0 | RPMESSAGE_RSC_TYPE_TRACE),
        (uint32_t)gDebugMemLog, DebugP_MEM_LOG_SIZE,
        0, "trace:r5fss0_0",
    },
};


void *get_resource_table (int rsc_id, int *len)
{
	(void) rsc_id;
	*len = sizeof(resource_table);
	return &resource_table;
}
