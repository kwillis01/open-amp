/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 * Copyright (c) 2017 Xilinx, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <metal/atomic.h>
#include <metal/assert.h>
#include <metal/device.h>
#include <metal/irq.h>
#include <metal/utilities.h>
#include <openamp/rpmsg_virtio.h>
#include <errno.h>

#include "r5/kernel/dpl/HwiP.h"
#include "platform_info.h"
#include "rsc_table.h"
#include "mailbox.h"

#define KICK_DEV_NAME         "mailbox"
#define KICK_BUS_NAME         "generic"

#ifndef SHARED_MEM_PA
#define SHARED_MEM_PA  0xA2000000UL
#endif /* !SHARED_MEM_PA */

#ifndef SHARED_MEM_SIZE
#define SHARED_MEM_SIZE 0x100000UL
#endif /* !SHARED_MEM_SIZE */

#ifndef SHARED_BUF_OFFSET
#define SHARED_BUF_OFFSET 0x8000UL
#endif /* !SHARED_BUF_OFFSET */

#ifndef RPMSG_NO_IPI
#define _rproc_wait() asm volatile("wfi")
#endif /* !RPMSG_NO_IPI */

/* Polling information used by remoteproc operations */
static metal_phys_addr_t poll_phys_addr = MAILBOX_BASE_ADDR;
struct metal_device mailbox_device = {
	.name = KICK_DEV_NAME,
	.bus = NULL,
	.num_regions = 1,
	.regions = {
		{
			.virt = (void *)MAILBOX_BASE_ADDR,
			.physmap = &poll_phys_addr,
			.size = 0x1000,
			.page_shift = -1UL,
			.page_mask = -1UL,
			.mem_flags = DEVICE_NONSHARED | PRIV_RW_USER_RW,
			.ops = {NULL},
		}
	},
	.node = {NULL},

	.irq_num = 1,
	.irq_info = (void *)65,
};

static struct remoteproc_priv rproc_priv = {
	.kick_dev_name = KICK_DEV_NAME,
	.kick_dev_bus_name = KICK_BUS_NAME,
};

static struct remoteproc rproc_inst;
int messageFlag = 0;


void getMessageISR(void *args){
	printf("In ISR");
	messageFlag = 1;
	HwiP_clearInt(98);
}

/* processor operations from r5 to a53 */
static struct remoteproc * am64_r5_a53_proc_init(struct remoteproc *rproc,
						 const struct remoteproc_ops *ops,
						 void *arg)
{
	struct remoteproc_priv *prproc = arg;
//	struct metal_device *kick_dev;
//	unsigned int irq_vect;
//	int ret;

	if (!rproc || !prproc || !ops)
		return NULL;
//	ret = metal_device_open(prproc->kick_dev_bus_name,
//				prproc->kick_dev_name,
//				&kick_dev);
//	if (ret) {
//		printf("failed to open polling device: %d.\r\n", ret);
//		return NULL;
//	}
//	rproc->priv = prproc;
//	prproc->kick_dev = kick_dev;
//	prproc->kick_io = metal_device_io_region(kick_dev, 0);
//	if (!prproc->kick_io)
//		goto err1;

//	(void)irq_vect;
//	metal_io_write32(prproc->kick_io, 0, !POLL_STOP);

	HwiP_Params hwiParams;
    HwiP_Object hwiObj;
 
    HwiP_Params_init(&hwiParams);
    /* for R5F, interrupt #10 at VIM */
    hwiParams.intNum = 98; 
    /* for M4F, external interrupt #10 at NVIC is 
       16 internal interrupts + external interrupt number at NVIC 
       i.e hwiParams.intNum = 16 + 10; 
     */
    hwiParams.callback = getMessageISR;
    hwiParams.args = NULL;
 
	HwiP_init();
    HwiP_construct(&hwiObj, &hwiParams);
	HwiP_enable();
	messageFlag = 0;

	rproc->ops = ops;

	return rproc;
//err1:
//	metal_device_close(kick_dev);
	return NULL;
}

static void am64_r5_a53_proc_remove(struct remoteproc *rproc)
{
	struct remoteproc_priv *prproc;

	if (!rproc)
		return;
	prproc = rproc->priv;

	metal_device_close(prproc->kick_dev);
}

static void *
am64_r5_a53_proc_mmap(struct remoteproc *rproc, metal_phys_addr_t *pa,
			metal_phys_addr_t *da, size_t size,
			unsigned int attribute, struct metal_io_region **io)
{
	struct remoteproc_mem *mem;
	metal_phys_addr_t lpa, lda;
	struct metal_io_region *tmpio;

	lpa = *pa;
	lda = *da;

	if (lpa == METAL_BAD_PHYS && lda == METAL_BAD_PHYS)
		return NULL;
	if (lpa == METAL_BAD_PHYS)
		lpa = lda;
	if (lda == METAL_BAD_PHYS)
		lda = lpa;

	if (!attribute)
		attribute = NORM_SHARED_NCACHE | PRIV_RW_USER_RW;
	mem = metal_allocate_memory(sizeof(*mem));
	if (!mem)
		return NULL;
	tmpio = metal_allocate_memory(sizeof(*tmpio));
	if (!tmpio) {
		metal_free_memory(mem);
		return NULL;
	}
	remoteproc_init_mem(mem, NULL, lpa, lda, size, tmpio);
	/* va is the same as pa in this platform */
	metal_io_init(tmpio, (void *)lpa, &mem->pa, size,
		      sizeof(metal_phys_addr_t) << 3, attribute, NULL);
	remoteproc_add_mem(rproc, mem);
	*pa = lpa;
	*da = lda;
	if (io)
		*io = tmpio;
	return metal_io_phys_to_virt(tmpio, mem->pa);
}

static int am64_r5_a53_proc_notify(struct remoteproc *rproc, uint32_t id)
{
	(void)rproc;

	// Put message in mailbox
	if (MailboxSendMessage(AM64_R5FSS1_MAILBOX, 0, id) == 0)
		printf("Sent on queue 0: %lu\n", id);

	return 0;
}

/* processor operations from r5 to a53. It defines
 * notification operation and remote processor managementi operations. */
const struct remoteproc_ops am64_r5_a53_proc_ops = {
	.init = am64_r5_a53_proc_init,
	.remove = am64_r5_a53_proc_remove,
	.mmap = am64_r5_a53_proc_mmap,
	.notify = am64_r5_a53_proc_notify,
	.start = NULL,
	.stop = NULL,
	.shutdown = NULL,
};

/* RPMsg virtio shared buffer pool */
static struct rpmsg_virtio_shm_pool shpool;

static struct remoteproc *
platform_create_proc(int proc_index, int rsc_index)
{
	void *rsc_table;
	int rsc_size;
	int ret;
	metal_phys_addr_t pa;

	(void) proc_index;
	rsc_table = get_resource_table(rsc_index, &rsc_size);

	/* Register IPI device */
	if (metal_register_generic_device(&mailbox_device))
		return NULL;

	/* Initialize remoteproc instance */
	if (!remoteproc_init(&rproc_inst, &am64_r5_a53_proc_ops, &rproc_priv))
		return NULL;

	/*
	 * Mmap shared memories
	 * Or shall we constraint that they will be set as carved out
	 * in the resource table?
	 */
	/* mmap resource table */
	pa = (metal_phys_addr_t)rsc_table;
	(void *)remoteproc_mmap(&rproc_inst, &pa,
				NULL, rsc_size,
				NORM_NSHARED_NCACHE|PRIV_RW_USER_RW,
				&rproc_inst.rsc_io);
	/* mmap shared memory */
	pa = SHARED_MEM_PA;
	(void *)remoteproc_mmap(&rproc_inst, &pa,
				NULL, SHARED_MEM_SIZE,
				NORM_NSHARED_NCACHE|PRIV_RW_USER_RW,
				NULL);

	/* parse resource table to remoteproc */
	ret = remoteproc_set_rsc_table(&rproc_inst, rsc_table, rsc_size);
	if (ret) {
		printf("Failed to initialize remoteproc (%d)\n", ret);
		remoteproc_remove(&rproc_inst);
		return NULL;
	}
	printf("Initialize remoteproc successfully.\r\n");

	return &rproc_inst;
}

int platform_init(int argc, char *argv[], void **platform)
{
	unsigned long proc_id = 0;
	unsigned long rsc_id = 0;
	struct remoteproc *rproc;

	if (!platform) {
		printf("Failed to initialize platform,"
			   "NULL pointer to store platform data.\r\n");
		return -EINVAL;
	}
	/* Initialize HW system components */
//	init_system();

	/* Low level abstraction layer for openamp initialization */
	struct metal_init_params init_param = METAL_INIT_DEFAULTS;
	metal_init(&init_param);


	if (argc >= 2) {
		proc_id = strtoul(argv[1], NULL, 0);
	}

	if (argc >= 3) {
		rsc_id = strtoul(argv[2], NULL, 0);
	}

	rproc = platform_create_proc(proc_id, rsc_id);
	if (!rproc) {
		printf("Failed to create remoteproc device.\r\n");
		return -EINVAL;
	}
	*platform = rproc;
	return 0;
}

struct  rpmsg_device *
platform_create_rpmsg_vdev(void *platform, unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_bind_cb)
{
	struct remoteproc *rproc = platform;
	struct rpmsg_virtio_device *rpmsg_vdev;
	struct virtio_device *vdev;
	void *shbuf;
	struct metal_io_region *shbuf_io;
	int ret;

	rpmsg_vdev = metal_allocate_memory(sizeof(*rpmsg_vdev));
	if (!rpmsg_vdev)
		return NULL;
	shbuf_io = remoteproc_get_io_with_pa(rproc, SHARED_MEM_PA);
	if (!shbuf_io)
		goto err1;
	shbuf = metal_io_phys_to_virt(shbuf_io,
				      SHARED_MEM_PA + SHARED_BUF_OFFSET);

	printf("creating remoteproc virtio\r\n");
	/* TODO: can we have a wrapper for the following two functions? */
	vdev = remoteproc_create_virtio(rproc, vdev_index, role, rst_cb);
	if (!vdev) {
		printf("failed remoteproc_create_virtio\r\n");
		goto err1;
	}

	printf("initializing rpmsg shared buffer pool\r\n");
	/* Only RPMsg virtio driver needs to initialize the shared buffers pool */
	rpmsg_virtio_init_shm_pool(&shpool, shbuf,
				   (SHARED_MEM_SIZE - SHARED_BUF_OFFSET));

	printf("initializing rpmsg vdev\r\n");
	/* RPMsg virtio device can set shared buffers pool argument to NULL */
	ret =  rpmsg_init_vdev(rpmsg_vdev, vdev, ns_bind_cb,
			       shbuf_io,
			       &shpool);
	if (ret) {
		printf("failed rpmsg_init_vdev\r\n");
		goto err2;
	}
	printf("initializing rpmsg vdev\r\n");
	return rpmsg_virtio_get_rpmsg_device(rpmsg_vdev);
err2:
	remoteproc_remove_virtio(rproc, vdev);
err1:
	metal_free_memory(rpmsg_vdev);
	return NULL;
}

int platform_poll(void *priv)
{
	struct remoteproc *rproc = priv;
	uint32_t msg;
	int ret;

	// mailbox interrupt for getMessage here 
	while(1) {
		/*
		if (MailboxGetMessage(MAILBOX_BASE_ADDR, 1, &msg) == MESSAGE_VALID) {
			ret = remoteproc_get_notification(rproc, msg);
			if (ret)
				return ret;
			break;
		}*/
		if (messageFlag)
		{
			messageFlag = 0;
			MailboxGetMessage(MAILBOX_BASE_ADDR, 1, &msg);
			ret = remoteproc_get_notification(rproc, msg);
			if (ret)
				return ret;
			break;
		}
//		_rproc_wait();
	}
	return 0;
}

void platform_release_rpmsg_vdev(struct rpmsg_device *rpdev, void *platform)
{
	struct rpmsg_virtio_device *rpvdev;
	struct remoteproc *rproc;

	rpvdev = metal_container_of(rpdev, struct rpmsg_virtio_device, rdev);
	rproc = platform;

	rpmsg_deinit_vdev(rpvdev);
	remoteproc_remove_virtio(rproc, rpvdev->vdev);
}

void platform_cleanup(void *platform)
{
	struct remoteproc *rproc = platform;

	if (rproc)
		remoteproc_remove(rproc);
//	cleanup_system();

	metal_finish();
}
