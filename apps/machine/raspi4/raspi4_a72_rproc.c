/*
 * Copyright (c) 2018 Xilinx, Inc. All rights reserved.
 * Copyright (c) 2020 Takayuki Imada <takayuki.imada@gmail.com>. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**************************************************************************
 * FILE NAME
 *
 *       raspi4_a72_rproc.c
 *
 * DESCRIPTION
 *
 *       This file define Raspberry pi 4B A72 platform specific remoteproc
 *       implementation.
 *
 **************************************************************************/

#include <openamp/remoteproc.h>
#include <metal/atomic.h>
#include <metal/device.h>
#include <metal/irq.h>
#if !defined(__linux__) /* FreeRTOS */
#include <metal/sys.h>
#endif
#include <platform_info.h>
#include <lprintf.h>
#include "interrupt.h"
#include "board.h"

/* GIC device definition */
static metal_phys_addr_t gic_phys_addr = GIC_DIST_BASE;
struct metal_device gic_device = {
	.name = GIC_DEV_NAME,
	.bus = NULL,
	.num_regions = 1,
	.regions = {
		{
			.virt = (void *)GIC_DIST_BASE,
			.physmap = &gic_phys_addr,
			.size = 0x4000,
			.page_shift = -1UL,
			.page_mask = -1UL,
			.mem_flags = DEVICE_MEMORY,
			.ops = {NULL},
		},
	},
	.node = {NULL},
	.irq_num = 0,
	.irq_info = 0,
};

static metal_phys_addr_t armlocal_phys_addr = ARMLOCAL_BASE;
struct metal_device armlocal_device = {
	.name = ARMLOCAL_DEV_NAME,
	.bus = NULL,
	.num_regions = 1,
	.regions = {
		{
			.virt = (void *)ARMLOCAL_BASE,
			.physmap = &armlocal_phys_addr,
			.size = 0x1000,
			.page_shift = -1UL,
			.page_mask = -1UL,
			.mem_flags = DEVICE_MEMORY,
			.ops = {NULL},
		},
	},
	.node = {NULL},
	.irq_num = 1,
	.irq_info = (void *)MBOX_IRQ,
};

static int raspi4_a72_proc_irq_handler(int vect_id, void *data)
{
	struct remoteproc *rproc = data;
	struct remoteproc_priv *prproc;

	(void)vect_id;
	if (!rproc)
		return METAL_IRQ_NOT_HANDLED;
	prproc = rproc->priv;

	/* Clear the mailbox data */
	metal_io_write32(prproc->armlocal_io, MBOX_CLR_OFFSET, MBOX_DATA);

	atomic_flag_clear(&prproc->nokick);
	return METAL_IRQ_HANDLED;
}

static struct remoteproc *
raspi4_a72_proc_init(struct remoteproc *rproc,
			struct remoteproc_ops *ops, void *arg)
{
	struct remoteproc_priv *prproc = arg;
	struct metal_device *dev;
	unsigned int irq_vect;
	int ret;

	if (!rproc || !prproc || !ops)
		return NULL;

#if defined(__linux__)
    /* Device registration is done by a device tree in Linux */
#else /* FreeRTOS */
	(void)metal_register_generic_device(&gic_device);
	(void)metal_register_generic_device(&armlocal_device);
#endif

    /* GIC device open */
	ret = metal_device_open(prproc->bus_name, prproc->gic_name,
				&dev);
	if (ret) {
		LPERROR("failed to open the GIC device: %d.\r\n", ret);
		return NULL;
	}
	prproc->gic_dev = dev;
	prproc->gic_io = metal_device_io_region(dev, 0);
	if (!prproc->gic_io) {
		goto err1;
    }

    /* ARM local device open */
	ret = metal_device_open(prproc->bus_name, prproc->armlocal_name,
				&dev);
	if (ret) {
		LPERROR("failed to open the ARM local device: %d.\r\n", ret);
		goto err1;
	}
	prproc->armlocal_dev = dev;
	prproc->armlocal_io = metal_device_io_region(dev, 0);
	if (!prproc->armlocal_io) {
		goto err2;
    }

    /* rproc/prproc settings */
	rproc->priv = prproc;
	rproc->ops = ops;
	atomic_flag_test_and_set(&prproc->nokick);

	/* Register interrupt handler and enable interrupt */
	irq_vect = prproc->armlocal_dev->irq_info;
#if defined(__linux__)
    ret = gic_register((uint32_t)irq_vect, MBOX_PRIORITY, (0x1U << 0x0U), prproc->gic_io);
    if (ret) {
        LPERROR("[Error] gic_enable failed with %d\n", ret);
        goto err2;
    }
#else /* FreeRTOS */
    ret = metal_raspi4_irq_init(irq_vect);
    if (ret) {
        LPERROR("[Error] metal_raspi4_irq_init failed with %d\n", ret);
        goto err2;
    }
    ret = isr_register((uint32_t)IRQ_MBOX, MBOX_PRIORITY, (0x1U << 0x3U), metal_raspi4_irq_isr);
    if (ret) {
        LPERROR("[Error] isr_register failed with %d\n", ret);
        goto err2;
    }
#endif
	metal_irq_register(irq_vect, raspi4_a72_proc_irq_handler, rproc);
	metal_irq_enable(irq_vect);

#if defined(__linux__)
	/* Shared memory device open */
	ret = metal_device_open(prproc->bus_name, prproc->shm_name,
				&dev);
	if (ret) {
		LPERROR("failed to open the shared memory device: %d.\r\n", ret);
		goto err2;
	}
	prproc->shm_dev = dev;
	prproc->shm_io = metal_device_io_region(dev, 0);
	if (!prproc->shm_io) {
		goto err3;
    }
    rproc->rsc_io = prproc->shm_io;
#endif

	LPRINTF("Successfully intialized remoteproc.\r\n");
	return rproc;

#if defined(__linux__)
err3:
    if (!prproc->shm_dev) {
    	metal_device_close(prproc->shm_dev);
    }
#endif
err2:
    if (!prproc->armlocal_dev) {
    	metal_device_close(prproc->armlocal_dev);
    }
err1:
    if (!prproc->gic_dev) {
    	metal_device_close(prproc->gic_dev);
    }
	return NULL;
}

static void raspi4_a72_proc_remove(struct remoteproc *rproc)
{
	struct remoteproc_priv *prproc;
	struct remoteproc_mem *mem;
    struct metal_list *node;
	struct metal_device *dev;

	if (!rproc)
		return;

	prproc = rproc->priv;
    if (!prproc)
        return;

#if !defined(__linux__) /* FreeRTOS */
    if (rproc->rsc_io)
        metal_free_memory(rproc->rsc_io);
#endif
    metal_list_for_each(&rproc->mems, node) {
        mem = metal_container_of(node, struct remoteproc_mem, node);
        if (mem)
            metal_free_memory(mem);
    }

	metal_irq_disable((unsigned int)prproc->armlocal_dev->irq_info);
	metal_irq_unregister((int)prproc->armlocal_dev->irq_info);

#if defined(__linux__)
	dev = prproc->shm_dev;
	if (dev)
		metal_device_close(dev);
#endif
	dev = prproc->armlocal_dev;
	if (dev)
		metal_device_close(dev);
	dev = prproc->gic_dev;
	if (dev)
		metal_device_close(dev);

#if defined(__linux__)
    /* No need to unregister the GIC device in Linux */
#else /* FreeRTOS */
	metal_list_del(&gic_device.node); /* should be done in metal_unregister_generic_device() */
	metal_list_del(&armlocal_device.node); /* should be done in metal_unregister_generic_device() */
#endif
}

static void *
raspi4_a72_proc_mmap(struct remoteproc *rproc, metal_phys_addr_t *pa,
			metal_phys_addr_t *da, size_t size,
			unsigned int attribute, struct metal_io_region **io)
{
    (void)attribute;
#if defined(__linux__)
    (void)size;
	struct remoteproc_priv *prproc;
	void *va;
#else /* FreeRTOS */
	struct metal_io_region *tmpio;
#endif
	struct remoteproc_mem *mem;
	metal_phys_addr_t lpa, lda;

    if (!rproc) {
        return NULL;
    }

	lpa = *pa;
	lda = *da;

	if (lpa == METAL_BAD_PHYS && lda == METAL_BAD_PHYS)
		return NULL;
	if (lpa == METAL_BAD_PHYS)
		lpa = lda;
	if (lda == METAL_BAD_PHYS)
		lda = lpa;

	mem = metal_allocate_memory(sizeof(*mem));
	if (!mem)
		return NULL;

#if defined(__linux__)
    prproc = rproc->priv;
	remoteproc_init_mem(mem, NULL, lpa, lda, size, prproc->shm_io);
	va = metal_io_phys_to_virt(mem->io, lpa);
	if (va) {
		if (io)
			*io = mem->io;
	    remoteproc_add_mem(rproc, mem);
	}

	return va;
#else /* FreeRTOS */
	tmpio = metal_allocate_memory(sizeof(*tmpio));
	if (!tmpio) {
		metal_free_memory(mem);
		return NULL;
	}
	remoteproc_init_mem(mem, NULL, lpa, lda, size, tmpio);
	/* va is the same as pa in this platform */
	metal_io_init(tmpio, (void *)lpa, &mem->pa, size,
		      sizeof(metal_phys_addr_t)<<3, 0x0, NULL);
	remoteproc_add_mem(rproc, mem);

	if (io)
		*io = tmpio;

	return metal_io_phys_to_virt(tmpio, mem->pa);
#endif
}

static int raspi4_a72_proc_notify(struct remoteproc *rproc, uint32_t id)
{
	struct remoteproc_priv *prproc;

	(void)id;
	if (!rproc)
		return -1;
	prproc = rproc->priv;
	if (!prproc->armlocal_io)
		return -1;

	/* Trigger an interrupt by ARM mailbox */
	metal_io_write32(prproc->armlocal_io, MBOX_SET_OFFSET, MBOX_DATA);
	return 0;
}

/* processor operations from between a72 cores. It defines
 * notification operation and remote processor managementi operations. */
struct remoteproc_ops raspi4_a72_proc_ops = {
	.init = raspi4_a72_proc_init,
	.remove = raspi4_a72_proc_remove,
	.mmap = raspi4_a72_proc_mmap,
	.notify = raspi4_a72_proc_notify,
	.start = NULL,
	.stop = NULL,
	.shutdown = NULL,
};
