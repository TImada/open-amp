/*
 * Copyright (c) 2014, Mentor Graphics Corporation. All rights reserved.
 * Copyright (c) 2017 - 2018 Xilinx, Inc. All rights reserved.
 * Copyright (c) 2021 Takayuki Imada <takayuki.imada@gmail.com>. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**************************************************************************
 * FILE NAME
 *
 *       platform_info.c
 *
 * DESCRIPTION
 *
 *       This file define platform specific data and implements APIs to set
 *       platform specific information for OpenAMP.
 *
 **************************************************************************/

#include <openamp/remoteproc.h>
#include <openamp/rpmsg_virtio.h>
#include <metal/atomic.h>
#include <metal/device.h>
#include <metal/io.h>
#include <metal/irq.h>
#include <metal/sys.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "platform_info.h"
#include "rsc_table.h"
#include "lprintf.h"
#if !defined(__linux__) /* FreeRTOS */
#include "interrupt.h"
#include "board.h"
#endif

extern struct remoteproc_ops raspi4_a72_proc_ops;

/* Remoteproc private data */
static struct remoteproc_priv rproc_priv = {
    .bus_name = GIC_BUS_NAME,
    .gic_name = GIC_DEV_NAME,
    .gic_dev = NULL,
    .gic_io = NULL,
    .armlocal_name = ARMLOCAL_DEV_NAME,
    .armlocal_dev = NULL,
    .armlocal_io = NULL,
    .shm_name = SHM_DEV_NAME,
    .shm_dev = NULL,
    .shm_io = NULL,
};

/* Remoteproc instance */
static struct remoteproc rproc_inst;

static struct remoteproc *
platform_create_proc(int proc_index, int rsc_index)
{
    (void) proc_index;
    (void) rsc_index;
    void *rsc_table, *buf;
    int ret;
    metal_phys_addr_t pa;

    /* Initialize remoteproc instance */
    if (!remoteproc_init(&rproc_inst, &raspi4_a72_proc_ops, &rproc_priv))
        return NULL;

    /* Mmap resource table */
    pa = RSC_MEM_PA;
    LPRINTF("Calling mmap resource table.\r\n");
    rsc_table = remoteproc_mmap(&rproc_inst, &pa, NULL, RSC_MEM_SIZE,
                    0, &rproc_inst.rsc_io);
    if (!rsc_table) {
        LPERROR("ERROR: Failed to mmap resource table.\r\n");
        goto err;
    }   
    LPRINTF("Successfully mmap resource table.\r\n");
    /* Mmap shared memory */
    pa = SHARED_BUF_PA;
    LPRINTF("Calling mmap shared memory.\r\n");
    buf = remoteproc_mmap(&rproc_inst, &pa, NULL, SHARED_BUF_SIZE,
                    0, NULL);
    if (!buf) {
        LPERROR("ERROR: Failed to mmap shared memory.\r\n");
        goto err;
    }   
    LPRINTF("Successfully mmap the buffer region.\r\n");

    /* parse resource table to remoteproc */
    ret = remoteproc_set_rsc_table(&rproc_inst, rsc_table, RSC_MEM_SIZE);
    if (ret) {
        LPERROR("Failed to intialize remoteproc\r\n");
        goto err;
    }
    LPRINTF("Initialize remoteproc successfully.\r\n");

    return &rproc_inst;
err:
    remoteproc_remove(&rproc_inst);
    return NULL;
}

int platform_init(int argc, char *argv[], void **platform)
{
    unsigned long proc_id = 0;
    unsigned long rsc_id = 0;
    struct remoteproc *rproc;

    if (!platform) {
        LPERROR("Failed to initialize platform,"
               "NULL pointer to store platform data.\r\n");
        return -EINVAL;
    }

    if (argc >= 2) {
        proc_id = strtoul(argv[1], NULL, 0);
    }

    if (argc >= 3) {
        rsc_id = strtoul(argv[2], NULL, 0);
    }

    rproc = platform_create_proc(proc_id, rsc_id);
    if (!rproc) {
        LPERROR("Failed to create remoteproc device.\r\n");
        return -EINVAL;
    }
    *platform = rproc;
    return 0;
}

/* RPMsg virtio shared buffer pool */
static struct rpmsg_virtio_shm_pool shpool;

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
    memset(rpmsg_vdev, 0x0, sizeof(struct rpmsg_virtio_device));
    shbuf_io = remoteproc_get_io_with_pa(rproc, SHARED_BUF_PA);
    if (!shbuf_io)
        return NULL;
    shbuf = metal_io_phys_to_virt(shbuf_io, SHARED_BUF_PA);

    LPRINTF("creating remoteproc virtio\r\n");
    /* TODO: can we have a wrapper for the following two functions? */
    vdev = remoteproc_create_virtio(rproc, vdev_index, role, rst_cb);
    if (!vdev) {
        LPERROR("failed remoteproc_create_virtio\r\n");
        goto err1;
    }

    LPRINTF("Initializing rpmsg vdev\r\n");
    if (role == VIRTIO_DEV_MASTER) {
        /* Only RPMsg virtio master needs to initialize the
         * shared buffers pool
         */
        rpmsg_virtio_init_shm_pool(&shpool, shbuf, SHARED_BUF_SIZE);

        /* RPMsg virtio slave can set shared buffers pool
         * argument to NULL
         */
        ret =  rpmsg_init_vdev(rpmsg_vdev, vdev, ns_bind_cb,
                       shbuf_io, &shpool);
    } else {
        ret =  rpmsg_init_vdev(rpmsg_vdev, vdev, ns_bind_cb,
                       shbuf_io, NULL);
    }
    if (ret) {
        LPERROR("failed rpmsg_init_vdev\r\n");
        goto err2;
    }
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
    struct remoteproc_priv *prproc;
    unsigned int flags;

    prproc = rproc->priv;
    while(1) {
        flags = metal_irq_save_disable();
        if (!(atomic_flag_test_and_set(&prproc->nokick))) {
            metal_irq_restore_enable(flags);
            remoteproc_get_notification(rproc, RSC_NOTIFY_ID_ANY);
            break;
        }
        _rproc_wait();
        metal_irq_restore_enable(flags);
    }
    return 0;
}

void platform_release_rpmsg_vdev(struct rpmsg_device *rpdev)
{
    struct rpmsg_virtio_device *rpmsg_vdev;
    
    rpmsg_vdev = metal_container_of(rpdev, struct rpmsg_virtio_device, rdev);
    if (rpmsg_vdev)
        metal_free_memory(rpmsg_vdev);
    
    /* TODO: Add release operations if needed */

    return;
}

void platform_cleanup(void *platform)
{
    struct remoteproc *rproc = platform;

    if (rproc)
        remoteproc_remove(rproc);

    return;
}
