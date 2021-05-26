#ifndef PLATFORM_INFO_H_
#define PLATFORM_INFO_H_

#include <openamp/remoteproc.h>
#include <openamp/virtio.h>
#include <openamp/rpmsg.h>

#if defined __cplusplus
extern "C" {
#endif

/* Another APU core ID. In this demo, the other APU core is 0. */
#if defined(__linux__)
#define RECV_CPU_ID	(3UL)
#else /* FreeRTOS */
#define RECV_CPU_ID	(0UL)
#endif

/* System bus name */
#if defined(__linux__)
#define GIC_BUS_NAME	"platform"
#else /* FreeRTOS */
#define GIC_BUS_NAME	"generic"
#endif

/* gic device */
#if defined(__linux__)
#define GIC_DEV_NAME    "40041000.gic_uio"
#define GIC_DIST_BASE   (0x40041000U)
#else /* FreeRTOS */
#define GIC_DEV_NAME    "ff841000.gic"
#define GIC_DIST_BASE   (0xFF841000U)
#endif

/* ARM mailbox */
#if defined(__linux__)
#define MBOX_SET_OFFSET (0xB0U) /* Set mailbox#12 */
#define MBOX_CLR_OFFSET (0xC0U) /* Clear mailbox#0 */
#define ARMLOCAL_DEV_NAME   "ff800000.armlocal_uio"
#define ARMLOCAL_BASE  (0xff800000U)
#else /* FreeRTOS */
#define MBOX_SET_OFFSET (0x80U) /* Set mailbox#0 */
#define MBOX_CLR_OFFSET (0xF0U) /* Clear mailbox#12 */
#define ARMLOCAL_DEV_NAME   "ff800000.mbox"
#define ARMLOCAL_BASE  (0xff800000U)
#endif
#define MBOX_DATA   (0x1U)
#define MBOX_IRQ    (IRQ_MBOX) /* Linux: SPI#0, FreeRTOS: SPI#12 */
#define MBOX_PRIORITY (0xA0)

/* RPMSG service name */
#define RPMSG_SERVICE_NAME  "rpmsg-service0"

/* Shared memory */
#define SHM_DEV_NAME        "20600000.shm"
#define SHM_MEM_PA          (0x20600000UL)
#define SHM_MEM_SIZE        (0x00200000UL)
#define RSC_MEM_PA          (SHM_MEM_PA)
#define RSC_MEM_SIZE        (0x00002000UL)
#define VRING_BASE          (SHM_MEM_PA + 0x00080000UL)
#define VRING_TX_BASE       (VRING_BASE)
#define VRING_RX_BASE       (VRING_TX_BASE + 0x00040000UL)
#define SHARED_BUF_PA       (SHM_MEM_PA + 0x00100000UL)
#define SHARED_BUF_SIZE     (0x00100000UL)

/* Memory attributes */
#define NORM_NONCACHE       (0x11DE2)	/* Normal Non-cacheable */
#define STRONG_ORDERED      (0xC02)	/* Strongly ordered */
#define DEVICE_MEMORY       (0xC06)	/* Device memory */
#define RESERVED            (0x0)		/* reserved memory */

#define _rproc_wait() asm volatile("wfi")

/* Remoteproc private data struct */
struct remoteproc_priv {
    /* system bus */
    const char *bus_name; /* bus name */
    /* GIC device */
	const char *gic_name; /* GIC device name */
	struct metal_device *gic_dev; /* pointer to GIC device */
	struct metal_io_region *gic_io; /* pointer to GIC i/o region */
    /* ARM local device */
	const char *armlocal_name; /* ARM local device name */
	struct metal_device *armlocal_dev; /* pointer to ARM local device */
	struct metal_io_region *armlocal_io; /* pointer to ARM local I/O region */
    /* Shared memory device */
    const char *shm_name; /* shared memory device name */
	struct metal_device *shm_dev; /* pointer to shared memory device */
	struct metal_io_region *shm_io; /* pointer to shared memory i/o region */
    /* Misc */
	atomic_int nokick; /* 0 for kick from other side */
};

/**
 * platform_init - initialize the platform
 *
 * It will initialize the platform.
 *
 * @argc: number of arguments
 * @argv: array of the input arguements
 * @platform: pointer to store the platform data pointer
 *
 * return 0 for success or negative value for failure
 */
int platform_init(int argc, char *argv[], void **platform);

/**
 * platform_create_rpmsg_vdev - create rpmsg vdev
 *
 * It will create rpmsg virtio device, and returns the rpmsg virtio
 * device pointer.
 *
 * @platform: pointer to the private data
 * @vdev_index: index of the virtio device, there can more than one vdev
 *              on the platform.
 * @role: virtio master or virtio slave of the vdev
 * @rst_cb: virtio device reset callback
 * @ns_bind_cb: rpmsg name service bind callback
 *
 * return pointer to the rpmsg virtio device
 */
struct rpmsg_device *
platform_create_rpmsg_vdev(void *platform, unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_bind_cb);

/**
 * platform_poll - platform poll function
 *
 * @platform: pointer to the platform
 *
 * return negative value for errors, otherwise 0.
 */
int platform_poll(void *platform);

/**
 * platform_release_rpmsg_vdev - release rpmsg virtio device
 *
 * @rpdev: pointer to the rpmsg device
 */
void platform_release_rpmsg_vdev(struct rpmsg_device *rpdev);

/**
 * platform_cleanup - clean up the platform resource
 *
 * @platform: pointer to the platform
 */
void platform_cleanup(void *platform);

#if defined __cplusplus
}
#endif

#endif /* PLATFORM_INFO_H_ */
