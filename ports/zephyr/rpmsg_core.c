/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Linaro Limited
 * Copyright (c) 2024 TI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "py/mpconfig.h"
#include "py/runtime.h"
#include "src/zephyr_getchar.h"

// Zephyr headers
#include <zephyr/kernel.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ipm.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/io.h>
#include <resource_table.h>

#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Sample requires definition of shared memory for rpmsg"
#endif

/* Constants derived from device tree */
#define SHM_NODE	DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR	DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE	DT_REG_SIZE(SHM_NODE)

#define APP_TASK_STACK_SIZE (1024)

/* Add 1024 extra bytes for the TTY task stack for the "tx_buff" buffer. */
#define APP_TTY_TASK_STACK_SIZE (1536)
K_THREAD_STACK_DEFINE(thread_mng_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_mng_data;

static const struct device *const ipm_handle = DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;
static metal_phys_addr_t rsc_tab_physmap;

static struct metal_io_region shm_io_data; /* shared memory */
static struct metal_io_region rsc_io_data; /* rsc_table memory */

struct rpmsg_rcv_msg
{
	void *data;
	size_t len;
};

static struct metal_io_region *shm_io = &shm_io_data;

static struct metal_io_region *rsc_io = &rsc_io_data;
static struct rpmsg_virtio_device rvdev;

static struct fw_resource_table *rsc_table;

static struct rpmsg_endpoint tty_ept;

static K_SEM_DEFINE (data_sem, 0, 1);
static K_SEM_DEFINE (data_tty_sem, 0, 1);

extern int mp_interrupt_char;
void mp_sched_keyboard_interrupt(void);
void mp_hal_signal_event(void);
//void mp_hal_wait_sem(struct k_sem *sem, uint32_t timeout_ms);

//static struct k_sem uart_sem;
#define UART_BUFSIZE 256

/** Queue for received data. */
struct k_msgq rx_q;

/** Buffer for received messages */
uint8_t rx_buf[UART_BUFSIZE];

static int rpmsg_recv_tty_callback(struct rpmsg_endpoint *ept, void *data,
				   size_t len, uint32_t src, void *priv)
{
	for (size_t i = 0; i < len; i++)
	{
		char ch = ((char*)data)[i];

		if (ch == mp_interrupt_char) {
			k_msgq_put(&rx_q, &ch, K_FOREVER);
			mp_hal_signal_event();
			mp_sched_keyboard_interrupt();
			k_yield();
			return RPMSG_SUCCESS;
		} else {
			k_msgq_put(&rx_q, &ch, K_FOREVER);
		}
	}

	// printk("%x\n", ch);
	k_yield();

	return RPMSG_SUCCESS;
}

static void platform_ipm_callback(const struct device *dev, void *context, uint32_t id, volatile void *data)
{
//	printk("msg received from mb %d: data: 0x%08x, size: %d", id, *((uint32_t *)data), 4);

	k_sem_give(&data_sem);
}

int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

//	printk("msg to send to host with id: %d", id);
	ipm_send(ipm_handle, 0, id, &id, 4);

	return 0;
}

int platform_init(void)
{
	int rsc_size;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		printk("metal_init: failed: %d", status);
		return -1;
	}

	/* declare shared memory region */
	metal_io_init(shm_io, (void*)SHM_START_ADDR, &shm_physmap, SHM_SIZE, -1, 0, NULL);

	/* declare resource table region */
	rsc_table_get(&rsc_table, &rsc_size);
	rsc_tab_physmap = (uintptr_t)rsc_table;

	metal_io_init(rsc_io, rsc_table, &rsc_tab_physmap, rsc_size, -1, 0, NULL);

	/* setup IPM */
	if (!device_is_ready(ipm_handle)) {
		printk("IPM device is not ready");
		return -1;
	}

	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	status = ipm_set_enabled(ipm_handle, 1);
	if (status) {
		printk("ipm_set_enabled failed");
		return -1;
	}

	return 0;
}

struct rpmsg_device*
platform_create_rpmsg_vdev(void)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_ID,
			rsc_table_to_vdev(rsc_table),
			rsc_io, NULL, mailbox_notify, NULL);

	if (!vdev) {
		printk("failed to create vdev");
		return NULL;
	}

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
			(void*)vring_rsc->da, rsc_io,
			vring_rsc->num, vring_rsc->align);
	if (ret) {
		printk("failed to init vring 0");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
			(void*)vring_rsc->da, rsc_io,
			vring_rsc->num, vring_rsc->align);
	if (ret) {
		printk("failed to init vring 1");
		goto failed;
	}

	ret = rpmsg_init_vdev(&rvdev, vdev, NULL, shm_io, NULL);
	if (ret) {
		printk("failed rpmsg_init_vdev");
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}

static void cleanup_system(void)
{
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
}

void zephyr_rpmsg_init(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	/* Initialize platform */
	int ret = platform_init();
	if (ret) {
		printk("Failed to initialize platform");
		ret = -1;
		goto task_end;
	}

	struct rpmsg_device *rpdev = platform_create_rpmsg_vdev();
	if (!rpdev) {
		printk("Failed to create rpmsg virtio device");
		ret = -1;
		goto task_end;
	}

	ret = rpmsg_create_ept(&tty_ept, rpdev, "rpmsg-tty",
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_tty_callback, NULL);
	if (ret < 0)
		return;

	/* start the rpmsg clients */

	k_sem_give(&data_tty_sem);

	while (1) {
		int status = k_sem_take(&data_sem, K_FOREVER);

		if (status == 0) {
			rproc_virtio_notified(rvdev.vdev, VRING1_ID);
		}
	}

task_end:
	cleanup_system();

	printk("RPMsg thread ended");
}

/*
 * Core UART functions to implement for a port
 */

// Receive single character
int mp_hal_stdin_rx_chr(void) {
//	mp_hal_wait_sem(&uart_sem, -1);
	uint8_t c;
	k_msgq_get(&rx_q, &c, K_FOREVER);
	return c;
}

// Send string of given length
mp_uint_t mp_hal_stdout_tx_strn(const char *str, mp_uint_t len)
{
	int ret = rpmsg_send(&tty_ept, str, (int)len);
	if (ret < 0)
		return ret;

	return len;
}

int mp_console_init(void)
{
//	k_sem_init(&uart_sem, 0, UINT_MAX);

	k_msgq_init(&rx_q, (char*)rx_buf, sizeof(uint8_t), UART_BUFSIZE);

	k_thread_create(&thread_mng_data, thread_mng_stack, APP_TASK_STACK_SIZE,
			zephyr_rpmsg_init,
			NULL, NULL, NULL, 8, 0, K_NO_WAIT);

	k_sem_take(&data_tty_sem, K_FOREVER);

//	printk("starting real_main\n");

	return 0;
}
