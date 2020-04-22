/*
 * vpu.c
 *
 * linux device driver for VPU.
 *
 * Copyright (C) 2006 - 2013  CHIPS&MEDIA INC.
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2.1 of the License, or (at your option)
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>

#include <uapi/video/rcm-vpu-coda980-config.h>

#define LOG_TAG "RCM_VPU_DRV"
#include <uapi/video/rcm-vpu-coda980.h>


/* definitions to be changed as customer  configuration */
/* if you want to have clock gating scheme frame by frame */
/* #define VPU_SUPPORT_CLOCK_CONTROL */

/* if the driver want to use interrupt service from kernel ISR */
#define VPU_SUPPORT_ISR
#ifdef VPU_SUPPORT_ISR
/* if the driver want to disable and enable IRQ whenever interrupt asserted. */
//#define VPU_IRQ_CONTROL
#endif

/* if the platform driver knows the name of this driver */
/* VPU_PLATFORM_DEVICE_NAME */

#define VPU_SUPPORT_PLATFORM_DRIVER_REGISTER

/* if this driver knows the dedicated video memory address */
// #define VPU_SUPPORT_RESERVED_VIDEO_MEMORY

#define VPU_PLATFORM_DEVICE_NAME "vdec"
#define VPU_CLK_NAME "vcodec"
#define VPU_DEV_NAME "vpu"

/* if the platform driver knows this driver */
/* the definition of VPU_REG_BASE_ADDR and VPU_REG_SIZE are not meaningful */

#define VPU_REG_BASE_ADDR 0x75000000
#define VPU_REG_SIZE (0x4000*MAX_NUM_VPU_CORE)

#ifdef VPU_SUPPORT_ISR
#define VPU_IRQ_NUM (23+32)
#endif

/* this definition is only for chipsnmedia FPGA board env */
/* so for SOC env of customers can be ignored */


#ifndef VM_RESERVED	/*for kernel up to 3.7.0 version*/
# define  VM_RESERVED   (VM_DONTEXPAND | VM_DONTDUMP)
#endif




typedef struct vpu_drv_context_t {
	struct fasync_struct *async_queue;
    u32 open_count;                     /*!<< device reference count. Not instance count */
} vpu_drv_context_t;

/* To track the allocated memory buffer */
typedef struct vpudrv_buffer_pool_t {
	struct list_head list;
	struct vpudrv_buffer_t vb;
	struct file *filp;
} vpudrv_buffer_pool_t;

/* To track the instance index and buffer in instance pool */
typedef struct vpudrv_instanace_list_t {
	struct list_head list;
	unsigned long inst_idx;
	unsigned long core_idx;
	struct file *filp;
} vpudrv_instanace_list_t;

#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY

#include "rcm-vpu-coda980-vmm.h"
static video_mm_t s_vmem;
static vpudrv_buffer_t s_video_memory = {0};
#else
#endif /*VPU_SUPPORT_RESERVED_VIDEO_MEMORY*/

typedef struct vpudrv_instance_pool_t {
	unsigned char codecInstPool[MAX_NUM_INSTANCE][MAX_INST_HANDLE_SIZE];	
	int vpu_instance_num;	
} vpudrv_instance_pool_t;

static int vpu_hw_reset(void);

#if defined(CONFIG_PM) || defined(VPU_SUPPORT_CLOCK_CONTROL)
static void vpu_clk_disable(struct clk *clk);
#endif

static int vpu_clk_enable(struct clk *clk);
static struct clk *vpu_clk_get(struct device *dev);
static void vpu_clk_put(struct clk *clk);

static struct platform_device *g_pdev;

/* end customer definition */
static vpudrv_buffer_t s_instance_pool = {0};
static vpudrv_buffer_t s_common_memory = {0};
static vpu_drv_context_t s_vpu_drv_context;
static dev_t s_vpu_major;
static struct cdev s_vpu_cdev;

static struct clk *s_vpu_clk;

static int s_vpu_open_ref_count;

#ifdef VPU_SUPPORT_ISR
static int s_vpu_irq = VPU_IRQ_NUM;
#endif

static phys_addr_t s_vpu_reg_phy_addr = VPU_REG_BASE_ADDR;

static void __iomem *s_vpu_reg_virt_addr;

static int s_interrupt_flag;
static wait_queue_head_t s_interrupt_wait_q;

static spinlock_t s_vpu_lock = __SPIN_LOCK_UNLOCKED(s_vpu_lock);
static DEFINE_SEMAPHORE(s_vpu_sem);
static struct list_head s_vbp_head = LIST_HEAD_INIT(s_vbp_head);
static struct list_head s_inst_list_head = LIST_HEAD_INIT(s_inst_list_head);


static vpu_bit_firmware_info_t s_bit_firmware_info[MAX_NUM_VPU_CORE];


#define BIT_BASE                    0x0000
#define BIT_CODE_RUN                (BIT_BASE + 0x000)
#define BIT_CODE_DOWN               (BIT_BASE + 0x004)
#define BIT_INT_CLEAR               (BIT_BASE + 0x00C)
#define BIT_INT_STS                 (BIT_BASE + 0x010)
#define BIT_CODE_RESET				(BIT_BASE + 0x014)
#define BIT_BUSY_FLAG               (BIT_BASE + 0x160)
#define BIT_RUN_COMMAND             (BIT_BASE + 0x164)
#define BIT_RUN_INDEX               (BIT_BASE + 0x168)
#define BIT_RUN_COD_STD             (BIT_BASE + 0x16C)
#define BIT_CUR_PC                  (BIT_BASE + 0x018)
#define BIT_INT_REASON              (BIT_BASE + 0x174)
#define VPU_PRODUCT_CODE_REGISTER   (BIT_BASE + 0x1044)

#ifdef CONFIG_PM
/* implement to power management functions */
static u32	s_vpu_reg_store[MAX_NUM_VPU_CORE][64];
static int vpu_resume(struct platform_device *pdev);
static int vpu_suspend(struct platform_device *pdev, pm_message_t state);
#endif

#define ReadVpuRegister(addr) ioread32(s_vpu_reg_virt_addr + s_bit_firmware_info[core].reg_base_offset + addr)
#define WriteVpuRegister(addr, val) iowrite32(val, s_vpu_reg_virt_addr + s_bit_firmware_info[core].reg_base_offset + addr)



static int vpu_alloc_dma_buffer(vpudrv_buffer_t *vb)
{
	if (!vb)
		return -1;

#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
	vb->phys_addr = (unsigned long)vmem_alloc(&s_vmem, vb->size, 0);
	if ((unsigned long)vb->phys_addr  == (unsigned long)-1) {
		vpu_loge("reserved Physical memory allocation error size=%d, base_addr=0x%x, mem_size=%d\n", vb->size, (int)s_vmem.base_addr, (int)s_vmem.mem_size);
		return -1;
	}

	vb->base = (unsigned long)(s_video_memory.base + (vb->phys_addr - s_video_memory.phys_addr));
#else
	{
		dma_addr_t dma_addr;
		vb->base = (unsigned long)dma_alloc_coherent(&g_pdev->dev, PAGE_ALIGN(vb->size), &dma_addr, GFP_DMA | GFP_KERNEL);
		vb->phys_addr = dma_addr;
		if ((void *)(vb->base) == NULL)	{
			vpu_loge("dynamic Physical memory allocation error size=%d\n", vb->size);
			return -1;
		}
	}
#endif

	return 0;
}

static void vpu_free_dma_buffer(vpudrv_buffer_t *vb)
{
	if (!vb)
		return;

#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
	if (vb->base)
		vmem_free(&s_vmem, vb->phys_addr, 0);
#else
	if (vb->base)
		dma_free_coherent(&g_pdev->dev, PAGE_ALIGN(vb->size), (void *)vb->base, vb->phys_addr);
#endif

}

static int vpu_free_instances(struct file *filp)
{
	vpudrv_instanace_list_t *vil, *n;
	vpudrv_instance_pool_t *vip;
	unsigned char *vip_base;
	int instance_pool_size_per_core;
	unsigned char *vdi_mutexes_base;
	const int PTHREAD_MUTEX_T_DESTROY_VALUE = 0xdead10cc;

	vpu_logd("vpu_free_instances inter. sizeof(vpudrv_instance_pool_t)=%d \n", sizeof(vpudrv_instance_pool_t));

	instance_pool_size_per_core = (s_instance_pool.size/MAX_NUM_VPU_CORE); /* s_instance_pool.size  assigned to the size of all core once call VDI_IOCTL_GET_INSTANCE_POOL by user. */

	list_for_each_entry_safe(vil, n, &s_inst_list_head, list)
	{
		if (vil->filp == filp) {
			s_vpu_open_ref_count--;
			vip_base = (unsigned char *)(s_instance_pool.base + (instance_pool_size_per_core*vil->core_idx));
			vpu_logd("vpu_free_instances detect instance crash\n");
			vpu_logd("instIdx=%d, coreIdx=%d, vip_base=%p, instance_pool_size_per_core=%d\n", (int)vil->inst_idx, (int)vil->core_idx, vip_base, (int)instance_pool_size_per_core);
			vip = (vpudrv_instance_pool_t *)vip_base;
			if (vip) {
				memset(&vip->codecInstPool[vil->inst_idx], 0x00, 4);	/* only first 4 byte is key point to free the corresponding instance. */
				vip->vpu_instance_num = s_vpu_open_ref_count;
#define PTHREAD_MUTEX_T_HANDLE_SIZE 4
				vdi_mutexes_base = (unsigned char *)(vip_base + (instance_pool_size_per_core - PTHREAD_MUTEX_T_HANDLE_SIZE*4));
				vpu_logd("vpu_free_instances : force to destroy vdi_mutexes_base=%p in userspace, vip->vpu_instance_num=%d\n",
				        vdi_mutexes_base, vip->vpu_instance_num);
				if (vdi_mutexes_base) {
					int i;
					for (i=0; i < 4; i++) {
						memcpy(vdi_mutexes_base, &PTHREAD_MUTEX_T_DESTROY_VALUE, PTHREAD_MUTEX_T_HANDLE_SIZE);
						vdi_mutexes_base += PTHREAD_MUTEX_T_HANDLE_SIZE;
					}
				}
			}
			list_del(&vil->list);
			kfree(vil);
		}
	}

	return 1;
}

static int vpu_free_buffers(struct file *filp)
{
	vpudrv_buffer_pool_t *pool, *n;
	vpudrv_buffer_t vb;

	vpu_logd("vpu_free_buffers\n");

	list_for_each_entry_safe(pool, n, &s_vbp_head, list)
	{
		if (pool->filp == filp) {
			vb = pool->vb;
			if (vb.base) {
				vpu_free_dma_buffer(&vb);
				list_del(&pool->list);
				kfree(pool);
			}
		}

	}

	return 0;
}

static irqreturn_t vpu_irq_handler(int irq, void *dev_id)
{
	vpu_drv_context_t *dev = (vpu_drv_context_t *)dev_id;

	/* this can be removed. it also work in VPU_WaitInterrupt of API function */
	int core;
#ifdef VPU_IRQ_CONTROL
	disable_irq_nosync(s_vpu_irq);
#endif

	//vpu_logi("[VPUDRV] vpu_irq_handler\n");

	for (core = 0; core < MAX_NUM_VPU_CORE; core++)	{
		if (s_bit_firmware_info[core].size == 0) /*it means that we didn't get an information the current core from API layer. No core activated.*/
			continue;

		if (ReadVpuRegister(BIT_INT_STS))
		{
            //vpu_logd("BIT_INT_STS. int reason=0x%x", ReadVpuRegister(BIT_INT_REASON));
			WriteVpuRegister(BIT_INT_CLEAR, 0x1);
		}
	}

	if (dev->async_queue)
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);	/* notify the interrupt to user space */

	s_interrupt_flag = 1;

	wake_up_interruptible(&s_interrupt_wait_q);
	return IRQ_HANDLED;
}

static int vpu_open(struct inode *inode, struct file *filp)
{
	spin_lock(&s_vpu_lock);
	vpu_logi("[VPUDRV] vpu_open\n");

    s_vpu_drv_context.open_count++;
	filp->private_data = (void *)(&s_vpu_drv_context);
	spin_unlock(&s_vpu_lock);

	return 0;
}

/*static int vpu_ioctl(struct inode *inode, struct file *filp, u_int cmd, u_long arg) // for kernel 2.6.9 of C&M*/
static long vpu_ioctl(struct file *filp, u_int cmd, u_long arg)
{
	int ret = 0;
	switch (cmd) {
	case VDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY:
		{
			vpudrv_buffer_pool_t *vbp;

			down(&s_vpu_sem);

			vbp = kzalloc(sizeof(*vbp), GFP_KERNEL);
			if (!vbp) {
				up(&s_vpu_sem);
				return -ENOMEM;
			}

			ret = copy_from_user(&(vbp->vb), (vpudrv_buffer_t *)arg, sizeof(vpudrv_buffer_t));
			if (ret) {
				kfree(vbp);
				up(&s_vpu_sem);
				return -EFAULT;
			}

			ret = vpu_alloc_dma_buffer(&(vbp->vb));
			if (ret == -1) {
				ret = -ENOMEM;
				kfree(vbp);
				up(&s_vpu_sem);
				break;
			}
			ret = copy_to_user((void __user *)arg, &(vbp->vb), sizeof(vpudrv_buffer_t));
			if (ret) {
				kfree(vbp);
				ret = -EFAULT;
				up(&s_vpu_sem);
				break;
			}

			vbp->filp = filp;
			spin_lock(&s_vpu_lock);
			list_add(&vbp->list, &s_vbp_head);
			spin_unlock(&s_vpu_lock);

			up(&s_vpu_sem);
		}
		break;
	case VDI_IOCTL_FREE_PHYSICALMEMORY:
		{
			vpudrv_buffer_pool_t *vbp, *n;
			vpudrv_buffer_t vb;

			down(&s_vpu_sem);

			ret = copy_from_user(&vb, (vpudrv_buffer_t *)arg, sizeof(vpudrv_buffer_t));
			if (ret) {
				up(&s_vpu_sem);
				return -EACCES;
			}

			if (vb.base)
				vpu_free_dma_buffer(&vb);

			spin_lock(&s_vpu_lock);
			list_for_each_entry_safe(vbp, n, &s_vbp_head, list)
			{
				if (vbp->vb.base == vb.base) {
					list_del(&vbp->list);
					kfree(vbp);
					break;
				}
			}
			spin_unlock(&s_vpu_lock);

			up(&s_vpu_sem);

		}
		break;
	case VDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO:
		{
#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
			if (s_video_memory.base != 0) {
				ret = copy_to_user((void __user *)arg, &s_video_memory, sizeof(vpudrv_buffer_t));
				if (ret != 0)
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}
#endif
		}
		break;

	case VDI_IOCTL_WAIT_INTERRUPT:
		{
            u32 timeout = (u32) arg;

			ret = wait_event_interruptible_timeout(s_interrupt_wait_q, s_interrupt_flag != 0, msecs_to_jiffies(timeout));
			if (ret == 0) {
				ret = -ETIME;
				break;
			}

			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}

			ret = 0;
			s_interrupt_flag = 0;
#ifdef VPU_IRQ_CONTROL
			enable_irq(s_vpu_irq);
#endif
		}
		break;

	case VDI_IOCTL_SET_CLOCK_GATE:
		{
			u32 clkgate;

			if (get_user(clkgate, (u32 __user *) arg))
				return -EFAULT;
#ifdef VPU_SUPPORT_CLOCK_CONTROL
			if (clkgate)
				vpu_clk_enable(s_vpu_clk);
			else
				vpu_clk_disable(s_vpu_clk);
#endif

		}
		break;
	case VDI_IOCTL_GET_INSTANCE_POOL:
		{
			down(&s_vpu_sem);

			if (s_instance_pool.base != 0) {
				ret = copy_to_user((void __user *)arg, &s_instance_pool, sizeof(vpudrv_buffer_t));
				if (ret != 0)
					ret = -EFAULT;
			} else {
				ret = copy_from_user(&s_instance_pool, (vpudrv_buffer_t *)arg, sizeof(vpudrv_buffer_t));
				if (ret == 0) {
                    if (vpu_alloc_dma_buffer(&s_instance_pool) != -1)
					{
						memset((void *)s_instance_pool.base, 0x0, s_instance_pool.size); /*clearing memory*/
						ret = copy_to_user((void __user *)arg, &s_instance_pool, sizeof(vpudrv_buffer_t));
					    if (ret == 0) {
							/* success to get memory for instance pool */
							up(&s_vpu_sem);
							break;
						}
					}

				}
				ret = -EFAULT;
			}

			up(&s_vpu_sem);
		}
		break;
	case VDI_IOCTL_GET_COMMON_MEMORY:
		{
			if (s_common_memory.base != 0) {
				ret = copy_to_user((void __user *)arg, &s_common_memory, sizeof(vpudrv_buffer_t));
				if (ret != 0)
					ret = -EFAULT;
			} else {
				ret = copy_from_user(&s_common_memory, (vpudrv_buffer_t *)arg, sizeof(vpudrv_buffer_t));
				if (ret == 0) {
					if (vpu_alloc_dma_buffer(&s_common_memory) != -1) {
						ret = copy_to_user((void __user *)arg, &s_common_memory, sizeof(vpudrv_buffer_t));
					    if (ret == 0) {
							/* success to get memory for common memory */
							break;
						}
					}
				}

				ret = -EFAULT;
			}
		}
		break;
	case VDI_IOCTL_OPEN_INSTANCE:
		{
			vpudrv_inst_info_t inst_info;
			vpudrv_instanace_list_t *vil, *n;

			vil = kzalloc(sizeof(*vil), GFP_KERNEL);
			if (!vil)
				return -ENOMEM;

			if (copy_from_user(&inst_info, (vpudrv_inst_info_t *)arg, sizeof(vpudrv_inst_info_t))) {
				kfree(vil);
				return -EFAULT;
			}

			vil->inst_idx = inst_info.inst_idx;
			vil->core_idx = inst_info.core_idx;
			vil->filp = filp;
			spin_lock(&s_vpu_lock);
			list_add(&vil->list, &s_inst_list_head);

			 inst_info.inst_open_count = 0; /* counting the current open instance number */
			list_for_each_entry_safe(vil, n, &s_inst_list_head, list)
			{
				if (vil->core_idx == inst_info.core_idx)
					inst_info.inst_open_count++;
			}
			spin_unlock(&s_vpu_lock);
			s_vpu_open_ref_count++; /* flag just for that vpu is in opened or closed */

			if (copy_to_user((void __user *)arg, &inst_info, sizeof(vpudrv_inst_info_t))) {
				kfree(vil);
				return -EFAULT;
			}

			vpu_logd("[VPUDRV] VDI_IOCTL_OPEN_INSTANCE core_idx=%d, inst_idx=%d, s_vpu_open_ref_count=%d, inst_open_count=%d\n", (int)inst_info.core_idx, (int)inst_info.inst_idx, s_vpu_open_ref_count, inst_info.inst_open_count);
		}
		break;
	case VDI_IOCTL_CLOSE_INSTANCE:
		{
			vpudrv_inst_info_t inst_info;
			vpudrv_instanace_list_t *vil, *n;

			if (copy_from_user(&inst_info, (vpudrv_inst_info_t *)arg, sizeof(vpudrv_inst_info_t)))
				return -EFAULT;
			spin_lock(&s_vpu_lock);
			list_for_each_entry_safe(vil, n, &s_inst_list_head, list)
			{
				if (vil->inst_idx == inst_info.inst_idx && vil->core_idx == inst_info.core_idx) {
					list_del(&vil->list);
					kfree(vil);
					break;
				}
			}

			inst_info.inst_open_count = 0; /* counting the current open instance number */
			list_for_each_entry_safe(vil, n, &s_inst_list_head, list)
			{
				if (vil->core_idx == inst_info.core_idx)
					inst_info.inst_open_count++;
			}
			spin_unlock(&s_vpu_lock);
			s_vpu_open_ref_count--; /* flag just for that vpu is in opened or closed */

			if (copy_to_user((void __user *)arg, &inst_info, sizeof(vpudrv_inst_info_t)))
				return -EFAULT;

			vpu_logd("[VPUDRV] VDI_IOCTL_CLOSE_INSTANCE core_idx=%d, inst_idx=%d, s_vpu_open_ref_count=%d, inst_open_count=%d\n", (int)inst_info.core_idx, (int)inst_info.inst_idx, s_vpu_open_ref_count, inst_info.inst_open_count);
		}
		break;
	case VDI_IOCTL_GET_INSTANCE_NUM:
		{
			vpudrv_inst_info_t inst_info;
			vpudrv_instanace_list_t *vil, *n;

			ret = copy_from_user(&inst_info, (vpudrv_inst_info_t *)arg, sizeof(vpudrv_inst_info_t));
			if (ret != 0)
				break;

			inst_info.inst_open_count = 0;
			spin_lock(&s_vpu_lock);
			list_for_each_entry_safe(vil, n, &s_inst_list_head, list)
			{
				if (vil->core_idx == inst_info.core_idx)
					inst_info.inst_open_count++;
			}
			spin_unlock(&s_vpu_lock);
			ret = copy_to_user((void __user *)arg, &inst_info, sizeof(vpudrv_inst_info_t));

			vpu_logd("[VPUDRV] VDI_IOCTL_GET_INSTANCE_NUM core_idx=%d, inst_idx=%d, open_count=%d\n", (int)inst_info.core_idx, (int)inst_info.inst_idx, inst_info.inst_open_count);

		}
		break;
	case VDI_IOCTL_RESET:
		{
			vpu_hw_reset();
		}
		break;
	default:
		{
			printk(KERN_ERR "[VPUDRV] No such IOCTL, cmd is %d\n", cmd);
		}
		break;
	}
	return ret;
}


static ssize_t vpu_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{

	return -1;
}

static ssize_t vpu_write(struct file *filp, const char __user *buf, size_t len, loff_t *ppos)
{

	/* vpu_logi("[VPUDRV] vpu_write len=%d\n", (int)len); */
	if (!buf) {
		vpu_loge("vpu_write buf = NULL error \n");
		return -EFAULT;
	}

	if (len == sizeof(vpu_bit_firmware_info_t))	{
		vpu_bit_firmware_info_t *bit_firmware_info;

		bit_firmware_info = kzalloc(sizeof(vpu_bit_firmware_info_t), GFP_KERNEL);
		if (!bit_firmware_info) {
			vpu_loge("vpu_write  bit_firmware_info allocation error \n");
			return -EFAULT;
		}

		if (copy_from_user(bit_firmware_info, buf, len)) {
			vpu_loge("vpu_write copy_from_user error for bit_firmware_info\n");
			kfree(bit_firmware_info);
			return -EFAULT;
		}

		if (bit_firmware_info->size == sizeof(vpu_bit_firmware_info_t)) {
			vpu_logd("vpu_write set bit_firmware_info coreIdx=0x%x, reg_base_offset=0x%x size=0x%x, bit_code[0]=0x%x\n", bit_firmware_info->core_idx, (int)bit_firmware_info->reg_base_offset, bit_firmware_info->size, bit_firmware_info->bit_code[0]);

			if (bit_firmware_info->core_idx > MAX_NUM_VPU_CORE) {
				vpu_loge("vpu_write coreIdx[%d] is exceeded than MAX_NUM_VPU_CORE[%d]\n", bit_firmware_info->core_idx, MAX_NUM_VPU_CORE);
				kfree(bit_firmware_info);
				return -ENODEV;
			}

			memcpy((void *)&s_bit_firmware_info[bit_firmware_info->core_idx], bit_firmware_info, sizeof(vpu_bit_firmware_info_t));
            kfree(bit_firmware_info);
			return len;
		}

		kfree(bit_firmware_info);
	}




	return -1;
}

static int vpu_release(struct inode *inode, struct file *filp)
{
	vpu_logi("[VPUDRV] vpu_release\n");
	
	spin_lock(&s_vpu_lock);

	/* found and free the not handled buffer by user applications */
	vpu_free_buffers(filp);

	/* found and free the not closed instance by user applications */
	vpu_free_instances(filp);
    s_vpu_drv_context.open_count--;
    if (s_vpu_drv_context.open_count == 0) {
	// [***] VPU processors must be stopped first
	// because the firmware code is into the common memory buffer
	{
		int core;
		for (core = 0; core < MAX_NUM_VPU_CORE; core++) {
			if (s_bit_firmware_info[core].size == 0)
				continue;
			WriteVpuRegister(BIT_CODE_RUN, 0);
			WriteVpuRegister(BIT_BUSY_FLAG, 1);
			WriteVpuRegister(BIT_CODE_RESET, 1);
		}
	}
	// [***]
        if (s_instance_pool.base) {
            vpu_logi("[VPUDRV] free instance pool\n");
            vpu_free_dma_buffer(&s_instance_pool);
            s_instance_pool.base = 0;
        }

        if (s_common_memory.base) {
            vpu_logi("[VPUDRV] free common memory\n");
            vpu_free_dma_buffer(&s_common_memory);
            s_common_memory.base = 0;
        }
    }
    vpu_logi("%s open count: %d\n", __func__, s_vpu_drv_context.open_count);

	spin_unlock(&s_vpu_lock);



	return 0;
}


static int vpu_fasync(int fd, struct file *filp, int mode)
{
	struct vpu_drv_context_t *dev = (struct vpu_drv_context_t *)filp->private_data;
	return fasync_helper(fd, filp, mode, &dev->async_queue);
}


static int vpu_map_to_register(struct file *fp, struct vm_area_struct *vm)
{
	unsigned long pfn;

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	pfn = s_vpu_reg_phy_addr >> PAGE_SHIFT;

	return remap_pfn_range(vm, vm->vm_start, pfn, vm->vm_end-vm->vm_start, vm->vm_page_prot) ? -EAGAIN : 0;
}

static int vpu_map_to_physical_memory(struct file *fp, struct vm_area_struct *vm)
{
	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_writecombine(vm->vm_page_prot);

	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff, vm->vm_end-vm->vm_start, vm->vm_page_prot) ? -EAGAIN : 0;
}
static int vpu_map_to_instance_pool_memory(struct file *fp, struct vm_area_struct *vm)
{
	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff, vm->vm_end-vm->vm_start, vm->vm_page_prot) ? -EAGAIN : 0;
}

/*!
 * @brief memory map interface for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_mmap(struct file *fp, struct vm_area_struct *vm)
{
	if (vm->vm_pgoff) {
		if (vm->vm_pgoff == (s_instance_pool.phys_addr>>PAGE_SHIFT))
			return vpu_map_to_instance_pool_memory(fp, vm);

		return vpu_map_to_physical_memory(fp, vm);
	} else {
		return vpu_map_to_register(fp, vm);
	}
}

struct file_operations vpu_fops = {
	.owner = THIS_MODULE,
	.open = vpu_open,
	.read = vpu_read,
	.write = vpu_write,
	/*.ioctl = vpu_ioctl, // for kernel 2.6.9 of C&M*/
	.unlocked_ioctl = vpu_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vpu_compat_ioctl,
#endif //CONFIG_COMPAT
	.release = vpu_release,
	.fasync = vpu_fasync,
	.mmap = vpu_mmap,
};





static int vpu_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res = NULL;

	g_pdev = pdev;

	vpu_logi("[VPUDRV] vpu_probe\n");
	if (pdev)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {/* if platform driver is implemented */
		s_vpu_reg_phy_addr = res->start;
		s_vpu_reg_virt_addr = ioremap(res->start, res->end - res->start);
		vpu_logi("[VPUDRV] : vpu base address get from platform driver physical base addr=0x%x, virtual base=0x%p\n", (unsigned)s_vpu_reg_phy_addr , s_vpu_reg_virt_addr);
	} else {
		s_vpu_reg_virt_addr = ioremap(s_vpu_reg_phy_addr, VPU_REG_SIZE);
		vpu_logi("[VPUDRV] : vpu base address get from defined value physical base addr=0x%x, virtual base=0x%p\n", (unsigned)s_vpu_reg_phy_addr, s_vpu_reg_virt_addr);

	}

	/* get the major number of the character device */
	if ((alloc_chrdev_region(&s_vpu_major, 0, 1, VPU_DEV_NAME)) < 0) {
		err = -EBUSY;
		printk(KERN_ERR "could not allocate major number\n");
		goto ERROR_PROVE_DEVICE;
	}

	/* initialize the device structure and register the device with the kernel */
	cdev_init(&s_vpu_cdev, &vpu_fops);
	if ((cdev_add(&s_vpu_cdev, s_vpu_major, 1)) < 0) {
		err = -EBUSY;
		printk(KERN_ERR "could not allocate chrdev\n");

		goto ERROR_PROVE_DEVICE;
	}

	if (pdev)
		s_vpu_clk = vpu_clk_get(&pdev->dev);
	else
		s_vpu_clk = vpu_clk_get(NULL);

	if (!s_vpu_clk)
		printk(KERN_ERR "[VPUDRV] : fail to get clock controller, but, do not treat as error, \n");
	else
		vpu_logi("[VPUDRV] : get clock controller s_vpu_clk=%p\n", s_vpu_clk);

#ifdef VPU_SUPPORT_CLOCK_CONTROL
#else
	vpu_clk_enable(s_vpu_clk);
#endif


#ifdef VPU_SUPPORT_ISR
	if (pdev)
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res) {/* if platform driver is implemented */
		s_vpu_irq = res->start;
		vpu_logi("[VPUDRV] : vpu irq number get from platform driver irq=0x%x\n", s_vpu_irq);
	} else {
		vpu_logi("[VPUDRV] : vpu irq number get from defined value irq=0x%x\n", s_vpu_irq);
	}


	err = request_irq(s_vpu_irq, vpu_irq_handler, 0, "VPU_CODEC_IRQ", (void *)(&s_vpu_drv_context));
	if (err) {
		printk(KERN_ERR "[VPUDRV] :  fail to register interrupt handler\n");
		goto ERROR_PROVE_DEVICE;
	}
#endif


#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
	{
		int ret;
		struct device_node *mem_buffer;
		struct resource res;

		mem_buffer = of_parse_phandle(pdev->dev.of_node, "memory-buffer", 0);
		if (mem_buffer == NULL) {
			printk(KERN_ERR "[VPUDRV] :  fail to get memory-buffer resource\n");
			goto ERROR_PROVE_DEVICE;
		}

		ret = of_address_to_resource(mem_buffer, 0, &res);
		if (ret != 0) {
			printk(KERN_ERR "[VPUDRV] :  failed to locate DT /reserved-memory resource\n");
			goto ERROR_PROVE_DEVICE;
		}

		s_video_memory.phys_addr = res.start;
		s_video_memory.size = res.end - res.start;
	}
	s_video_memory.base = (unsigned long)ioremap(s_video_memory.phys_addr, PAGE_ALIGN(s_video_memory.size));
	if (!s_video_memory.base) {
		printk(KERN_ERR "[VPUDRV] :  fail to remap video memory physical phys_addr=0x%08X, base=0x%08X, size=%d\n", (int)s_video_memory.phys_addr, (int)s_video_memory.base, (int)s_video_memory.size);
		goto ERROR_PROVE_DEVICE;
	}

	if (vmem_init(&s_vmem, s_video_memory.phys_addr, s_video_memory.size) < 0) {
		printk(KERN_ERR "[VPUDRV] :  fail to init vmem system\n");
		goto ERROR_PROVE_DEVICE;
	}
	vpu_logi("[VPUDRV] success to probe vpu device with reserved video memory phys_addr=0x%x, base = 0x%x\n", (int) s_video_memory.phys_addr, (int)s_video_memory.base);
#else
	vpu_logi("[VPUDRV] success to probe vpu device with non reserved video memory\n");
#endif


	return 0;


ERROR_PROVE_DEVICE:

	if (s_vpu_major)
		unregister_chrdev_region(s_vpu_major, 1);

	if (s_vpu_reg_virt_addr)
		iounmap(s_vpu_reg_virt_addr);

	return err;
}

static int vpu_remove(struct platform_device *pdev)
{
	vpu_logd("vpu_remove\n");
#ifdef VPU_SUPPORT_PLATFORM_DRIVER_REGISTER


	if (s_instance_pool.base) {
		vpu_free_dma_buffer(&s_instance_pool);
		s_instance_pool.base = 0;
	}

	if (s_common_memory.base) {
		vpu_free_dma_buffer(&s_common_memory);
		s_common_memory.base = 0;
	}


#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
	if (s_video_memory.base) {
		iounmap((void *)s_video_memory.base);
		s_video_memory.base = 0;

		vmem_exit(&s_vmem);
	}
#endif


	if (s_vpu_major > 0) {
		cdev_del(&s_vpu_cdev);
		unregister_chrdev_region(s_vpu_major, 1);
		s_vpu_major = 0;
	}

#ifdef VPU_SUPPORT_ISR
	if (s_vpu_irq)
		free_irq(s_vpu_irq, &s_vpu_drv_context);
#endif

	if (s_vpu_reg_virt_addr)
		iounmap(s_vpu_reg_virt_addr);

    vpu_clk_put(s_vpu_clk);

#endif /*VPU_SUPPORT_PLATFORM_DRIVER_REGISTER*/


	return 0;
}

#ifdef CONFIG_PM
static int vpu_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;
	int core;
	unsigned long timeout = jiffies + HZ;	/* vpu wait timeout to 1sec */

	vpu_logd("vpu_suspend\n");

    vpu_clk_enable(s_vpu_clk);

	if (s_vpu_open_ref_count > 0) {
		for (core = 0; core < MAX_NUM_VPU_CORE; core++)	{
			if (s_bit_firmware_info[core].size == 0)
				continue;
			while (ReadVpuRegister(BIT_BUSY_FLAG)) {
				if (time_after(jiffies, timeout))
					goto DONE_SUSPEND;
			}

		}

	}

	for (core = 0; core < MAX_NUM_VPU_CORE; core++) {
		if (s_bit_firmware_info[core].size == 0)
			continue;
		for (i = 0; i < 64; i++)
			s_vpu_reg_store[core][i] = ReadVpuRegister(BIT_BASE+(0x100+(i * 4)));
	}

    vpu_clk_disable(s_vpu_clk);

	return 0;

DONE_SUSPEND:

    vpu_clk_disable(s_vpu_clk);

	return -EAGAIN;

}
static int vpu_resume(struct platform_device *pdev)
{
	int i;
	int core;
	u32 val;
	unsigned long timeout = jiffies + HZ;	/* vpu wait timeout to 1sec */

	vpu_logd("vpu_resume\n");


    vpu_clk_enable(s_vpu_clk);

	for (core = 0; core < MAX_NUM_VPU_CORE; core++) {
		if (s_bit_firmware_info[core].size == 0)
			continue;

		WriteVpuRegister(BIT_CODE_RUN, 0);

		/*---- LOAD BOOT CODE*/
		for (i = 0; i < 512; i++) {
			val = s_bit_firmware_info[core].bit_code[i];
			WriteVpuRegister(BIT_CODE_DOWN, ((i << 16) | val));
		}

		for (i = 0; i < 64; i++)
			WriteVpuRegister(BIT_BASE+(0x100+(i * 4)), s_vpu_reg_store[core][i]);

		WriteVpuRegister(BIT_BUSY_FLAG, 1);
		WriteVpuRegister(BIT_CODE_RESET, 1);
		WriteVpuRegister(BIT_CODE_RUN, 1);

		while (ReadVpuRegister(BIT_BUSY_FLAG)) {
			if (time_after(jiffies, timeout))
				goto DONE_WAKEUP;
		}

	}

	if (s_vpu_open_ref_count == 0)
		vpu_clk_disable(s_vpu_clk);

DONE_WAKEUP:

    if (s_vpu_open_ref_count > 0)
      vpu_clk_enable(s_vpu_clk);

	return 0;
}
#else
#define	vpu_suspend	NULL
#define	vpu_resume	NULL
#endif				/* !CONFIG_PM */


static int vpu_hw_reset(void)
{
	vpu_logd("request vpu reset from application. \n");
	return 0;
}

static struct clk *vpu_clk_get(struct device *dev)
{
	return clk_get(dev, VPU_CLK_NAME);
}
static void vpu_clk_put(struct clk *clk)
{
	if (!(clk == NULL || IS_ERR(clk)))
	clk_put(clk);
}

static int vpu_clk_enable(struct clk *clk)
{

		if (!(clk == NULL || IS_ERR(clk))) {
		/* the bellow is for C&M EVB.*/
	    /*
		{
			struct clk *s_vpuext_clk = NULL;
			s_vpuext_clk = clk_get(NULL, "vcore");
			if (s_vpuext_clk)
			{
				vpu_logi("[VPUDRV] vcore clk=%p\n", s_vpuext_clk);
				clk_enable(s_vpuext_clk);
			}

			vpu_logi("[VPUDRV] vbus clk=%p\n", s_vpuext_clk);
			if (s_vpuext_clk)
			{
				s_vpuext_clk = clk_get(NULL, "vbus");
				clk_enable(s_vpuext_clk);
			}
		}
		*/
		/* for C&M EVB. */

		vpu_logd("[VPUDRV] vpu_clk_enable\n");
		return clk_enable(clk);
	}

	return 0;
}

#if defined(CONFIG_PM) || defined(VPU_SUPPORT_CLOCK_CONTROL)
static void vpu_clk_disable(struct clk *clk)
{
	if (!(clk == NULL || IS_ERR(clk))) {
		vpu_logd("[VPUDRV] vpu_clk_disable\n");
		clk_disable(clk);
	}
}
#endif


static const struct of_device_id rcm_vpu_coda980_of_match[] = {
	{
		.compatible = "rcm,vpu-coda980"
	},
	{ },
};
MODULE_DEVICE_TABLE(of, rcm_vpu_coda980_of_match);

static struct platform_driver vpu_driver = {
	.driver = {
		.name = VPU_PLATFORM_DEVICE_NAME,
		.of_match_table = rcm_vpu_coda980_of_match
	},
	.probe = vpu_probe,
	.remove = vpu_remove,
	.suspend = vpu_suspend,
	.resume = vpu_resume
};


static int __init vpu_init(void)
{
	int res;

	vpu_logd("vpu_init\n");


	init_waitqueue_head(&s_interrupt_wait_q);
	s_common_memory.base = 0;
	s_instance_pool.base = 0;
#ifdef VPU_SUPPORT_PLATFORM_DRIVER_REGISTER
	res = platform_driver_register(&vpu_driver);
#else
	res = platform_driver_register(&vpu_driver);
	res = vpu_probe(NULL);
#endif

	vpu_logd("end vpu_init result=0x%x\n", res);
	return res;
}

static void __exit vpu_exit(void)
{
#ifdef VPU_SUPPORT_PLATFORM_DRIVER_REGISTER
	vpu_logd("vpu_exit\n");

	platform_driver_unregister(&vpu_driver);

#else

	vpu_clk_put(s_vpu_clk);

	if (s_instance_pool.base) {
		vpu_free_dma_buffer(&s_instance_pool);
		s_instance_pool.base = 0;
	}

	if (s_common_memory.base) {
		vpu_free_dma_buffer(&s_common_memory);
		s_common_memory.base = 0;
	}

#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
	if (s_video_memory.base) {
		iounmap((void *)s_video_memory.base);
		s_video_memory.base = 0;

		vmem_exit(&s_vmem);
	}
#endif

	if (s_vpu_major > 0) {
		cdev_del(&s_vpu_cdev);
		unregister_chrdev_region(s_vpu_major, 1);
		s_vpu_major = 0;
	}

#ifdef VPU_SUPPORT_ISR
	if (s_vpu_irq)
		free_irq(s_vpu_irq, &s_vpu_drv_context);
#endif

	if (s_vpu_reg_virt_addr) {
		iounmap(s_vpu_reg_virt_addr);
		s_vpu_reg_virt_addr = (void *)0x00;
	}

#endif

	return;
}



MODULE_AUTHOR("A customer using C&M VPU, Inc.");
MODULE_DESCRIPTION("RCM VPU Coda980 linux driver");
MODULE_LICENSE("GPL");

module_init(vpu_init);
module_exit(vpu_exit);
