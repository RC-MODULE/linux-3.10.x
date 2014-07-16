/*
 * EasyNMC Framework driver
 * (c) RC Module 2014
 *
 * This driver provides a simple interface to userspace apps and 
 * is designed to be as simple as possible. 
 * Cores are registered with this framework by calling 
 *
 * easynmc_register_core() from respective platform drivers. 
 * 
 * A  
 *
 * 
 * License terms: GNU General Public License (GPL) version 2
 * Author: Andrew Andrianov <andrew@ncrmnt.org>
 */
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <generated/utsrelease.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/easynmc.h>

#define DRVNAME "easynmc"

/* TODO: Make nmc cores appear under a proper device class and fix numbering  */
static unsigned int freecoreid;
static unsigned int num_cores_registered;
 
static DEFINE_SPINLOCK(rlock);

#define DECLARE_PARAM(name, def)			\
	static int g_##name = def;			\
	module_param_named(name, g_##name, int, 0644);

DECLARE_PARAM(debug, 1);

#define dbg(format, args... )						\
	do {								\
		if(g_debug) {						\
			printk(KERN_DEBUG DRVNAME "@%d: " format, __LINE__, ##args); \
		}							\
	} while(0);


static irqreturn_t easynmc_handle_lp(int irq, void *dev_id)
{
	struct nmc_core *core = (struct nmc_core *) dev_id;	
	core->stats.irqs_recv[HOST_IRQ_LP]++;
	wake_up(&core->qh);
	return IRQ_HANDLED;
}

static irqreturn_t easynmc_handle_hp(int irq, void *dev_id)
{
	struct nmc_core *core = (struct nmc_core *) dev_id;	
	core->stats.irqs_recv[HOST_IRQ_HP]++;
	wake_up(&core->qh);
	return IRQ_HANDLED;
}

/* These ioctls are the same for both io and mem devices */
static long easynmc_ioctl(struct file *filp, unsigned int ioctl_num, unsigned long ioctl_param)
{
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	switch (ioctl_num) {
	case IOCTL_NMC3_RESET:
		if (core->started)
			return -ENOSYS; 
                /* 
		 * NMC3 doesn't like being reset more than once (facepalm.jpg) 
		 * Do not even try itm unless you want a system fuckup.
		 * This driver assumes that nmc core is untouched upon system boot
		 */  

		core->started++;
		core->reset(core);
		break;
	case IOCTL_NMC3_SEND_IRQ:
	{
		enum nmc_irq i;
		int ret = get_user(i,  (uint32_t __user *) ioctl_param);
		if (ret)
			return ret;
		if (i >= NMC_NUM_IRQS)
			return -EIO;
		core->interrupt(core, i);
		core->stats.irqs_sent[i]++; 
		break;
	}
	case IOCTL_NMC3_GET_NAME:
	{
		int ret = copy_to_user((uint32_t __user *) ioctl_param, core->name, strlen(core->name)+1);
		if (ret)
			return -EFAULT;
		break;
	}
	case IOCTL_NMC3_GET_TYPE:
	{
		int ret = copy_to_user((uint32_t __user *) ioctl_param, core->type, strlen(core->type)+1);
		if (ret)
			return -EFAULT;
		break;
	}
	case IOCTL_NMC3_GET_STATS:
	{
		DEFINE_SPINLOCK(lock);
		unsigned long flags; 
		int ret;
		spin_lock_irqsave(&lock, flags);
		ret = copy_to_user((uint32_t __user *) ioctl_param, &core->stats, sizeof(struct nmc_core_stats));
		spin_unlock_irqrestore(&lock, flags);

		if (ret)
			return -EFAULT;
		break;
	}
	case IOCTL_NMC3_RESET_STATS:
	{
		int i;
		DEFINE_SPINLOCK(lock);
		unsigned long flags; 
		spin_lock_irqsave(&lock, flags);
		for (i=0; i<NMC_NUM_IRQS; i++)
			core->stats.irqs_sent[i]=0;
		for (i=0; i<HOST_NUM_IRQS; i++)
			core->stats.irqs_recv[i]=0;
		spin_unlock_irqrestore(&lock, flags);
		break;
	}
	case IOCTL_NMC3_WAIT_IRQ:
	{
		uint32_t params[3];
		enum host_irq i;
		uint32_t timeout, tocount;
		int ret = copy_from_user(
			params, 
			(uint32_t __user *) ioctl_param, 
			sizeof(uint32_t) * 3
			);
		if (ret)
			return -EFAULT;
		i = params[0];
		timeout = params[1];
		tocount = params[2];
		if (i >= HOST_NUM_IRQS)
			return -EIO;
		ret = wait_event_interruptible_timeout(
			core->qh, 
			(tocount >= core->stats.irqs_recv[i]), 
			timeout);
		if (ret < 0)
			return ret;
		if (ret == 0) 
			return -ETIMEDOUT;
		break;
	}
	case IOCTL_NMC3_ATTACH_STDIO:
	{
		uint32_t params[2];
		uint32_t type, addr;
		int ret = copy_from_user(
			params, 
			(uint32_t __user *) ioctl_param, 
			sizeof(uint32_t) * 2
			);
		if (ret)
			return -EFAULT;
		type = params[0]; 
		addr = params[1];
		ret = 0;
		if (0 == type) {/* stdout */
			core->stdout = addr;
		} else if (1 == type) { /* stdout */
			core->stdin = addr;
		} else 
			ret = -EFAULT;
		wake_up(&core->qh);
		return ret;
	}
	}
	return 0;
}


/* 
   Internal memory operations, dump,  simple and straightforward 
   Userspace will work with this to do abs uploads
 */ 

static int imem_open(struct inode *inode, struct file *filp)
{
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	dbg("open core %d (imem io)\n", core->id);
	return 0;
}

static int imem_release(struct inode *inode, struct file *filp)
{
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	dbg("release core %d (imem io)\n", core->id);
	return 0;
}

static ssize_t imem_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	loff_t notcopied, tocopy;
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	if (*offp > core->imem_size) 
		return 0; /* Tried to read beyond end of mem */
	tocopy = min_t(unsigned long, (core->imem_size - *offp), count);
	notcopied = copy_to_user(buff, &core->imem_virt[*offp], tocopy);
	*offp += (tocopy - notcopied);
	return (tocopy - notcopied);
}

static ssize_t imem_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	loff_t notcopied, tocopy;
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	if (*offp > core->imem_size) 
		return 0; /* Tried to read beyond end of mem */
	tocopy = min_t(unsigned long, (core->imem_size - *offp), count);
	notcopied = copy_from_user(&core->imem_virt[*offp], buff, tocopy);
	*offp += (tocopy - notcopied);
	return (tocopy - notcopied);
}



static const struct vm_operations_struct mmap_mem_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys
#endif
};

static int imem_mmap(struct file * filp, struct vm_area_struct * vma)
{
	size_t size = vma->vm_end - vma->vm_start;
	unsigned long phys_start = vma->vm_pgoff << PAGE_SHIFT; 
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	
	if (!core->imem_phys) { 
		printk("easynmc: This core doesn't support imem mmapping (test core? )\n");
		return -EIO;
	}
	
	dbg("Requested mmap: start 0x%lx len 0x%lx \n", 
	    (unsigned long) phys_start, 
	    (unsigned long) size);
	
	/* Dumb bounds check */

	if ((phys_start <  (core->imem_phys)) || 
	    (phys_start >= (core->imem_phys + core->imem_size))) { 
		printk("easynmc: Requested mmap outsize imem range, fix your app!\n");
		return -EAGAIN;
	}

	if (((phys_start + core->imem_size) <  (core->imem_phys)) || 
	    ((phys_start + core->imem_size) >= (core->imem_phys + core->imem_size))) { 
		printk("easynmc: Requested mmap outsize imem range, fix your app!\n");
		return -EAGAIN;
	}

	/* 
	   Remap-pfn-range will mark the range VM_IO and VM_RESERVED 
	   Since nmc can and will modify imem, we should map it with caching off  
	*/
	
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    vma->vm_pgoff,
			    size,
			    pgprot_noncached(vma->vm_page_prot)))
		return -EAGAIN;
	return 0;
}

unsigned int imem_poll(struct file *filp, poll_table *wait)
{
	DEFINE_SPINLOCK(lock);
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	unsigned long flags;
	unsigned int mask = 0;
	uint32_t irqs_stat[HOST_NUM_IRQS];

	spin_lock_irqsave(&lock, flags);
	memcpy(irqs_stat, core->stats.irqs_sent, (sizeof(uint32_t) * HOST_NUM_IRQS));
	spin_unlock_irqrestore(&lock, flags);

	poll_wait(filp, &core->qh, wait);

	/* If stdin is attached, we do NOT report LP here */
	if ((!core->stdin) && (!core->stdout) &&irqs_stat[HOST_IRQ_LP])
		mask |= POLLIN | POLLRDNORM;    /* readable */

	if (irqs_stat[HOST_IRQ_HP])
		mask |= POLLPRI | POLLRDBAND;    /* readable */
	
	return mask;
}

/* stdio ops */ 

static int stdio_open(struct inode *inode, struct file *filp)
{
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	dbg("open core %d (stdio)\n", core->id);
	return 0;
}

static int stdio_release(struct inode *inode, struct file *filp)
{
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	dbg("release core %d (stdio)\n", core->id);
	return 0;
}

static ssize_t stdio_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	loff_t notcopied, tocopy;
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);

	if (!core->stdout) /* read nothing if no stdio configured */
		return 0;
	/* TODO */
	return 0;
}

static ssize_t stdio_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	loff_t notcopied, tocopy;
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	if (!core->stdin) /* write nothing if no stdio configured */
		return 0;
	/* TODO */
	return 0;
}

unsigned int stdio_poll(struct file *filp, poll_table *wait)
{
	return 0;
}


static struct file_operations io_ops = { 
	.unlocked_ioctl   = easynmc_ioctl,
	.read             = stdio_read,
	.open             = stdio_open,
	.write            = stdio_write,
	.release          = stdio_release,
	.poll             = stdio_poll,
};

/* 
   Polling convention:  
    POLLIN  == LP,
    POLLPRI == HP,
    POLLHUP == NMI has been sent by user app   
 */

static struct file_operations imem_ops = { 
	.open             = imem_open,
	.release          = imem_release,
	.read             = imem_read,
	.write            = imem_write,
	.mmap             = imem_mmap,
	.poll             = imem_poll,
	.unlocked_ioctl   = easynmc_ioctl,
};

#define nmc_read_reg(r) ioread32((core->imem_virt + r))


int easynmc_register_core(struct nmc_core *core) 
{
	int err = -EIO; 
	spin_lock(&rlock);
	/* TODO: Locking */

	printk("easynmc: registering core %s (%s) with id %d\n", core->name, core->type, freecoreid);
	core->id = freecoreid++;
		
	snprintf(core->devname_io,  EASYNMC_DEVNAME_LEN, "nmc%dio",  core->id);
	snprintf(core->devname_mem, EASYNMC_DEVNAME_LEN, "nmc%dmem", core->id);

	core->mdev_io.core  = core; 
	core->mdev_mem.core = core;

	core->mdev_io.mdev.minor	= MISC_DYNAMIC_MINOR;
	core->mdev_io.mdev.name	        = core->devname_io;
	core->mdev_io.mdev.fops	        = &io_ops;

	core->mdev_mem.mdev.minor	= MISC_DYNAMIC_MINOR;
	core->mdev_mem.mdev.name	= core->devname_mem;
	core->mdev_mem.mdev.fops	= &imem_ops;

	core->stdout = NULL;
	core->stdin  = NULL;

	err = misc_register(&core->mdev_io.mdev);
	if (err)
		goto error; 

	err = misc_register(&core->mdev_mem.mdev);
	if (err)
		goto free_io;


	if (!core->irqs[HOST_IRQ_HP]) {
		printk(KERN_ERR "easynmc: Missing HP Interrupt !\n");
		goto free_mem;
	}

	if (!core->irqs[HOST_IRQ_LP]) {
		printk(KERN_ERR "easynmc: Missing LP Interrupt !\n");
		goto free_mem;
	}

	err = request_irq(core->irqs[HOST_IRQ_HP], easynmc_handle_hp, 0x0, DRVNAME, core);
	if (err)
	{
		printk(KERN_ERR "easynmc: HP IRQ request failed!\n");
		goto free_mem;
	}
	
	err = request_irq(core->irqs[HOST_IRQ_LP], easynmc_handle_lp, 0x0, DRVNAME, core);
	if (err)
	{
		printk(KERN_ERR "easynmc: LP IRQ request failed!\n");
		goto free_hp_irq;
	}

	init_waitqueue_head(&core->qh);
	num_cores_registered++;
	spin_unlock(&rlock);
	return 0;	

free_hp_irq:
	free_irq(core->irqs[HOST_IRQ_HP], core);

free_mem:
	misc_deregister(&core->mdev_mem.mdev);

free_io:
	misc_deregister(&core->mdev_io.mdev);

error:
	freecoreid--;
	spin_unlock(&rlock);
	return err;

}
EXPORT_SYMBOL(easynmc_register_core);

int easynmc_deregister_core(struct nmc_core *core) 
{
	printk("easynmc: deregistering core %d\n", core->id);
	/* TODO: Actual cleanup */
	spin_lock(&rlock);
	misc_deregister(&core->mdev_io.mdev);
	misc_deregister(&core->mdev_mem.mdev);
	free_irq(core->irqs[HOST_IRQ_LP], core);
	free_irq(core->irqs[HOST_IRQ_HP], core);
	spin_unlock(&rlock);
	num_cores_registered--;
	return 0;
}
EXPORT_SYMBOL(easynmc_deregister_core);


static int __init easynmc_init(void)
{
	printk("EasyNMC Unified DSP Framework. (c) RC Module 2014\n");
	return 0;
}

static void __exit easynmc_exit(void)
{
	printk("easynmc: Bye!\n");
	BUG_ON(num_cores_registered); /* Being paranoid is sometimes good */		
}

module_init(easynmc_init);
module_exit(easynmc_exit);

MODULE_AUTHOR("Andrew Andrianov <andrew@ncrmnt.org>");
MODULE_DESCRIPTION("EasyNMC Framework driver");
MODULE_LICENSE("GPLv2");
