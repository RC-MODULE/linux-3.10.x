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
#include <linux/wait.h>
#include <generated/utsrelease.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/easynmc.h>
#include <linux/semaphore.h>

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
	core->stats.irqs_recv[NMC_IRQ_LP]++;
	core->clear_interrupt(core, NMC_IRQ_LP);
	wake_up(&core->qh);
	return IRQ_HANDLED;
}

static irqreturn_t easynmc_handle_hp(int irq, void *dev_id)
{
	struct nmc_core *core = (struct nmc_core *) dev_id;	
	core->stats.irqs_recv[NMC_IRQ_HP]++;
	core->clear_interrupt(core, NMC_IRQ_HP);
	wake_up(&core->qh);
	return IRQ_HANDLED;
}

/* Returns event number if available and increment counters
 * Return -1 if no events avail on the supplied stats struc
 */
static int next_event(struct nmc_core *core, uint32_t mask, struct nmc_core_stats *stats)
{
	int i;
	dbg("next_evt mask %x \n", mask);
	for (i=0; i<NMC_NUM_IRQS; i++)
		if ((mask & (1<<i)) && stats->irqs_recv[i] != core->stats.irqs_recv[i]) {
			stats->irqs_recv[i]++;
			return 1<<i;
		}			
	return -1; /* No events avail */
}

/* These ioctls are the same for both io and mem devices */
static long easynmc_ioctl(struct file *filp, unsigned int ioctl_num, unsigned long ioctl_param)
{
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	dbg("ioctl %d, param %lx\n", ioctl_num, ioctl_param);
	switch (ioctl_num) {
	case IOCTL_NMC3_RESET:
		if (core->started)
			return -ENOSYS; 
                /* 
		 * NMC3 doesn't like being reset more than once (facepalm.jpg) 
		 * Do not even try itm unless you want a system fuckup.
		 * This driver assumes that nmc core is untouched upon system boot
		 */  

		dbg("resetting core %s (%d)\n", core->name, core->id);
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
		core->send_interrupt(core, i);
		core->stats.irqs_sent[i]++; 


		/* Some special handling for NMI: 
		 * - Increment counter in recv.
		 * - wake the qh
		 * - Mark core as 'started'
		 */
		if (i == NMC_IRQ_NMI) {
			core->stats.irqs_recv[i]++; 
			wake_up(&core->qh);
			core->stats.started = 1; 
		}
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
	case IOCTL_NMC3_GET_IMEMSZ:
	{
		int ret = put_user(core->imem_size, (uint32_t __user *) ioctl_param);
		if (ret)
			return ret;
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
		for (i=0; i<NMC_NUM_IRQS; i++)
			core->stats.irqs_recv[i]=0;
		spin_unlock_irqrestore(&lock, flags);
		break;
	}
	case IOCTL_NMC3_POLLMARK:
	{
		/*   Store our event counters, we'll be using them 
		 *   to generate events. 
		 */
		DEFINE_SPINLOCK(lock);
		unsigned long flags; 
		spin_lock_irqsave(&lock, flags);
		core->pollstats = core->stats;
		spin_unlock_irqrestore(&lock, flags);
		break;
	}
	case IOCTL_NMC3_RESET_TOKEN:
	{
		DEFINE_SPINLOCK(lock);
		unsigned long flags; 
		struct nmc_irq_token itoken;

		int ret = copy_from_user(
			&itoken, 
			(uint32_t __user *) ioctl_param, 
			sizeof(struct nmc_irq_token)
			);

		if (ret) 
			return -EFAULT;

		spin_lock_irqsave(&lock, flags);
		itoken.stats = core->stats;
		spin_unlock_irqrestore(&lock, flags);
		itoken.id = ++core->token_id;

		ret = copy_to_user(
			(uint32_t __user *) ioctl_param, 
			&itoken, 
			sizeof(struct nmc_irq_token)
			);

		if (ret) 
			return -EFAULT;

		break;
	}
	case IOCTL_NMC3_CANCEL_BY_TOKEN:
	{
		struct nmc_irq_token itoken;
		int ret = copy_from_user(
			&itoken, 
			(uint32_t __user *) ioctl_param, 
			sizeof(struct nmc_irq_token)
			);

		if (ret) 
			return -EFAULT;

		/* TODO: 
		   The current implementation is very naive and
		   assumes that there IS a token with the ID requested.
		   If there's none - we'll just wait here the timeout time.
		   Users are adviced to use poll over synchronos wait where
		   possible for now. 
		 */

		ret = down_interruptible(&core->cancel_sem);
		if (ret) 
			return ret;

		core->cancel_id = itoken.id;

		wake_up(&core->qh);

		ret = wait_event_interruptible_timeout(
			core->qh, 
			(core->cancel_id == 0), 
			itoken.timeout * HZ / 1000);
		
		up(&core->cancel_sem);

		if (ret < 0)
			return ret;
		
		if (ret == 0) 
			return -ETIMEDOUT;
		
		break;
	}
	case IOCTL_NMC3_WAIT_ON_TOKEN:
	{
		struct nmc_irq_token itoken;
		int ret = copy_from_user(
			&itoken, 
			(uint32_t __user *) ioctl_param, 
			sizeof(struct nmc_irq_token)
			);

		if (ret)
			return -EFAULT;
		itoken.event = EASYNMC_EVT_CANCELLED;
		/* Wait for a valid event */
		ret = wait_event_interruptible_timeout(
			core->qh, 
			(core->cancel_id == itoken.id) ||
			(-1 != (itoken.event = 
				next_event(core, 
					   itoken.events_enabled, 
					   &itoken.stats))), 
			itoken.timeout * HZ / 1000);

		if (ret < 0)
			return ret;

		if (ret == 0) { 	
			dbg("timeout %d\n", itoken.timeout);
			itoken.event = EASYNMC_EVT_TIMEOUT;
		}
		/* Handle cancellation */
		if (itoken.event == EASYNMC_EVT_CANCELLED) {
			core->cancel_id = 0;
			wake_up(&core->qh);
		}

		ret = copy_to_user(
			(uint32_t __user *) ioctl_param, 
			&itoken, 
			sizeof(struct nmc_irq_token)
			);
		
		if (ret)
			return -EFAULT;
		break;
	}
	case IOCTL_NMC3_ATTACH_STDIN:
	case IOCTL_NMC3_ATTACH_STDOUT:
	{
		uint32_t addr;
		int ret = get_user(addr,  (uint32_t __user *) ioctl_param);
		if (ret || (addr > core->imem_size) ||
		    (addr + sizeof(struct nmc_stdio_channel) > core->imem_size))
			return -EFAULT;
		/* FixMe: Proper stdio buffer addressing */
		/* BUG HERE HERE HERE HERE */
		if (ioctl_num == IOCTL_NMC3_ATTACH_STDIN)
			core->stdin = addr;
		else
			core->stdout = addr; 
		wake_up(&core->qh);
		break;
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
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	size_t size = vma->vm_end - vma->vm_start;
	unsigned long phys_start = (vma->vm_pgoff << PAGE_SHIFT) + core->imem_phys; 
	vma->vm_pgoff = phys_start >> PAGE_SHIFT;
	
	if (!core->imem_phys) { 
		printk("easynmc: This core doesn't support imem mmapping (test core? )\n");
		return -EIO;
	}
	
	dbg("Requested mmap: start 0x%lx len 0x%lx \n", 
	    (unsigned long) phys_start, 
	    (unsigned long) size);
	
	/* Dumb bounds check */

	if ((phys_start <  (core->imem_phys)) || 
	    (phys_start >  (core->imem_phys + core->imem_size))) { 
		printk("easynmc: Requested mmap outsize imem range, fix your app!\n");
		return -EAGAIN;
	}

	if (((phys_start + core->imem_size) <  (core->imem_phys)) || 
	    ((phys_start + core->imem_size) >  (core->imem_phys + core->imem_size))) { 
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

static int pollflags[NMC_NUM_IRQS] = {
	POLLHUP,
	POLLPRI | POLLRDBAND,
	POLLIN  | POLLRDNORM
};

unsigned int imem_poll(struct file *filp, poll_table *wait)
{
	DEFINE_SPINLOCK(lock);
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	unsigned long flags;
	int i; 
	unsigned int mask = 0;
	poll_wait(filp, &core->qh, wait);
	spin_lock_irqsave(&lock, flags);	
	for (i=0; i<NMC_NUM_IRQS; i++) {
		if (core->stats.irqs_recv[i] != core->pollstats.irqs_recv[i]) {
			mask |= pollflags[i];
			core->pollstats.irqs_recv[i]++;
		}
	}
	spin_unlock_irqrestore(&lock, flags);
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

	core->stdout  = NULL;
	core->stdin   = NULL;

	core->token_id  = 0;
	core->cancel_id = 0;

	memset(&core->stats, 0x0, sizeof(core->stats));
	memset(&core->pollstats, 0x0, sizeof(core->pollstats));

	sema_init(&core->cancel_sem, 1);

	err = misc_register(&core->mdev_io.mdev);
	if (err)
		goto error; 

	err = misc_register(&core->mdev_mem.mdev);
	if (err)
		goto free_io;


	if (!core->irqs[NMC_IRQ_HP]) {
		printk(KERN_ERR "easynmc: Warning, missing HP Interrupt on core %s!\n", core->name);
	}

	if (!core->irqs[NMC_IRQ_LP]) {
		printk(KERN_ERR "easynmc: Warning, missing LP Interrupt on core %s!\n", core->name);
	}

	if (core->irqs[NMC_IRQ_HP]) { 
		err = request_irq(core->irqs[NMC_IRQ_HP], easynmc_handle_hp, 0x0, DRVNAME, core);
		if (err)
		{
			printk(KERN_ERR "easynmc: HP IRQ request failed!\n");
			goto free_mem;
		}
	}

	if (core->irqs[NMC_IRQ_LP]) {
		err = request_irq(core->irqs[NMC_IRQ_LP], easynmc_handle_lp, 0x0, DRVNAME, core);
		if (err)
		{
			printk(KERN_ERR "easynmc: LP IRQ request failed!\n");
			goto free_hp_irq;
		}
	}

	init_waitqueue_head(&core->qh);
	num_cores_registered++;
	spin_unlock(&rlock);
	return 0;	

free_hp_irq:
	free_irq(core->irqs[NMC_IRQ_HP], core);

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
	free_irq(core->irqs[NMC_IRQ_LP], core);
	free_irq(core->irqs[NMC_IRQ_HP], core);
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
