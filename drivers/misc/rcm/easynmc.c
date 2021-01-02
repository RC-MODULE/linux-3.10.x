/*
 * EasyNMC Framework driver
 * (c) RCM 2014
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
#include <linux/circ_buf.h>
#include <linux/proc_fs.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/dma-buf.h> // [***] EasyNMC prototype

#define DRVNAME "easynmc"

/* TODO: Make nmc cores appear under a proper device class and fix numbering  */
static unsigned int freecoreid;
static unsigned int num_cores_registered;

/* A list of struct easynmc_core */
static LIST_HEAD(core_list);

static DEFINE_SPINLOCK(rlock);

#define IRQ_POLLING_SLEEP_MIN 9000
#define IRQ_POLLING_SLEEP_MAX 11000

#define DECLARE_PARAM(name, def)			\
	static int g_##name = def;			\
	module_param_named(name, g_##name, int, 0644);

DECLARE_PARAM(debug, 0);
DECLARE_PARAM(max_appdata_len, 4096);

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

static int easynmc_irq_polling(void* context)
{
	struct nmc_core *core = (struct nmc_core *)context;

	int mask;

	while (!kthread_should_stop()) {
		mask = core->check_interrupts(core);

		if (mask & BIT(NMC_IRQ_HP))
			easynmc_handle_hp(0, core);

		if (mask & BIT(NMC_IRQ_LP))
			easynmc_handle_lp(0, core);

		usleep_range(IRQ_POLLING_SLEEP_MIN, IRQ_POLLING_SLEEP_MAX);
	}

	return 0;
}

static void appdata_drop(struct nmc_core *core)
{
	if (core->appdata) { 
		kfree(core->appdata);
		core->appdata = NULL;
		core->appdata_len = 0;
	}
	dbg("Dropping current appdata\n");
}

static int appdata_set(struct nmc_core *core, void __user *buf, size_t len)
{
	int ret = 0;
	appdata_drop(core);
	if (len) { 
		core->appdata = kmalloc(len, GFP_KERNEL);
		if (!core->appdata) 
			return -ENOMEM;
		core->appdata_len = len;
		ret = copy_from_user(core->appdata, buf, len);
		if (ret)
			core->appdata_len = 0;
	}
	return ret; 
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

static void easynmc_send_irq(struct nmc_core *core, enum nmc_irq i)
{
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

	case IOCTL_NMC3_NMI_ON_CLOSE:
	{
		int i;
		int ret = get_user(i,  (int __user *) ioctl_param);
		if (ret)
			return ret;		
		core->nmi_on_close = i;
		dbg("core->nmi_on_close == %d\n", i);
		break;
	}
	case IOCTL_NMC3_SEND_IRQ:
	{
		enum nmc_irq i;
		int ret = get_user(i,  (int __user *) ioctl_param);
		if (ret)
			return ret;
		if (i >= NMC_NUM_IRQS)
			return -EIO;
		
		easynmc_send_irq(core, i);

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
			itoken.event = EASYNMC_EVT_CANCELLED;


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
		struct nmc_stdio_channel * chan;
		int ret = get_user(addr,  (uint32_t __user *) ioctl_param);
		
		if (ret || (addr > core->imem_size) ||
		    (addr + sizeof(struct nmc_stdio_channel) > core->imem_size))
			return -EFAULT;

		chan = (struct nmc_stdio_channel *) &core->imem_virt[addr];

		if ((le32_to_cpu(chan->size) & (le32_to_cpu(chan->size) - 1)) != 0) {
			printk("easynmc: IO buffer size not power of 2, not attaching\n");
			return -EIO;
		}

		if (ioctl_num == IOCTL_NMC3_ATTACH_STDIN)
			core->stdin  = chan;
		else
			core->stdout  = chan;
		
		dbg("Attached %s length %d words", 
		    (ioctl_num == IOCTL_NMC3_ATTACH_STDIN) ? "stdin" : "stdout",
		     le32_to_cpu(chan->size));
		chan->isr_on_io = 1; /* Enable interrupt transport */

		/* Wake up. We may have data in there already! */
		wake_up(&core->qh);
		break;
	}
	case IOCTL_NMC3_REFORMAT_STDIN:
	case IOCTL_NMC3_REFORMAT_STDOUT:
	{
		uint32_t r;
		int ret = get_user(r,  (uint32_t __user *) ioctl_param);
		if (ret)
			return -EFAULT;
		
		if (ioctl_num == IOCTL_NMC3_REFORMAT_STDIN)
			core->reformat_stdin  = r;
		else
			core->reformat_stdout  = r;
		break;
	}

	case IOCTL_NMC3_GET_APPDATA:
	{		
		struct nmc_ioctl_buffer ibuf;
		size_t len;
		int ret = copy_from_user(
			&ibuf, 
			(void __user *) ioctl_param, 
			sizeof(struct nmc_ioctl_buffer)
			);

		if (ret)
			return -EFAULT;
		
		len = min_t(size_t, core->appdata_len, ibuf.len);
		if (len) 
			ret = copy_to_user((void __user *) ibuf.data, core->appdata, len);

		if (ret)
			return -EFAULT;

		/* Set actual length in buffer */
		ibuf.len = len;
		
		ret = copy_to_user(
			(void __user *) ioctl_param,
			&ibuf,
			sizeof(struct nmc_ioctl_buffer)
			);
		
		if (ret)
			return -EFAULT;
		
		break;
	}

	case IOCTL_NMC3_SET_APPDATA:
	{
		struct nmc_ioctl_buffer ibuf;
		int ret = copy_from_user(
			&ibuf, 
			(void __user *) ioctl_param, 
			sizeof(struct nmc_ioctl_buffer)
			);
		if (ret)
			return -EFAULT;

		if (ibuf.len >= g_max_appdata_len) { 
			printk(KERN_INFO "easynmc: Requested appdata too big. Increase max_appdata_len\n");
			return -ENOMEM;
		}
		
		return appdata_set(core, 
				   (void __user *) ibuf.data, ibuf.len);
		
		break;
	}

// [***] EasyNMC prototype (start) FixMe!!!
#ifdef CONFIG_ION
	case IOCTL_NMC3_ION2NMC:
	{
		uint32_t fd;
		phys_addr_t paddr;
		struct dma_buf *buffer;
		struct dma_buf_attachment *attachement;
		struct sg_table *sg_tbl;
#ifdef CONFIG_TARGET_1879VM8YA
		int is_arm_core = (strcmp(core->type, "arm") == 0);
#endif		
		int ret = get_user(fd,  (uint32_t __user *) ioctl_param);
		if (ret)
			return -EFAULT;

		buffer = dma_buf_get(fd);
		if (IS_ERR(buffer))
			return PTR_ERR(buffer);

		attachement = dma_buf_attach(buffer, core->dev);
		if (IS_ERR(attachement)) {
			dma_buf_put(buffer);
			return PTR_ERR(attachement);
		}

		sg_tbl = dma_buf_map_attachment(attachement, DMA_BIDIRECTIONAL);
		if (IS_ERR(sg_tbl)) {
			dma_buf_detach(buffer, attachement);
			dma_buf_put(buffer);
			return PTR_ERR(attachement);
		}

		paddr = page_to_phys(sg_page(sg_tbl->sgl));

		dma_buf_unmap_attachment(attachement, sg_tbl, DMA_BIDIRECTIONAL);
		dma_buf_detach(buffer, attachement);
		dma_buf_put(buffer);

#ifdef CONFIG_TARGET_1879VM8YA
		if (is_arm_core)
			ret = put_user((uint32_t)(paddr), (uint32_t __user *) ioctl_param);
		else
			ret = put_user((uint32_t)(paddr >> 2), (uint32_t __user *) ioctl_param);
#elif defined CONFIG_1888TX018
		ret = put_user((uint32_t)((paddr >> 2) + 0x10000000), (uint32_t __user *) ioctl_param);
#else
		ret = put_user((uint32_t)(paddr >> 2), (uint32_t __user *) ioctl_param);		
#endif
		if (ret)
			return -EFAULT;
		break;
	}
#endif
// [***] EasyNMC prototype (end)

	default:
		return -ENOTTY;

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
	dbg("release core %d (imem io) %d \n", core->id, core->nmi_on_close);
	if (core->stats.started && core->nmi_on_close) { 
		easynmc_send_irq(core, NMC_IRQ_NMI);
	}
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
	phys_addr_t phys_start = (vma->vm_pgoff << PAGE_SHIFT) + core->imem_phys; 
	vma->vm_pgoff = phys_start >> PAGE_SHIFT;
	
	if (!core->imem_phys) { 
		printk("easynmc: This core doesn't support imem mmapping (test core? )\n");
		return -EIO;
	}
	
	dbg("Requested mmap: start 0x%llx len 0x%lx \n", 
	    (long long unsigned int)phys_start, 
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
//	POLLPRI | POLLRDBAND, // [***] EasyNMC prototype
	POLLOUT | POLLWRNORM, // [***] EasyNMC prototype
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

static size_t copy_from_nmc_to_user(unsigned char __user* to, const uint32_t* from, size_t len, int reformat)
{
	int i;
	size_t ret=0;
	if (!reformat)
		return (copy_to_user(to, from, len * sizeof(uint32_t)) >> 2);
	
	for (i = 0; i < len; i++) {
		unsigned char data = (unsigned char) (le32_to_cpu(from[i]) & 0xFF);
		if (0 > put_user(data, to++))
			ret++;
	}
	
	return ret;
}

static size_t copy_from_user_to_nmc(uint32_t* to, const unsigned char __user* from, size_t len, int reformat)
{
	int i;
	size_t ret=0;
	if (!reformat)
		return (copy_from_user(to, from, len * sizeof(uint32_t)) >> 2);
	
	for (i = 0; i < len; i++) {
		unsigned char data;
		if (0 > get_user(data, from++))
			ret++;
		else
			to[i] = cpu_to_le32((uint32_t) data);		
	}

	return ret;
}


static ssize_t stdio_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t notcopied, tocopy, copied;
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);

	if (!core->stdout) /* read nothing if no stdio configured */
		return 0;

	copied = 0;
	notcopied = 0;
	while (count && (notcopied==0)) { 
		size_t processed; 
		size_t occupied = CIRC_CNT_TO_END(le32_to_cpu(core->stdout->head), 
						  le32_to_cpu(core->stdout->tail), 
						  le32_to_cpu(core->stdout->size));
		if (!core->reformat_stdout) 
			occupied = occupied << 2; /* If no reformatting 4x more data! */
		
		tocopy = min_t(size_t, occupied, count);

		if (0 == tocopy) { 
			if (copied || (filp->f_flags & O_NONBLOCK)) {
				/* Return what we've read so far */
				break;
			} else {
				int ret; 
				/* If nothing read, block until something's avail */
				ret = wait_event_interruptible(
					core->qh, 
					CIRC_CNT(le32_to_cpu(core->stdout->head), 
						 le32_to_cpu(core->stdout->tail),
						 le32_to_cpu(core->stdout->size))

					);
				if (ret < 0) 
					break;
			};	
		};

		notcopied = copy_from_nmc_to_user(
			&buff[copied],
			&core->stdout->data[le32_to_cpu(core->stdout->tail)],
			tocopy,
			core->reformat_stdout
			);

		/* FixMe: Looks way too many maths, have a fresh look here sometime later. */
		processed = tocopy - notcopied;
		count  -= processed; 
		copied += processed;
		core->stdout->tail = cpu_to_le32( le32_to_cpu(core->stdout->tail) + processed);
		core->stdout->tail = cpu_to_le32( le32_to_cpu(core->stdout->tail) & (le32_to_cpu(core->stdout->size) - 1));
	}
	
	*offp += copied;

	if (!copied && (filp->f_flags & O_NONBLOCK))
		return -EAGAIN;

	return copied;
}

static ssize_t stdio_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	size_t notcopied, tocopy, copied;
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);

	if (!core->stdin) /* read nothing if no stdio configured */
		return 0;

	copied = 0;
	notcopied = 0;
	while (count && (notcopied==0)) { 
		size_t processed; 
		size_t numfree = CIRC_SPACE_TO_END(le32_to_cpu(core->stdin->head), 
						  le32_to_cpu(core->stdin->tail), 
						  le32_to_cpu(core->stdin->size));
		if (!core->reformat_stdin) 
			numfree = numfree << 2; /* If no reformatting 4x more data! */
		
		tocopy = min_t(size_t, numfree, count);

		if (0 == tocopy) { 
			if (copied || (filp->f_flags & O_NONBLOCK)) {
				/* Return what we've written so far */
				break;
			} else {
				int ret; 
				/* If nothing can be written, block until we write more */
				ret = wait_event_interruptible(
					core->qh, 
					CIRC_SPACE(le32_to_cpu(core->stdin->head), 
						   le32_to_cpu(core->stdin->tail),
						   le32_to_cpu(core->stdin->size))
					);
				if (ret < 0) 
					break;
				continue;
			};	
		};

		notcopied = copy_from_user_to_nmc(
			&core->stdin->data[le32_to_cpu(core->stdin->head)],
			&buff[copied],
			tocopy,
			core->reformat_stdin
			);

		/* FixMe: Looks way too many maths, have a fresh look here sometime later. */
		processed = tocopy - notcopied;
		count  -= processed; 
		copied += processed;
		core->stdin->head = cpu_to_le32(le32_to_cpu(core->stdin->head) + processed);
		core->stdin->head = cpu_to_le32(le32_to_cpu(core->stdin->head) & (le32_to_cpu(core->stdin->size) - 1));
	}
	
	*offp += copied;

	if (!copied && (filp->f_flags & O_NONBLOCK))
		return -EAGAIN;

	return copied;
}

unsigned int stdio_poll(struct file *filp, poll_table *wait)
{
	struct nmc_core *core = EASYNMC_MISC2CORE(filp->private_data);
	unsigned int mask = 0;	

	poll_wait(filp, &core->qh, wait);

	if (core->stdout &&
	    CIRC_CNT(le32_to_cpu(core->stdout->head), 
		     le32_to_cpu(core->stdout->tail), 
		     le32_to_cpu(core->stdout->size)))
		mask|= (POLLIN | POLLRDNORM);

	if (core->stdin &&
	    CIRC_SPACE(le32_to_cpu(core->stdin->head), 
		     le32_to_cpu(core->stdin->tail), 
		     le32_to_cpu(core->stdin->size)))
		mask|= (POLLOUT | POLLWRNORM);

	return mask;
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
	list_add_tail(&core->linkage, &core_list);

	/* TODO: 
	 *  Dynamically allocate and register devices whenever stdio and the rest
	 *  are attached.
	 */
	   
	snprintf(core->devname_io,  EASYNMC_DEVNAME_LEN, "nmc%dio",  core->id);
	snprintf(core->devname_mem, EASYNMC_DEVNAME_LEN, "nmc%d",    core->id);

	core->mdev_io.core  = core; 
	core->mdev_mem.core = core;

	/* Initialize app data with zero values */ 
	core->appdata     = NULL;
	core->appdata_len = 0;
		
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
	/* Default to nmi-on-close */
	core->nmi_on_close = 1;

	memset(&core->stats, 0x0, sizeof(core->stats));
	memset(&core->pollstats, 0x0, sizeof(core->pollstats));

	sema_init(&core->cancel_sem, 1);

	err = misc_register(&core->mdev_io.mdev);
	if (err)
		goto error; 

	err = misc_register(&core->mdev_mem.mdev);
	if (err)
		goto free_io;

	if (core->do_irq_polling) {
		core->thread_irq_polling = kthread_run(easynmc_irq_polling, core, "easynmc_irq_polling");

		if (IS_ERR(core->thread_irq_polling))
		{
			printk(KERN_ERR "easynmc: Failed to create IRQ polling thread\n");
			goto free_mem;
		}
	}
	else {
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
	list_del_init(&core->linkage);
	misc_deregister(&core->mdev_io.mdev);
	misc_deregister(&core->mdev_mem.mdev);
	if (core->thread_irq_polling) {
		kthread_stop(core->thread_irq_polling);
		core->thread_irq_polling = NULL;
	}
	free_irq(core->irqs[NMC_IRQ_LP], core);
	free_irq(core->irqs[NMC_IRQ_HP], core);
	num_cores_registered--;
	spin_unlock(&rlock);
	return 0;
}
EXPORT_SYMBOL(easynmc_deregister_core);



/* ProcFS stuff */

static ssize_t proc_read(struct file *filp, char __user *buffer, size_t buffer_length, loff_t *offset)
{
	struct list_head *iter; 
	int copied=0;
	struct nmc_core *core;
	int len;
	char name_buf[32];

	/* 
	 * We give all of our information in one go, so if the
	 * user asks us if we have more information the
	 * answer should always be no.
	 *
	 * This is important because the standard read
	 * function from the library would continue to issue
	 * the read system call until the kernel replies
	 * that it has no more information, or until its
	 * buffer is filled.
	 */

	if (*offset > 0)
		return 0;
	
	/* Lock us against any core removal that might happen */
	spin_lock(&rlock);
	list_for_each(iter, &core_list) {
		core = list_entry(iter, struct nmc_core, linkage);
		len = snprintf(name_buf, sizeof name_buf, "/dev/nmc%d\n", core->id);
		if ((len >= 0)  && (len <= buffer_length)) {
			if (copy_to_user(&buffer[copied], name_buf, len)) {
				spin_unlock(&rlock);
				return -EFAULT;
			}
			copied += len;
			buffer_length -= len;
			*offset += len;
		}
	}	
	spin_unlock(&rlock);

	return copied; /* Do not return NULL byte */
}

static struct proc_dir_entry *nmc_proc_entry;
static const struct file_operations proc_fops = { /*wherefore? struct proc_ops?*/
	.read = proc_read,
	.owner = THIS_MODULE
};

static int __init easynmc_init(void)
{
	printk("EasyNMC Unified DSP Framework. (c) RCM 2018\n");
	nmc_proc_entry = proc_create("nmc", 0644, NULL, &proc_fops);
	if (nmc_proc_entry == NULL) { 
//		remove_proc_entry("nmc", &proc_root);
		printk(KERN_ALERT "Error: Could not initialize /proc/nmc\n");
		return -ENOMEM;
	}

	return 0;
}

static void __exit easynmc_exit(void)
{
	printk("easynmc: Bye!\n");
	BUG_ON(num_cores_registered); /* Being paranoid is sometimes good */		
//	remove_proc_entry("nmc", &proc_root);
}

module_init(easynmc_init);
module_exit(easynmc_exit);

MODULE_AUTHOR("Andrew Andrianov <andrew@ncrmnt.org>");
MODULE_DESCRIPTION("EasyNMC Framework driver");
MODULE_LICENSE("GPLv2");
