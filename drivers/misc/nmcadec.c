#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/nmcadec.h>
#include <linux/rcmod_soc_nmc3.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/slab.h>

#define DRIVER_NAME "nmcadec"
#define NMC3_DEVICE_NAME "NMC3_0"
#define FIRMWARE_FILENAME "nmcadec.abs"


struct nmcadec_shared {
	u32 ib_phys;
	u32 ib_size;
	u32 ib_head, ib_tail;
	u32 ob_phys;
	u32 ob_size;
	u32 ob_head, ob_tail;
	u32 chan_n;
	u32 ib_ack;

	u32 nm_area[6];
	u32 debug[8];
};

struct nmcadec_device {

	struct miscdevice mdev;
	unsigned int opened;

	struct __NMC3_thread_descriptor *desc;
	u32 entry;
	int decoder_state;
#define DECODER_STOPPED		0
#define DECODER_STARTING	1
#define DECODER_RUNNING		2
#define DECODER_FAILED		3

	struct nmcadec_shared *shared;
	dma_addr_t shared_dma;

	char *ib;
	dma_addr_t ib_dma;

	struct nmcadec_frame *ob;
	u32 *ob_ring;
	dma_addr_t ob_ring_dma;

	/* After flush, some random ob is "owned" by userspace.
	 * However, driver has to to fill ob ring from the beginning.
	 * This makes policy "put i-th ob always into i-th position of the
	 * ring" problematic.
	 *
	 * So driver maintains "next-to-provide" and "next-to-get" positions,
	 * that differ from ob ring positions.
	 * Also it maintains "still-busy" position to ensure that
	 * userspace-owned buffer will never enter ob ring.
	 */

	/* Position that we still can't provide, because it's still not
	 * freed from previous usage. Atomic_t because used in several
	 * contexts */
	atomic_t busy_pos;

	/* Position to provide next. Never increases over busy_pos.
	 *
	 * Ring position for this is ->shared->ib_head.
	 *
	 * No need for atomic_t, because used when preparing start, when
	 * pocessing tick, and when cleaning up after stop - all that is
	 * mutual-exclusive */
	unsigned int next_provide_pos;

	/* Need to transparently drop errored ob's.
	 *
	 * Can't do that at read() time, because then it could happen that
	 * poll() returns read is ready, but read() drops errored ob's and
	 * then blocks.
	 * So have to drop before poll() notices, that is - in ob_ready().
	 * To find out if ob should be dropped or not, need first to
	 * dma_unmap() it.
	 *
	 * next_unmap_pos and next_unmap_ring_pos - position and ring position
	 * to unmap and check next; advance is limited by ->shared->ob_tail.
	 *
	 * next_consume_pos - position to consume next; advance is limited by 
	 * next_unmap_pos.
	 *
	 * Since ob_ready() is called from several contexts (read(), poll(),
	 * timer tick) and is no more a simple readonly check, have to use
	 * locking to protect these fields */
	unsigned int next_unmap_pos, next_unmap_ring_pos, next_consume_pos;
	spinlock_t ob_lock;

	/* after flushes, stream offsaet as visible by userspace and by
	 * decoder start to differ. Here the difference is stored */
	u32 offset_delta;

	int written_after_last_start;

	struct timer_list timer;
	wait_queue_head_t ib_wait;
	wait_queue_head_t ob_wait;
	wait_queue_head_t flush_wait;

	/* Need to allow read() and write() to be entered in parallel,
	 * but not more than by one thread each.
	 * So have to use separate mutexes */
	struct mutex open_mutex;
	struct mutex read_mutex;
	struct mutex write_mutex;
	struct mutex read_vs_flush_mutex;
};

static struct nmcadec_device the_adec;

/* Comarisin that handles wraparounds */
#define is_less_then(a, b) (((int)(a) - (int)(b)) < 0)

static inline int ib_free_space(struct nmcadec_device *dev)
{
	return NMCADEC_IRING_BYTES -
	       (dev->shared->ib_head - dev->shared->ib_tail);
}

static int ob_ready(struct nmcadec_device *dev)
{
	int i, ri, ret;

	spin_lock(&dev->ob_lock);

	while (1) {

		if (dev->next_consume_pos != dev->next_unmap_pos) {
			ret = 1;
			break;
		}

		if (dev->next_unmap_ring_pos == dev->shared->ob_tail) {
			ret = 0;
			break;
		}

		i = dev->next_unmap_pos % NMCADEC_ORING_SIZE;
		ri = dev->next_unmap_ring_pos % NMCADEC_ORING_SIZE;

		dma_unmap_single(0, (dma_addr_t)dev->ob_ring[ri],
					sizeof(*dev->ob), DMA_FROM_DEVICE);

		dev->next_unmap_pos++;
		dev->next_unmap_ring_pos++;

		if (dev->ob[i].status == NMCADEC_STATUS_OK) {
			ret = 1;
			break;
		}

		printk(KERN_DEBUG DRIVER_NAME ": dropping broken ob\n");
		dev->next_consume_pos++;
	}

	spin_unlock(&dev->ob_lock);
	return ret;
}

static void nmcadec_provide_obs(struct nmcadec_device *dev)
{
	unsigned int i, ri;
	dma_addr_t dma;

	/* Since total number of ob's equals to ob ring size, there is
	 * always room in ob ring when next_provide_pos < busy_pos */

	while (is_less_then(dev->next_provide_pos,
			atomic_read(&dev->busy_pos))) {

		i = dev->next_provide_pos % NMCADEC_ORING_SIZE;
		ri = dev->shared->ob_head % NMCADEC_ORING_SIZE;

		dma = dma_map_single(0, &dev->ob[i],
				sizeof(*dev->ob), DMA_FROM_DEVICE);
		if (unlikely(!dma))
			break;

		dev->ob_ring[ri] = dma;
		wmb();
		dev->shared->ob_head++;
		
		dev->next_provide_pos++;
	}
}

/* Undo what was done in nmcadec_provide_obs(), for cleanup paths.
 *
 * This breaks into how ob's are manages, so care must be taken to avoid
 * breakage:
 * - this changes ob_head in way that does not match decoder's expectations
 *   => decoder must NOT be running
 * - this is the reverse of nmcadec_provide_obs() so that must not be
 *   called in parallel => tick timer must be stopped
 */
static void nmcadec_unprovide_obs(struct nmcadec_device *dev)
{
	int ri;

	while (dev->shared->ob_head != dev->shared->ob_tail) {

		dev->shared->ob_head--;
		dev->next_provide_pos--;

		ri = dev->shared->ob_head % NMCADEC_ORING_SIZE;
		dma_unmap_single(0, (dma_addr_t)dev->ob_ring[ri],
					sizeof(*dev->ob), DMA_FROM_DEVICE);
	}
}

static void nmcadec_tick(unsigned long data)
{
	struct nmcadec_device *dev = (struct nmcadec_device *)data;

	nmcadec_provide_obs(dev);

	if (ob_ready(dev))
		wake_up_interruptible_all(&dev->ob_wait);

	if (ib_free_space(dev) != 0)
		wake_up_interruptible_all(&dev->ib_wait);

	mod_timer(&dev->timer, jiffies + 1);

#if 0
	if (jiffies % (HZ / 5) == 0)
		printk("ibh %d ibt %d obh %d obt %d [0]=%08x [1]=%08x [2]=%08x [3]=%08x\n",
			dev->shared->ib_head,
			dev->shared->ib_tail,
			dev->shared->ob_head,
			dev->shared->ob_tail,
			dev->shared->debug[0], dev->shared->debug[1],
			dev->shared->debug[2], dev->shared->debug[3]);
#endif
}

static void nmcadec_decoder_terminated(struct __NMC3_thread_descriptor *desc,
		int ret)
{
	struct nmcadec_device *dev = &the_adec;

	BUG_ON(desc != dev->desc);

	printk(KERN_ERR DRIVER_NAME ": unexpected decoder termination\n");

	dev->decoder_state = DECODER_FAILED;
	wake_up_interruptible_all(&dev->ib_wait);
	wake_up_interruptible_all(&dev->ob_wait);
	wake_up_interruptible_all(&dev->flush_wait);
}

static int nmcadec_start_decoder(struct nmcadec_device *dev)
{
	int ret;

	if (dev->decoder_state != DECODER_STOPPED)
		return -EIO;

	/* If called from flush(), then below code could race against ib_ready()
	 * called from poll() - but we don't care, because it's userspace
	 * problem if they call flush() and in parallel are waiting for write */
	dev->offset_delta += dev->shared->ib_head;
	dev->shared->ib_head = dev->shared->ib_tail = 0;
	dev->shared->ib_ack = 0;

	/* At this point, ob_head and ob_tail must be equal, and next_unmap_pos
	 * must be equal to next_provide_pos.
	 * - for first start, this is how things are set up in open()
	 * - for non-first start, this is result of unprovide_obs().
	 *
	 * Touching unmap_ring_pos, ob_tail and consume_pos requires taking
	 * dev->ob_lock to avoid race against ob_ready() called form poll() */
	dev->shared->ob_head = 0;
	spin_lock(&dev->ob_lock);
	dev->next_unmap_ring_pos = dev->shared->ob_tail = 0;
	dev->next_consume_pos = dev->next_provide_pos;
	spin_unlock(&dev->ob_lock);
	nmcadec_provide_obs(dev);

	dev->decoder_state = DECODER_STARTING;
	ret = nmc3_app_run(dev->desc, dev->entry, dev->shared_dma,
			&nmcadec_decoder_terminated);
	if (unlikely(ret)) {
		dev->decoder_state = DECODER_FAILED;
		printk(KERN_ERR DRIVER_NAME ": could not start NMC code\n");
		return ret;
	}
	dev->decoder_state = DECODER_RUNNING;
	dev->written_after_last_start = 0;

	mod_timer(&dev->timer, jiffies + 1);
	return 0;
}

static void nmcadec_stop_decoder(struct nmcadec_device *dev)
{
	del_timer_sync(&dev->timer);

	nmc3_app_stop_irq(dev->desc);
	dev->decoder_state = DECODER_STOPPED;

	nmcadec_unprovide_obs(dev);
}

static int nmcadec_open(struct inode *inode, struct file *file)
{
	struct nmcadec_device *dev = &the_adec;
	const struct firmware *fw_entry;
	int ret;

	mutex_lock(&dev->open_mutex);
	if (dev->opened) {
		ret = -EBUSY;
		goto out;
	}

	/* Load possibly-updated firmware (fixme?) */

	ret = request_firmware(&fw_entry, FIRMWARE_FILENAME,
			dev->mdev.this_device);
	if (ret) {
		printk(KERN_ERR DRIVER_NAME ": could not get NMC code %s\n",
				FIRMWARE_FILENAME);
		goto out;
	}

	ret = nmc3_elf_upload(dev->desc,
		(void *)fw_entry->data, fw_entry->size, &dev->entry);
	if (ret) {
		printk(KERN_ERR DRIVER_NAME ": could not upload NMC code %s\n",
				FIRMWARE_FILENAME);
		release_firmware(fw_entry);
		goto out;
	}

	printk(KERN_INFO DRIVER_NAME ": loaded NMC code %s (%d bytes)\n",
			FIRMWARE_FILENAME, fw_entry->size);

	release_firmware(fw_entry);

	dev->next_unmap_pos = dev->next_provide_pos = 0;
	atomic_set(&dev->busy_pos, NMCADEC_ORING_SIZE);
	dev->shared->ib_head = 0;	/* used in _start() */
	dev->offset_delta = 0;
	dev->shared->chan_n = 0;

	ret = nmcadec_start_decoder(dev);
	if (unlikely(ret))
		goto out;

	dev->opened = 1;

out:
	mutex_unlock(&dev->open_mutex);
	return ret;
}

static int nmcadec_release(struct inode *inode, struct file *file)
{
	struct nmcadec_device *dev = &the_adec;

	mutex_lock(&dev->open_mutex);

	BUG_ON(dev->opened != 1);
	dev->opened = 0;

	nmcadec_stop_decoder(dev);
	
	mutex_unlock(&dev->open_mutex);

	return 0;
}

static int nmcadec_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct nmcadec_device *dev = &the_adec;
	unsigned long vsize = vma->vm_end - vma->vm_start;

	vma->vm_flags |= (VM_IO/* | VM_DONTEXPAND | VM_DONTDUMP*/);

	if (vma->vm_pgoff != 0 || vsize != NMCADEC_MAP_SIZE)
		return -EINVAL;

	if ((vma->vm_flags & VM_WRITE) != 0)
		return -EPERM;

	return remap_pfn_range(vma,
			vma->vm_start, page_to_pfn(virt_to_page(dev->ob)),
			vsize, vma->vm_page_prot);
}

static ssize_t nmcadec_write(struct file *file, const char __user *buf, 
			size_t count, loff_t *ppos)
{
	struct nmcadec_device *dev = &the_adec;
	int ret, written, chunk, offset;

	ret = mutex_lock_interruptible(&dev->write_mutex);
	if (ret)
		return ret;

	/* Handle interrupted flush */
	if (unlikely(dev->decoder_state == DECODER_STOPPED)) {
		ret = nmcadec_start_decoder(dev);
		if (unlikely(ret))
			goto out;
	}

	written = 0;

	while (count > 0) {

		ret = wait_event_interruptible(dev->ib_wait,
			((chunk = ib_free_space(dev)) != 0 ||
			 (file->f_flags & O_NONBLOCK) ||
			 unlikely(dev->decoder_state == DECODER_FAILED)));

		if (ret) {
			if (ret == -EINTR && written)
				ret = written;
			goto out;
		}

		if (unlikely(dev->decoder_state == DECODER_FAILED)) {
			ret = -EIO;
			goto out;
		}
		if (!chunk) {
			ret = written;
			goto out;
		}

		offset = dev->shared->ib_head % NMCADEC_IRING_BYTES;
		chunk = min_t(int, chunk, count);
		chunk = min_t(int, chunk, NMCADEC_IRING_BYTES - offset);

		dma_sync_single_for_cpu(0, dev->ib_dma + offset, chunk,
				DMA_TO_DEVICE);
		ret = copy_from_user(dev->ib + offset, buf, chunk);
		dma_sync_single_for_device(0, dev->ib_dma + offset, chunk,
				DMA_TO_DEVICE);
		dev->written_after_last_start = 1;	/* mark unclean even if error */
		if (ret) {
			ret = -EFAULT;
			goto out;
		}

		dev->shared->ib_head += chunk;
		written += chunk;
		buf += chunk;
		count -= chunk;
	}

	ret = written;
out:
	mutex_unlock(&dev->write_mutex);
	return ret;
}

static ssize_t nmcadec_read(struct file *file, char __user *buf, 
			size_t count, loff_t *ppos)
{
	struct nmcadec_device *dev = &the_adec;
	unsigned int i;
	int ret;

	/* FIXME: more than one buffer per syscall? */

	if (count != sizeof(i))
		return -EINVAL;

	ret = mutex_lock_interruptible(&dev->read_mutex);
	if (ret)
		return ret;

	ret = wait_event_interruptible(dev->ob_wait,
			(ob_ready(dev) || (file->f_flags & O_NONBLOCK) ||
			 unlikely(dev->decoder_state == DECODER_FAILED)));
	if (ret)
		goto out;
	if (unlikely(dev->decoder_state == DECODER_FAILED)) {
		ret = -EIO;
		goto out;
	}

	/* To avoid race against flush [that could change positions that
	 * ob_ready() depends on], need to re-test ob_ready() condition
	 * under mutex */

	mutex_lock(&dev->read_vs_flush_mutex);

	if (!ob_ready(dev)) {
		ret = 0;
		goto out2;
	}

	/* Since ob_ready() returned true, and dev->read_vs_flush_mutex
	 * is locked (so flush can't run in parallel), it is guaranteed
	 * that dev->next_consume_pos won't be changed by ob_ready(),
	 * so can read it without taking dev->ob_lock.
	 *
	 * But must not change it without taking lock, since behaviour of
	 * ob_ready() does depend on such a change. At a glance it looks
	 * safe even to increment without locking, but better not to do it
	 * to avoid potential races caused by caching or reordering or similar
	 * lowlevel things */

	i = dev->next_consume_pos % NMCADEC_ORING_SIZE;
	dev->ob[i].offset += dev->offset_delta;

	if (copy_to_user(buf, &i, sizeof(i))) {
		ret = -EFAULT;
		goto out2;
	}

	spin_lock(&dev->ob_lock);
	dev->next_consume_pos++;
	spin_unlock(&dev->ob_lock);

	/* For now, allow userspace to own only last returned buffer. FIXME */
	atomic_set(&dev->busy_pos,
			dev->next_consume_pos + NMCADEC_ORING_SIZE - 1);

	ret = sizeof(i);
out2:
	mutex_unlock(&dev->read_vs_flush_mutex);

	if (!ob_ready(dev))
		wake_up_interruptible(&dev->flush_wait);

out:
	mutex_unlock(&dev->read_mutex);
	return ret;
}

static unsigned int nmcadec_poll(struct file *file,
				struct poll_table_struct *table)
{
	struct nmcadec_device *dev = &the_adec;
	unsigned int ret;

	poll_wait(file, &dev->ib_wait, table);
	poll_wait(file, &dev->ob_wait, table);

	ret = 0;
	if (ob_ready(dev))
		ret |= POLLIN | POLLRDNORM;
	if (ib_free_space(dev))
		ret |= POLLOUT | POLLWRNORM;

	return ret;
}

static int nmcadec_flush(struct nmcadec_device *dev, int wait)
{
	int ret;

	/* FIXME: maybe better to wake sleeping writer? */
	ret = mutex_lock_interruptible(&dev->write_mutex);
	if (unlikely(ret))
		return ret;

	ret = wait_event_interruptible(dev->ib_wait,
		!wait || dev->shared->ib_head == dev->shared->ib_ack ||
		unlikely(dev->decoder_state == DECODER_FAILED));
	if (unlikely(ret))
		goto out;
	if (unlikely(dev->decoder_state == DECODER_FAILED)) {
		ret = -EIO;
		goto out;
	}

	/* No need to restart decoder if nothing was written since last start.
	 *
	 * Can't just check for zero ib_head because there is theoretical
	 * possibility of wraparound. So use a separate flag instead */
	if (dev->written_after_last_start)
		nmcadec_stop_decoder(dev);

	if (wait) {
		ret = wait_event_interruptible(dev->flush_wait, !ob_ready(dev));
		if (unlikely(ret))
			goto out;
	}

	mutex_lock(&dev->read_vs_flush_mutex);
	if (dev->written_after_last_start)
		ret = nmcadec_start_decoder(dev);
	else
		ret = 0;
	mutex_unlock(&dev->read_vs_flush_mutex);

out:
	mutex_unlock(&dev->write_mutex);
	return ret;
}

static int nmcadec_request_channels(struct nmcadec_device *dev, int n)
{
	switch (n) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 6:
	case 8:
		dev->shared->chan_n = n;
		return 0;
	default:
		return -EINVAL;
	}
}

static long nmcadec_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct nmcadec_device *dev = &the_adec;

	switch (cmd) {

	case NMCADEC_IOCTL_FLUSH:
		return nmcadec_flush(dev, arg);

	case NMCADEC_IOCTL_REQUEST_CHANNELS:
		return nmcadec_request_channels(dev, arg);

	default:
		return -EINVAL;
	}
}

static struct file_operations nmcadec_fops = {
	.owner = THIS_MODULE,
	.open = nmcadec_open,
	.release = nmcadec_release,
	.mmap = nmcadec_mmap,
	.write = nmcadec_write,
	.read = nmcadec_read,
	.poll = nmcadec_poll,
	.unlocked_ioctl = nmcadec_ioctl,
};

static int __init nmcadec_init(void)
{
	struct nmcadec_device *dev = &the_adec;
	int ret;

	dev->desc = nmc3_find_device(NMC3_DEVICE_NAME);
	if (!dev->desc) {
		printk(KERN_ERR DRIVER_NAME ": could not find %s\n",
				NMC3_DEVICE_NAME);
		ret = -ENODEV;
		goto nmc3_find_failed;
	}

	ret = nmc3_start(dev->desc, 0, 0);
	if (ret) {
		printk(KERN_ERR DRIVER_NAME ": could not initialize %s\n",
				NMC3_DEVICE_NAME);
		goto nmc3_start_failed;
	}

	dev->decoder_state = DECODER_STOPPED;

	mutex_init(&dev->open_mutex);
	mutex_init(&dev->read_mutex);
	mutex_init(&dev->write_mutex);
	mutex_init(&dev->read_vs_flush_mutex);
	spin_lock_init(&dev->ob_lock);

	setup_timer(&dev->timer, nmcadec_tick, (unsigned long)dev);
	
	init_waitqueue_head(&dev->ib_wait);
	init_waitqueue_head(&dev->ob_wait);
	init_waitqueue_head(&dev->flush_wait);

	dev->shared = dma_alloc_coherent(0, sizeof(*dev->shared),
			&dev->shared_dma, GFP_KERNEL);
	if (!dev->shared) {
		printk(KERN_ERR DRIVER_NAME ": could not alloc shared space\n");
		ret = -ENOMEM;
		goto shared_alloc_failed;
	}

	dev->ib = kmalloc(NMCADEC_IRING_BYTES, GFP_KERNEL);
	if (!dev->ib) {
		printk(KERN_ERR DRIVER_NAME ": could not alloc input buffer\n");
		ret = -ENOMEM;
		goto ib_alloc_failed;
	}
	dev->ib_dma = dma_map_single(0, dev->ib,
			NMCADEC_IRING_BYTES, DMA_TO_DEVICE);
	if (!dev->ib_dma) {
		printk(KERN_ERR DRIVER_NAME ": could not map input buffer\n");
		ret = -ENOMEM;
		goto ib_map_failed;
	}

	dev->ob = alloc_pages_exact(NMCADEC_MAP_SIZE, GFP_KERNEL);
	if (!dev->ob) {
		printk(KERN_ERR DRIVER_NAME ": could not alloc output buffer\n");
		ret = -ENOMEM;
		goto ob_alloc_failed;
	}

	dev->ob_ring = dma_alloc_coherent(0, NMCADEC_ORING_SIZE * sizeof(u32),
			&dev->ob_ring_dma, GFP_KERNEL);
	if (!dev->ob_ring) {
		printk(KERN_ERR DRIVER_NAME ": could not alloc output ring\n");
		ret = -ENOMEM;
		goto ob_ring_alloc_failed;
	}

	dev->shared->ib_phys = dev->ib_dma;
	dev->shared->ib_size = NMCADEC_IRING_BYTES;
	dev->shared->ob_phys = dev->ob_ring_dma;
	dev->shared->ob_size = NMCADEC_ORING_SIZE;

	dev->mdev.minor = MISC_DYNAMIC_MINOR;
	dev->mdev.name = DRIVER_NAME;
	dev->mdev.fops = &nmcadec_fops;
	/* FIXME: want NMC3 device to set parent for our device, but
	 *        library does not export it currently */
	ret = misc_register(&dev->mdev);
	if (ret) {
		printk(KERN_ERR DRIVER_NAME ": failed to register\n");
		goto misc_register_failed;
	}

	return 0;

misc_register_failed:
	dma_free_coherent(0, NMCADEC_ORING_SIZE * sizeof(u32),
					dev->ob_ring, dev->ob_ring_dma);
ob_ring_alloc_failed:
	free_pages_exact(dev->ob, NMCADEC_MAP_SIZE);
ob_alloc_failed:
	dma_unmap_single(0, dev->ib_dma, NMCADEC_IRING_BYTES, DMA_TO_DEVICE);
ib_map_failed:
	kfree(dev->ib);
ib_alloc_failed:
	dma_free_coherent(0, sizeof(*dev->shared),
					dev->shared, dev->shared_dma);
shared_alloc_failed:
nmc3_start_failed:
	nmc3_release_dscr(dev->desc);
nmc3_find_failed:
	return ret;
}

static void __exit nmcadec_exit(void)
{
	struct nmcadec_device *dev = &the_adec;

	misc_deregister(&dev->mdev);
	dma_free_coherent(0, NMCADEC_ORING_SIZE * sizeof(u32),
					dev->ob_ring, dev->ob_ring_dma);
	free_pages_exact(dev->ob, NMCADEC_MAP_SIZE);
	dma_unmap_single(0, dev->ib_dma, NMCADEC_IRING_BYTES, DMA_TO_DEVICE);
	kfree(dev->ib);
	dma_free_coherent(0, sizeof(*dev->shared),
					dev->shared, dev->shared_dma);
	nmc3_release_dscr(dev->desc);
}

module_init(nmcadec_init);
module_exit(nmcadec_exit);

MODULE_AUTHOR("Nikita V. Youshchenko <yoush@cs.msu.su>");
MODULE_DESCRIPTION("NMC3-based audio (ac3) decoder - ARM part");
MODULE_LICENSE("GPL");
