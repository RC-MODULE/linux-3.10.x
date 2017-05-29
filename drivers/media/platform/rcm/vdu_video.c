/*
 * drivers/video/module_vdu_video.c - Module's VDU V4L2 support.
 *
 *	Copyright (C) 2008 Module
 *	Written by MSU, CMC dept., LVK lab http://lvk.cs.msu.su
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */
#undef DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/mm.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <linux/rcm_vdu.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stanislav Bezzubtcev <stas@lvk.cs.msu.su>, "
	      "Nikita Youshchenko <yoush@cs.msu.su>");
MODULE_DESCRIPTION("Module VDU V4L2 driver");

#define DRIVER_NAME	"module_vdu_video"
#define CARD_NAME	"module_vdu"	/* FIXME: this should come from outsize */

#ifndef DEBUG
#define dprintk(info...)
#else
#define dprintk(info...)		\
	printk(KERN_DEBUG DRIVER_NAME ": " info)
#endif

/******************************************************************************
 * Private data
 ******************************************************************************/

/* This data describes each of buffers */
struct mvv_buffer_struct {
	struct v4l2_buffer v4l2_buf;
	dma_addr_t base_pa;

	int queued_nr, done_nr;
};

/* This data is given to module_vdu_core */
struct core_data_struct {
	struct mvdu_video_buf buf;
	int params_valid;
	struct mvdu_video_params params;
	struct mvv_buffer_struct *b1, *b2;
	struct list_head list;
};

struct mvdu_video_data {
	struct video_device vdev;
	struct mvdu_device *core_device;

	/* Data containers */
	struct mvv_buffer_struct *mbs;
	struct core_data_struct *cds;
	int num_bufs;
	int requested_bufs;

	/* Free core_data_struct to use */
	struct list_head free;
	wait_queue_head_t cds_wait;

	/* Data stream to _core */
	struct list_head ready;
	struct mvv_buffer_struct *delayed_buf;
	int bufs_in_core;
	int stream_on;
	wait_queue_head_t flush_wait;

	/* Data stream from _core */
	struct list_head used;
	struct mvv_buffer_struct *next_dequeue_buf;
	wait_queue_head_t dq_wait;

	/* Parameters handling */
	struct mvdu_video_params params;	/* current setting */
	int params_set;		/* ... successfully on this open */
	int params_changed;	/* .. since last given to _core */

	/* Current owner with streaming permissions */
	struct file *owner;

	/* Locking */
	struct mutex mutex;
	spinlock_t lock;
};


static inline void mbs_to_queued(struct mvv_buffer_struct *mbs)
{
	if (!mbs->queued_nr++)
		mbs->v4l2_buf.flags |= V4L2_BUF_FLAG_QUEUED;
}

static inline void mbs_to_done(struct mvv_buffer_struct *mbs)
{
	if (!--mbs->queued_nr)
		mbs->v4l2_buf.flags &= ~V4L2_BUF_FLAG_QUEUED;
	if (!mbs->done_nr++)
		mbs->v4l2_buf.flags |= V4L2_BUF_FLAG_DONE;
}

static inline void mbs_release(struct mvv_buffer_struct *mbs)
{
	if (!--mbs->done_nr)
		mbs->v4l2_buf.flags &= ~V4L2_BUF_FLAG_DONE;
}

static inline void mbs_release_from_ready(struct mvv_buffer_struct *mbs)
{
	if (!--mbs->queued_nr)
		mbs->v4l2_buf.flags &= ~V4L2_BUF_FLAG_QUEUED;
}


#define MVDU_VIDEO_MODE_USE_ONE_FRAME(x) \
	(((x) == MVDU_VIDEO_IN_FRAME_TO_P) || \
	 ((x) == MVDU_VIDEO_IN_FRAME_TO_PP) || \
	 ((x) == MVDU_VIDEO_IN_FRAME_TO_I))

#define MVDU_VIDEO_MODE_USE_TWO_FRAMES(x) \
	(!MVDU_VIDEO_MODE_USE_ONE_FRAME(x))

static inline int enqueue_to_delayed(struct mvdu_video_data *mvd)
{
	return MVDU_VIDEO_MODE_USE_TWO_FRAMES(mvd->params.in_mode) &&
	       !mvd->delayed_buf;
}

/* Called under mvd->mutex */
static void start_streaming(struct mvdu_video_data *mvd)
{
	if (mvd->stream_on)
		return;

	mvd->stream_on = 1;
	if (!list_empty(&mvd->ready))
		mvdu_mvl_announce_buffer(mvd->core_device);
}

/* Called under mvd->mutex */
static void stop_streaming(struct mvdu_video_data *mvd)
{
	if (!mvd->stream_on)
		return;

	mvdu_mvl_stop_streaming(mvd->core_device);
	wait_event(mvd->flush_wait, mvd->bufs_in_core == 0);
	mvd->stream_on = 0;
}

/* Called under mvd->mutex */
static void enqueue_core_data_struct(struct mvdu_video_data *mvd,
		struct core_data_struct *cds,
		struct mvv_buffer_struct *mbs1,
		struct mvv_buffer_struct *mbs2)
{
	struct mvdu_video_buf *f = &cds->buf;
	struct mvdu_video_params *cp = &mvd->params;
	dma_addr_t pa1, pa2;
	int need_announce;
	unsigned long flags;

	cds->b1 = mbs1;
	cds->b2 = mbs2;

	pa1 = mbs1->base_pa;
	pa2 = mbs2 ? mbs2->base_pa : pa1;

	if (cp->planes_nr == MVDU_TWO_PLANES) {
		MVDU_ADDR_FR1_Y(f) = pa1 + MVDU_OFFSET_Y(cp);
		MVDU_ADDR_FR1_C(f) = pa1 + MVDU_OFFSET_C(cp);
		MVDU_ADDR_FR2_Y(f) = pa2 + MVDU_OFFSET_Y(cp);
		MVDU_ADDR_FR2_C(f) = pa2 + MVDU_OFFSET_C(cp);
	} else {
		MVDU_ADDR_FR1_Y(f) = pa1 + MVDU_OFFSET_Y(cp);
		MVDU_ADDR_FR1_CB(f) = pa1 + MVDU_OFFSET_CB(cp);
		MVDU_ADDR_FR1_CR(f) = pa1 + MVDU_OFFSET_CR(cp);
		MVDU_ADDR_FR2_Y(f) = pa2 + MVDU_OFFSET_Y(cp);
		MVDU_ADDR_FR2_CB(f) = pa2 + MVDU_OFFSET_CB(cp);
		MVDU_ADDR_FR2_CR(f) = pa2 + MVDU_OFFSET_CR(cp);
	}

	spin_lock_irqsave(&mvd->lock, flags);

	if (mvd->params_changed) {
		cds->params_valid = 1;
		cds->params = mvd->params;
		mvd->params_changed = 0;
	} else
		cds->params_valid = 0;

	need_announce = mvd->stream_on && list_empty(&mvd->ready);
	list_add_tail(&cds->list, &mvd->ready);

	spin_unlock_irqrestore(&mvd->lock, flags);

	if (need_announce)
		mvdu_mvl_announce_buffer(mvd->core_device);
}

/* Called from _core */
static int fetch_buf(struct mvdu_device *device,
		struct mvdu_video_buf **data_ptr,
		struct mvdu_video_params **params_ptr)
{
	struct mvdu_video_data *mvd = device->mvd;
	struct core_data_struct *cds;
	int have_more;
	unsigned long flags;

	spin_lock_irqsave(&mvd->lock, flags);

	BUG_ON(list_empty(&mvd->ready));

	cds = list_first_entry(&mvd->ready, struct core_data_struct, list);
	*data_ptr = &cds->buf;
	*params_ptr = cds->params_valid ? &cds->params : 0;
	list_del(&cds->list);
	have_more = !list_empty(&mvd->ready);
	mvd->bufs_in_core++;

	spin_unlock_irqrestore(&mvd->lock, flags);

	dprintk("core fetched buf at %08x\n", (unsigned int)(*data_ptr));
	return have_more;
}

/* Called from _core */
static void release_buf(struct mvdu_device *device,
		struct mvdu_video_buf *arg)
{
	struct mvdu_video_data *mvd = device->mvd;
	struct core_data_struct *cds;
	unsigned long flags;

	cds = container_of(arg, struct core_data_struct, buf);

	spin_lock_irqsave(&mvd->lock, flags);

	mbs_to_done(cds->b1);
	if (cds->b2)
		mbs_to_done(cds->b2);

	list_add_tail(&cds->list, &mvd->used);
	wake_up_interruptible(&mvd->dq_wait);

	mvd->bufs_in_core--;
	if (!mvd->bufs_in_core)
		wake_up(&mvd->flush_wait);

	spin_unlock_irqrestore(&mvd->lock, flags);

	dprintk("core released buf at %08x\n", (unsigned int)arg);
}

static void release_params(struct mvdu_device *device,
		struct mvdu_video_params *arg)
{
}


static int vdu_video_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct mvdu_video_data *mvd =
		container_of(vdev, struct mvdu_video_data, vdev);

	file->private_data = mvd;
	mvdu_enable_vdu(mvd->core_device);
	return 0;
}

static int vdu_video_close(struct file *file)
{
	struct mvdu_video_data *mvd = file->private_data;
	struct core_data_struct *cds;

	mvdu_disable_vdu(mvd->core_device);

	mutex_lock(&mvd->mutex);

	if (mvd->owner == file) {

		mvd->owner = 0;

		stop_streaming(mvd);

		while (!list_empty(&mvd->ready)) {

			cds = list_first_entry(&mvd->ready, typeof(*cds), list);
			list_del(&cds->list);

			mbs_release_from_ready(cds->b1);
			if (cds->b2)
				mbs_release_from_ready(cds->b2);

			list_add_tail(&cds->list, &mvd->free);
		}

		if (mvd->delayed_buf) {
			mbs_release_from_ready(mvd->delayed_buf);
			mvd->delayed_buf = 0;
		}

		while (!list_empty(&mvd->used)) {

			cds = list_first_entry(&mvd->used, typeof(*cds), list);
			list_del(&cds->list);

			mbs_release(cds->b1);
			if (cds->b2)
				mbs_release(cds->b2);

			list_add_tail(&cds->list, &mvd->free);
		}

		if (mvd->next_dequeue_buf) {
			mbs_release(mvd->next_dequeue_buf);
			mvd->next_dequeue_buf = 0;
		}

		mvd->requested_bufs = 0;
		mvd->params_set = 0;
		mvd->params.valid = 0;
	}

	mutex_unlock(&mvd->mutex);
	return 0;
}

static ssize_t vdu_video_read(struct file *file, char __user *data,
		size_t count, loff_t *ppos)
{
	return -EINVAL;
}

static ssize_t vdu_video_write(struct file *file, const char __user *data,
		size_t count, loff_t *ppos)
{
	return -EINVAL;
}

static unsigned int vdu_video_poll(struct file *file, poll_table *table)
{
	struct mvdu_video_data *mvd = file->private_data;
	unsigned int ret;

	poll_wait(file, &mvd->dq_wait, table);
	poll_wait(file, &mvd->cds_wait, table);

	ret = 0;
	if (mvd->next_dequeue_buf || !list_empty(&mvd->used))
		ret |= POLLIN | POLLRDNORM;
	if (enqueue_to_delayed(mvd) || !list_empty(&mvd->free))
		ret |= POLLOUT | POLLWRNORM;

	return ret;
}

/* Mapping: anyone may map, we don't care. It is not really related
 * to streaming.
 *
 * We just map video buffer area, either entire or in parts.
 *
 * Code below taken from fb_mmap().
 */
static int vdu_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct mvdu_video_data *mvd = file->private_data;
	unsigned long off, start;
	u32 len;

	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;
	off = vma->vm_pgoff << PAGE_SHIFT;

	start = mvd->core_device->video_buffer_dma;
	len = PAGE_ALIGN((start & ~PAGE_MASK) +
			mvd->core_device->video_buffer_size);
	if (off >= len)
		return -EINVAL;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
	return io_remap_pfn_range(vma, vma->vm_start,
			(off + start) >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static int s_params(struct file *file, struct mvdu_video_params *p);
static int g_params(struct file *file, struct mvdu_video_params *p);

static long vdu_video_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int ret;
	struct mvdu_video_params e;

	switch(cmd) {
	case VIDIOC_S_PARAMS:
		ret = __copy_from_user(&e, (void __user *)arg, sizeof(e));
		if (ret)
			break;
		ret = s_params(file, &e);
		break;
	case VIDIOC_G_PARAMS:
		ret = g_params(file, &e);
		if (ret)
			break;
		ret = __copy_to_user((void __user *)arg, &e, sizeof(e));
		break;
	default:
		ret = video_ioctl2(file, cmd, arg);
		break;
	} /* switch(cmd) */

	return ret;
}

static const struct v4l2_file_operations mvdu_video_fops = {
	.owner = THIS_MODULE,
	.open = vdu_video_open,
	.release = vdu_video_close,
	.unlocked_ioctl = vdu_video_ioctl,
	.read = vdu_video_read,
	.write = vdu_video_write,
	.mmap = vdu_video_mmap,
	.poll = vdu_video_poll,
};


static int querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct mvdu_video_data *mvd = file->private_data;
	memset(cap, 0, sizeof(*cap));

	strncpy(cap->card, CARD_NAME, sizeof(cap->card) - 1);
	strncpy(cap->driver, DRIVER_NAME, sizeof(cap->driver) - 1);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "MMIO %08x",
		 (unsigned int)mvd->core_device->regs_phys);
	cap->version = KERNEL_VERSION(0, 0, 1);
    cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	MVDU_VIDEO_BASE_PHYS(cap) = mvd->core_device->video_buffer_dma;

	return 0;
}

static int reqbufs(struct file *file, void *h, struct v4l2_requestbuffers *b)
{
	struct mvdu_video_data *mvd = file->private_data;
	int ret = 0;

	if (b->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ||
			b->memory != V4L2_MEMORY_MMAP ) {
		dprintk("invalid reqbufs params\n");
		return -EINVAL;
	}

	if (!b->count)
		return -EINVAL;

	mutex_lock(&mvd->mutex);

	if (mvd->owner && mvd->owner != file) {
		dprintk("reqbufs by non-owner\n");
		ret = -EBUSY;
		goto out_unlock;
	}

	if (mvd->stream_on && b->count != mvd->requested_bufs) {
		dprintk("reqbufs while streaming active\n");
		ret = -EBUSY;
		goto out_unlock;
	}

	mvd->owner = file;
	if (b->count > mvd->num_bufs)
		b->count = mvd->num_bufs;
	mvd->requested_bufs = b->count;

out_unlock:

	if (!ret)
		dprintk("requested %d buffers\n", b->count);

	mutex_unlock(&mvd->mutex);
	return ret;
}

static int querybuf(struct file *file, void *h, struct v4l2_buffer *b)
{
	struct mvdu_video_data *mvd = file->private_data;
	struct v4l2_buffer *db;

	if (b->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		dprintk("querybuf() video buffer type mismatch\n");
		return -EINVAL;
	}

	if (file != mvd->owner) {
		dprintk("only an owner can query buffer's status\n");
		return -EBUSY;
	}

	if (b->index >= mvd->requested_bufs) {
		dprintk("querybuf() index %d, attached bufs number %d\n",
			b->index, mvd->requested_bufs);
		return -EINVAL;
	}

	db = &mvd->mbs[b->index].v4l2_buf;

	b->flags = db->flags;
	b->length = db->length;
	b->memory = V4L2_MEMORY_MMAP;
	b->m.offset = db->m.offset;
	return 0;
}

static int streamon(struct file *file, void *h, enum v4l2_buf_type i)
{
	struct mvdu_video_data *mvd = file->private_data;
	int ret;

	if (i != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		dprintk("wrong v4l2_buf_type: got 0x%x, expected 0x%x\n",
			i, V4L2_BUF_TYPE_VIDEO_OUTPUT);
		return -EINVAL;
	}

	mutex_lock(&mvd->mutex);

	if (mvd->owner && mvd->owner != file) {
		dprintk("streamon called not by owner\n");
		ret = -EBUSY;
		goto out;
	}
	mvd->owner = file;

	start_streaming(mvd);
	dprintk("streaming started\n");
	ret = 0;
out:
	mutex_unlock(&mvd->mutex);
	return ret;
}

static int streamoff(struct file *file, void *h, enum v4l2_buf_type i)
{
	struct mvdu_video_data *mvd = file->private_data;
	int ret;

	if (i != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		dprintk("wrong v4l2_buf_type: got 0x%x, expected 0x%x\n",
			i, V4L2_BUF_TYPE_VIDEO_OUTPUT);
		return -EINVAL;
	}

	mutex_lock(&mvd->mutex);

	if (mvd->owner != file) {
		dprintk("streamoff called not by owner\n");
		ret = -EBUSY;
	} else {
		stop_streaming(mvd);
		dprintk("streaming stopped\n");
		ret = 0;
	}

	mutex_unlock(&mvd->mutex);
	return ret;
}

static int qbuf(struct file *file, void *h, struct v4l2_buffer *b)
{
	struct mvdu_video_data *mvd = file->private_data;
	struct mvv_buffer_struct *mbs;
	struct core_data_struct *cds;
	unsigned long flags;
	int ret;

	if (b->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ||
			b->memory != V4L2_MEMORY_MMAP) {
		dprintk("invalid parameters to qbuf\n");
		return -EINVAL;
	}

	mutex_lock(&mvd->mutex);

	if (mvd->owner != file) {
		dprintk("qbuf called not by owner\n");
		ret = -EBUSY;
		goto out;
	}

	if (!mvd->params_set) {
		dprintk("params were not set before buffer enqueue\n");
		ret = -EINVAL;
		goto out;
	}

	if (b->index >= mvd->requested_bufs) {
		dprintk("cannot enqueue buffer %d - index is out of range\n",
				b->index);
		ret = -EINVAL;
		goto out;
	}

	if (enqueue_to_delayed(mvd))
		cds = 0;
	else {

		spin_lock_irqsave(&mvd->lock, flags);

		while (list_empty(&mvd->free)) {

			spin_unlock_irqrestore(&mvd->lock, flags);

			if (file->f_flags & O_NONBLOCK) {
				ret = -EWOULDBLOCK;
				goto out;
			}

			ret = wait_event_interruptible(mvd->cds_wait,
						!list_empty(&mvd->free));
			if (ret)
				goto out;

			spin_lock_irqsave(&mvd->lock, flags);
		}

		cds = list_first_entry(&mvd->free, typeof(*cds), list);
		list_del(&cds->list);

		spin_unlock_irqrestore(&mvd->lock, flags);
	}

	mbs = &mvd->mbs[b->index];
	mbs_to_queued(mbs);

	dprintk("enqueue buffer %d\n", b->index);

	if (enqueue_to_delayed(mvd))
		mvd->delayed_buf = mbs;
	else if (MVDU_VIDEO_MODE_USE_TWO_FRAMES(mvd->params.in_mode))
		enqueue_core_data_struct(mvd, cds, mvd->delayed_buf, mbs);
	else
		enqueue_core_data_struct(mvd, cds, mbs, 0);

	ret = 0;

out:
	mutex_unlock(&mvd->mutex);
	return ret;
}

static int dqbuf(struct file *file, void *h, struct v4l2_buffer *b)
{
	struct mvdu_video_data *mvd = file->private_data;
	struct core_data_struct *cds;
	struct mvv_buffer_struct *mbs;
	int ret;
	unsigned long flags;

	if (b->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ||
			b->memory != V4L2_MEMORY_MMAP) {
		dprintk("invalid parameters to dqbuf\n");
		return -EINVAL;
	}

	if (mvd->owner != file) {
		dprintk("dqbuf called not by owner\n");
		return -EBUSY;
	}

	spin_lock_irqsave(&mvd->lock, flags);

	if (mvd->next_dequeue_buf) {
		mbs = mvd->next_dequeue_buf;
		mvd->next_dequeue_buf = 0;
	} else {

		while (list_empty(&mvd->used)) {

			spin_unlock_irqrestore(&mvd->lock, flags);

			if (file->f_flags & O_NONBLOCK)
				return -EWOULDBLOCK;

			ret = wait_event_interruptible(mvd->dq_wait,
						!list_empty(&mvd->used));
			if (ret)
				return ret;

			spin_lock_irqsave(&mvd->lock, flags);
		}

		cds = list_first_entry(&mvd->used, typeof(*cds), list);
		list_del(&cds->list);

		mbs = cds->b1;
		mvd->next_dequeue_buf = cds->b2;

		list_add_tail(&cds->list, &mvd->free);
		wake_up_interruptible(&mvd->cds_wait);
	}

	mbs_release(mbs);

	spin_unlock_irqrestore(&mvd->lock, flags);

	b->index = mbs->v4l2_buf.index;

	dprintk("dequeued buffer %d\n", b->index);
	return 0;
}

/* FIXME: locking for s_params, g_params */

static int s_params(struct file *file, struct mvdu_video_params *new_p)
{
	struct mvdu_video_data *mvd = file->private_data;
	struct mvdu_video_params tp;
	int ret;

	mutex_lock(&mvd->mutex);

	if (mvd->owner != file) {
		dprintk("s_params called not by owner\n");
		ret = -EBUSY;
		goto out;
	}

	if (unlikely(mvd->delayed_buf)) {
		dprintk("cannot set params while buffer is delayed\n");
		ret = -EBUSY;
		goto out;
	}

	tp = mvd->params;
	mvdu_mvl_merge_params(&tp, new_p);
	ret = mvdu_mvl_check_params(mvd->core_device, &tp);
	if (ret) {
		dprintk("parameters are not consistent\n");
		goto out;
	}
	mvd->params = tp;

	mvd->params_changed = 1;
	mvd->params_set = 1;
	ret = 0;

	dprintk("changed parameters\n");
out:
	mutex_unlock(&mvd->mutex);
	return ret;
}

static int g_params(struct file *file, struct mvdu_video_params *p)
{
	struct mvdu_video_data *mvd = file->private_data;

	memcpy(p, &mvd->params, sizeof(*p));

	return 0;
}

/* Have to provide this to make VIDIOC_REQBUF working */
static int g_fmt_vid_out(struct file *file, void *h, struct v4l2_format *fmt)
{
	return -EINVAL;
}

static const struct v4l2_ioctl_ops mvdu_video_ioctl_ops = {
	.vidioc_querycap = querycap,
	.vidioc_g_fmt_vid_out = g_fmt_vid_out,
	.vidioc_reqbufs = reqbufs,
	.vidioc_querybuf = querybuf,
	.vidioc_streamon = streamon,
	.vidioc_streamoff = streamoff,
	.vidioc_qbuf = qbuf,
	.vidioc_dqbuf = dqbuf,
};


static void vdev_release(struct video_device *vdev)
{
}

static int __init video_dev_init(struct mvdu_device *core_device)
{
	struct mvdu_video_data *mvd;
	struct video_device *vd;
	int ret = 0;
	int i;

	mvd = (struct mvdu_video_data *)kzalloc(sizeof(*mvd), GFP_KERNEL);
	if (!mvd) {
		dev_err(core_device->dev,
				"failed to allocate memory for device "
				"driver structures\n");
		ret = -ENOMEM;
		goto fail_mvd_allocate;
	}

	core_device->mvd = mvd;
	mvd->core_device = core_device;

	INIT_LIST_HEAD(&mvd->ready);
	INIT_LIST_HEAD(&mvd->used);
	INIT_LIST_HEAD(&mvd->free);
	spin_lock_init(&mvd->lock);
	mutex_init(&mvd->mutex);
	init_waitqueue_head(&mvd->dq_wait);
	init_waitqueue_head(&mvd->cds_wait);
	init_waitqueue_head(&mvd->flush_wait);

	ret = (core_device->allocate_video_buffer) ?
			core_device->allocate_video_buffer(core_device) : 0;
	mvd->num_bufs =
		core_device->video_buffer_size / MVDU_VIDEO_MAX_BUFFER_SIZE;
	if (!mvd->num_bufs) {
		dev_err(core_device->dev, "no video buffers available\n");
		if (!ret)
			ret = -ENOMEM;
		goto no_buffers;
	}

	mvd->mbs = (struct mvv_buffer_struct *)
		kzalloc(sizeof(struct mvv_buffer_struct) * mvd->num_bufs,
		GFP_KERNEL);
	if (!mvd->mbs) {
		dev_err(core_device->dev,
				"failed to allocate memory for mbs objects\n");
		goto fail_mbs_allocate;
	}
	for (i = 0; i < mvd->num_bufs; i++) {
		struct mvv_buffer_struct *mbs = &mvd->mbs[i];
		struct v4l2_buffer *b = &mbs->v4l2_buf;

		b->index = i;
		b->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		b->length = MVDU_VIDEO_MAX_BUFFER_SIZE;
		b->memory = V4L2_MEMORY_MMAP;
		b->m.offset = i * MVDU_VIDEO_MAX_BUFFER_SIZE;
		mbs->base_pa = core_device->video_buffer_dma + b->m.offset;
	}

	mvd->cds = (struct core_data_struct *)
		kzalloc(sizeof(struct core_data_struct) * mvd->num_bufs * 2,
		GFP_KERNEL);
	if (!mvd->cds) {
		dev_err(core_device->dev,
				"failed to allocate memory for cds objects\n");
		goto fail_cds_allocate;
	}

	for (i = 0; i < mvd->num_bufs * 2; i++)
		list_add_tail(&mvd->cds[i].list, &mvd->free);

	vd = &mvd->vdev;

	strncpy(vd->name, DRIVER_NAME, sizeof(vd->name) - 1);
	vd->fops = &mvdu_video_fops;
	vd->ioctl_ops = &mvdu_video_ioctl_ops;
	vd->release = &vdev_release;
    vd->v4l2_dev = &core_device->v4l2_dev;

	/* placeholder */
	vd->tvnorms = V4L2_STD_PAL | V4L2_STD_NTSC | V4L2_STD_SECAM;
//	vd->current_norm = V4L2_STD_PAL;
	vd->vfl_dir = VFL_DIR_TX;

	core_device->mvl_fetch_buffer = &fetch_buf;
	core_device->mvl_release_buffer = &release_buf;
	core_device->mvl_release_params = &release_params;

	memset(vd->v4l2_dev, 0x0, sizeof(*vd->v4l2_dev));
	ret = v4l2_device_register(core_device->dev, vd->v4l2_dev);
	if (ret) {
		dev_err(core_device->dev,
				"failed to register v4l2 device\n");
		goto fail_video_device_register;
	}
	/* We need to select some device type to call
	 * video_register_device function. Type value defines
	 * class/device name (we want "video").
	 */
	ret = video_register_device(vd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		dev_err(core_device->dev,
				"failed to register video device\n");
		goto fail_video_device_register;
	}

	dprintk("driver loaded\n");
	return 0;

fail_video_device_register:
	core_device->mvl_fetch_buffer = 0;
	core_device->mvl_release_buffer = 0;
	core_device->mvl_release_params = 0;
	kfree(mvd->cds);
fail_cds_allocate:
	kfree(mvd->mbs);
fail_mbs_allocate:
	if (core_device->free_video_buffer)
		core_device->free_video_buffer(core_device);
no_buffers:
	kfree(mvd);
	core_device->mvd = 0;
fail_mvd_allocate:
	return ret;
}

static void __exit video_dev_remove(struct mvdu_device *core_device)
{
	struct mvdu_video_data *mvd = core_device->mvd;

	video_unregister_device(&mvd->vdev);

	core_device->mvl_fetch_buffer = 0;
	core_device->mvl_release_buffer = 0;
	core_device->mvl_release_params = 0;

	kfree(mvd->cds);
	kfree(mvd->mbs);
	if (core_device->free_video_buffer)
		core_device->free_video_buffer(core_device);
	kfree(mvd);
	core_device->mvd = 0;

	dprintk("driver removed\n");
}

static int __init module_vdu_video_init(void)
{
	struct mvdu_device *core_device;
	int i = 0;

	while ((core_device = mvdu_get_device(i++)) != 0) {
		video_dev_init(core_device);
	}

	return 0;
}

static void __exit module_vdu_video_exit(void)
{
	struct mvdu_device *core_device;
	int i = 0;

	while ((core_device = mvdu_get_device(i++)) != 0) {
		if (core_device->mvd)
			video_dev_remove(core_device);
	}
}

late_initcall(module_vdu_video_init);
module_exit(module_vdu_video_exit);
