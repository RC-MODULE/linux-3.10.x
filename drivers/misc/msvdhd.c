#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>

#include <linux/msvdhd.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#define DRIVER_NAME "msvdhd"

struct msvd_regs {
	u32 vd_clc;	/* Clock control (rwh) */
	u32 vd_id;	/* Revision identification (r) */
	u32 vd_imsc;	/* Interrupt mask (rw) */
	u32 vd_ris;	/* Raw interrupt status (r) */
	u32 vd_mis;	/* Masked interrupt status (r) */
	u32 vd_icr;	/* Interrupt status clear (w) */
};

struct msvdhd_device_data {

	struct device *dev;
	struct miscdevice mdev;

	struct msvd_regs __iomem *regs;
	unsigned long regs_phys;

	void *buf;
	unsigned long buf_phys;

	unsigned long extmem_phys;
	unsigned long intmem_phys;

	int irq;

	int irq_enabled;
	unsigned int irq_status;
	spinlock_t lock;

	wait_queue_head_t irq_wq;
};

static struct msvdhd_device_data the_msvdhd_device;


static irqreturn_t msvdhd_irq(int irq, void *dev_id)
{
	struct msvdhd_device_data *dev = dev_id;
	unsigned int status;
	unsigned long flags;

	status = ioread32(&dev->regs->vd_mis);
	iowrite32(status, &dev->regs->vd_icr);

	spin_lock_irqsave(&dev->lock, flags);
	dev->irq_status |= status;
	spin_unlock_irqrestore(&dev->lock, flags);

	wake_up_interruptible(&dev->irq_wq);

	return IRQ_HANDLED;
}

static long msvdhd_ioctl(struct file *filp,
			 unsigned int cmd, unsigned long arg)
{
	int ret;
	unsigned int status;
	struct msvdhd_device_data *dev = filp->private_data;
	struct msvd_wait_irq_params __user *p_irq;
	struct msvd_get_buf_phys_params __user *p_buf;
	struct msvd_get_extmem_phys_params __user *p_extmem;
	struct msvd_get_intmem_phys_params __user *p_intmem;
	unsigned long flags;

	switch (cmd) {
		case MSVD_ENABLE_IRQ:
			if (dev->irq_enabled)
				return 0;
			dev->irq_status = 0;
			ret = request_irq(dev->irq, msvdhd_irq, 0,
					DRIVER_NAME, dev);
			if (ret) {
				dev_err(dev->dev,
						"failed to set irq handler\n");
				return ret;
			}
			dev->irq_enabled = 1;
			return 0;

		case MSVD_DISABLE_IRQ:
			if (!dev->irq_enabled)
				return 0;
			free_irq(dev->irq, dev);
			dev->irq_enabled = 0;
			wake_up_interruptible(&dev->irq_wq);
			return 0;

		case MSVD_WAIT_IRQ:
			ret = wait_event_interruptible(dev->irq_wq,
					dev->irq_status != 0 ||
					unlikely(!dev->irq_enabled));
			if (ret)
				return ret;

			spin_lock_irqsave(&dev->lock, flags);
			status = xchg(&dev->irq_status, 0);
			spin_unlock_irqrestore(&dev->lock, flags);

			p_irq = (void __user *) arg;
			return put_user(status, &p_irq->status);

		case MSVD_GET_BUF_PHYS:
			p_buf = (void __user *) arg;
			return put_user(dev->buf_phys, &p_buf->addr);

		case MSVD_GET_EXTMEM_PHYS:
			p_extmem = (void __user *) arg;
			return put_user(dev->extmem_phys, &p_extmem->addr);

		case MSVD_GET_INTMEM_PHYS:
			p_intmem = (void __user *) arg;
			return put_user(dev->intmem_phys, &p_intmem->addr);

		default:
			return -EINVAL;
	}
}

static int msvdhd_open(struct inode *inode, struct file *filp)
{
	struct msvdhd_device_data *dev = &the_msvdhd_device;

	dev_dbg(dev->dev, "open\n");
	filp->private_data = dev;

	return 0;
}

static int msvdhd_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct msvdhd_device_data *dev = filp->private_data;
	unsigned long vsize = vma->vm_end - vma->vm_start;

	vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);

	if (vma->vm_pgoff == (MSVD_MAP_REGS_OFFSET >> PAGE_SHIFT) &&
	    vsize == MSVD_MAP_REGS_SIZE) {

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		return remap_pfn_range(vma, vma->vm_start,
				dev->regs_phys >> PAGE_SHIFT,
				vsize, vma->vm_page_prot);
	}

	if (vma->vm_pgoff == (MSVD_MAP_BUF_OFFSET >> PAGE_SHIFT) &&
	    vsize == MSVD_MAP_BUF_SIZE) {

		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
		return remap_pfn_range(vma, vma->vm_start,
				dev->buf_phys >> PAGE_SHIFT,
				vsize, vma->vm_page_prot);
	}

	if (vma->vm_pgoff == (MSVD_MAP_EXTMEM_OFFSET >> PAGE_SHIFT) &&
	    vsize == MSVD_MAP_EXTMEM_SIZE) {

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		return remap_pfn_range(vma, vma->vm_start,
				dev->extmem_phys >> PAGE_SHIFT,
				vsize, vma->vm_page_prot);
	}

	if (vma->vm_pgoff == (MSVD_MAP_INTMEM_OFFSET >> PAGE_SHIFT) &&
	    vsize == MSVD_MAP_INTMEM_SIZE) {

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		return remap_pfn_range(vma, vma->vm_start,
				dev->intmem_phys >> PAGE_SHIFT,
				vsize, vma->vm_page_prot);
	}

	return -EINVAL;
}

static int msvdhd_release(struct inode *inode, struct file *filp)
{
	struct msvdhd_device_data *dev = filp->private_data;

	dev_dbg(dev->dev, "release\n");

	if (dev->irq_enabled) {
		dev_dbg(dev->dev, "disabling irq\n");
		free_irq(dev->irq, dev);
		dev->irq_enabled = 0;
	}

	return 0;
}

static struct file_operations msvdhd_ops = {
	owner:	 THIS_MODULE,
	unlocked_ioctl:	 msvdhd_ioctl,
	open:	 msvdhd_open,
	mmap:	 msvdhd_mmap,
	release: msvdhd_release,
};

static int msvdhd_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	struct msvdhd_device_data *dev = &the_msvdhd_device;
	unsigned int val;

	dev->dev = &pdev->dev;
	init_waitqueue_head(&dev->irq_wq);
	platform_set_drvdata(pdev, dev);

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(dev->dev, "irq not defined\n");
		ret = -ENODEV;
		goto failed_get_irq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev->dev, "registers not defined\n");
		ret = -ENODEV;
		goto failed_get_regs;
	}

	dev->regs = devm_request_and_ioremap(&pdev->dev, res);
	if (dev->regs == NULL) {
		dev_err(dev->dev, "failed to map registers\n");
		ret = -ENOMEM;
		goto failed_remap;
	}
	dev->regs_phys = res->start;

	val = ioread32(&dev->regs->vd_id);
	if (((val >> 12) & 0xFF) != 0x53) {
		dev_err(dev->dev, "no MSVD-HD signature at 0x%lx\n",
				dev->regs_phys);
		ret = -ENODEV;
		goto failed_read_reg;
	}
	dev_info(dev->dev, "found device at 0x%lx, id 0x%08x\n",
			dev->regs_phys, val);

	dev->buf = alloc_pages_exact(MSVD_MAP_BUF_SIZE, GFP_KERNEL);
	if (!dev->buf) {
		dev_err(dev->dev, "failed to allocate input buffer");
		ret = -ENOMEM;
		goto failed_alloc_buf;
	}
	dev->buf_phys = __virt_to_phys((unsigned long)dev->buf);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "extmem");
	if (!res || resource_size(res) < MSVD_MAP_EXTMEM_SIZE) {
		dev_err(dev->dev, "extmem not defined properly\n");
		ret = -ENODEV;
		goto failed_get_extmem;
	}
	dev->extmem_phys = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "intmem");
	if (!res || resource_size(res) < MSVD_MAP_INTMEM_SIZE) {
		dev_err(dev->dev, "intmem not defined properly\n");
		ret = -ENODEV;
		goto failed_get_intmem;
	}
	dev->intmem_phys = res->start;

	dev->mdev.minor = MISC_DYNAMIC_MINOR;
	dev->mdev.name = DRIVER_NAME;
	dev->mdev.parent = dev->dev;
	dev->mdev.fops = &msvdhd_ops;
	ret = misc_register(&dev->mdev);
	if (ret) {
		dev_err(dev->dev, "failed to register\n");
		goto failed_register_dev;
	}

	return 0;

failed_register_dev:
failed_get_intmem:
failed_get_extmem:
	free_pages_exact(dev->buf, MSVD_MAP_BUF_SIZE);
failed_alloc_buf:
failed_read_reg:
failed_remap:
failed_get_regs:
failed_get_irq:
	platform_set_drvdata(pdev, 0);
	return ret;
}

static int msvdhd_remove(struct platform_device *pdev)
{
	struct msvdhd_device_data *dev = platform_get_drvdata(pdev);

	misc_deregister(&dev->mdev);
	//free_pages_exact(dev->extmem, MSVD_MAP_BUF_SIZE);
	free_pages_exact(dev->buf, MSVD_MAP_BUF_SIZE);
	platform_set_drvdata(pdev, 0);
	return 0;
}

static const struct of_device_id msvdhd_dt_ids[] = {
	{ .compatible = "module,msvdhd" },
	{ }
};

MODULE_DEVICE_TABLE(of, msvdhd_dt_ids);

static struct platform_driver msvdhd_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(msvdhd_dt_ids),
	},
	.probe = msvdhd_probe,
	.remove = msvdhd_remove,
};

module_platform_driver(msvdhd_driver);

MODULE_DESCRIPTION("MSVD-HD driver");
MODULE_LICENSE("GPL");
