/* RC Module BBP Soc. Test of ARM-NMC3 interface driver
 *
 * Copyright RC Module 2011
 *
 * Author Gennadiy Kurtsman <gkurtsman@module.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <asm/ioctl.h>

#define DEBUG
#undef DEBUG

#include <linux/rcmod_soc_nmc3.h>

#include "rcmod_soc_nmc3_test.h"

#define TEST_NMC3_DRIVER_VERSION     "0.1"

static struct class *NMC3_class;
struct NMC3_extension * this_device;
static volatile int app_start_sign;
struct __xan_data {
	__u32 data;
	struct semaphore xan_sync;
};
static struct __xan_data xan_data; 

void app_callback (struct __NMC3_thread_descriptor* thr, int ret_code)
{
	struct NMC3_extension *dev_ex = thr->data;

	dbg ("%s, ret_code=0x%x",__func__,ret_code);
	dev_ex->ret_code = ret_code;
	dbg ("dev_ex=%p, ret_code = dev_ex->ret_code=0x%x",dev_ex, dev_ex->ret_code);
	up (&dev_ex->app_complete_sem);
}

void irq_cb (int irq, __u32 msg)
{
	if (msg == NM_APP_START_CODE)
		app_start_sign = 1;
	dbg ("%s: irq=%d, msg=0x%x",__func__,irq,msg);
}

static unsigned int mark_2cb;
static unsigned int mark_2cb_set;

void xan_cb (int irq, __u32 msg)
{
	dbg ("%s", __func__);
	xan_data.data = msg;
	mark_2cb = 0;
	up (&xan_data.xan_sync);
	dbg ("%s: irq=%d, msg=0x%x",__func__,irq,msg);
}

void xan_2cb (int irq, __u32 msg)
{
	dbg ("%s", __func__);
	xan_data.data = msg;
	mark_2cb = 1;
	up (&xan_data.xan_sync);
	dbg ("%s: irq=%d, msg=0x%x",__func__,irq,msg);
}

static int NMC3_open(struct inode *inode, struct file *filp)
{
	struct NMC3_extension *dev_ex = container_of(inode->i_cdev,
		struct NMC3_extension, cdev);
	int minor = MINOR(inode->i_rdev);

	filp->private_data = dev_ex;
	dev_ex->nmc3_dscr[minor] = nmc3_find_device (dev_ex->Name);
	if (!dev_ex->nmc3_dscr[minor]) {
		printk (KERN_INFO "nmc3_find_device() ERROR\n");
		return -ENODEV;
	}

	dbg("%s, Return: 0, filp=%p, dev_ex=%p", __func__, filp, dev_ex);
	return 0;
}

static int NMC3_release(struct inode *inode, struct file *filp)
{
	struct NMC3_extension *dev_ex = filp->private_data;
	struct __NMC3_thread_descriptor* thr_dscr;
	int minor = MINOR(inode->i_rdev);

	thr_dscr = dev_ex->nmc3_dscr[minor];
	dev_ex->nmc3_dscr[minor] = NULL;
	nmc3_release_dscr (thr_dscr);
	return 0;
}

static loff_t NMC3_llseek(struct file *filp, loff_t offset, int from)
{
	struct NMC3_extension *dev_ex = filp->private_data;
	int ret;

	dbg ("XXXXXXXXXXXXXXX %s, offset=0x%llx, from=%d,dev_ex=%p", __func__, offset,
		from,dev_ex);
	if ((ret = down_interruptible(&dev_ex->sem)) < 0) {
		printk(KERN_NOTICE "NMC3: LLSEEK operation interrupted.\n");
		return ret;
	}
	switch (from) {
	case SEEK_SET:
		filp->f_pos = offset;
		break;
	case SEEK_CUR:
		filp->f_pos += offset;
		break;
	case SEEK_END:
		filp->f_pos = -offset;
		break;
	default:
		printk(KERN_INFO "NMC3: ERROR: Seek ERROR %d\n", from);
		up(&dev_ex->sem);
		return -EINVAL;
	}
	up(&dev_ex->sem);
	dbg ("f_pos=0x%llx", filp->f_pos);
	return filp->f_pos;
}

static ssize_t NMC3_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	struct NMC3_extension *dev_ex = filp->private_data;
	__u8 kbuff [1024];	/* 64 KB */
	int ln;
	int chunk;
	const char __user *ubuff = buff;
	int ret;
	int minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	dbg ("%s, offp=0x%llx",__func__,*offp);
	if ((ret = down_interruptible(&dev_ex->sem)) < 0) {
		printk(KERN_NOTICE "NMC3: WRITE operation interrupted.\n");
		return ret;
	}
	for (ln = count; ln > 0; ln -= chunk, ubuff += chunk) {
		chunk = (ln < sizeof (kbuff)) ? ln : sizeof (kbuff);
		dbg ("ln=%d,chunk=%d",ln,chunk);
		nmc3_mem_read (dev_ex->nmc3_dscr[minor], kbuff, *offp, chunk);
		if ( copy_to_user(buff, kbuff, chunk)) {
			ret = -EFAULT;
			goto read_out;
		}
		dbg ("offset = 0x%llx, chunk = %d,kbuff = 0x%x 0x%x 0x%x",
			*offp, chunk, ((__u32*)kbuff)[0],
			((__u32*)kbuff)[1], ((__u32*)kbuff)[2]);
	}
read_out:
	up(&dev_ex->sem);
	return count;
}

static ssize_t NMC3_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	struct NMC3_extension *dev_ex = filp->private_data;
	__u8 kbuff [1024];	/* 64 KB */
	int ln;
	int chunk;
	const char __user *ubuff = buff;
	int ret;
	int minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	dbg ("XXXXXXXXXXX%s,offp=0x%llx,count=%d,dev_ex=%p", __func__, *offp,
		count, dev_ex);
	if ((ret = down_interruptible(&dev_ex->sem)) < 0) {
		printk(KERN_NOTICE "NMC3: WRITE operation interrupted.\n");
		return ret;
	}
	dbg ("before for");
	for (ln = count; ln > 0; ln -= chunk, ubuff += chunk) {
		chunk = (ln < sizeof (kbuff)) ? ln : sizeof (kbuff);
		dbg ("ln=%d,chunk=%d",ln,chunk);
		if ( copy_from_user(kbuff, buff, chunk)) {
			ret = -EFAULT;
			goto write_out;
		}
		dbg ("offset = 0x%llx, chunk = %d,kbuff = 0x%x 0x%x 0x%x",
			*offp, chunk, ((__u32*)kbuff)[0],
			((__u32*)kbuff)[1], ((__u32*)kbuff)[2]);
		nmc3_mem_write (dev_ex->nmc3_dscr[minor], *offp, kbuff, chunk);
	}
write_out:
	up(&dev_ex->sem);
	return 0;
}

static long NMC3_ioctl(struct file *filp, unsigned int ioctl_num, unsigned long ioctl_param)
{
	struct NMC3_extension *dev_ex = filp->private_data;
	int ret;
	struct __app_run_param app_run_param;
	struct __app_irq_param app_irq_param;
	struct nm_irq_data app_irqwait_param;
	void* kernel_app_array = NULL;
	unsigned long lost_app_ln;
	__u32 app_entry;
	int minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	if ((ret = down_interruptible(&dev_ex->sem)) < 0) {
		printk(KERN_NOTICE "NMC3: IOCTL operation interrupted.\n");
		return ret;
	}
		
	switch (ioctl_num) {
	case IOCTL_NMC3_START:
		ret = nmc3_start (dev_ex->nmc3_dscr[minor], NULL, 0); /* TODO: to try
								initcode as file */
		if (ret < 0) {
			dbg ("nmc3_start ERROR %d",ret);
			goto ioctl_out;
		}
		break;
	case IOCTL_NMC3_APP_RUN:
		if ( copy_from_user(&app_run_param,
				(struct __app_run_param*)ioctl_param,
				sizeof (app_run_param))) {
			ret = -EFAULT;
			goto ioctl_out;
		}
#if 0
		if (!(kernel_app_array = kmalloc (app_run_param.app_ln, GFP_KERNEL))) {
			ret = -ENOMEM;
			goto ioctl_out;
		}
#else
		if (!(kernel_app_array = vmalloc (app_run_param.app_ln))) {
			ret = -ENOMEM;
			goto ioctl_out;
		}
		dbg ("OK");
#endif
		dbg ("array = %p, app_ptr = %p, ln = %d",
			kernel_app_array, app_run_param.app_ptr,app_run_param.app_ln);
		if ((lost_app_ln = copy_from_user (kernel_app_array,
				app_run_param.app_ptr, app_run_param.app_ln))) {
			dbg ("copy_from_user ERROR, %ld", lost_app_ln);
			ret = -EFAULT;
			goto ioctl_run_free;
		}
		dbg ("lost_app_ln=%ld",lost_app_ln);
		if ((ret = nmc3_app_stop_irq (dev_ex->nmc3_dscr[minor])) < 0) {
			dbg ("ERROR %d", ret);
			goto ioctl_run_free;
		}
		dbg ("appstop(), ret=%d",ret);
		if ((ret = nmc3_elf_upload (dev_ex->nmc3_dscr[minor], kernel_app_array, 
				app_run_param.app_ln, &app_entry)) < 0) {
			dbg ("ERROR %d", ret);
			goto ioctl_run_free;
		};
		dbg ("nmc3_elf_upload() OK, ret=%d", ret);
		((struct __NMC3_thread_descriptor*)dev_ex->nmc3_dscr[minor])->data = dev_ex;
		if ((ret = nmc3_set_handler ((struct __NMC3_thread_descriptor*)dev_ex->nmc3_dscr[minor],
				irq_cb)) < 0)
			goto ioctl_out;
		if ((ret = nmc3_app_run (dev_ex->nmc3_dscr[minor], app_entry,
			app_run_param.param, app_callback)) < 0) { 
			dbg ("ERROR %d",ret);
			goto ioctl_run_free;
		}
		dbg ("nmc3_app_run() OK");
		if (app_run_param.wait_nmapp_app)
			while (!app_start_sign);
		app_start_sign = 0;
		dbg ("app start irq waiting OK");
ioctl_run_free:
#if 0
		kfree(kernel_app_array);
#else
		vfree(kernel_app_array);
#endif
		break;
	case IOCTL_NMC3_APP_STOP_NMI:
		if ((ret = nmc3_app_stop_nmi (dev_ex->nmc3_dscr[minor])) < 0) {
			dbg ("ERROR %d",ret);
			goto ioctl_out;
		}
		break;
	case IOCTL_NMC3_APP_STOP_IRQ:
		if ((ret = nmc3_app_stop_irq (dev_ex->nmc3_dscr[minor])) < 0) {
			dbg ("ERROR %d",ret);
			goto ioctl_out;
		}
		print_if((struct __NMC3_thread_descriptor*)dev_ex->nmc3_dscr[minor]);
		break;
	case IOCTL_NMC3_IRQREQUEST:
		dbg ("IOCTL_NMC3_IRQREQUEST");
		if (copy_from_user(&app_irq_param,
				(struct __app_irq_param*)ioctl_param,
				sizeof app_irq_param)) {
			ret = -EFAULT;
			goto ioctl_out;
		}
		switch (app_irq_param.irq_cb_type) {
		case NM_STEST_TYPE:
			if ((ret = nmc3_set_handler (
					(struct __NMC3_thread_descriptor*)dev_ex->nmc3_dscr[minor],
					xan_cb)) < 0)
				goto ioctl_out;
			mark_2cb_set = 0;
			break;
		case NM_STOP_TYPE:
			if ((ret = nmc3_set_handler (
					(struct __NMC3_thread_descriptor*)dev_ex->nmc3_dscr[minor],
					 irq_cb)) < 0)
				goto ioctl_out;
			break;
		case NM_2PTEST_TYPE:
			if ((ret = nmc3_set_handler (
					(struct __NMC3_thread_descriptor*)dev_ex->nmc3_dscr[minor],
					xan_2cb)) < 0)
				goto ioctl_out;
			mark_2cb_set = 1;
		default:
				printk ("wrong interrupt type ERROR 0x%x\n",app_irq_param.irq_cb_type);
				ret = -EFAULT;
				goto ioctl_out;
		}
		if ((ret = nmc3_request_interrupt (dev_ex->nmc3_dscr[minor],
				app_irq_param.param)) < 0) {
			dbg ("ERROR %d",ret);
			goto ioctl_out;
		}
		break;
	case IOCTL_ARM_IRQWAIT:
		if ((ret = down_interruptible(&xan_data.xan_sync)) < 0) {
			printk(KERN_NOTICE "%s#%d: NMC3: xan irq waiting interrupted %d\n",
				__FILE__,__LINE__,ret);
			goto ioctl_out;
		};
		dbg ("xan_sync, ret = %d, mark_2cb_set=0x%x, mark_2cb=0x%x",ret,
			mark_2cb_set, mark_2cb);
		if (mark_2cb_set ^ mark_2cb) {
			ret = -EINVAL;
			goto ioctl_out;
		}
		ret = nmc3_mem_read ( (struct __NMC3_thread_descriptor*)dev_ex->nmc3_dscr[minor],
			&app_irqwait_param, xan_data.data, sizeof(app_irqwait_param));
		if (ret < 0)
			goto ioctl_out;
		if (ret != sizeof(app_irqwait_param)) {
			dbg ("wrong nmc3_mem_read() length read %d",ret);
			ret = -EBADE;
			goto ioctl_out;
		}
		if (copy_to_user ((void*)ioctl_param, &app_irqwait_param, ret)) {
			ret = -EFAULT;
			goto ioctl_out;
		}
		break;
	case IOCTL_NMC3_APP_WAIT:
		if ((ret = down_interruptible(&dev_ex->app_complete_sem)) < 0) {
			printk(KERN_NOTICE "%s#%d: NMC3: app_complete op interrupted %d\n",
				__FILE__,__LINE__,ret);
			print_if((struct __NMC3_thread_descriptor*)dev_ex->nmc3_dscr[minor]);
			goto ioctl_out;
		}
		dbg ("nmc3_app_run COMPLETED");
		print_if((struct __NMC3_thread_descriptor*)dev_ex->nmc3_dscr[minor]);
		ret = 0;
		dbg ("dev_ex=%p, ret_code = dev_ex->ret_code=0x%x, ret=0x%x",
			dev_ex, dev_ex->ret_code, ret);
		if (copy_to_user((void*)ioctl_param, &dev_ex->ret_code, sizeof(dev_ex->ret_code))) {
			dbg ("copy_to_user ERROR");
			ret = -EINVAL;
			goto ioctl_out;
		}
		break;
	}
ioctl_out:
	up(&dev_ex->sem);
	return ret;
}

struct file_operations fops =
{
	.owner          = THIS_MODULE,
	.open           = NMC3_open,
	.release        = NMC3_release,
	.llseek         = NMC3_llseek,
	.read           = NMC3_read,
	.write          = NMC3_write,
	.unlocked_ioctl = NMC3_ioctl,
};

static int __init rcmod_soc_nmc3_test_init (void)
{
	int ret;
	struct NMC3_extension *dev_ex;
	struct device* NMC3_device [TEST_NMC3_MINORS];
	int i;

	dbg("%s", __func__);

	NMC3_class = class_create(THIS_MODULE, NMC3_FILENAME);
	dbg ("NMC3_class=%p",NMC3_class);
	if (IS_ERR(NMC3_class))
	{
		printk(KERN_ERR "%s#%d: NMC3: ERROR: Can not create device class!\n",
			__FILE__,__LINE__);
		ret = -ENXIO;
		goto Fail_NMC3_init_0;
	}

	ret = register_chrdev_region(MKDEV(NMC3_MAJOR, 0), TEST_NMC3_MINORS, NMC3_FILENAME);
	if (ret)
	{
		printk(KERN_ERR "NMC3: ERROR: Can not register device numbers!\n");
		goto Fail_NMC3_init_1;
	}
	dbg ("ret = %d",ret);
	dev_ex = kzalloc(sizeof(struct NMC3_extension), GFP_KERNEL);
	if (!dev_ex)
	{
		ret = -ENOMEM;
		printk(KERN_ERR "NMC3: ERROR: Insufficient memory!\n");
		goto Fail_NMC3_init_2;
	}

	sprintf(dev_ex->Name, "%s_%u", NMC3_FILENAME, 0);

	printk ("dev_ex->Name=%s",dev_ex->Name);
	sema_init(&dev_ex->sem, 1);
	sema_init(&dev_ex->app_complete_sem, 0);
	sema_init(&xan_data.xan_sync, 0);

	cdev_init(&dev_ex->cdev, &fops);
	dev_ex->cdev.owner = THIS_MODULE;
	dev_ex->cdev.ops = &fops;
	this_device = dev_ex;

	ret = cdev_add(&dev_ex->cdev, MKDEV(NMC3_MAJOR, 0), TEST_NMC3_MINORS);
	if (ret) {
		printk(KERN_ERR "NMC3: ERROR: Can not adding char device!\n");
		goto Fail_NMC3_init_3;
	}

	dbg ("ret = %d, dev_numbers=0x%08x", ret, MKDEV(NMC3_MAJOR, 0));
	for (i=0; i<TEST_NMC3_MINORS; i++) {
		NMC3_device[i] = device_create(NMC3_class, NULL, MKDEV(NMC3_MAJOR,i), NULL, "nmc3_%u", i);
		if (IS_ERR(NMC3_device[i]))
		{
			printk(KERN_ERR "NMC3: ERROR: Can not adding device to class!\n");
			ret = -ENXIO;
			goto Fail_NMC3_init_4;
		}
		dbg ("NMC3_device = %p", NMC3_device[i]);
	}
	dbg("%s OK", __func__);
	ret = 0;
	goto Fail_NMC3_init_0;

Fail_NMC3_init_4:
	for (; i>0; i--)
		device_destroy(NMC3_class, MKDEV(NMC3_MAJOR, i-1));
	cdev_del(&dev_ex->cdev);
Fail_NMC3_init_3:
	kfree(dev_ex);
Fail_NMC3_init_2:
	unregister_chrdev_region(MKDEV(NMC3_MAJOR, 0), TEST_NMC3_MINORS);

Fail_NMC3_init_1:
	class_destroy(NMC3_class);

Fail_NMC3_init_0:

	return ret;
}

static void /*__exit*/ rcmod_soc_nmc3_test_exit(void)
{
	dev_t dev_numbers = MKDEV(NMC3_MAJOR, 0);
	int i;

	dbg("%s, Begin.", __func__);

	for (i=0; i<TEST_NMC3_MINORS; i++)
		device_destroy(NMC3_class, MKDEV(NMC3_MAJOR, i));
	cdev_del(&this_device->cdev);
	unregister_chrdev_region(dev_numbers, TEST_NMC3_MINORS);
	class_destroy(NMC3_class);
	kfree(this_device);
	this_device = NULL;

	dbg("%s, End.", __func__);
}


module_init(rcmod_soc_nmc3_test_init);
module_exit(rcmod_soc_nmc3_test_exit);
        
/* Module parameters */
MODULE_AUTHOR ("Gennadiy Kurtsman <gkurtsman@module.ru>");
MODULE_DESCRIPTION ("RC MODULE SOC: Testing driver for Driver of ARM-NMC3 interface");
MODULE_LICENSE("GPL");
MODULE_VERSION(TEST_NMC3_DRIVER_VERSION);
        

