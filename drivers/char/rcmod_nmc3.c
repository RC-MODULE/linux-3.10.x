/* RC Module BBP Soc. ARM-NMC3 interface driver
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
#include <linux/irq.h>
#include <linux/of.h>

#include "rcmod_nmc3.h"
#include "rcmod_nmc3_ioctl.h"



static struct class *NMC3_class;

static void __iomem *NMUREGS_m;

static int NMC3_open(struct inode *inode, struct file *filp)
{
	struct NMC3_extension *dev_ex = container_of(inode->i_cdev, struct NMC3_extension, cdev);

	dbg("Begin; Num = %u.", dev_ex->Num);

	filp->private_data = dev_ex;

	dbg("End; Return: 0.");
	return 0;
}

static int NMC3_release(struct inode *inode, struct file *filp)
{
#ifdef DEBUG
	struct NMC3_extension *dev_ex = container_of(inode->i_cdev, struct NMC3_extension, cdev);
#endif

	dbg("Begin; Num = %u.", dev_ex->Num);

	dbg("End; Return: 0.");
	return 0;
}

static loff_t NMC3_llseek(struct file *filp, loff_t offset, int from)
{
	struct NMC3_extension *dev_ex = filp->private_data;
	loff_t New_offset;

	dbg("Begin; Num = %u; offset = %llXh.", dev_ex->Num, offset);

	switch (from)
	{
	case SEEK_SET:
		dbg("case SEEK_SET.");
		New_offset = offset;
		break;

	case SEEK_CUR:
		dbg("case SEEK_CUR.");
		New_offset = filp->f_pos + offset;
		break;

	case SEEK_END:
		dbg("case SEEK_END.");
		New_offset = dev_ex->mem_im1_3_size + offset;
		break;

	default:
		dbg("case default.");
		printk(KERN_ERR "NMC3: ERROR: Seek ERROR!\n");
		dbg("End (With ERROR!); Return: -EINVAL!");
		return -EINVAL;
	}

	if (!( ((New_offset >= dev_ex->mem_im0_u) && (New_offset < dev_ex->mem_im0_u + dev_ex->mem_im0_size)) ||
		((New_offset >= dev_ex->mem_im1_3_u) && (New_offset < dev_ex->mem_im1_3_u + dev_ex->mem_im1_3_size)) ||
		((New_offset >= dev_ex->mem_em1_u) && (New_offset < dev_ex->mem_em1_u + dev_ex->mem_em1_size))))
	{
		printk(KERN_ERR "NMC3: ERROR: Can not set offset %llXh!\n", New_offset);
        printk(KERN_ERR "mem_im0_u %08x - %08x\n", dev_ex->mem_im0_u, dev_ex->mem_im0_u + dev_ex->mem_im0_size);
        printk(KERN_ERR "mem_im1_3_u %08x - %08x\n", dev_ex->mem_im1_3_u, dev_ex->mem_im1_3_u + dev_ex->mem_im1_3_size);
        printk(KERN_ERR "mem_em1_u %08x - %08x\n", dev_ex->mem_em1_u, dev_ex->mem_em1_u + dev_ex->mem_em1_size);
		dbg("End (With ERROR!); Return: -EINVAL!");
		return -EINVAL;
	}

	filp->f_pos = New_offset;

	dbg("End; filp->f_pos = %llXh = %lld.", filp->f_pos, filp->f_pos);
	return filp->f_pos;
}

static ssize_t NMC3_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	struct NMC3_extension *dev_ex = filp->private_data;
	u32 Real_offset = (*offp >> 2) << 2;
	size_t Real_count = (count >> 2) << 2;
	u32 Mem_u, Mem_size;
	void __iomem *Mem_m;

	dbg("Begin; Num = %u.", dev_ex->Num);

	if (Real_offset != *offp)
		printk(KERN_WARNING "NMC3: WARNING: Read offset %llXh is not multiple 4! Truncated.\n", *offp);
	if (Real_count != count)
		printk(KERN_WARNING "NMC3: WARNING: Read count %Xh is not multiple 4! Truncated.\n", count);

	if (down_interruptible(&dev_ex->sem))
	{
		printk(KERN_NOTICE "NMC3: Read operation interrupted.\n");
		dbg("End; Return: -ERESTARTSYS!");
		return -ERESTARTSYS;
	}

	if (!Real_count)
	{
		printk(KERN_WARNING "NMC3: WARNING: Requested read 0 DWORDS!\n");
		up(&dev_ex->sem);
		dbg("End (With WARNING!); Return: 0!");
		return 0;
	}

	if ((Real_offset >= dev_ex->mem_im0_u) && (Real_offset < dev_ex->mem_im0_u + dev_ex->mem_im0_size))
	{
		Mem_u = dev_ex->mem_im0_u;
		Mem_size = dev_ex->mem_im0_size;
		Mem_m = dev_ex->mem_im0_m;
	}
	else if ((Real_offset >= dev_ex->mem_im1_3_u) && (Real_offset < dev_ex->mem_im1_3_u + dev_ex->mem_im1_3_size))
	{
		Mem_u = dev_ex->mem_im1_3_u;
		Mem_size = dev_ex->mem_im1_3_size;
		Mem_m = dev_ex->mem_im1_3_m;
	}
	else if ((Real_offset >= dev_ex->mem_em1_u) && (Real_offset < dev_ex->mem_em1_u + dev_ex->mem_em1_size))
	{
		Mem_u = dev_ex->mem_em1_u;
		Mem_size = dev_ex->mem_em1_size;
		Mem_m = dev_ex->mem_em1_m;
	}
	else
	{
		printk(KERN_ERR "NMC3: ERROR: Can not read with offset %8.8Xh!\n", Real_offset);
		up(&dev_ex->sem);
		dbg("End (With ERROR!); Return: 0!");
		return 0;
	}

	if (Real_count > Mem_u + Mem_size - Real_offset)
	{
		printk(KERN_WARNING "NMC3: WARNING: Requested read area is out of memory! Truncated.\n");
		Real_count = Mem_u + Mem_size - Real_offset;
	}

	if (copy_to_user(buff, Mem_m + (Real_offset - Mem_u), Real_count))
	{
		printk(KERN_ERR "NMC3: ERROR: Can not copy to user space!\n");
		up(&dev_ex->sem);
		dbg("End (With ERROR!); Return: -EFAULT!");
		return -EFAULT;
	}

	*offp += Real_count;

	up(&dev_ex->sem);

	dbg("End; Real_count = %Xh = %d.", Real_count, Real_count);
	return Real_count;
}

static ssize_t NMC3_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	struct NMC3_extension *dev_ex = filp->private_data;
	u32 Real_offset = (*offp >> 2) << 2;
	size_t Real_count = (count >> 2) << 2;
	u32 Mem_u, Mem_size;
	void __iomem *Mem_m;

	dbg("Begin; Num = %u.", dev_ex->Num);

	if (Real_offset != *offp)
		printk(KERN_WARNING "NMC3: WARNING: Write offset %llXh is not multiple 4! Truncated.\n", *offp);
	if (Real_count != count)
		printk(KERN_WARNING "NMC3: WARNING: Write count %Xh is not multiple 4! Truncated.\n", count);

	if (down_interruptible(&dev_ex->sem))
	{
		printk(KERN_NOTICE "NMC3: Write operation interrupted.\n");
		dbg("End; Return: -ERESTARTSYS!");
		return -ERESTARTSYS;
	}

	if (!Real_count)
	{
		printk(KERN_WARNING "NMC3: WARNING: Requested write 0 DWORDS!\n");
		up(&dev_ex->sem);
		dbg("End (With WARNING!); Return: 0!");
		return 0;
	}

	if ((Real_offset >= dev_ex->mem_im0_u) && (Real_offset < dev_ex->mem_im0_u + dev_ex->mem_im0_size))
	{
		Mem_u = dev_ex->mem_im0_u;
		Mem_size = dev_ex->mem_im0_size;
		Mem_m = dev_ex->mem_im0_m;
	}
	else if ((Real_offset >= dev_ex->mem_im1_3_u) && (Real_offset < dev_ex->mem_im1_3_u + dev_ex->mem_im1_3_size))
	{
		Mem_u = dev_ex->mem_im1_3_u;
		Mem_size = dev_ex->mem_im1_3_size;
		Mem_m = dev_ex->mem_im1_3_m;
	}
	else if ((Real_offset >= dev_ex->mem_em1_u) && (Real_offset < dev_ex->mem_em1_u + dev_ex->mem_em1_size))
	{
		Mem_u = dev_ex->mem_em1_u;
		Mem_size = dev_ex->mem_em1_size;
		Mem_m = dev_ex->mem_em1_m;
	}
	else
	{
		printk(KERN_ERR "NMC3: ERROR: Can not write with offset %8.8Xh!\n", Real_offset);
		up(&dev_ex->sem);
		dbg("End (With ERROR!); Return: 0!");
		return 0;
	}

	if (Real_count > Mem_u + Mem_size - Real_offset)
	{
		printk(KERN_WARNING "NMC3: WARNING: Requested write area is out of memory! Truncated.\n");
		Real_count = Mem_u + Mem_size - Real_offset;
	}

	if (copy_from_user(Mem_m + (Real_offset - Mem_u), buff, Real_count))
	{
		printk(KERN_ERR "NMC3: ERROR: Can not copy from user space!\n");
		up(&dev_ex->sem);
		dbg("End (With ERROR!); Return: -EFAULT!");
		return -EFAULT;
	}

	*offp += Real_count;

	up(&dev_ex->sem);

	dbg("End; Real_count = %Xh = %d.", Real_count, Real_count);
	return Real_count;
}

static long NMC3_ioctl(struct file *filp, unsigned int ioctl_num, unsigned long ioctl_param)
{
	struct NMC3_extension *dev_ex = filp->private_data;
	int ret = 0;

	dbg("Begin; Num = %u.", dev_ex->Num);

	if (down_interruptible(&dev_ex->sem))
	{
		printk(KERN_NOTICE "NMC3: IOCTL operation interrupted.\n");
		dbg("End; Return: -ERESTARTSYS!");
		return -ERESTARTSYS;
	}

	switch (ioctl_num)
	{
    case IOCTL_NMC3_RESET:
		dbg("case IOCTL_NMC3_RESET.");
		iowrite32(0xFF, NMUREGS_m + NMURESET_offset);
		break;

	case IOCTL_NMC3_SEND_NMI:
		dbg("case IOCTL_NMC3_SEND_NMI.");
        iowrite32(0x1, NMUREGS_m + NMUINTREQ_offset);
		break;

	case IOCTL_NMC3_SEND_HPINT:
		dbg("case IOCTL_NMC3_SEND_HPINT.");
        iowrite32(0x1, NMUREGS_m + NMUINTREQH_offset);
		break;

	case IOCTL_NMC3_SEND_LPINT:
		dbg("case IOCTL_NMC3_SEND_LPINT.");
        iowrite32(0x1, NMUREGS_m + NMUINTREQL_offset);
		break;

	case IOCTL_NMC3_MASK_HP_INTERRUPT:
	case IOCTL_NMC3_MASK_LP_INTERRUPT:
	{
		u32 User_mask = 0x0;
		int irq_XP;
		struct irq_desc *irq_XP_desc;

		dbg("case IOCTL_NMC3_MASK_XP_INTERRUPT.");

		ret = get_user(User_mask, (u32 __user *)ioctl_param);
		if (ret)
		{
			printk(KERN_ERR "NMC3: ERROR: Can not get new mask!\n");
			break;
		}

		User_mask &= 0x1;

		dbg("User_mask = %8.8Xh.", User_mask);

		if (ioctl_num == IOCTL_NMC3_MASK_HP_INTERRUPT)
		{
			dbg("irq_HP = %d", dev_ex->irq_HP);
			irq_XP = dev_ex->irq_HP;
		}
		else
		{
			dbg("irq_LP = %d", dev_ex->irq_LP);
			irq_XP = dev_ex->irq_LP;
		}

		irq_XP_desc = irq_to_desc(irq_XP);

		if (User_mask)
		{
			while (irq_XP_desc->depth)
			{
				enable_irq(irq_XP);
			}
		}
		else
		{
			if (!irq_XP_desc->depth)
			{
				disable_irq(irq_XP);
			}
		}

		break;
	}

	case IOCTL_NMC3_WAIT_HP_INTERRUPT:
	case IOCTL_NMC3_WAIT_LP_INTERRUPT:
	{
		struct Wait_interrupt *wi;
		u32 Time_out = 0, int_status = 0;
		unsigned long jif = 0;

		dbg("case IOCTL_NMC3_WAIT_XP_INTERRUPT.");

		ret = get_user(Time_out, (u32 __user *)ioctl_param);
		if (ret)
		{
			printk(KERN_ERR "NMC3: ERROR: Can not get timeout value for wait interrupt!\n");
			break;
		}

		wi = kzalloc(sizeof(struct Wait_interrupt), GFP_KERNEL);
		if (!wi)
		{
			printk(KERN_ERR "NMC3: ERROR: Insufficient memory!\n");
			ret = -ENOMEM;
			break;
		}

		wi->tgid = current->tgid;
		wi->int_type = (ioctl_num == IOCTL_NMC3_WAIT_HP_INTERRUPT) ? 1 : 2;

		dbg("tgid = %Xh; int_type = %u.", wi->tgid, wi->int_type);

		up(&dev_ex->sem);

		spin_lock_bh(&dev_ex->Wait_interrupt_lock);
		list_add_tail(&wi->list, &dev_ex->Wait_interrupt_list);
		spin_unlock_bh(&dev_ex->Wait_interrupt_lock);
	
		if (Time_out == 0xFFFFFFFF)
		{
			dbg("Wake down (sleep)...");
			ret = wait_event_interruptible(dev_ex->queue, wi->int_NM);
		}
		else if (!Time_out)
		{
			dbg("Wake down (sleep with 1 jiff)...");
			ret = wait_event_interruptible_timeout(dev_ex->queue, wi->int_NM, 1);
		}
		else
		{
			if (Time_out > (0xFFFFFFFF / HZ))
			{
				dbg("Timeout (%u) greater than maximum. Truncated.", Time_out);
				Time_out = 0xFFFFFFFF / HZ;
			}

			jif = Time_out * HZ / 1000;
			if (!jif)
			{
				jif = 1;
			}

			dbg("Wake down (sleep with timeout %u ms (%lu jiffies))...", Time_out, jif);
			ret = wait_event_interruptible_timeout(dev_ex->queue, wi->int_NM, jif);
		}
		dbg("Wake up.");

		spin_lock_bh(&dev_ex->Wait_interrupt_lock);
		list_del(&wi->list);
		spin_unlock_bh(&dev_ex->Wait_interrupt_lock);

		int_status = (wi->int_NM) ? (wi->int_status) : (-1);

		kfree(wi);

		if (ret != -ERESTARTSYS)
		{
			dbg("int_status = %d.", int_status);
			ret = put_user(int_status, (u32 __user *)ioctl_param);
			if (ret)
			{
				printk(KERN_ERR "NMC3: ERROR: Can not write interrupt status to user space!\n");
			}
		}
		else
		{
			printk(KERN_NOTICE "NMC3: Wait interrupt interrupted.\n");
		}

		dbg("End; ret = %Xh = %d.", ret, ret);
		return ret;
	}

	case IOCTL_NMC3_CANCEL_WAIT_HP_INTERRUPT:
	case IOCTL_NMC3_CANCEL_WAIT_LP_INTERRUPT:
	{
		struct list_head *ptr;
		struct Wait_interrupt *wi;
		unsigned int int_type = (ioctl_num == IOCTL_NMC3_CANCEL_WAIT_HP_INTERRUPT) ? 1 : 2;

		dbg("case IOCTL_NMC3_CANCEL_WAIT_XP_INTERRUPT; int_type = %u.", int_type);

		up(&dev_ex->sem);

		spin_lock_bh(&dev_ex->Wait_interrupt_lock);

		list_for_each(ptr, &dev_ex->Wait_interrupt_list)
		{
			wi = list_entry(ptr, struct Wait_interrupt, list);

			dbg("Before iteration: tgid = %Xh; int_type = %u.", wi->tgid, wi->int_type);

			if ((wi->tgid == current->tgid) && (wi->int_type == int_type))
			{
				wi->int_NM = 1;
			}

			dbg("After iteration: int_NM = %d.", wi->int_NM);
		}

		spin_unlock_bh(&dev_ex->Wait_interrupt_lock);

		wake_up_interruptible(&dev_ex->queue);

		dbg("End; Return: 0.");
		return 0;
	}

	case IOCTL_NMC3_WAIT_SYSTEM_INTERRUPT:
	{
		u32 IOCTL_parameters[2] = {0, 0};
		u32 Int_type = 0, Timeout = 0;
		u32 Int_status = 0;
		unsigned long jif = 0, jif_end = 0;

		dbg("case IOCTL_NMC3_WAIT_SYSTEM_INTERRUPT.");

		ret = copy_from_user(IOCTL_parameters, (void __user *)ioctl_param, 8);
		if (ret)
		{
			printk(KERN_ERR "NMC3: ERROR: Can not get IOCTL parameters!\n");
			break;
		}

		Int_type = IOCTL_parameters[0];
		Timeout = IOCTL_parameters[1];

		up(&dev_ex->sem);

		if ((Timeout) && (Timeout != 0xFFFFFFFF))
		{
			jif = jiffies;

			if (Timeout > (0xFFFFFFFF / HZ))
			{
				dbg("Timeout (%u) greater than maximum. Truncated.", Timeout);
				Timeout = 0xFFFFFFFF / HZ;
			}
			dbg("Timeout = %u.", Timeout);

			jif_end = jif + Timeout * HZ / 1000;
		}

		while (1)
		{
			if (Timeout == 0xFFFFFFFF)
			{
				if (Int_type == READY_FOR_COMMAND)
				{
					dbg("Int_type = READY_FOR_COMMAND.");
					dbg("Wake down (sleep)...");
					ret = wait_event_interruptible(
						dev_ex->queue,
						(!(dev_ex->User_core_lock)) && (dev_ex->Ready_for_command));
					dbg("Wake up; ret = %Xh = %d.", ret, ret);
				}
				else if (Int_type == PROGRAM_STOPPED)
				{
					dbg("Int_type = PROGRAM_STOPPED.");
					dbg("Wake down (sleep)...");
					ret = wait_event_interruptible(
						dev_ex->queue,
						(!(dev_ex->User_core_lock)) && (dev_ex->Program_stopped));
					dbg("Wake up; ret = %Xh = %d.", ret, ret);
				}
				else if (Int_type == SYNC_FLAG)
				{
					dbg("Int_type = SYNC_FLAG.");
					dbg("Wake down (sleep)...");
					ret = wait_event_interruptible(
						dev_ex->queue,
						(!(dev_ex->User_core_lock)) && (dev_ex->Sync_flag));
					dbg("Wake up; ret = %Xh = %d.", ret, ret);
				}
				else
				{
					dbg("Int_type: User core lock.");
					dbg("Wake down (sleep)...");
					ret = wait_event_interruptible(
						dev_ex->queue,
						!(dev_ex->User_core_lock));
					dbg("Wake up; ret = %Xh = %d.", ret, ret);
				}

				if (ret == -ERESTARTSYS)
				{
					printk(KERN_NOTICE "NMC3: Wait interrupt interrupted.\n");
					dbg("End; Return: -ERESTARTSYS.");
					return -ERESTARTSYS;
				}
			}
			else if (Timeout) // Timeout
			{
				jif = jiffies;

				if (time_before(jif, jif_end))
				{
					if (Int_type == READY_FOR_COMMAND)
					{
						dbg("Int_type = READY_FOR_COMMAND.");
						dbg("Wake down (sleep with timeout %lu jiffies)...", (long)jif_end - (long)jif);
						ret = wait_event_interruptible_timeout(
							dev_ex->queue,
							(!(dev_ex->User_core_lock)) && (dev_ex->Ready_for_command),
							(long)jif_end - (long)jif);
						dbg("Wake up; ret = %Xh = %d.", ret, ret);
					}
					else if (Int_type == PROGRAM_STOPPED)
					{
						dbg("Int_type = PROGRAM_STOPPED.");
						dbg("Wake down (sleep with timeout %lu jiffies)...", (long)jif_end - (long)jif);
						ret = wait_event_interruptible_timeout(
							dev_ex->queue,
							(!(dev_ex->User_core_lock)) && (dev_ex->Program_stopped),
							(long)jif_end - (long)jif);
						dbg("Wake up; ret = %Xh = %d.", ret, ret);
					}
					else if (Int_type == SYNC_FLAG)
					{
						dbg("Int_type = SYNC_FLAG.");
						dbg("Wake down (sleep with timeout %lu jiffies)...", (long)jif_end - (long)jif);
						ret = wait_event_interruptible_timeout(
							dev_ex->queue,
							(!(dev_ex->User_core_lock)) && (dev_ex->Sync_flag),
							(long)jif_end - (long)jif);
						dbg("Wake up; ret = %Xh = %d.", ret, ret);
					}
					else
					{
						dbg("Int_type: User core lock.");
						dbg("Wake down (sleep with timeout %lu jiffies)...", (long)jif_end - (long)jif);
						ret = wait_event_interruptible_timeout(
							dev_ex->queue,
							!(dev_ex->User_core_lock),
							(long)jif_end - (long)jif);
						dbg("Wake up; ret = %Xh = %d.", ret, ret);
					}

					if (ret == -ERESTARTSYS)
					{
						printk(KERN_NOTICE "NMC3: Wait interrupt interrupted.\n");
						dbg("End; Return: -ERESTARTSYS.");
						return -ERESTARTSYS;
					}

					if (!ret)
					{
						dbg("Time almost elapsed.");
						Int_status = -1;
					}
				}
				else
				{
					dbg("Time almost elapsed.");
					Int_status = -1;
				}
			} // Timeout

			dbg("Attempt of interrupt grab...");

			spin_lock_bh(&dev_ex->NM_status_lock);

			if (!(dev_ex->User_core_lock))
			{
				if (Int_type == READY_FOR_COMMAND)
				{
					dbg("Int_type = READY_FOR_COMMAND.");

					if (dev_ex->Ready_for_command)
					{
						dev_ex->User_core_lock = 1;
						dev_ex->Ready_for_command = 0;
						Int_status = 1;

						spin_unlock_bh(&dev_ex->NM_status_lock);

						dbg("Interrupt grabbed.");
						break;
					}
				}
				else if (Int_type == PROGRAM_STOPPED)
				{
					dbg("Int_type = PROGRAM_STOPPED.");

					if (dev_ex->Program_stopped)
					{
						dev_ex->User_core_lock = 1;
						dev_ex->Program_stopped = 0;
						Int_status = 1;

						spin_unlock_bh(&dev_ex->NM_status_lock);

						dbg("Interrupt grabbed.");
						break;
					}
				}
				else if (Int_type == SYNC_FLAG)
				{
					dbg("Int_type = SYNC_FLAG.");

					if (dev_ex->Sync_flag)
					{
						dev_ex->User_core_lock = 1;
						dev_ex->Sync_flag = 0;
						Int_status = 1;

						spin_unlock_bh(&dev_ex->NM_status_lock);

						dbg("Interrupt grabbed.");
						break;
					}
				}
				else
				{
					dbg("Int_type: User core lock.");

					dev_ex->User_core_lock = 1;
					Int_status = 1;

					spin_unlock_bh(&dev_ex->NM_status_lock);

					dbg("Interrupt grabbed.");
					break;
				}
			} // if (User_core_lock == 0)

			spin_unlock_bh(&dev_ex->NM_status_lock);

			dbg("Interrupt not grabbed.");

			if (!Timeout)
			{
				break;
			}

			if (Int_status == -1)
			{
				dbg("Time elapsed.");
				break;
			}
		} // while (1)

		dbg("Int_status = %d.", Int_status);

		ret = put_user(Int_status, (u32 __user *)ioctl_param);
		if (ret)
		{
			printk(KERN_ERR "NMC3: ERROR: Can not write interrupt status to user space!\n");
		}

		dbg("End; ret = %Xh = %d.", ret, ret);
		return ret;
	}

	case IOCTL_NMC3_RESET_SYSTEM_INTERRUPT:
		dbg("case IOCTL_NMC3_RESET_SYSTEM_INTERRUPT.");
		spin_lock_bh(&dev_ex->NM_status_lock);
		dbg("Before resetting: Ready_for_command = %d; Program_stopped = %d; Sync_flag = %d; User_core_lock = %d.",
			dev_ex->Ready_for_command, dev_ex->Program_stopped, dev_ex->Sync_flag, dev_ex->User_core_lock);
		dev_ex->Ready_for_command = 0;
		dev_ex->Program_stopped = 0;
		dev_ex->Sync_flag = 0;
		spin_unlock_bh(&dev_ex->NM_status_lock);
		break;

	case IOCTL_NMC3_USER_CORE_UNLOCK:
		dbg("case IOCTL_NMC3_USER_CORE_UNLOCK.");
		spin_lock_bh(&dev_ex->NM_status_lock);
		dev_ex->User_core_lock = 0;
		spin_unlock_bh(&dev_ex->NM_status_lock);
		wake_up_interruptible(&dev_ex->queue);
		break;

#ifdef UNDOC
	case IOCTL_NMC3_GET_REG:
	{
		u32 Reg_addr_u, Reg_val;
		static void __iomem *Reg_addr_m;

		dbg("case IOCTL_NMC3_GET_REG.");

		ret = get_user(Reg_addr_u, (u32 __user *)ioctl_param);
		if (ret)
		{
			printk(KERN_ERR "NMC3: ERROR: Can not get register address!\n");
			break;
		}

		dbg("Reg_addr_u = 0x%8.8X", Reg_addr_u);

		Reg_addr_m = ioremap(Reg_addr_u, 0x4);
		if (!Reg_addr_m)
		{
			printk(KERN_ERR "NMC3: ERROR: Can not map register!\n");
			ret = -ENOMEM;
			break;
		}

		dbg("Reg_addr_m = 0x%8.8X", (u32)Reg_addr_m);

		Reg_val = ioread32(Reg_addr_m);

		dbg("Reg_val = 0x%8.8X", Reg_val);

		iounmap(Reg_addr_m);

		ret = put_user(Reg_val, (u32 __user *)ioctl_param);
		if (ret)
		{
			printk(KERN_ERR "NMC3: ERROR: Can not write register value to user space!\n");
		}

		break;
	}

	case IOCTL_NMC3_SET_REG:
	{
		u32 IOCTL_parameters[2];
		u32 Reg_addr_u, Reg_val;
		static void __iomem *Reg_addr_m;

		dbg("case IOCTL_NMC3_SET_REG.");

		ret = copy_from_user(IOCTL_parameters, (void __user *)ioctl_param, 8);
		if (ret)
		{
			printk(KERN_ERR "NMC3: ERROR: Can not get IOCTL parameters!\n");
			break;
		}

		Reg_addr_u = IOCTL_parameters[0];
		Reg_val = IOCTL_parameters[1];

		dbg("Reg_addr_u = 0x%8.8X; Reg_val = 0x%8.8X", Reg_addr_u, Reg_val);

		Reg_addr_m = ioremap(Reg_addr_u, 0x4);
		if (!Reg_addr_m)
		{
			printk(KERN_ERR "NMC3: ERROR: Can not map register!\n");
			ret = -ENOMEM;
			break;
		}

		dbg("Reg_addr_m = 0x%8.8X", (u32)Reg_addr_m);

		iowrite32(Reg_val, Reg_addr_m);

		iounmap(Reg_addr_m);

		break;
	}
#endif // UNDOC

	default:
		dbg("case default.");
		printk(KERN_ERR "NMC3: ERROR: Unknown IOCTL!\n");
		ret = -ENOTTY;
	} // switch

	up(&dev_ex->sem);

	dbg("End; ret = %Xh = %d.", ret, ret);
	return ret;
}

static int NMC3_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct NMC3_extension *dev_ex = filp->private_data;
	unsigned long Phys_addr = vma->vm_pgoff << PAGE_SHIFT;
	u32 Mem_u, Mem_size;

	dbg("Begin; Num = %u; vm_start = %lXh; vm_end = %lXh; vm_pgoff = %lXh.",
		dev_ex->Num, vma->vm_start, vma->vm_end, vma->vm_pgoff);

	if ((Phys_addr >= (dev_ex->mem_im0_u >> PAGE_SHIFT) << PAGE_SHIFT)
		&& (Phys_addr < dev_ex->mem_im0_u + dev_ex->mem_im0_size))
	{
		Mem_u = dev_ex->mem_im0_u;
		Mem_size = dev_ex->mem_im0_size;
	}
	else if ((Phys_addr >= (dev_ex->mem_im1_3_u >> PAGE_SHIFT) << PAGE_SHIFT)
		&& (Phys_addr < dev_ex->mem_im1_3_u + dev_ex->mem_im1_3_size))
	{
		Mem_u = dev_ex->mem_im1_3_u;
		Mem_size = dev_ex->mem_im1_3_size;
	}
	else if ((Phys_addr >= (dev_ex->mem_em1_u >> PAGE_SHIFT) << PAGE_SHIFT)
		&& (Phys_addr < dev_ex->mem_em1_u + dev_ex->mem_em1_size))
	{
		Mem_u = dev_ex->mem_em1_u;
		Mem_size = dev_ex->mem_em1_size;
	}
	else
	{
		printk(KERN_ERR "NMC3: ERROR: Requested area is out of memory!\n");
		dbg("End (With ERROR!); Return: -EINVAL!");
		return -EINVAL;
	}

	if (vma->vm_end - vma->vm_start > Mem_u + Mem_size - Phys_addr)
	{
		printk(KERN_ERR "NMC3: ERROR: Requested area is out of memory!\n");
		dbg("End (With ERROR!); Return: -EINVAL!");
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, vma->vm_end - vma->vm_start, vma->vm_page_prot))
	{
		printk(KERN_ERR "NMC3: ERROR: Can not map memory!\n");
		dbg("End; Return: -EAGAIN.");
		return -EAGAIN;
	}

	dbg("End; Return: 0.");
	return 0;
}

static irqreturn_t NMC3_ISR_HP(int irq, void *dev_id)
{
	struct NMC3_extension *dev_ex = (struct NMC3_extension *)dev_id;

	dbg("Begin; Num = %u.", dev_ex->Num);

	tasklet_schedule(&dev_ex->HP_tasklet);

	iowrite32(1, NMUREGS_m + NMUINTCLRH_offset);

	/* WHAT DA FUQ? */
//	iowrite32(0xFFFFFFFF, VICADDRESS_m);

	dbg("End; Return: IRQ_HANDLED.");
	return IRQ_HANDLED;
}

static irqreturn_t NMC3_ISR_LP(int irq, void *dev_id)
{
	struct NMC3_extension *dev_ex = (struct NMC3_extension *)dev_id;

	dbg("Begin; Num = %u.", dev_ex->Num);
	
	tasklet_schedule(&dev_ex->LP_tasklet);
	
	iowrite32(1, NMUREGS_m + NMUINTCLRL_offset);
//	iowrite32(0xFFFFFFFF, VICADDRESS_m);

	dbg("End; Return: IRQ_HANDLED.");
	return IRQ_HANDLED;
}

void NMC3_HP_tasklet(unsigned long num)
{
	struct NMC3_extension *dev_ex = (struct NMC3_extension *)num;
	struct list_head *ptr;
	struct Wait_interrupt *wi;
	u32 From_NM;

	dbg("Begin; Num = %u.", dev_ex->Num);

	From_NM = ioread32((u32 *)dev_ex->mem_im1_3_m + FROM_NM_offset);

	dbg("From_NM = %8.8Xh.", From_NM);

	if (From_NM == READY_FOR_COMMAND)
	{
		dbg("Interrupt type: READY_FOR_COMMAND.");

		spin_lock(&dev_ex->NM_status_lock);
		dev_ex->Ready_for_command = 1;
		spin_unlock(&dev_ex->NM_status_lock);
	}
	else if (From_NM == PROGRAM_STOPPED)
	{
		dbg("Interrupt type: PROGRAM_STOPPED.");

		spin_lock(&dev_ex->NM_status_lock);
		dev_ex->Program_stopped = 1;
		spin_unlock(&dev_ex->NM_status_lock);
	}
	else if (From_NM == SYNC_FLAG)
	{
		dbg("Interrupt type: SYNC_FLAG.");

		spin_lock(&dev_ex->NM_status_lock);
		dev_ex->Sync_flag = 1;
		spin_unlock(&dev_ex->NM_status_lock);
	}
	else
	{
		dbg("User interrupt.");

		spin_lock(&dev_ex->Wait_interrupt_lock);

		list_for_each(ptr, &dev_ex->Wait_interrupt_list)
		{
			wi = list_entry(ptr, struct Wait_interrupt, list);

			dbg("Before iteration: tgid = %Xh, int_type = %u.", wi->tgid, wi->int_type);

			if (wi->int_type == 1)
			{
				wi->int_NM = 1;
				wi->int_status = 1;
			}

			dbg("After iteration: int_NM = %d.", wi->int_NM);
		}

		spin_unlock(&dev_ex->Wait_interrupt_lock);
	}

    dbg("wake_up_interruptible %08x", (u32 *)dev_ex->mem_im1_3_m + HP_INT_SEND_offset);
	wake_up_interruptible(&dev_ex->queue);
	iowrite32(0, (u32 *)dev_ex->mem_im1_3_m + HP_INT_SEND_offset);

	dbg("End.");
}

void NMC3_LP_tasklet(unsigned long num)
{
	struct NMC3_extension *dev_ex = (struct NMC3_extension *)num;
	struct list_head *ptr;
	struct Wait_interrupt *wi;

	dbg("Begin; Num = %u.", dev_ex->Num);

	spin_lock(&dev_ex->Wait_interrupt_lock);

	list_for_each(ptr, &dev_ex->Wait_interrupt_list)
	{
		wi = list_entry(ptr, struct Wait_interrupt, list);

		dbg("Before iteration: tgid = %Xh, int_type = %u.", wi->tgid, wi->int_type);

		if (wi->int_type == 2)
		{
			wi->int_NM = 1;
			wi->int_status = 1;
		}

		dbg("After iteration: int_NM = %d.", wi->int_NM);
	}

	spin_unlock(&dev_ex->Wait_interrupt_lock);

	wake_up_interruptible(&dev_ex->queue);

	dbg("End.");
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
	.mmap           = NMC3_mmap,
};

static int __init NMC3_probe(struct platform_device *pdev)
{
	struct NMC3_extension *dev_ex;
	struct resource *mem_im0_res, *mem_im1_res, *mem_im3_res, *mem_em1_res;
	struct resource *nmc_HPint_res, *nmc_LPint_res;
	dev_t dev_number = MKDEV(NMC3_MAJOR, pdev->id);
	struct device *NMC3_device;
	int ret;


	if (of_property_read_u32(pdev->dev.of_node, "device-id", &pdev->id) < 0) {
		dev_err(&pdev->dev, "Missing required parameter 'device-id'\n");
		return -ENODEV;
	}

	dbg("Begin; id = %u.", pdev->id);

	mem_im0_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "im0");
	if (!mem_im0_res)
	{
		ret = -ENODEV;
		printk(KERN_ERR "NMC3: ERROR: Can not get \"im0\" resource!\n");
		goto Fail_NMC3_probe_0;
	}

	mem_im1_res  = platform_get_resource_byname(pdev, IORESOURCE_MEM, "im1");
	if (!mem_im1_res)
	{
		ret = -ENODEV;
		printk(KERN_ERR "NMC3: ERROR: Can not get \"im1\" resource!\n");
		goto Fail_NMC3_probe_0;
	}

	mem_im3_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "im3");
	if (!mem_im3_res)
	{
		ret = -ENODEV;
		printk(KERN_ERR "NMC3: ERROR: Can not get \"im3\" resource!\n");
		goto Fail_NMC3_probe_0;
	}

	mem_em1_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "em1");
	if (!mem_em1_res)
	{
		ret = -ENODEV;
		printk(KERN_ERR "NMC3: ERROR: Can not get \"em1\" resource!\n");
		goto Fail_NMC3_probe_0;
	}

	nmc_HPint_res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!nmc_HPint_res)
	{
		ret = -ENODEV;
		printk(KERN_ERR "NMC3: ERROR: Can not get HP interrupt resource!\n");
		goto Fail_NMC3_probe_0;
	}

	nmc_LPint_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!nmc_LPint_res)
	{
		ret = -ENODEV;
		printk(KERN_ERR "NMC3: ERROR: Can not get LP interrupt resource!\n");
		goto Fail_NMC3_probe_0;
	}

	dev_ex = kzalloc(sizeof(struct NMC3_extension), GFP_KERNEL);
	if (!dev_ex)
	{
		ret = -ENOMEM;
		printk(KERN_ERR "NMC3: ERROR: Insufficient memory!\n");
		goto Fail_NMC3_probe_0;
	}

	dev_ex->Num = pdev->id;
	sprintf(dev_ex->Name, "%s_%u", NMC3_FILENAME, pdev->id);

	dev_ex->mem_im0_u = mem_im0_res->start;
	dev_ex->mem_im1_3_u = mem_im1_res->start;
	dev_ex->mem_em1_u = mem_em1_res->start;

	dev_ex->mem_im0_size = resource_size(mem_im0_res);
	dev_ex->mem_im1_3_size = resource_size(mem_im1_res) + resource_size(mem_im3_res);
	dev_ex->mem_em1_size = resource_size(mem_em1_res);

	dbg("mem_im0 resource (unmapped): Start: %8.8Xh; Size: %8.8Xh.", dev_ex->mem_im0_u, dev_ex->mem_im0_size);
	dbg("mem_im1_3 resource (unmapped): Start: %8.8Xh; Size: %8.8Xh.", dev_ex->mem_im1_3_u, dev_ex->mem_im1_3_size);
	dbg("mem_em1 resource (unmapped): Start: %8.8Xh; Size: %8.8Xh.", dev_ex->mem_em1_u, dev_ex->mem_em1_size);

	dev_ex->irq_HP = nmc_HPint_res->start;
	dev_ex->irq_LP = nmc_LPint_res->start;

	dbg("HP interrupt resource: %Xh.", dev_ex->irq_HP);
	dbg("LP interrupt resource: %Xh.", dev_ex->irq_LP);

	sema_init(&dev_ex->sem, 1);
	init_waitqueue_head(&dev_ex->queue);
	spin_lock_init(&dev_ex->Wait_interrupt_lock);
	INIT_LIST_HEAD(&dev_ex->Wait_interrupt_list);
	spin_lock_init(&dev_ex->NM_status_lock);

	tasklet_init(&dev_ex->HP_tasklet, NMC3_HP_tasklet, (unsigned long)dev_ex);
	tasklet_init(&dev_ex->LP_tasklet, NMC3_LP_tasklet, (unsigned long)dev_ex);

	dev_ex->mem_im0_m = ioremap(dev_ex->mem_im0_u, dev_ex->mem_im0_size);
	if (!dev_ex->mem_im0_m)
	{
		ret = -ENOMEM;
		printk(KERN_ERR "NMC3: ERROR: Can not map IM0 memory!\n");
		goto Fail_NMC3_probe_1;
	}

	dev_ex->mem_im1_3_m = ioremap(dev_ex->mem_im1_3_u, dev_ex->mem_im1_3_size);
	if (!dev_ex->mem_im1_3_m)
	{
		ret = -ENOMEM;
		printk(KERN_ERR "NMC3: ERROR: Can not map IM1 & IM3 memory!\n");
		goto Fail_NMC3_probe_2;
	}

	if (dev_ex->mem_em1_size)
	{
		dev_ex->mem_em1_m = ioremap(dev_ex->mem_em1_u, dev_ex->mem_em1_size);
		if (!dev_ex->mem_em1_m)
		{
			ret = -ENOMEM;
			printk(KERN_ERR "NMC3: ERROR: Can not map EM1 memory!\n");
			goto Fail_NMC3_probe_3;
		}
	}
	else
	{
		dbg("All external SDRAM used by linux.");
	}

	dbg("mem_im0_m = %8.8Xh; mem_im0_size = %8.8Xh.", (u32)dev_ex->mem_im0_m, (u32)dev_ex->mem_im0_size);
	dbg("mem_im1_3_m = %8.8Xh; mem_im1_3_size = %8.8Xh.", (u32)dev_ex->mem_im1_3_m, (u32)dev_ex->mem_im1_3_size);
	dbg("mem_em1_m = %8.8Xh; mem_em1_size = %8.8Xh.", (u32)dev_ex->mem_em1_m, (u32)dev_ex->mem_em1_size);

	ret = request_irq(dev_ex->irq_HP, NMC3_ISR_HP, 0x0, dev_ex->Name, dev_ex);
	if (ret)
	{
		printk(KERN_ERR "NMC3: ERROR: HP IRQ request failed!\n");
		goto Fail_NMC3_probe_4;
	}

	ret = request_irq(dev_ex->irq_LP, NMC3_ISR_LP, 0x0, dev_ex->Name, dev_ex);
	if (ret)
	{
		printk(KERN_ERR "NMC3: ERROR: LP IRQ request failed!\n");
		goto Fail_NMC3_probe_6;
	}

	dev_set_drvdata(&pdev->dev, dev_ex);

	cdev_init(&dev_ex->cdev, &fops);
	dev_ex->cdev.owner = THIS_MODULE;

	ret = cdev_add(&dev_ex->cdev, dev_number, 1);
	if (ret)
	{
		printk(KERN_ERR "NMC3: ERROR: Can not adding char device!\n");
		goto Fail_NMC3_probe_7;
	}

	NMC3_device = device_create(NMC3_class, NULL, dev_number, NULL, "nmc3_%u", pdev->id);
	if (IS_ERR(NMC3_device))
	{
		printk(KERN_ERR "NMC3: ERROR: Can not adding device to class!\n");
		ret = -ENXIO;
		goto Fail_NMC3_probe_8;
	}

	dbg("End; Return: 0.");
	return 0;

Fail_NMC3_probe_8:
	cdev_del(&dev_ex->cdev);

Fail_NMC3_probe_7:
	free_irq(dev_ex->irq_LP, dev_ex);

Fail_NMC3_probe_6:
	free_irq(dev_ex->irq_HP, dev_ex);

Fail_NMC3_probe_4:
	iounmap(dev_ex->mem_em1_m);

Fail_NMC3_probe_3:
	iounmap(dev_ex->mem_im1_3_m);

Fail_NMC3_probe_2:
	iounmap(dev_ex->mem_im0_m);

Fail_NMC3_probe_1:
	tasklet_kill(&dev_ex->HP_tasklet);
	tasklet_kill(&dev_ex->LP_tasklet);
	kfree(dev_ex);

Fail_NMC3_probe_0:

	dbg("End (with ERROR); ret = %Xh = %d!", ret, ret);
	return ret;
}

static int __exit NMC3_remove(struct platform_device *pdev)
{
	struct NMC3_extension *dev_ex = dev_get_drvdata(&pdev->dev);

	dbg("Begin; id = %u.", pdev->id);

	device_destroy(NMC3_class, MKDEV(NMC3_MAJOR, dev_ex->Num));
	cdev_del(&dev_ex->cdev);
	free_irq(dev_ex->irq_LP, dev_ex);
	free_irq(dev_ex->irq_HP, dev_ex);
	iounmap(dev_ex->mem_em1_m);
	iounmap(dev_ex->mem_im1_3_m);
	iounmap(dev_ex->mem_im0_m);
	tasklet_kill(&dev_ex->HP_tasklet);
	tasklet_kill(&dev_ex->LP_tasklet);
	kfree(dev_ex);

	dbg("End; Return: 0.");
	return 0;
}



static int __init NMC3_init(void)
{
	dev_t dev_numbers = MKDEV(NMC3_MAJOR, 0);
	int ret;

	dbg("Begin.");

	NMC3_class = class_create(THIS_MODULE, NMC3_FILENAME);
	if (IS_ERR(NMC3_class))
	{
		printk(KERN_ERR "NMC3: ERROR: Can not create device class!\n");
		ret = -ENXIO;
		goto Fail_NMC3_init_0;
	}

	ret = register_chrdev_region(dev_numbers, NMC3_COUNT, NMC3_FILENAME);
	if (ret)
	{
		printk(KERN_ERR "NMC3: ERROR: Can not register device numbers!\n");
		goto Fail_NMC3_init_1;
	}

	NMUREGS_m = ioremap(NMUREGS_u, NMUREGS_size);
	if (!NMUREGS_m)
	{
		ret = -ENOMEM;
		printk(KERN_ERR "NMC3: ERROR: Can not map NMUREGS!\n");
		goto Fail_NMC3_init_2;
	}

	dbg("++++++++++++ NMUREGS_m = %8.8Xh    NMUREGS_u = %8.8Xh", (u32)NMUREGS_m, (u32)NMUREGS_u);

	/* 
	   Should now be automatic ;)
	ret = platform_driver_probe(&NMC3_driver, NMC3_probe);
	if (ret)
	{
		printk(KERN_ERR "NMC3: ERROR: Can not register driver!\n");
		goto Fail_NMC3_init_4;
	}
	*/

	dbg("End; Return: 0.");
	return 0;


Fail_NMC3_init_2:
	unregister_chrdev_region(dev_numbers, NMC3_COUNT);

Fail_NMC3_init_1:
	class_destroy(NMC3_class);

Fail_NMC3_init_0:

	dbg("End (with ERROR); ret = %Xh = %d!", ret, ret);
	return ret;
}

static void __exit NMC3_exit(void)
{
	dev_t dev_numbers = MKDEV(NMC3_MAJOR, 0);

	dbg("Begin.");

	//platform_driver_unregister(&NMC3_driver);
	iounmap(NMUREGS_m);
	unregister_chrdev_region(dev_numbers, NMC3_COUNT);
	class_destroy(NMC3_class);

	dbg("End.");
}

static const struct of_device_id module_nmc3_of_match_table[] = {
	{ .compatible = "module,nmc3", },
	{ /* end of list */ }
};

MODULE_DEVICE_TABLE(of, module_nmc3_of_match_table);

static struct platform_driver nmc3_driver =
{
	.probe = NMC3_probe,
	.remove = NMC3_remove,
        .driver =
	{
		.name	= "rcmod_nmc3",
		.of_match_table = of_match_ptr(module_nmc3_of_match_table),
        },
};

MODULE_ALIAS("platform:rcmod_nmc3");
module_platform_driver(nmc3_driver);

MODULE_LICENSE("GPL");

module_init(NMC3_init);
module_exit(NMC3_exit);
