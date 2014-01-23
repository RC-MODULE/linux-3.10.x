/* RC Module SoC. ARM-NMC3 interface driver
 *
 * Copyright (C) RC Module 2011
 *
 * 	Gennadiy Kurtsman <gkurtsman@module.ru>
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
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>

#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/delay.h>

#define DEBUG
//#undef DEBUG

#include <linux/rcmod_soc_nmc3.h>
#include "rcmod_soc_nmc3_initnmc.h"

#define NMC3_DRIVER_VERSION	"0.1"

__u32 magic_nmc3 [] = {0x9d7256f3, 0x326be47f, 0xf9d57299, 0x1ad13486, 0x1f9fb475};

static LIST_HEAD(nmc3_dscr);
/*************************************************************/
#ifdef DEBUG
void print_if (struct __NMC3_thread_descriptor* thr_dscr)
{
	struct __NMC3_descriptor* dscr = thr_dscr->dscr;
	PRINT_IF;
}
EXPORT_SYMBOL(print_if);
#endif

static inline struct __NMC3_descriptor* seek_dscr_byname (char* device_name)
{
	struct __NMC3_descriptor* dscr;
	struct list_head* lptr;

	list_for_each(lptr,&nmc3_dscr) {
		dscr = container_of(lptr, struct __NMC3_descriptor, list);
		dbg ("dscr->name = %s, device_name=%s", dscr->Name, device_name);
		if (!strcmp(dscr->Name, device_name))
			break;
		else
			dscr = NULL;
	}
	return dscr;
}

static inline struct __NMC3_descriptor* seek_dscr_byptr (struct __NMC3_descriptor* dscr)
{
	struct list_head* lptr;

	list_for_each(lptr,&nmc3_dscr)
		if (dscr == container_of(lptr, struct __NMC3_descriptor, list)) {
			return dscr;
		}
	return NULL;
}

static void request_nmc3_mem (struct __NMC3_descriptor* dscr)
{
	iowrite32(CTRL_NM_INT_SET_VAL, dscr->nmHPintr_set);
}

static int irq_cmd_to_NMC3 (struct __NMC3_descriptor* dscr, __u32 code)
{
	__u32 sync_value = ioread32(&(dscr->NMC3_if_memory->ISR_FROM_ARM));
	int wait_time;

	for (wait_time=0; wait_time < IRQ_CONTEXT_LIFE_TIME; wait_time++) {
		if (sync_value == ANY_COMMAND)
			break;
		udelay(1000);
		sync_value = ioread32(&(dscr->NMC3_if_memory->ISR_FROM_ARM));
	}
	if (wait_time >= IRQ_CONTEXT_LIFE_TIME) {
		printk (KERN_INFO "%s#%d: sync value ERROR code=0x%x, memval=0x%x\n",
			__FILE__,__LINE__, code, sync_value);
		return -EIO;
	}
	dbg ("iowrite32(0x%x,%p);",code,&(dscr->NMC3_if_memory->ISR_FROM_ARM));
	iowrite32(code, &(dscr->NMC3_if_memory->ISR_FROM_ARM));
	return 0;
}
 
static int poll_nmc3_ready (struct __NMC3_descriptor* dscr)
{
	int i;

	for (i=0; i<1000; i++)
		if (ioread32(&(dscr->NMC3_if_memory->FROM_ARM)) == ANY_COMMAND) {
			dbg ("end %s",__func__);
			return 0;
		} else
			udelay(1000);
	return -EBUSY; 
}

static inline int cmd_to_NMC3 (struct __NMC3_descriptor* dscr, __u32 code, __u32 app_entry)
{
	int ret;

	dbg ("FROM_ARM = 0x%x",dscr->NMC3_if_memory->FROM_ARM);
	if ((ret = poll_nmc3_ready (dscr)) < 0) {
		dbg ("can't issue command");
//		DUMP_NMC3(0,0x400,0x20);
		return -EBUSY;
	}
	iowrite32(app_entry, &(dscr->NMC3_if_memory->SYNC_FROM_ARM)); /* description of the field is
							       in NM startup source files */
	iowrite32(code, &(dscr->NMC3_if_memory->FROM_ARM));
	return 0;
}

static int free_nmc3_mem (struct __NMC3_descriptor* dscr)
{
	return irq_cmd_to_NMC3 (dscr, STOP_UNLOCK_CMD);
}
static enum __nmc3_states check_run (struct __NMC3_descriptor* dscr)
{
	__u32* if_magic_nmc3 = (__u32*)dscr->NMC3_if_memory->MAGIC_CODE;
	int i;

	if (dscr->status == running)
		return running;
	for (i=0; i<ARRAY_SIZE(magic_nmc3); i++)
		if (magic_nmc3[i] != ioread32(if_magic_nmc3 + i))
			return reset;
	return running;
}

static inline int check_Ehdr (Elf32_Ehdr* ehdr)
{
	if (memcmp (ehdr->e_ident, "\x7f" "ELF", 4)) {
		printk (KERN_INFO "%s#%d: elf file header ERROR\n", __FILE__,__LINE__);
		return -ENOEXEC;
	}
	if (ehdr->e_type != 2) {
		printk (KERN_INFO "%s#%d: no executable elf file ERROR\n", __FILE__,__LINE__);
		return -ENOEXEC;
	}
	if (ehdr->e_machine != 64051) {		/* no -soc translation */
		printk (KERN_INFO "%s#%d: translation option ERROR %d\n", __FILE__,__LINE__,
			ehdr->e_machine);
		return -ENOEXEC;
	}
	return 0;
}

static int seek_nmc3_bank (struct __NMC3_descriptor* dscr, __u32 offset)
{
	int i;
	int bank_n = -1;

	for (i=0; i<dscr->nmc3_bank_count; i++)
		if (offset < dscr->nmc3_bank_base_phys_internal[i]) {
			bank_n = i-1;
			break;
		}
		if (bank_n < 0) {
			dbg ("offset=0x%x,nmc3_bank_base_phys_internal[%d]=0x%x",
				offset, i, dscr->nmc3_bank_base_phys_internal[i]);
			bank_n = dscr->nmc3_bank_count - 1;
		}
		if (offset > dscr->nmc3_bank_base_phys_internal[bank_n] +
				(dscr->nmc3_bank_ln[bank_n] >> 2) - 1) {
			dbg ("offset=0x%x, nmc3_bank_base_phys_internal[%d]=0x%x,"
					"nmc3_bank_ln[%d]=0x%lx /4=0x%lx",
				offset, bank_n, dscr->nmc3_bank_base_phys_internal[bank_n],
				bank_n, dscr->nmc3_bank_ln[bank_n],
				(dscr->nmc3_bank_ln[bank_n] >> 2));
			return -ENXIO;
		}
		return bank_n;
}

static __u32 seek_nmc3_off (struct __NMC3_descriptor* dscr, int bank_n, __u32 offset)
{
	return (__u32)(dscr->nmc3_bank_base_virt[bank_n]) + ((offset -
		dscr->nmc3_bank_base_phys_internal[bank_n]) << 2);
}

static int loadelf (struct __NMC3_descriptor* dscr, void* ptr, size_t ln, __u32* app_entry)
{
	Elf32_Ehdr* ehdr = ptr;
	Elf32_Phdr* phdr;
	int ret;
	int i;
	int j;
	int bank_n;
	__u32 target_seg;
	unsigned long* addr_seg;
	int wr_count;
	int wr_count_B;
	int seg_valid;
	Elf32_Addr seg_current_addr;
	int current_ln_B;
	int current_ln;
	__u32 bank_ln_internal;

	if((ret =check_Ehdr(ehdr)) < 0) {
		printk (KERN_INFO "%s#%d: elf file format ERR %d\n", __FILE__,__LINE__, ret);
		return ret;
	}
	if (app_entry)
		*app_entry = ehdr->e_entry;
	phdr = ptr+ehdr->e_phoff;
	for (i=0; i < ehdr->e_phnum; i++) {
		dbg ("p_offset = 0x%x, p_paddr = 0x%x,p_type = 0x%x,p_memsz=0x%x", phdr[i].p_offset,
			phdr[i].p_paddr,phdr[i].p_type,phdr[i].p_memsz);
		if((phdr[i].p_type != PT_LOAD) || (!phdr[i].p_memsz))
			continue;
		addr_seg = ptr + phdr[i].p_offset;
		seg_valid = 1;
		seg_current_addr = phdr[i].p_paddr;
		wr_count_B = phdr[i].p_filesz;
		while (seg_valid) { 
			if ((bank_n = seek_nmc3_bank (dscr, seg_current_addr)) < 0)
				return bank_n;
			dbg ("bank_n = %d,ptr=%p,offset=0x%x,addr_seg=%p",bank_n,
				ptr, phdr[i].p_offset, addr_seg);
			dbg ("p_offset = 0x%x, p_paddr = 0x%x", phdr[i].p_offset,
				seg_current_addr);
			if ((target_seg = seek_nmc3_off (dscr, bank_n, seg_current_addr)) >
					(u32)(dscr->nmc3_bank_base_virt[bank_n]) + 
					dscr->nmc3_bank_ln[bank_n]) {
				dbg ("target_seg=0x%x,bank_n=%d,bank_base=0x%x,bank_ln=0x%lx",
					target_seg,bank_n,(u32)(dscr->nmc3_bank_base_virt[bank_n]),
					dscr->nmc3_bank_ln[bank_n]);
				printk (KERN_INFO "%s#%d: offset ERROR 0x%x\n", __FILE__,__LINE__,
					target_seg);
				return -ENXIO;
			}
			dbg ("after seek_nmc3_off(), target_seg=0x%x",target_seg);
			dbg ("bank_base_phys_internal[%d]=0x%x,bank_ln[%d]=0x%lx,"
				"current_addr=0x%x,wr_count_B=0x%x",
				bank_n,dscr->nmc3_bank_base_phys_internal[bank_n],
				bank_n,dscr->nmc3_bank_ln[bank_n],
				seg_current_addr,wr_count_B);
			bank_ln_internal = dscr->nmc3_bank_ln[bank_n] >> 2;
			wr_count = wr_count_B >> 2;
			if (dscr->nmc3_bank_base_phys_internal[bank_n] + bank_ln_internal <
					seg_current_addr + wr_count) {
				current_ln = dscr->nmc3_bank_base_phys_internal[bank_n] +
					bank_ln_internal - seg_current_addr;
				current_ln_B = current_ln << 2;
				wr_count = current_ln;
				wr_count_B -= current_ln_B;
				seg_current_addr += wr_count;
			} else {
				wr_count = (wr_count_B + 3) >> 2; /* p_filesz - size in bytes */
				seg_valid = 0;
				seg_current_addr += wr_count;
			}
			dbg ("wr_count = 0x%x/%d, wr_count_B = 0x%x/%d, seg_current_addr = 0x%x",
				wr_count,wr_count,wr_count_B,wr_count_B,seg_current_addr);
			for (j=0; j<wr_count; j++)	/* iowrite32_rep doesn't work with NMC3
							   internal memory*/
				iowrite32 (addr_seg[j], ((__u32*)target_seg) + j);
			addr_seg += wr_count;
			dbg ("addr_seg = %p",addr_seg);
		}
		/* To fill empty memory of segment by zeroes */
		seg_valid = 1;
		wr_count_B = phdr[i].p_memsz - phdr[i].p_filesz;
		while (seg_valid) {
			dbg ("seg_current_addr = 0x%x",seg_current_addr);
			if ((bank_n = seek_nmc3_bank (dscr, seg_current_addr)) < 0)
				return bank_n;
			dbg ("bank_n = %d",bank_n);
			if ((target_seg = seek_nmc3_off (dscr, bank_n, seg_current_addr)) >
					(u32)(dscr->nmc3_bank_base_virt[bank_n]) +
					dscr->nmc3_bank_ln[bank_n]) {
				dbg ("target_seg=0x%x,bank_n=%d,bank_base=0x%x,bank_ln=0x%lx",
					target_seg,bank_n,(u32)(dscr->nmc3_bank_base_virt[bank_n]),
					dscr->nmc3_bank_ln[bank_n]);
				printk (KERN_INFO "%s#%d: offset ERROR 0x%x\n", __FILE__,__LINE__,
					target_seg);
				return -ENXIO;
			}
			bank_ln_internal = dscr->nmc3_bank_ln[bank_n] >> 2;
			wr_count = wr_count_B >> 2;
			if (dscr->nmc3_bank_base_phys_internal[bank_n] + bank_ln_internal <
					seg_current_addr + wr_count) {
				current_ln = dscr->nmc3_bank_base_phys_internal[bank_n] +
					bank_ln_internal - seg_current_addr;
				current_ln_B = current_ln << 2;
				wr_count = current_ln;
				wr_count_B -= current_ln_B;
				seg_current_addr += wr_count;
			} else {
				wr_count = (wr_count_B + 3) >> 2; /* p_filesz - size in bytes */
				seg_valid = 0;
				seg_current_addr += wr_count;
			}
			for (j=0; j<wr_count; j++)
				iowrite32 (0x00000000, ((__u32*)target_seg) + j);
			addr_seg += wr_count;
		}
	}
	dbg ("end of %s",__func__);
	return 0;
}

static int irq_poll_nmc3_ready (struct __NMC3_descriptor* dscr)
{
	int i;

	for (i=0; i<1000; i++)
		if (ioread32 (&(dscr->NMC3_if_memory->ISR_FROM_ARM)) == ANY_COMMAND) {
				dbg ("end %s",__func__);
				return 0;
		} else
			udelay(1000);
		return -EBUSY;
}

/********************* interrupts **************************/
static irqreturn_t HPisr (int irq, void *dev_id)
{
	struct __NMC3_descriptor* dscr = dev_id;
	struct __NMC3_descriptor* tmp;
	struct __NMC3_thread_descriptor* thr_dscr;
	struct ToArmBlock *msg_memory;
	int ret_code;
	__u32 msg_code;
	void (*app_cb) (struct __NMC3_thread_descriptor* thr, int ret_code);
	__u32 irq_param;
	irqreturn_t ret;

	dbg ("%s, irq=%d", __func__, irq);
//	DUMP_NMC3(0,0x400,0x0c);	// 0x100 - internal

	tmp = seek_dscr_byptr (dscr);
	if (!tmp)
		BUG_ON(!tmp);
	if (dscr->status != running) {
		printk(KERN_INFO "NMC3 doesn't run during interrupt\n");
		ret = IRQ_NONE;
		goto HPisr_out;
	}
	msg_memory = (struct ToArmBlock *)(dscr->NMC3_if_memory->SYNC_TO_ARM);
	msg_code = ioread32(&(dscr->NMC3_if_memory->TO_ARM));
	switch (msg_code) {
	case PROGRAM_FINISH:
		if ((dscr->app_thr_active < 1) ||
				(dscr->app_thr_active >= CONFIG_NMC3_DESCRIPTORS_LIMIT)) {
			printk(KERN_INFO "interrupt isn't awaited %d\n", dscr->app_thr_active);
			ret = IRQ_NONE;
			goto HPisr_out;
		}
		thr_dscr = dscr->NMC3_thread_dscr + dscr->app_thr_active - 1;
		if (!(thr_dscr->busy)) {
			printk(KERN_INFO "%s#%d: thread descriptor hasn't been started\n",
				__FILE__,__LINE__);
			ret = IRQ_NONE;
			goto HPisr_out;
		}
		app_cb = thr_dscr->app_cb;
		dscr->app_thr_active = 0; /* freeing descr. for new NMC3 apps */
		if (app_cb) {
			ret_code = ioread32(&msg_memory->ret_low);
			app_cb (thr_dscr, ret_code);
		}
		break;
	case IRQ_FROM_NMC3:
		if (dscr->irq_cb) {
			irq_param = ioread32(&msg_memory->irq_param);
			dscr->irq_cb (dscr->HPirq, irq_param);
		}
		break;
	default:
		printk ("%s#%d: unknown interrupt from NMC3, code=0x%08x\n",
			__FILE__,__LINE__,msg_code);
	}
	ret = IRQ_HANDLED;
HPisr_out:
	iowrite32(CTRL_NM_INT_CLR_VAL, dscr->nmHPintr_clr);
	iowrite32(VIC_LPINTRS_ENA_VAL, dscr->vic_LPintrs_ena);
	iowrite32(ANY_MESSAGE, &(dscr->NMC3_if_memory->TO_ARM));
	return ret;
}

static irqreturn_t LPisr (int irq, void *dev_id)
{
	struct __NMC3_descriptor* dscr = dev_id;
	struct __NMC3_descriptor* tmp;
	irqreturn_t ret;

	dbg ("%s, irq=%d", __func__, irq);
	tmp = seek_dscr_byptr (dscr);
	if (!tmp)
		BUG_ON(!tmp);
	if (dscr->status != running) {
		printk(KERN_INFO "NMC3 doesn't run during interrupt\n");
		ret = IRQ_NONE;
		goto LPisr_out;
	}
	ret = IRQ_HANDLED;
LPisr_out:
	iowrite32(CTRL_NM_INT_CLR_VAL, dscr->nmLPintr_clr);
	iowrite32(VIC_LPINTRS_ENA_VAL, dscr->vic_LPintrs_ena);
	return ret;
}

/********************** NMC3 API ********************/

struct __NMC3_thread_descriptor* nmc3_find_device (char* device_name)
{
	int i;
	unsigned long flags;
	struct __NMC3_descriptor* dscr;
	void* ret;

	dbg ("%s, name=%s",__func__,device_name);
	dscr = seek_dscr_byname (device_name);
	dbg("dscr=%p,thr_dscr=%p",dscr,dscr->NMC3_thread_dscr);
	if (!dscr) {
		printk(KERN_INFO "%s: -ENXIO error\n", __func__);
		return NULL;
	}
	spin_lock_irqsave(&dscr->load_lock, flags);
	for (i=0; i<CONFIG_NMC3_DESCRIPTORS_LIMIT; i++) {
		if (!(dscr->NMC3_thread_dscr[i].busy)) {
			dscr->NMC3_thread_dscr[i].busy = -1;
			ret = &dscr->NMC3_thread_dscr[i];
			goto nmc3_find_out;
		}
	}
	printk(KERN_NOTICE "%s: -EBUSY error\n",__func__);
	ret = NULL;
nmc3_find_out:
	spin_unlock_irqrestore(&dscr->load_lock, flags);
	return ret;
}
EXPORT_SYMBOL(nmc3_find_device);

int nmc3_release_dscr (struct __NMC3_thread_descriptor* thr_dscr) {
	struct __NMC3_descriptor* dscr;
	unsigned long flags;

	if (!(thr_dscr->busy)) {
		printk(KERN_INFO "%s: the thread descriptor doesn't belong to any caller\n",
			__func__);
			return -EINVAL;
	}
	dscr = seek_dscr_byptr ((struct __NMC3_descriptor*)thr_dscr->dscr);
	dbg("dscr=%p,thr_dscr=%p",dscr,dscr->NMC3_thread_dscr);
	if (!dscr) {
		printk(KERN_NOTICE "%s: -ENXIO error\n", __func__);
		return -ENXIO;
	}
	spin_lock_irqsave(&dscr->load_lock, flags);
	thr_dscr->busy = 0;
	thr_dscr->app_cb = NULL;
	spin_unlock_irqrestore(&dscr->load_lock, flags);
	return 0;
}
EXPORT_SYMBOL(nmc3_release_dscr);

int nmc3_start (struct __NMC3_thread_descriptor* thr_dscr, void* init_code, size_t init_code_ln)
{
	struct __NMC3_descriptor* dscr;
	void* ptr = (init_code) ? init_code : __initnmc_abs;
	size_t ln = (init_code) ? init_code_ln : __initnmc_abs_len;
	unsigned long flags;
	enum __nmc3_states status;
	int ret = 0;

	if (!(thr_dscr->busy)) {
		printk(KERN_INFO "%s: the thread descriptor doesn't belong to any caller\n",
			__func__);
			return -EINVAL;
	}
	dscr = seek_dscr_byptr (thr_dscr->dscr);
	if (!dscr)
		return -ENXIO;
	spin_lock_irqsave(&dscr->load_lock, flags);

	request_nmc3_mem (dscr);
	status = check_run (dscr); /* this has an effect if only NMC3 is running */
	if (status == running) {
		dscr->status = running;
		dbg ("NMC3 is running, ISR_FROM_ARM = 0x%x",
			ioread32(&(dscr->NMC3_if_memory->ISR_FROM_ARM)));
		if ((ret = free_nmc3_mem (dscr)) < 0)
			dbg ("ERROR %d",ret);
		goto start_out;
	}
	ret = loadelf (dscr, ptr, ln, NULL);
	if (ret < 0) {
		printk (KERN_INFO "%s#%d: loadelf ERROR %d\n",__FILE__,__LINE__,ret);
		goto start_out;
	}
	iowrite32(ANY_MESSAGE, &(dscr->NMC3_if_memory->TO_ARM));
	iowrite32(CTRL_NM_NMI_REG_VAL, dscr->nmnmi);
	if ((ret = poll_nmc3_ready (dscr)) < 0) {
		printk (KERN_INFO "%s#%d: NMC3 doesn't start ERROR %d\n",
			__FILE__,__LINE__,ret);
//		DUMP_NMC3(0,0x400,0x20);
		goto start_out;
	}
	status = check_run (dscr);
	if (status != running) {
		dscr->status = status;
		dbg ("NMC3 can't run;");
//		DUMP_NMC3(0,0x400,0x20);
		ret = -ENODEV;
		goto start_out;
	}
	dscr->status = running;
	ret = 0;
	goto start_out;
start_out:
	spin_unlock_irqrestore(&dscr->load_lock, flags);
	return ret;
}
EXPORT_SYMBOL(nmc3_start);

int nmc3_elf_upload (struct __NMC3_thread_descriptor* thr_dscr,
	void* app_ptr, size_t app_ln, __u32* app_entry)
{
	struct __NMC3_descriptor* dscr;
	int ret;
	unsigned long flags;

	dbg ("%s",__func__);
	if (!(thr_dscr->busy)) {
		printk(KERN_INFO "%s: the thread descriptor doesn't belong to any caller\n",
			__func__);
			return -EINVAL;
	}
	dscr = seek_dscr_byptr ((thr_dscr)->dscr);
	if (!dscr) {
		printk(KERN_INFO "%s: ERROR, dscr=%p\n",__func__,dscr);
		return -ENXIO;
	}
	spin_lock_irqsave(&dscr->load_lock, flags);
	request_nmc3_mem (dscr);
	if ((ret = loadelf (dscr, app_ptr, app_ln, app_entry)) < 0) {
		printk(KERN_INFO "%s(): elf file load ERROR %d\n",__func__,ret);
		goto elf_out;
	}
elf_out:
	if (ret < 0)
		free_nmc3_mem (dscr);
	else
		ret = free_nmc3_mem (dscr);
	spin_unlock_irqrestore(&dscr->load_lock, flags);
	dbg ("end of %s, ret=%d, ISR_FROM_ARM = 0x%x", __func__, ret,
		ioread32(&(dscr->NMC3_if_memory->ISR_FROM_ARM)));
	return ret;
}
EXPORT_SYMBOL(nmc3_elf_upload);

int nmc3_app_run (struct __NMC3_thread_descriptor* thr_dscr, unsigned long app_entry,
		unsigned long param,
		void (*app_cb) (struct __NMC3_thread_descriptor* thr, int ret_code))
{
	struct __NMC3_descriptor* dscr;
	unsigned long flags;
	int ret;
	struct FromArmBlock* fromArm;

	if (!(thr_dscr->busy)) {
		printk(KERN_INFO "%s: the thread descriptor doesn't belong to any caller\n",
			__func__);
			return -EINVAL;
	}
	dscr = seek_dscr_byptr (thr_dscr->dscr);
	if (!dscr)
		return -ENXIO;
	if (dscr->status == reset) {
		printk(KERN_INFO "%s(): NMC3 hasn't started\n",__func__);
		return -EPERM;
	}

	if ((dscr->app_thr_active) && (dscr->app_thr_active != thr_dscr->num))
		return -EBUSY;
	spin_lock_irqsave(&dscr->load_lock, flags);

	if (dscr->app_thr_active) {
		ret = -EBUSY;
		goto apprun_out;
	}
	request_nmc3_mem (dscr);
	fromArm = (struct FromArmBlock*)dscr->NMC3_if_memory->SYNC_FROM_ARM;
	iowrite32 (param,&fromArm->param);
	if ((ret = cmd_to_NMC3 (dscr, RUN_PROGRAM, app_entry)) < 0)
		goto apprun_out;
	dscr->app_thr_active = thr_dscr->num;
	thr_dscr->app_cb = app_cb;
	ret = 0;
apprun_out:
	if (ret < 0)
		free_nmc3_mem (dscr);
	else
		ret = free_nmc3_mem (dscr);
	spin_unlock_irqrestore(&dscr->load_lock, flags);
#ifdef DEBUG
	udelay(2000);
#endif
	dbg ("end of %s, ret = %d, ISR_FROM_ARM = 0x%x",__func__, ret,
		ioread32(&(dscr->NMC3_if_memory->ISR_FROM_ARM)));
	PRINT_IF;
	return ret;
}
EXPORT_SYMBOL(nmc3_app_run);

int nmc3_app_stop_nmi (struct __NMC3_thread_descriptor* thr_dscr)
{
	struct __NMC3_descriptor* dscr;
	unsigned long flags;
	__u32* if_magic_nmc3;
#ifdef DEBUG
	int i;
#endif
	int ret;

	if (!(thr_dscr->busy))
		return -EPERM;
	dscr = seek_dscr_byptr (thr_dscr->dscr);
	if (!dscr)
		return -ENXIO;
	if (dscr->status != running)
		return - ENODEV;
	if ((dscr->app_thr_active) && (dscr->app_thr_active != thr_dscr->num))
		return -EBUSY;
	if_magic_nmc3 = (__u32*)dscr->NMC3_if_memory->MAGIC_CODE;
	spin_lock_irqsave(&dscr->load_lock, flags);
	dscr->status = reset;
#ifdef DEBUG
	request_nmc3_mem (dscr);
	for (i=0; i<ARRAY_SIZE(magic_nmc3); i++) 
		iowrite32(~magic_nmc3[i], if_magic_nmc3 + i);
	free_nmc3_mem (dscr);
	dbg ("magic code on target");
//	DUMP_NMC3(0,0x428,8);
//	DUMP_NMC3(0,0x440,8);
#if 0	/* this is made for test.
	iowrite32(TEST_NMI, &(dscr->NMC3_if_memory->FROM_ARM)); */
#endif
#endif
	thr_dscr->app_cb = NULL;
	dscr->app_thr_active = 0;
	iowrite32(CTRL_NM_NMI_REG_VAL, dscr->nmnmi);
	udelay (2000);
	iowrite32(ANY_MESSAGE, &(dscr->NMC3_if_memory->TO_ARM));
	iowrite32(NO_COMMAND, &(dscr->NMC3_if_memory->FROM_ARM));
	if ((ret = poll_nmc3_ready (dscr)) < 0) {
		printk (KERN_INFO "%s#%d: NMC3 doesn't restart ERROR %d\n",
			__FILE__,__LINE__,ret);
//		DUMP_NMC3(0,0x400,0x20);
		ret = -EIO;
		goto nmi_out;
	}
	dscr->status = check_run (dscr);
	if (dscr->status != running) {
		printk (KERN_INFO "%s#%d: magic code on target is wrong\n",
			__FILE__,__LINE__);
		ret = -EIO;
		goto nmi_out;
	}
	ret = 0;
nmi_out:
	spin_unlock_irqrestore(&dscr->load_lock, flags);
	dbg ("end of %s",__func__);
	return ret;
}
EXPORT_SYMBOL(nmc3_app_stop_nmi);

int nmc3_app_stop_irq (struct __NMC3_thread_descriptor* thr_dscr)
{
	struct __NMC3_descriptor* dscr;
	unsigned long flags;
	int ret;

	if (!(thr_dscr->busy))
		return -EPERM;
	dscr = seek_dscr_byptr (thr_dscr->dscr);
	if (!dscr)
		return -ENXIO;
	if (dscr->status != running) 
		return -ENODEV;
	if ((dscr->app_thr_active) && (dscr->app_thr_active != thr_dscr->num))
		return -EBUSY;
	spin_lock_irqsave(&dscr->load_lock, flags);
#ifdef DEBUG
#if 0 /* this is necessary only for testing: NMC3 goes in uncompleted loop. */
	if ((ret = cmd_to_NMC3 (dscr, TEST_NMI, 0)) < 0) {
		dbg ("ERROR %d", ret);
		goto irq_out;
	}
	udelay (2000);
#endif
#endif
	request_nmc3_mem (dscr);
	thr_dscr->app_cb = NULL;
	dscr->app_thr_active = 0;
	if ((ret = irq_cmd_to_NMC3 (dscr, STOP_APP)) < 0) {
		dbg ("ERROR %d", ret);
		goto irq_out;
	}
/* free_nmc3_mem () isn't necessary, because NMC3 has to go to command polling mode. */
	dscr->irq_cb = NULL;	/* to clear interrupt handler */
	if ((ret = irq_poll_nmc3_ready (dscr)) < 0) {
		printk (KERN_INFO "%s#%d: NMC3 doesn't restart ERROR %d\n",
			__FILE__,__LINE__,ret);
//		DUMP_NMC3(0,0x400,0x20);
		ret = -EIO;
		goto irq_out;
	}	
	ret = 0;
irq_out:
	spin_unlock_irqrestore(&dscr->load_lock, flags);
	dbg ("end of %s",__func__);
	return ret;
}
EXPORT_SYMBOL(nmc3_app_stop_irq);

/* nmc3_request_interrupt() warning: be sure that former interrupt request
 * has been serviced at NMC3 to this moment */
int nmc3_request_interrupt (struct __NMC3_thread_descriptor* thr_dscr, __u32 param)
{
	struct __NMC3_descriptor* dscr;
	unsigned long flags;
	struct FromArmBlock* fromArm;
	int ret;

	if (!(thr_dscr->busy))
		return -EPERM;
	dscr = seek_dscr_byptr (thr_dscr->dscr);
	if (!dscr)
		return -ENXIO;
	if (dscr->status != running) {
		printk(KERN_INFO "%s(): NMC3 hasn't started\n",__func__);
		return -EPERM;
	}
	spin_lock_irqsave(&dscr->load_lock, flags);
	request_nmc3_mem (dscr);
	fromArm = (struct FromArmBlock*)dscr->NMC3_if_memory->SYNC_FROM_ARM;
	iowrite32 (param,&fromArm->param);
	if ((ret = irq_cmd_to_NMC3 (dscr, IRQ_FROM_ARM)) < 0) {
		printk(KERN_INFO "%s(): target is busy", __func__);
		ret= -EBUSY;
		goto irq_nmc3_out;
	}
	ret = 0;
irq_nmc3_out:
	if (ret < 0)
		free_nmc3_mem (dscr);
	else
		ret = free_nmc3_mem (dscr);
	if (ret < 0)
		PRINT_IF;
	spin_unlock_irqrestore(&dscr->load_lock, flags);
	dbg ("ISR_FROM_ARM = 0x%x", ioread32(&(dscr->NMC3_if_memory->ISR_FROM_ARM))); 
	return ret;
}
EXPORT_SYMBOL(nmc3_request_interrupt);

int nmc3_mem_read (struct __NMC3_thread_descriptor* thr_dscr,
		void* dst, unsigned long src, size_t ln)
{
	struct __NMC3_descriptor* dscr;
	int ret;
	int bank_n;
	int bank_off;	
	unsigned long flags;
	int i;

	if (!(thr_dscr->busy))
		return -EPERM;
	dscr = seek_dscr_byptr (thr_dscr->dscr);
	if (!dscr)
		return -ENXIO;
#if 0 // this is necessary only for testing: NMC3 goes in uncompleted loop.
        if ((ret = cmd_to_NMC3 (dscr, TEST_NMI, 0)) < 0) {
                dbg ("ERROR %d", ret);
                goto read_out;
        }
        udelay (2000);
#endif

	if ((bank_n = seek_nmc3_bank(dscr, src)) < 0) {
		dbg ("ERROR %d", bank_n);
		ret = bank_n;
		goto read_out;
	}
	bank_off = seek_nmc3_off(dscr, bank_n, src);
	dbg ("bank_n=%d,bank_off=0x%x",bank_n,bank_off);
	if ((bank_off + ln) > ((u32)(dscr->nmc3_bank_base_virt[bank_n]) +
			dscr->nmc3_bank_ln[bank_n])) {
		dbg ("ERROR");
		ret = -EFAULT;
		goto read_out;
	}
	spin_lock_irqsave(&dscr->load_lock, flags);
	if (dscr->status == running)
		request_nmc3_mem (dscr);
	for (i=0; i < (ln>>2); i++)
		((__u32*)dst)[i] = ioread32 ((__u32*)bank_off + i);
	ret = ln;
	if (dscr->status == running)
		free_nmc3_mem (dscr);
	spin_unlock_irqrestore(&dscr->load_lock, flags);
read_out:
	dbg ("end of %s",__func__);
	return ret;
}
EXPORT_SYMBOL(nmc3_mem_read);

int nmc3_mem_write (struct __NMC3_thread_descriptor* thr_dscr,
		unsigned long dst, void* src, size_t ln)
{
	struct __NMC3_descriptor* dscr;
	int ret;
	int bank_n;
	int bank_off;	
	unsigned long flags;
	int i;

	if (!(thr_dscr->busy))
		return -EPERM;
	dscr = seek_dscr_byptr (thr_dscr->dscr);
	if (!dscr)
		return -ENXIO;
	if ((bank_n = seek_nmc3_bank(dscr, dst)) < 0) {
		dbg ("ERROR %d", bank_n);
		ret = bank_n;
		goto write_out;
	}
	bank_off = seek_nmc3_off(dscr, bank_n, dst);
	dbg ("bank_n=%d,bank_off=0x%x",bank_n,bank_off);
	if ((bank_off + ln) > ((u32)(dscr->nmc3_bank_base_virt[bank_n]) +
			dscr->nmc3_bank_ln[bank_n])) {
		dbg ("ERROR");
		ret = -EFAULT;
		goto write_out;
	}
	spin_lock_irqsave(&dscr->load_lock, flags);
	if (dscr->status == running)
		request_nmc3_mem (dscr);
#if 0
	iowrite32_rep ((void*)bank_off, src, ln); /* doesn't work with NMC3 memory */
#else
	for (i=0; i < (ln>>2); i++)
		iowrite32 (((__u32*)src)[i],(__u32*)bank_off + i);
#endif
//	DUMP_NMC3(2,0x400,4);
	ret = ln;
	if (dscr->status == running)
		free_nmc3_mem (dscr);
	spin_unlock_irqrestore(&dscr->load_lock, flags);
write_out:
	return ret;
}
EXPORT_SYMBOL(nmc3_mem_write);

/* nmc3_set_handler() warning: if a handler has been set to this moment
 * it will be changed by current one. NULL will remove NMC3 interrupt handler */
int nmc3_set_handler (struct __NMC3_thread_descriptor* thr_dscr,
	void (*irq_cb) (int irq, __u32 msg))
{
	struct __NMC3_descriptor* dscr;
	unsigned long flags;

	if (!(thr_dscr->busy))
		return -EPERM;
	dscr = seek_dscr_byptr (thr_dscr->dscr);
	if (!dscr)
		return -ENXIO;

	spin_lock_irqsave(&dscr->load_lock, flags);
	dscr->irq_cb = irq_cb;
	spin_unlock_irqrestore(&dscr->load_lock, flags);
	return 0;
}
EXPORT_SYMBOL(nmc3_set_handler);

/********************** end of NMC3 API ********************/

static int __init nmc3_probe (struct platform_device *pdev)
{
	struct __NMC3_descriptor* dscr;
	struct resource *vic_LPintrs_ena; // , *nm_ctrl;
	struct resource *nmrst_res, *nmnmi_res, *nmHPintrCLR_res, *nmLPintrCLR_res;
	struct resource *nmHPintrSET_res, *nmLPintrSET_res;
	struct resource *rc;
	int i, rcount = 0;
	int ret = 0;

	dbg ("%s",__func__);
	/******************* platform IO resources (control registers) *****************/
	rcount = pdev->num_resources;

	if (!(vic_LPintrs_ena = platform_get_resource_byname (pdev,
			IORESOURCE_MEM, "vic_LPintrs_ena"))) {
		printk (KERN_INFO "%s#%d: VIC resource getting ERROR\n", __FILE__,__LINE__);
		ret = -ENODEV;
		goto nmc3_probe_out;
	}
	if (!(nmnmi_res = platform_get_resource_byname (pdev,
			IORESOURCE_MEM, "nmc3_nmi_reg"))) {
		printk (KERN_INFO "%s#%d: resource getting ERROR\n", __FILE__,__LINE__);
		ret = -ENODEV;
		goto nmc3_probe_out;
	}

	if (!(nmrst_res = platform_get_resource_byname (pdev,
			IORESOURCE_MEM, "nmc3_reset_reg"))) {
		printk (KERN_INFO "%s#%d: resource getting ERROR\n", __FILE__,__LINE__);
		ret = -ENODEV;
		goto nmc3_probe_out;
	}

	if (!(nmHPintrCLR_res = platform_get_resource_byname (pdev,
			IORESOURCE_MEM, "nmc3_HPintrCLR_reg"))) {
		printk (KERN_INFO "%s#%d: resource getting ERROR\n", __FILE__,__LINE__);
		ret = -ENODEV;
		goto nmc3_probe_out;
	}
	if (!(nmLPintrCLR_res = platform_get_resource_byname (pdev,
			IORESOURCE_MEM, "nmc3_LPintrCLR_reg"))) {
		printk (KERN_INFO "%s#%d: resource getting ERROR\n", __FILE__,__LINE__);
		ret = -ENODEV;
		goto nmc3_probe_out;
	}
	if (!(nmHPintrSET_res = platform_get_resource_byname (pdev,
			IORESOURCE_MEM, "nmc3_HPintrSET_reg"))) {
		printk (KERN_INFO "%s#%d: resource getting ERROR\n", __FILE__,__LINE__);
		ret = -ENODEV;
		goto nmc3_probe_out;
	}
	if (!(nmLPintrSET_res = platform_get_resource_byname (pdev,
			IORESOURCE_MEM, "nmc3_LPintrSET_reg"))) {
		printk (KERN_INFO "%s#%d: resource getting ERROR\n", __FILE__,__LINE__);
		ret = -ENODEV;
		goto nmc3_probe_out;
	}

	/******************* NMC3 descriptor init *************/
	dscr = kzalloc (sizeof(struct __NMC3_descriptor), GFP_KERNEL);
	if (!dscr) {
		ret = -ENOMEM;
		printk(KERN_INFO "%s#%d, NMC3 ERROR: Insufficient memory!\n",
			__FILE__,__LINE__);
		goto nmc3_probe_out;
	}
	if (!(dscr->NMC3_thread_dscr = kzalloc (sizeof (struct __NMC3_thread_descriptor) *
			CONFIG_NMC3_DESCRIPTORS_LIMIT, GFP_KERNEL))) {
		ret = -ENOMEM;
		printk(KERN_INFO "%s#%d, NMC3 descr ERROR: Insufficient memory!\n",
			__FILE__,__LINE__);
		goto nmc3_probe_free;
	}
	for (i=0; i<CONFIG_NMC3_DESCRIPTORS_LIMIT; i++) {
		dscr->NMC3_thread_dscr[i].num = i+1; /* non-zero number in dscr->app_thr_active
					is sign of working NMC3 application */
		dscr->NMC3_thread_dscr[i].dscr = dscr;
	}
	if (!(dscr->vic_LPintrs_ena = ioremap(vic_LPintrs_ena->start,
		vic_LPintrs_ena->end - vic_LPintrs_ena->start + 1))) {
		printk(KERN_INFO "%s#%d, ioremap ERROR\n", __FILE__,__LINE__);
		ret = -ENOMEM;
		goto nmc3_probe_thr_free;
	}
	if (!(dscr->nmnmi = ioremap(nmnmi_res->start,
			nmnmi_res->end - nmnmi_res->start + 1))) {
		printk(KERN_INFO "%s#%d, ioremap ERROR\n", __FILE__,__LINE__);
		ret = -ENOMEM;
		goto nmc3_probe_vic_unmap;
	}
	if (!(dscr->nmreset = ioremap(nmrst_res->start,
		nmrst_res->end - nmrst_res->start + 1))) {
		printk(KERN_INFO "%s#%d, ioremap ERROR\n", __FILE__,__LINE__);
		ret = -ENOMEM;
		goto nmc3_probe_nmrst_unmap;
	}
	if (!(dscr->nmLPintr_clr = ioremap(nmLPintrCLR_res->start,
			nmHPintrCLR_res->end - nmHPintrCLR_res->start + 1))) {
		printk(KERN_INFO "%s#%d, ioremap ERROR\n", __FILE__,__LINE__);
		ret = -ENOMEM;
		goto nmc3_probe_nmrst_unmap;
	}
	if (!(dscr->nmHPintr_clr = ioremap(nmHPintrCLR_res->start,
			nmHPintrCLR_res->end - nmHPintrCLR_res->start + 1))) {
		printk(KERN_INFO "%s#%d, ioremap ERROR\n", __FILE__,__LINE__);
		ret = -ENOMEM;
		goto nmc3_probe_nmlpiclr_unmap;
	}
	if (!(dscr->nmLPintr_set = ioremap(nmLPintrSET_res->start,
			nmLPintrSET_res->end - nmLPintrSET_res->start + 1))) {
		printk(KERN_INFO "%s#%d, ioremap ERROR\n", __FILE__,__LINE__);
		ret = -ENOMEM;
		goto nmc3_probe_nmhpiclr_unmap;
	}
	if (!(dscr->nmHPintr_set = ioremap(nmHPintrSET_res->start,
			nmHPintrSET_res->end - nmHPintrSET_res->start + 1))) {
		printk(KERN_INFO "%s#%d, ioremap ERROR\n", __FILE__,__LINE__);
		ret = -ENOMEM;
		goto nmc3_probe_nmlpiset_unmap;
	}

	/********** NMC3 memory banks ***********/
	if (of_property_read_u32(pdev->dev.of_node, "bank-count", &dscr->nmc3_bank_count) < 0) {
		dev_err(&pdev->dev, "Missing required parameter 'bank-count'\n");
		return -ENODEV;
	}
	if (!dscr->nmc3_bank_count) {
		printk(KERN_INFO "%s#%d, ERROR no NMC3 mem banks are described\n", __FILE__,__LINE__);
		ret = -ENODEV;
		goto nmc3_probe_nmhpiset_unmap;
	}
	dbg ("bank_count=%d",dscr->nmc3_bank_count);

	for (i=0; i<dscr->nmc3_bank_count; i++) {
		rc = platform_get_resource(pdev, IORESOURCE_MEM, i + 7);
		dscr->nmc3_bank_ln[i] = rc->end - rc->start + 1;
		dscr->nmc3_bank_base_phys[i] = rc->start;
		if (!(dscr->nmc3_bank_base_virt[i] = ioremap_nocache (rc->start,
				dscr->nmc3_bank_ln[i]))) {
			printk(KERN_INFO "%s#%d, ioremap ERROR\n", __FILE__,__LINE__);
			ret = -ENOMEM;
			dscr->nmc3_bank_count = i;
			goto nmc3_probe_nmhpiset_unmap;
		}
		dbg ("bank%i: start 0x%x ln 0x%x", i, (unsigned int)dscr->nmc3_bank_base_phys[i],
			(unsigned int)dscr->nmc3_bank_ln[i]);
	}
	for (i=0; i<dscr->nmc3_bank_count; i++) {
		if (!(rc = platform_get_resource(pdev, IORESOURCE_MEM, i + 7 + dscr->nmc3_bank_count))) {
			printk (KERN_INFO "%s#%d: resource isn't found ERROR %d\n",
				__FILE__,__LINE__,i);
			ret = -ENODEV;
			goto nmc3_probe_nmhpiset_unmap;
		} else {
			dscr->nmc3_bank_base_phys_internal[i] = rc->start;
			dbg ("end=0x%x,ln=0x%lx",rc->end,dscr->nmc3_bank_ln[i]);
		}
	}
	dscr->NMC3_if_memory = (struct __NMC3_if_memory*)(dscr->nmc3_bank_base_virt[0] +
		NMC3_IF_MEM_OFF);
	dbg ("bank0 virt=%p, bank1=%p,bank2=%p,bank3=%p, NMC3_if_memory = %p",
		dscr->nmc3_bank_base_virt[0],dscr->nmc3_bank_base_virt[1],
		dscr->nmc3_bank_base_virt[2],dscr->nmc3_bank_base_virt[3],
		dscr->NMC3_if_memory);
	dbg ("bank0 phys=0x%x/0x%x, bank2 phys = 0x%x/0x%x)",
		dscr->nmc3_bank_base_phys[0],dscr->nmc3_bank_base_phys_internal[0],
		dscr->nmc3_bank_base_phys[2], dscr->nmc3_bank_base_phys_internal[2]);

	/********** NMC3 IRQs ***********/
	dscr->HPirq = platform_get_irq(pdev, 0);
	if (!dscr->HPirq) {
		printk(KERN_INFO "%s#%d: failed to find device HP irq\n", __FILE__,__LINE__);
		goto nmc3_probe_nmhpiset_unmap;
	}
	dscr->LPirq = platform_get_irq(pdev, 1);
	if (!dscr->LPirq) {
		printk(KERN_INFO "%s#%d: failed to find device LP irq\n", __FILE__,__LINE__);
		goto nmc3_probe_nmhpiset_unmap;
	}

	if ((ret = devm_request_irq (&pdev->dev, dscr->HPirq, HPisr, 0, NMC3_PROCNAME, dscr)) < 0) {
		printk (KERN_INFO "%s#%d: request_irq ERROR %d", __FILE__,__LINE__, ret);
		goto nmc3_probe_nmhpiset_unmap;
	} 
	if ((ret = devm_request_irq (&pdev->dev, dscr->LPirq, LPisr, 0, NMC3_PROCNAME, dscr)) < 0) {
		printk (KERN_INFO "%s#%d: request_irq ERROR %d", __FILE__,__LINE__, ret);
		goto nmc3_probe_hpirqfree;
	}

	INIT_LIST_HEAD(&dscr->list);
	if (of_property_read_u32(pdev->dev.of_node, "device-id", &pdev->id) < 0) {
		dev_err(&pdev->dev, "Missing required parameter 'device-id'\n");
		return -ENODEV;
	}
	sprintf (dscr->Name, "%s_%d", NMC3_PROCNAME, pdev->id);
	dbg ("nmc3_desc:next=%p,prev=%p",nmc3_dscr.next,nmc3_dscr.prev);
	list_add_tail(&dscr->list, &nmc3_dscr);
	dbg ("nmc3_desc:next=%p,prev=%p",nmc3_dscr.next,nmc3_dscr.prev);
	dev_set_drvdata(&pdev->dev, dscr);
	dbg ("dscr=%p,next=%p,prev=%p",dscr,dscr->list.next,dscr->list.prev);
	spin_lock_init (&dscr->load_lock);
	printk (KERN_INFO "NMC3_DRIVER probe OK\n");
//	DUMP_NMC3(0, 0x00000, 20);
//	DUMP_NMC3(2, 0x00000, 20);

	goto nmc3_probe_out;
nmc3_probe_hpirqfree:
	free_irq (dscr->HPirq, dscr);
nmc3_probe_nmhpiset_unmap:
	iounmap (dscr->nmHPintr_set);
	for (i=0; i < dscr->nmc3_bank_count; i++)
		iounmap (dscr->nmc3_bank_base_virt[i]);
nmc3_probe_nmlpiset_unmap:
	iounmap (dscr->nmLPintr_set);
nmc3_probe_nmhpiclr_unmap:
	iounmap (dscr->nmHPintr_clr);
nmc3_probe_nmlpiclr_unmap:
	iounmap (dscr->nmLPintr_clr);
nmc3_probe_nmrst_unmap:
	if (dscr->nmreset)
		iounmap (dscr->nmreset);
	iounmap (dscr->nmnmi);
nmc3_probe_vic_unmap:
	iounmap (dscr->vic_LPintrs_ena);
nmc3_probe_thr_free:
	kfree (dscr->NMC3_thread_dscr);
nmc3_probe_free:
	kfree (dscr);
nmc3_probe_out:
	return ret;
}

static int nmc3_remove (struct platform_device *pdev)
{
	struct __NMC3_descriptor* dscr = NULL;
	struct __NMC3_descriptor* pdscr;
	struct list_head* lptr;
	int ret;
	int i;

	list_for_each(lptr,&nmc3_dscr) {
		dscr = container_of(lptr, struct __NMC3_descriptor, list);
		if (pdev->id == dscr->num) {
			dbg ("list_head has been found");
			break;
		}
	}
	pdscr = dev_get_drvdata(&pdev->dev);
	list_del(&pdscr->list);
	if (dscr != pdscr) {
		printk ("removed descriptor ERROR %p/%p\n",dscr, pdscr);
		ret= -EFAULT;
	} else
		ret = 0;
	dbg ("dscr = %p",dscr);

	free_irq (dscr->HPirq, dscr);
	free_irq (dscr->LPirq, dscr);
	iounmap (dscr->nmHPintr_set);
	for (i=0; i < dscr->nmc3_bank_count; i++)
		iounmap (dscr->nmc3_bank_base_virt[i]);
	iounmap (dscr->nmLPintr_set);
	iounmap (dscr->nmHPintr_clr);
	iounmap (dscr->nmLPintr_clr);
	if (dscr->nmreset)
		iounmap (dscr->nmreset);
	iounmap (dscr->nmnmi);
	iounmap (dscr->vic_LPintrs_ena);

	kfree(pdscr->NMC3_thread_dscr);
	kfree(pdscr);		

	return ret;
}

static const struct of_device_id module_nmc3_of_match_table[] = {
	{ .compatible = "module,nmc3", },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, module_nmc3_of_match_table);

static struct platform_driver module_nmc3 = {
	.probe		= nmc3_probe,
	.remove		= nmc3_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "rcmod_soc_nmc3",
		.of_match_table = of_match_ptr(module_nmc3_of_match_table),
	},
};
MODULE_ALIAS("platform:rcmod_soc_nmc3");

module_platform_driver(module_nmc3);

/* Module parameters */
MODULE_AUTHOR ("Gennadiy Kurtsman <gkurtsman@module.ru>");
MODULE_DESCRIPTION ("RC MODULE SOC: Driver of ARM-NMC3 interface");
MODULE_LICENSE("GPL");
MODULE_VERSION(NMC3_DRIVER_VERSION);

