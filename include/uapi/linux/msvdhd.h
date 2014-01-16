#ifndef __UAPI_LINUX_MSVDHD_H__
#define __UAPI_LINUX_MSVDHD_H__

#ifndef __KERNEL__
#include <sys/ioctl.h>
#include <sys/mman.h>
#endif

#define MSVD_IO_MAGIC		'm'

#define MSVD_ENABLE_IRQ		_IO(MSVD_IO_MAGIC, 0)

#ifndef __KERNEL__
static inline int msvd_enable_irq(int fd)
{
	return ioctl(fd, MSVD_ENABLE_IRQ, 0);
}
#endif

#define MSVD_DISABLE_IRQ	_IO(MSVD_IO_MAGIC, 1)

#ifndef __KERNEL__
static inline int msvd_disable_irq(int fd)
{
	return ioctl(fd, MSVD_DISABLE_IRQ, 0);
}
#endif

struct msvd_wait_irq_params {
	unsigned int status;
};

#define MSVD_WAIT_IRQ		_IOR(MSVD_IO_MAGIC, 2, struct msvd_wait_irq_params)

#ifndef __KERNEL__
static inline int msvd_wait_for_irq(int fd, unsigned int *status)
{
	struct msvd_wait_irq_params params;
	int ret;

	ret = ioctl(fd, MSVD_WAIT_IRQ, &params);
	if (!ret)
		*status = params.status;
	return ret;
}
#endif

struct msvd_get_buf_phys_params {
	unsigned long addr;
};

#define MSVD_GET_BUF_PHYS	_IOR(MSVD_IO_MAGIC, 3, struct msvd_get_buf_phys_params)

#ifndef __KERNEL__
static inline int msvd_get_buf_phys(int fd, unsigned long *addr)
{
	struct msvd_get_buf_phys_params params;
	int ret;

	ret = ioctl(fd, MSVD_GET_BUF_PHYS, &params);
	if (!ret)
		*addr = params.addr;
	return ret;
}
#endif

struct msvd_get_extmem_phys_params {
	unsigned long addr;
};

#define MSVD_GET_EXTMEM_PHYS	_IOR(MSVD_IO_MAGIC, 4, struct msvd_get_extmem_phys_params)

#ifndef __KERNEL__
static inline int msvd_get_extmem_phys(int fd, unsigned long *addr)
{
	struct msvd_get_extmem_phys_params params;
	int ret;

	ret = ioctl(fd, MSVD_GET_EXTMEM_PHYS, &params);
	if (!ret)
		*addr = params.addr;
	return ret;
}
#endif

struct msvd_get_intmem_phys_params {
	unsigned long addr;
};

#define MSVD_GET_INTMEM_PHYS	_IOR(MSVD_IO_MAGIC, 5, struct msvd_get_intmem_phys_params)

#ifndef __KERNEL__
static inline int msvd_get_intmem_phys(int fd, unsigned long *addr)
{
	struct msvd_get_intmem_phys_params params;
	int ret;

	ret = ioctl(fd, MSVD_GET_INTMEM_PHYS, &params);
	if (!ret)
		*addr = params.addr;
	return ret;
}
#endif


#define MSVD_MAP_REGS_OFFSET		0
#define MSVD_MAP_REGS_SIZE		0x20000

#define MSVD_MAP_BUF_OFFSET		0x100000
#define MSVD_MAP_BUF_SIZE		(4 * 1024 * 1024)

#define MSVD_MAP_EXTMEM_OFFSET		0x1000000
#define MSVD_MAP_EXTMEM_SIZE		(8 * 1024 * 1024)

#define MSVD_MAP_INTMEM_OFFSET		0x10000000
#define MSVD_MAP_INTMEM_SIZE		(256 * 1024)

#ifndef __KERNEL__
static inline void *msvd_map_registers(int fd)
{
	return mmap(0, MSVD_MAP_REGS_SIZE, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, MSVD_MAP_REGS_OFFSET);
}
static inline int msvd_unmap_registers(void *regs)
{
	return munmap(regs, MSVD_MAP_REGS_SIZE);
}

static inline void *msvd_map_buffer(int fd)
{
	return mmap(0, MSVD_MAP_BUF_SIZE, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, MSVD_MAP_BUF_OFFSET);
}
static inline int msvd_unmap_buffer(void *buf)
{
	return munmap(buf, MSVD_MAP_BUF_SIZE);
}

static inline void *msvd_map_extmem(int fd)
{
	return mmap(0, MSVD_MAP_EXTMEM_SIZE, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, MSVD_MAP_EXTMEM_OFFSET);
}
static inline int msvd_unmap_extmem(void *extmem)
{
	return munmap(extmem, MSVD_MAP_EXTMEM_SIZE);
}

static inline void *msvd_map_intmem(int fd)
{
	return mmap(0, MSVD_MAP_INTMEM_SIZE, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, MSVD_MAP_INTMEM_OFFSET);
}
static inline int msvd_unmap_intmem(void *intmem)
{
	return munmap(intmem, MSVD_MAP_INTMEM_SIZE);
}

#endif

#endif /* __UAPI_LINUX_MSVDHD_H__ */
