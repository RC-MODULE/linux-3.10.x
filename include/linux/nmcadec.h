#ifndef NMCADEC_H
#define NMCADEC_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#endif

/* FIXME: channel pos codes? */

#define NMCADEC_MAXCHAN		8
#define NMCADEC_MAXSAMPLES	256

#define NMCADEC_STATUS_OK	0

struct nmcadec_frame {
	unsigned int offset;		/* from stream start */
	unsigned int rate;		/* samples per second */
	unsigned int chan_n;		/* chanels */
	unsigned int status;
	unsigned short samples[NMCADEC_MAXCHAN * NMCADEC_MAXSAMPLES];
} __attribute__((aligned(8)));

#define NMCADEC_IRING_BYTES	65536
#define NMCADEC_ORING_SIZE	64

#ifndef __KERNEL__
#define PAGE_SIZE	4096
#define PAGE_ALIGN(v)	(((v) + (PAGE_SIZE - 1)) & ~(PAGE_SIZE - 1))
#endif

#define NMCADEC_MAP_SIZE	PAGE_ALIGN(NMCADEC_ORING_SIZE * \
					sizeof(struct nmcadec_frame))

#define NMCADEC_IOCTL_MAGIC	0xc1
#define NMCADEC_IOCTL_FLUSH	_IO(NMCADEC_IOCTL_MAGIC, 0)
#define NMCADEC_IOCTL_REQUEST_CHANNELS _IO(NMCADEC_IOCTL_MAGIC, 1)

#ifndef __KERNEL__
static inline int nmcadec_open(const char *pathname)
{
	return open(pathname, O_RDWR);
}
static inline int nmcadec_open_nonblock(const char *pathname)
{
	return open(pathname, O_RDWR | O_NONBLOCK);
}

static inline int nmcadec_close(int fd)
{
	return close(fd);
}

static inline struct nmcadec_frame *nmcadec_map(int fd)
{
	return mmap(0, NMCADEC_MAP_SIZE, PROT_READ, MAP_SHARED, fd, 0);
}

static inline int nmcadec_unmap(struct nmcadec_frame *fbuf)
{
	return munmap(fbuf, NMCADEC_MAP_SIZE);
}

static inline int nmcadec_write(int fd, const char *buf, size_t size)
{
	return write(fd, buf, size);
}

static inline int nmcadec_fetch(int fd, struct nmcadec_frame *fbuf,
		struct nmcadec_frame **frame)
{
	unsigned int index;
	int ret;

	ret = read(fd, &index, sizeof(unsigned int));
	if (ret <= 0)
		return ret;

	*frame = &fbuf[index];
	return 1;
}

static inline int nmcadec_flush(int fd, int wait)
{
	return ioctl(fd, NMCADEC_IOCTL_FLUSH, wait);
}

static inline int nmcadec_request_channels(int fd, int n)
{
	return ioctl(fd, NMCADEC_IOCTL_REQUEST_CHANNELS, n);
}

#endif

#endif
