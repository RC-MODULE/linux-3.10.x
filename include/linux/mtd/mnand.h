#ifndef __LINUX_MTD_MNAND_H__
#define __LINUX_MTD_MNAND_H__

struct mnand_platform_data {
	struct mtd_partition	*parts;
	int			nr_parts;
};

#endif
