#ifndef __LINUX_MTD_MNAND_H__
#define __LINUX_MTD_MNAND_H__

#define OOB_USE // only one read and write function can be enabled for version 5.5 (read or read_oob and write or write_oob)

struct mnand_platform_data {
	struct mtd_partition	*parts;
	int			nr_parts;
};

#endif
