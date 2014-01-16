/*
 * i2c-ocores.h - definitions for the i2c-ocores interface
 *
 * Peter Korsgaard <jacmet@sunsite.dk>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef _LINUX_I2C_OCORES_GAISLER_H
#define _LINUX_I2C_OCORES_GAISLER_H

struct ocores_gaisler_i2c_platform_data {
	u32 clock_khz;	/* input clock in kHz */
	u32 i2c_khz;	/* i2c clock in kHz */
};

#endif /* _LINUX_I2C_OCORES_GAISLER_H */
