/*
 * include/linux/module_crypto_aes.h
 *
 *	Copyright (C) 2010 Module
 *	Written by Sergey Mironov <ierton@gmail.com>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#ifndef _MODULE_CRYPTO_AES_H_
#define _MODULE_CRYPTO_AES_H_


#define MCRYPTO_AES_REGISTERS_SIZE (0x80)

/* Registers */
#define MCRYPTO_AES_KEY_0       0x00
#define MCRYPTO_AES_KEY_1       0x04
#define MCRYPTO_AES_KEY_2       0x08
#define MCRYPTO_AES_KEY_3       0x0C

#define MCRYPTO_AES_DATA_IN_0   0x10
#define MCRYPTO_AES_DATA_IN_1   0x14
#define MCRYPTO_AES_DATA_IN_2   0x18
#define MCRYPTO_AES_DATA_IN_3   0x1C

#define MCRYPTO_AES_IRQMASK     0x20
#define MCRYPTO_AES_STATUS      0x24
#define MCRYPTO_AES_STATUS_IDLE 1

#define MCRYPTO_AES_DATA_OUT_0  0x28
#define MCRYPTO_AES_DATA_OUT_1  0x2C
#define MCRYPTO_AES_DATA_OUT_2  0x30
#define MCRYPTO_AES_DATA_OUT_3  0x34

#endif

