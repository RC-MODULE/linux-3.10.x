/*
 * include/linux/module_crypto.h
 *
 *	Copyright (C) 2010 Module
 *	Written by Sergey Mironov <ierton@gmail.com>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#ifndef _MODULE_CRYPTO_H_
#define _MODULE_CRYPTO_H_


/* Registers */
// CONF
//
#define MCRYPTO_CONF_S          (0x00*4)
#define MCRYPTO_CONF_W          (0x01*4)
/*MCRYPTO_CONF bits{{{*/
#define MCRYPTO_CONF_3DES           0
#define MCRYPTO_CONF_DES            (1<<31)

#define MCRYPTO_CONF_3DES_MODE_EDE  (1<<0 | 1<<2)
#define MCRYPTO_CONF_3DES_MODE_DED  (1<<1)
#define MCRYPTO_CONF_3DES_CBC_ENC   (1<<3)
#define MCRYPTO_CONF_3DES_CBC_DEC   (0<<3)
#define MCRYPTO_CONF_3DES_MODE_MASK (~( \
        MCRYPTO_CONF_3DES_MODE_DED | MCRYPTO_CONF_3DES_MODE_EDE |\
        MCRYPTO_CONF_3DES_CBC_ENC | MCRYPTO_CONF_3DES_CBC_DEC))

#define MCRYPTO_CONF_3DES_SCHEME_ECB   (0<<30)
#define MCRYPTO_CONF_3DES_SCHEME_CBC   (1<<30)
#define MCRYPTO_CONF_3DES_SCHEME_MASK  (~( \
        MCRYPTO_CONF_3DES_SCHEME_CBC | MCRYPTO_CONF_3DES_SCHEME_ECB))
/*}}}*/

// APB IN/OUT
//
#define MCRYPTO_APB_OUT_LOW     (0x10*4)
#define MCRYPTO_APB_OUT_HIGH    (0x11*4)
#define MCRYPTO_APB_IN_LOW      (0x08*4)
#define MCRYPTO_APB_IN_HIGH     (0x09*4)

// KEYS
//
#define MCRYPTO_KEY_S0_LOW      (0x02*4)
#define MCRYPTO_KEY_S0_HIGH     (0x03*4)
#define MCRYPTO_KEY_S1_LOW      (0x04*4)
#define MCRYPTO_KEY_S1_HIGH     (0x05*4)
#define MCRYPTO_KEY_S2_LOW      (0x06*4)
#define MCRYPTO_KEY_S2_HIGH     (0x07*4)

#define MCRYPTO_KEY_W0_LOW      (0x0A*4)
#define MCRYPTO_KEY_W0_HIGH     (0x0B*4)
#define MCRYPTO_KEY_W1_LOW      (0x0C*4)
#define MCRYPTO_KEY_W1_HIGH     (0x0D*4)
#define MCRYPTO_KEY_W2_LOW      (0x0E*4)
#define MCRYPTO_KEY_W2_HIGH     (0x0F*4)

// INIT VECTORS
//
#define MCRYPTO_IV_S_LOW        (0x50)
#define MCRYPTO_IV_S_HIGH       (0x54)

#define MCRYPTO_IV_W_LOW        (0x58)
#define MCRYPTO_IV_W_HIGH       (0x5C)

// STATUS
//
#define MCRYPTO_STATUS          (0x48)
#define MCRYPTO_STATUS_WBW      (0x4C)
#define MCRYPTO_MASK            (0x64)
/*MCRYPTO_STATUS bits{{{*/
#define MCRYPTO_NIRQ                     10
#define MCRYPTO_STATUS_INAXI             0
#define MCRYPTO_STATUS_INDMA_BUFEND      1
#define MCRYPTO_STATUS_INAXI_NONACT      2
#define MCRYPTO_STATUS_INAXI_BADCH       3
#define MCRYPTO_STATUS_INAXI_SEGEND      4
#define MCRYPTO_STATUS_OUTAXI            5
#define MCRYPTO_STATUS_OUTDMA_BUFEND     6
#define MCRYPTO_STATUS_OUTAXI_NONACT     7
#define MCRYPTO_STATUS_OUTAXI_BADCH      8
#define MCRYPTO_STATUS_OUTAXI_SEGEND     9
/*}}}*/

#define MCRYPTO_ENDIAN          (0x68)
/*MCRYPTO_ENDIAN bits{{{*/
#define MCRYPTO_ENDIAN_LE       (-1)
#define MCRYPTO_ENDIAN_BE       (0)
/*}}}*/

// AXI INPUT REGISTERS
//
#define MCRYPTO_AxiParamCR_o        0xC0
#define MCRYPTO_DataChConfigCR_o    0xBC
/*MCRYPTO_DataChConfigCR_o bits{{{*/
#define MCRYPTO_DataChConfigCR_o_BufReqEdge(x) ((x)<<0)
#define MCRYPTO_DataChConfigCR_o_StopForLastDmaBufferWord_1(x) ((x)<<7)
#define MCRYPTO_DataChConfigCR_o_StopForLastDmaBufferWord_2(x) ((x)<<8)
/*}}}*/

#define MCRYPTO_DataChEnableCR      0xB8
/*MCRYPTO_DataChEnableCR bits{{{*/
#define MCRYPTO_DataChEnableCR_EnableCh_1(x) ((x)<<0)
#define MCRYPTO_DataChEnableCR_EnableCh_2(x) ((x)<<1)
#define MCRYPTO_DataChEnableCR_ReqBufEnable(x) ((x)<<30)
#define MCRYPTO_DataChEnableCR_SWReset (1<<31)
/*}}}*/

#define MCRYPTO_AxiInIrqFlag        0xC4
#define MCRYPTO_AxiInIrqFlagMask    0xC8
#define MCRYPTO_AxiInSlvErrAddr     0xCC
#define MCRYPTO_AxiInStatus         0xB0
/*MCRYPTO_AxiInStatus bits{{{*/
#define MCRYPTO_AxiInStatus_ActiveTrans_MASK (~(1<<0))
#define MCRYPTO_AxiInStatus_ActiveTrans (1<<0)
/*}}}*/
#define MCRYPTO_AxiAwlen            0x130

// DMA REGISTERS
//
// DMAIN start
#define MCRYPTO_DMAIN_BASE      0x80
// DMAIN stop
#define MCRYPTO_DMAOUT_BASE     0x100

// Группа регистров DMA
#define MCRYPTO_DMA_DESC_1      (0x80 - MCRYPTO_DMAIN_BASE)
// Конечный адрес дескриптора DMA
#define MCRYPTO_DMA_DESC_2      (0x84 - MCRYPTO_DMAIN_BASE)
// Параметры дескриптора DMA
#define MCRYPTO_DMA_DESC_3      (0x88 - MCRYPTO_DMAIN_BASE)
/*MCRYPTO_DMA_DESC_3 bits{{{*/
#define MCRYPTO_DMA_DESC_3_DMA_CHANNEL(x) ((x)<<0)
#define MCRYPTO_DMA_DESC_3_DMA_SEGSIZE(x) ((x)<<6)

/* single buffer */
#define MCRYPTO_DMA_DESC_3_DMA_BUFTYPE_S (0<<22)
/* ring buffer   */
#define MCRYPTO_DMA_DESC_3_DMA_BUFTYPE_R (1<<22)
/* double buffer */
#define MCRYPTO_DMA_DESC_3_DMA_BUFTYPE_D (2<<22)

#define MCRYPTO_DMA_DESC_3_ROT_TABLE_WR (1<<29)
#define MCRYPTO_DMA_DESC_3_RD_DMA_DESCR (1<<30)
#define MCRYPTO_DMA_DESC_3_WR_DMA_DESCR (1<<31)
/*}}}*/

// Следующий для записи адрес в данном DMA дескрипторе
#define MCRYPTO_DMA_DESC_4      (0x8C - MCRYPTO_DMAIN_BASE)
// Регистр масок прерываний (установить в "1" для выработки соответствующего прерывания), используются младшие 4 разряда
#define MCRYPTO_DMA_CONF_1      (0x90 - MCRYPTO_DMAIN_BASE)
/*MCRYPTO_DMA_CONF_1 bits{{{*/
#define MCRYPTO_DMA_CONF_1_IRQ_LAST_SEGADDR(x)  ((x)<<3)
#define MCRYPTO_DMA_CONF_1_IRQ_BADCH(x)         ((x)<<2)
#define MCRYPTO_DMA_CONF_1_IRQ_BAD_DESC(x)      ((x)<<1)
#define MCRYPTO_DMA_CONF_1_IRQ_LAST_BADDR(x)    ((x)<<0)
#define MCRYPTO_DMA_CONF_1_DATA_SEND_IRQ(x)     ((x)<<4)
#define MCRYPTO_DMA_CONF_1_DMA_ADDR_STEP(x)     ((x)<<5)
/*}}}*/

// Регистр статуса
#define MCRYPTO_DMA_STATUS      (0xAC - MCRYPTO_DMAIN_BASE)
// Статус прерывания о конце буфера для младших 32 каналов
#define MCRYPTO_DMA_STATUS_BUFEND_1     (0x9C - MCRYPTO_DMAIN_BASE)
// Статус прерывания о конце буфера для старших 32 каналов
#define MCRYPTO_DMA_STATUS_BUFEND_2     (0xA0 - MCRYPTO_DMAIN_BASE)
// Статус прерывания об обращении к неактивному дескриптору для младших 32 каналов
#define MCRYPTO_DMA_STATUS_NONACT_1     (0xA4 - MCRYPTO_DMAIN_BASE)
// Статус прерывания об обращении к неактивному дескриптору для старших 32 каналов
#define MCRYPTO_DMA_STATUS_NONACT_2     (0xA8 - MCRYPTO_DMAIN_BASE)
// Статус прерывания о конце сегмента для младших 32 каналов
#define MCRYPTO_DMA_STATUS_SEGEND_1     (0x94 - MCRYPTO_DMAIN_BASE)
// Статус прерывания о конце сегмента для старших 32 каналов
#define MCRYPTO_DMA_STATUS_SEGEND_2     (0x98 - MCRYPTO_DMAIN_BASE)

#endif

