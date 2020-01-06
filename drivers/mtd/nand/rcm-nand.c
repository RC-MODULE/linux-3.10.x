/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <alexeis@astrosoft.ru>
 */

#include <linux/types.h>
#include <linux/io.h> 
#include <linux/mtd/partitions.h>
#include <linux/delay.h> 
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/crc32.h>
#include <linux/string.h>
#include <linux/mtd/mtd.h> 
#include <linux/mtd/rawnand.h>

#ifndef __UBOOT__
#include <linux/module.h>
#include <linux/unistd.h>
#include <linux/gfp.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/mtd/nand.h>
#include <linux/mfd/syscon.h>
#include <linux/mtd/physmap.h>
#include <linux/semaphore.h> 
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include "rcm-nandids.h"
#else // __UBOOT__
#include <fdtdec.h>
#include <dm/of.h>
#include <dm/device.h>
#include <dm/of_access.h>
#include <dm/of_addr.h>
#include <dm/uclass.h>
#include <linux/ioport.h>
#include <regmap.h>
#include <nand.h>
#include "rcm_nandids.h"
#endif

#define NAND_CTRL_BASE                  0x3C032000
#define NAND_REG_status                 0x00
#define NAND_REG_control                0x04
#define NAND_REG_command                0x08
#define NAND_REG_col_addr               0x0c
#define NAND_REG_row_addr_read_d        0x10
#define NAND_REG_irq_mask_nand          0x14
#define NAND_REG_cdb_buffer_enable      0x18
#define NAND_REG_cdb_buffer_type        0x1c
#define NAND_REG_cdb_ch0_buffer_status  0x20
#define NAND_REG_axi_arlen              0x24
#define NAND_REG_sw_rstn_r              0x28
#define NAND_REG_reserved_2c            0x2c
#define NAND_REG_axi_if_err_addr        0x30
#define NAND_REG_axi_if_status          0x34
#define NAND_REG_msb_lsbr               0x38
#define NAND_REG_reserved_3c            0x3c
#define NAND_REG_start_dma_r            0x40
#define NAND_REG_end_dma_r              0x44
#define NAND_REG_current_dma_r          0x48
#define NAND_REG_cntrl_dma_r            0x4c
#define NAND_REG_cntrl_sw_rst           0x50
#define NAND_REG_id                     0x54
#define NAND_REG_version                0x58
#define NAND_REG_reserved_5c            0x5c
#define NAND_REG_reserved_60            0x60
#define NAND_REG_reserved_64            0x64
#define NAND_REG_reserved_68            0x68
#define NAND_REG_awlen_max              0x6c
#define NAND_REG_sw_rst                 0x70
#define NAND_REG_msb_lsbw               0x74
#define NAND_REG_respw_status           0x78
#define NAND_REG_status_full            0x7c
#define NAND_REG_reserved_80            0x80
#define NAND_REG_reserved_84            0x84
#define NAND_REG_start_dma_w            0x88
#define NAND_REG_end_dma_w              0x8c
#define NAND_REG_current_dma_w          0x90
#define NAND_REG_cntrl_dma_w            0x94
#define NAND_REG_reserved_9c            0x9c
#define NAND_REG_reserved_a0            0xa0
#define NAND_REG_reserved_a4            0xa4
#define NAND_REG_reserved_a8            0xa8
#define NAND_REG_reserved_ac            0xac
#define NAND_REG_reserved_b0            0xb0
#define NAND_REG_irq_status             0xb4
#define NAND_REG_timing_0               0xb8
#define NAND_REG_timing_1               0xbc
#define NAND_REG_timing_2               0xc0
#define NAND_REG_timing_3               0xc4
#define NAND_REG_timing_4               0xc8
#define NAND_REG_timing_5               0xcc
#define NAND_REG_timing_6               0xd0
#define NAND_REG_timing_7               0xd4
// NAND_REG_status
#define STAT_REG_NAND_FAIL              0x00000100      // возникновение некорректируемой ошибки
#define STAT_REG_CONT_READY             0x00000200      // контроллер NAND готов к приему команды
#define STAT_REG_NAND_READY             0x00000400      // флэш-память готова к выполнению операций
// NAND_REG_control
#define CTRL_REG_OP_BEGIN_SHIFT         0               // 0-флаг начала операции (1-начать)
#define CTRL_REG_PAGE_SIZE_SHIFT        1               // 3..1-размер страницы (0-512..7-65536)
#define CTRL_REG_NUM_ADR_BYTES_SHIFT    4               // 6..4-количество байтов (циклов) адреса в текущей команде
#define CTRL_REG_ECC_MODE_SHIFT         7               // 8..7-режим коррекции ошибок
#define CTRL_REG_CE_SHIFT               9               // 9-номер кристалла для текущей операции
#define	CTRL_REG_HWP_SHIFT              11              // 11-аппаратная защита от записи(0-запись разрешена?)
#define CTRL_REG_OOB_ECC_SHIFT          12              // 12-разрешение оценки кодов коррекции ошибок для данных в поле OOB(рекомендуется 1)
#define CTRL_REG_OOB_SIZE_SHIFT         13              // 23..13-размер OOB
#define	CTRL_REG_FCMD_SHIFT             24              // 31..24-первая команда для отправки во флэш память
// NAND_REG_command
#define CMD_REG_READ                    0x01            // чтение NAND
#define CMD_REG_READ_ID                 0x03            // чтение идентификатора
#define CMD_REG_PROGRAM 	        0x04            // программирование NAND
#define CMD_REG_ERASE_BLOCK	        0x06            // стирание блока
#define CMD_REG_RESET                   0x07            // сброс флэш памяти NAND
// NAND_REG_cntrl_dma_w,NAND_REG_cntrl_dma_r
#define SEGM_SIZE_SHIFT                 5               // 28..5-размер сегмента (байты–1),по достижению конца сгенерируется прерывания
#define START_ADDR_GEN                  (1<<31)         // 31-после задания значения начать генерировать адреса с начального по конечный
// NAND_REG_irq_mask_nand (Таблица 1.829)
#define IRQ_READ_FINISH                 (1 << 0)        // прерывание по завершении выполнения команды чтения
#define IRQ_READID                      (1 << 1)        // прерывание по завершении выполнения команды считывания идентификационного номера
#define IRQ_PROGRAM_FINISH              (1 << 2)        // прерывание по завершении выполнения команды программирования
#define IRQ_ERASE                       (1 << 3)        // прерывание по завершении выполнения команды стирания блока
#define IRQ_RESET                       (1 << 4)        // прерывание по завершении выполнения команды сброса флэш-памяти NAND
#define IRQ_UNCORR_ERROR0               (1 << 5)        // прерывание при возникновении некорректируемой ошибки
#define IRQ_AXIW_ERROR                  (1 << 6)        // прерывание при возникновении ошибочной транзакции по интерфейсу записи AXI (неверное значение BRESP)
#define IRQ_LAST_ADR_AXIW               (1 << 7)        // прерывание по достижению последнего адреса ведущего интерфейса записи AXI
#define IRQ_LAST_SEGM_AXIW              (1 << 8)        // прерывание по достижению последнего сегмента адреса ведущего интерфейса записи AXI
#define IRQ_AXIR_ERROR                  (1 << 9)        // прерывание при возникновении ошибочной транзакции по шине чтения AXI(неверное значение RRESP)
#define IRQ_LAST_ADR_AXIR               (1 << 10)       // прерывание по достижению последнего адреса ведущего интерфейса чтения AXI
#define IRQ_LAST_SEGM_AXIR              (1 << 11)       // прерывание по достижению последнего сегмента адреса ведущего интерфейса чтения
#define IRQ_UNCORR_ERROR1               (1 << 12)       // прерывание при возникновении некорректируемой ошибки
#define IRQ_STATUS_EMPTY                (1 << 13)       // состояние «читаемая страница пуста»
#define IRQ_RCM_NAND_MASK               0x1FFF
// таймауты
#define NAND_READY_TIMEOUT              10000
#define NAND_READY_IRQ_TIMEOUT          50000
// размер области ПДП
#define DMA_SIZE                        4096
// макросы для расчета таймингов
#define RCM_NAND_DIV_ROUND_UP(x, y) (1 + (((x) - 1) / (y)))
#define RCM_NAND_TIMING_TO_CLOCKS(t,f) ((RCM_NAND_DIV_ROUND_UP((t+1)*f, 1000))  & 0xff)
// режим коррекции ошибок
#define NAND_ECC_MODE_UNDEFINED         (-1)
#define NAND_ECC_MODE_NO_ECC            0
#define NAND_ECC_MODE_4BITS             1               // USER: 4 ошибки на 512 байтов (7 байтов проверочных),OOB: 3 байта проверочных на 4 байта данных
#define NAND_ECC_MODE_24BITS            2               // 24 ошибки на 1024 байта (42 байта проверочных)
// текущий режим
#define NAND_ECC_MODE                   NAND_ECC_MODE_4BITS

#if ( NAND_ECC_MODE == NAND_ECC_MODE_NO_ECC )
        #define ENABLE_ECC_OOB          0               // запрет коррекции ООВ
        #define ECC_OOB_CTRL_USED       0               // занято контроллером для коррекции
        #define WRITE_ALIGNED           0               // поволяем запись по произвольному смещению и с любой длиной
        #define RDWR_OOB_SIZE           (chip->mtd.oobsize)     // вся область
#elif ( NAND_ECC_MODE == NAND_ECC_MODE_4BITS )
        #define ENABLE_ECC_OOB          1
        #define ECC_OOB_CTRL_USED       (7*4+5*3+1)     // 7 на 512-пров.польз.данных,3 на 4-пров.ООВ,1-хвост,44 байта,что байты 16..19 у меня не удается прочитать!!!
        #define WRITE_ALIGNED           1
        #define RDWR_OOB_SIZE           (chip->mtd.oobsize-1)   // без последнего байта?
#else
        #error "Unsupportded ECC mode"
#endif

#define PRINT_REGW(r,v) NAND_DBG_PRINT_INF("REGW %08x=%08x\n",r,v);
#define PRINT_REGR(r,v) NAND_DBG_PRINT_INF("REGR %08x=%08x\n",r,v);

// LSIF0_CTRL
#define LSIF0_CTRL_BASE                 0x3C03F000
#define NAND_RADDR_EXTEND               0x24            // Расширение адреса для 36-разрядной адресации канала NAND памяти:
#define NAND_WADDR_EXTEND               0x28            // Биты [3:0] - соответствуют 4 старшим разряда адреса
#define EXT_MEM_MUX_MODE                0x30            // Выбор типа используемых контроллеров для выводов EXT_MEM

// GPIO
#define MGPIO3_GPIOAFSEL                0x3C043420      // 0xFF
#define MGPIO4_GPIOAFSEL                0x3C044420      // 0x03
#define MGPIO7_GPIOAFSEL                0x3C047420      // 0xF0
#define MGPIO8_GPIOAFSEL                0x3C048420      // 0x83

#ifdef __UBOOT__
        #define SWAP_BYTES(VAL)         cpu_to_le32(VAL)
#else
        #define SWAP_BYTES(VAL)         (VAL)
#endif

#define IOWRITE32(D,P)                  iowrite32(SWAP_BYTES(D),P)
#define IOREAD32(P)                     SWAP_BYTES(ioread32(P))

// команды ONFI FLASH (см.стандарт)
#define FLASH_CMD_PAGE_READ             0x00            // ONFI: Read
#define FLASH_CMD_BLOCK_ERASE           0x60            // ONFI: Block Erase
#define FLASH_CMD_PROGRAM_PAGE          0x80            // ONFI: Page Program
#define FLASH_CMD_READ_ID               0x90            // ONFI: Read ID
#define FLASH_CMD_NAND_ONFI             0xEC            // ONFI: Read Parameter Page

#define RCM_NAND_CTRL_ID                0x444E414E
#define DRIVER_NAME                     "rcm-nand" 

#define NAND_DBG_READ_CHECK                             // чтение до 2-х одинаковых буферов
//#define NAND_DBG_WRITE_CHECK                          // запись с проверкой
#define READ_RETRY_CNT                  5               // число попыток чтения
#define NAND_DBG_PRINT

#ifndef __UBOOT__
        #define DBG_PRINT_PROC printk
#else
        #define DBG_PRINT_PROC printf
#endif

#if defined NAND_DBG_PRINT
        #define NAND_DBG_PRINT_INF(...) DBG_PRINT_PROC("RCM_NAND[INF] " __VA_ARGS__);
        #define NAND_DBG_PRINT_WRN(...) DBG_PRINT_PROC("RCM_NAND[WRN] " __VA_ARGS__);
        #define NAND_DBG_PRINT_ERR(...) DBG_PRINT_PROC("RCM_NAND[ERR] " __VA_ARGS__);
#else
        #define NAND_DBG_PRINT_INF(...) while(0);
        #define NAND_DBG_PRINT_WRN(...) while(0);
        #define NAND_DBG_PRINT_ERR(...) while(0);
#endif

#define PRINT_BUF_16(b,n) \
    NAND_DBG_PRINT_INF( "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", \
                        b[n], b[n+1], b[n+2], b[n+3], b[n+4], b[n+5], b[n+6], b[n+7], b[n+8], b[n+9], b[n+10], b[n+11], b[n+12], b[n+13], b[n+14], b[n+15] );
#define PRINT_BUF_64(b,n) PRINT_BUF_16(b,n) PRINT_BUF_16(b,n+16) PRINT_BUF_16(b,n+32) PRINT_BUF_16(b,n+48)
#define PRINT_BUF_256(b,n) PRINT_BUF_64(b,n) PRINT_BUF_64(b,n+64) PRINT_BUF_64(b,n+128) PRINT_BUF_64(b,n+192)
#define PRINT_BUF_512(b) PRINT_BUF_256(b,0) PRINT_BUF_256(b,256)
#define PRINT_BUF(s,b,n) printk( "%s:%*ph\n", s, n, b );

#define PRINT_INT_STATUS \
        NAND_DBG_PRINT_ERR( "%u: RF=%u,RI=%u,PF=%u,ER=%u,RS=%u,E0=%u,WE=%u,WA=%u,WS=%u,RE=%u,RA=%u,RS=%u,E1=%u,EM=%u\n", \
                            chip->rd_page_cnt, \
                            (bool)(chip->status_irq & IRQ_READ_FINISH ), \
                            (bool)(chip->status_irq & IRQ_READID ), \
                            (bool)(chip->status_irq & IRQ_PROGRAM_FINISH ), \
                            (bool)(chip->status_irq & IRQ_ERASE ), \
                            (bool)(chip->status_irq & IRQ_RESET ), \
                            (bool)(chip->status_irq & IRQ_UNCORR_ERROR0 ), \
                            (bool)(chip->status_irq & IRQ_AXIW_ERROR ), \
                            (bool)(chip->status_irq & IRQ_LAST_ADR_AXIW ), \
                            (bool)(chip->status_irq & IRQ_LAST_SEGM_AXIW ), \
                            (bool)(chip->status_irq & IRQ_AXIR_ERROR ), \
                            (bool)(chip->status_irq & IRQ_LAST_ADR_AXIR ), \
                            (bool)(chip->status_irq & IRQ_LAST_SEGM_AXIR ), \
                            (bool)(chip->status_irq & IRQ_UNCORR_ERROR1 ), \
                            (bool)(chip->status_irq & IRQ_STATUS_EMPTY ) )

/* static const const uint8_t oob_test_data[64] = {
        0xD6, 0x39, 0xDF, 0xF6, 0xBA, 0xBF, 0x53, 0xA6, 0x46, 0xA7, 0x5A, 0x85, 0xD2, 0x35, 0x2B, 0xF6,
        0xC1, 0x79, 0x51, 0x38, 0x76, 0x32, 0x91, 0xE8, 0xAB, 0x8A, 0xDB, 0x22, 0x86, 0x58, 0x4F, 0x01,
        0x68, 0x5C, 0x92, 0x35, 0x80, 0x1E, 0x97, 0x89, 0xD5, 0x7D, 0x4E, 0xC0, 0x63, 0x28, 0x57, 0xB1,
        0xCD, 0x35, 0x7E, 0xC4, 0xB5, 0x8B, 0xA6, 0xAF, 0x2D, 0xF3, 0xBE, 0x1E, 0x56, 0xAF, 0x99, 0x40 }; */

enum {
    MNAND_IDLE = 0, 
    MNAND_WRITE, 
    MNAND_READ, 
    MNAND_ERASE, 
    MNAND_READID, 
    MNAND_RESET 
}; 

#ifndef __UBOOT__
        #define SEMA_INIT(CHIP) sema_init(CHIP->mutex,1);
        #define DOWN_KILLABLE(SEMA) down_killable(SEMA);
        #define DOWN(SEMA) down(SEMA);
        #define UP(SEMA) up(SEMA);
#else // __UBOOT__
        #define of_property_read_u32(DEVNODE,NAME,PVAL) ofnode_read_u32(np_to_ofnode(DEVNODE),NAME,PVAL)
        #define of_property_read_u32_array(DEVNODE,NAME,PARR,PSIZE) ofnode_read_u32_array(np_to_ofnode(DEVNODE),NAME,PARR,PSIZE)
        #define SEMA_INIT(CHIP) while(0);
        #define DOWN_KILLABLE(SEMA) (0);
        #define DOWN(SEMA) while(0);
        #define UP(SEMA) while(0);
#endif

#ifndef __UBOOT__
        #define COMPLETE_INTERRUPT
#endif

#ifdef COMPLETE_INTERRUPT
        #define INIT_COMPLETION(CHIP) init_completion(&CHIP->cmpl);
        #define WAIT_FOR_COMPLETION_IO(CHIP) wait_for_completion_io(&CHIP->cmpl);
        #define COMPLETE_ALL(CHIP) complete_all(&CHIP->cmpl);
#else
        #define INIT_COMPLETION(CHIP) while(0);
        #define WAIT_FOR_COMPLETION_IO(CHIP) rcm_nand_wait_irq(CHIP);   // нет проверки возвращаемого значения,но результат определится по изменению state
        #define COMPLETE_ALL(CHIP) while(0);
#endif

#ifndef CONFIG_SPL_BUILD

struct rcm_nand_chip {
        int init_flag;
        unsigned rd_page_cnt;
        void __iomem* io;
        int irq;
        uint32_t state; 
        dma_addr_t dma_handle; 
        size_t dma_size; 
        void* dma_area; 
        struct mtd_info mtd; 
        uint32_t status_irq;
        uint32_t status;
        uint32_t ctrl_id;
        uint32_t ctrl_ver;
        int err;
        int empty;
        uint64_t chip_size[2];
#ifndef __UBOOT__
        struct completion cmpl;
        struct semaphore mutex;
        struct platform_device* dev;
#else
        struct udevice* dev;
#endif
}; 

static void rcm_nand_set( struct rcm_nand_chip* chip, uint32_t addr, uint32_t val ) {
        IOWRITE32( val, chip->io + addr );
        //PRINT_REGW( addr, val )
}

static uint32_t rcm_nand_get( struct rcm_nand_chip* chip, uint32_t addr ) {
        uint32_t val = IOREAD32( chip->io + addr );
        //PRINT_REGR( addr, val )
        return val;
}

static int rcm_nand_is_buffer_ff( const uint8_t* buf, uint32_t len ) {
        uint32_t i;
        for( i = 0; i < len; i++ ) {
                if( buf[i] != 0xFF ) return 0;
        }
        return 1;
}

static uint32_t rcm_nand_pagesize_code( uint32_t size ) {
        switch( size )
        {
        case 512:
                return 0;
        case 1024:
                return 1;
        case 2048:
                return 2;
        case 4096:
                return 3;
        case 8192:
                return 4;
        case 16384:
                return 5;
        case 32768:
                return 6;
        case 65536:
                return 7;
        default:
                NAND_DBG_PRINT_ERR( "get_pagesize_code: error size=%u\n", size )
                return 0;
        }
}

static int rcm_nand_chip_offset( struct rcm_nand_chip* chip, loff_t* size ) {
        int cs = 0; 
        if( *size >= chip->chip_size[0] ) {
                cs++;
                *size -= chip->chip_size[0];
        }
        return cs; 
}

static void update_chip_err_empty( struct rcm_nand_chip* chip ) {
        chip->empty = (chip->status_irq & IRQ_STATUS_EMPTY ) ? 1 : 0;
        chip->err = ( (chip->status_irq & ( IRQ_AXIW_ERROR | IRQ_AXIR_ERROR ) ) ||
                      ( !chip->empty && (chip->status_irq & ( IRQ_UNCORR_ERROR0 | IRQ_UNCORR_ERROR1 ) ) ) ) ? 1 : 0;
        if( chip->err ) {
                PRINT_INT_STATUS
        }
}

#ifdef COMPLETE_INTERRUPT

static irqreturn_t rcm_nand_interrupt_handler( int irq, void* data ) {

        struct rcm_nand_chip* chip = (struct rcm_nand_chip*)data;

        if(chip->init_flag == 0 ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_interrupt_handler: init_flag=0\n" ) 
                return IRQ_NONE;
        }

       chip->status_irq = rcm_nand_get( chip, NAND_REG_irq_status );
       chip->status = rcm_nand_get( chip, NAND_REG_status );

        if( !(chip->status_irq & IRQ_RCM_NAND_MASK ) )
                return IRQ_NONE;

        switch(chip->state ) {
        case MNAND_READ:
                if(chip->status_irq & IRQ_READ_FINISH ) {
                        //NAND_DBG_PRINT_INF( "rcm_nand_interrupt_handler: IRQ_READ_FINISH\n" )
                       chip->state = MNAND_IDLE;
                       chip->rd_page_cnt++;
                }
                break;
        case MNAND_READID: 
                if(chip->status_irq & IRQ_READID ) {
                        //NAND_DBG_PRINT_INF( "rcm_nand_interrupt_handler: IRQ_READID\n" )
                       chip->state = MNAND_IDLE; 
                }
                break; 
        case MNAND_WRITE: 
                if(chip->status_irq & IRQ_PROGRAM_FINISH ) {
                        //NAND_DBG_PRINT_INF( "rcm_nand_interrupt_handler: IRQ_PROGRAM_FINISH\n" )
                       chip->state = MNAND_IDLE; 
                } 
                break; 
        case MNAND_ERASE: 
                if(chip->status_irq & IRQ_ERASE ) {
                        //NAND_DBG_PRINT_INF( "rcm_nand_interrupt_handler: IRQ_ERASE\n" )
                       chip->state = MNAND_IDLE; 
                 }
                break; 
        case MNAND_RESET:
                if(chip->status_irq & IRQ_RESET ) {
                        //NAND_DBG_PRINT_INF( "rcm_nand_interrupt_handler: IRQ_RESET\n" )
                       chip->state = MNAND_IDLE; 
                }
                break;
        default:
                NAND_DBG_PRINT_ERR( "rcm_nand_interrupt_handler: interrupt, but idle state\n" )
                return IRQ_HANDLED; // IRQ_NONE
        } 

        update_chip_err_empty( chip );
        complete_all( &chip->cmpl );
        return IRQ_HANDLED; 
}

#else // !COMPLETE_INTERRUPT

static int rcm_nand_wait_irq( struct rcm_nand_chip* chip ) {
        int ret = -1;
        uint32_t timeout = NAND_READY_IRQ_TIMEOUT;
        while( --timeout ) {
                chip->status_irq = rcm_nand_get( chip, NAND_REG_irq_status );
                chip->status = rcm_nand_get( chip, NAND_REG_status );

                if( ( chip->state == MNAND_READ ) && ( chip->status_irq & IRQ_READ_FINISH ) ) {
                        chip->rd_page_cnt++;
                        ret = 0;
                        break;
                }
                else if( ( ( chip->state == MNAND_WRITE ) && ( chip->status_irq & IRQ_PROGRAM_FINISH ) ) ||
                         ( ( chip->state == MNAND_ERASE ) && ( chip->status_irq & IRQ_ERASE ) ) ||
                         ( ( chip->state == MNAND_READID ) && ( chip->status_irq & IRQ_READID ) ) ||
                         ( ( chip->state == MNAND_RESET ) && ( chip->status_irq & IRQ_RESET ) ) ) {
                        ret = 0;
                        break;
                }
        }
        if( ret == 0 ) {
                update_chip_err_empty( chip );
                chip->state = MNAND_IDLE;
                //NAND_DBG_PRINT_INF( "rcm_nand_wait_irq: complete\n" )
        }
        else {
                NAND_DBG_PRINT_ERR( "rcm_nand_wait_irq: ret=%d,irq status=%08x,timeout=%u\n", ret, chip->status_irq, timeout )
        }
        return ret;
}

#endif // !COMPLETE_INTERRUPT

static void rcm_nand_update_control( struct rcm_nand_chip* chip,
                                     uint32_t chip_select,
                                     uint32_t page_size,
                                     uint32_t oob_size,                // команды в отношении страницы – размер OOB,остальные команды–количество читаемых байтов данных
                                     uint32_t op_begin,
                                     uint32_t num_adr_bytes, 
                                     uint32_t ecc_mode, 
                                     uint32_t hw_w_protect,            // 0-команды записи/стирания,1-чтение
                                     uint32_t oob_ecc,
                                     uint32_t command ) {
        uint32_t wr_reg = ( op_begin << CTRL_REG_OP_BEGIN_SHIFT ) |
                          ( page_size << CTRL_REG_PAGE_SIZE_SHIFT ) |
                          ( num_adr_bytes << CTRL_REG_NUM_ADR_BYTES_SHIFT ) |
                          ( ecc_mode << CTRL_REG_ECC_MODE_SHIFT ) |
                          ( chip_select << CTRL_REG_CE_SHIFT ) |
                          ( hw_w_protect << CTRL_REG_HWP_SHIFT ) |
                          ( oob_ecc << CTRL_REG_OOB_ECC_SHIFT ) |
                          ( oob_size << CTRL_REG_OOB_SIZE_SHIFT ) |
                          ( command << CTRL_REG_FCMD_SHIFT );
        mb();
        rcm_nand_set( chip, NAND_REG_control, wr_reg );
}

static int rcm_nand_wait_ready( struct rcm_nand_chip* chip, uint32_t mask ) {
        int ret = -1;
        uint32_t status, timeout = NAND_READY_TIMEOUT;

        while( --timeout ) {
                status = rcm_nand_get( chip, NAND_REG_status );
                //if( ( status & 0x0000007F ) !=  0x00000060 ) {
                //        NAND_DBG_PRINT_ERR( "rcm_nand_wait_ready: flash status=%08x\n", status )
                //}
                if( ( status & mask ) == mask ) {
                        ret = 0;
                        break;
                }
        }
        //NAND_DBG_PRINT_INF( "rcm_nand_wait_ready: ret=%d,status=%08x,timeout=%u\n", ret, status, timeout )
        return ret;
}

static void rcm_nand_set_timing( struct rcm_nand_chip* chip, uint32_t offset, const uint32_t* timings, unsigned cnt, uint32_t freq ) {
        unsigned i;
        uint32_t timing_reg_wr = 0, n = 0;
        //uint32_t timing_reg_rd;

        for( i=0; i<cnt; i++ ) {
                timing_reg_wr |= ( RCM_NAND_TIMING_TO_CLOCKS( timings[i], freq ) << n );
                n += 8;
        }
        rcm_nand_set( chip, offset, timing_reg_wr );
        //timing_reg_rd = rcm_nand_get( chip, offset );
        //NAND_DBG_PRINT_INF( "rcm_nand_set_timing: reg=%02x,wr=%08x,rd=%08x(%u,%u,%u,%u)\n",
        //                    offset, timing_reg_wr, timing_reg_rd,
        //                    (timing_reg_rd>>24)&0xff, (timing_reg_rd>>16)&0xff, (timing_reg_rd>>8)&0xff, (timing_reg_rd>>0)&0xff )
}

static void rcm_nand_set_timings( struct rcm_nand_chip* chip, uint32_t* timings, uint32_t freq ) {
        //NAND_DBG_PRINT_INF( "rcm_nand_set_timings: freq=%u\n", freq )
        rcm_nand_set_timing( chip, NAND_REG_timing_0, timings+0, 4, freq );   // tADL_min, tALH_min, tALS_min, tCH_min
        rcm_nand_set_timing( chip, NAND_REG_timing_1, timings+4, 4, freq );   // tCLH_min, tCLS_min, tCS_min, tDH_min
        rcm_nand_set_timing( chip, NAND_REG_timing_2, timings+8, 4, freq );   // tDS_min, tWC_min, tWH_min, tWP_min
        rcm_nand_set_timing( chip, NAND_REG_timing_3, timings+12, 3, freq );  // tWW_min, tAR_min, tCLR_min
        rcm_nand_set_timing( chip, NAND_REG_timing_4, timings+15, 4, freq );  // tCOH_min, tIR_min, tRC_min, tREH_min
        rcm_nand_set_timing( chip, NAND_REG_timing_5, timings+19, 4, freq );  // tRHOH_min, tRHW_min, tRLOH_min, tRP_min
        timings[25] += 2;
        rcm_nand_set_timing( chip, NAND_REG_timing_6, timings+23, 4, freq );  // tRR_min, tWHR_min, tCEA_max+2, tCHZ_max
        timings[27] += 2;
        timings[29] += 2;
        rcm_nand_set_timing( chip, NAND_REG_timing_7, timings+27, 3, freq );  // tREA_max+2, tRHZ_max, tWB_max+2
}

static int rcm_nand_hw_init( struct rcm_nand_chip* chip, uint32_t* timings, uint32_t freq ) {

        chip->ctrl_id = rcm_nand_get( chip, NAND_REG_id );
        chip->ctrl_ver = rcm_nand_get( chip, NAND_REG_version );   // 0x000008B4
        NAND_DBG_PRINT_INF( "rcm_nand_init: id=%08x,version=%08x,ecc mode=%d\n", chip->ctrl_id, chip->ctrl_ver, NAND_ECC_MODE )

        if( chip->ctrl_id != RCM_NAND_CTRL_ID )
                return -ENXIO;

        rcm_nand_set( chip, NAND_REG_cntrl_sw_rst, 0x0);              // запись 0 в данный регистр инициирует программный сброс контроллера
        rcm_nand_set( chip, NAND_REG_sw_rst, 0x0 );                   // запись 0,затем 1 в данный регистр инициирует
        rcm_nand_set( chip, NAND_REG_sw_rst, 0x1 );                   // программный сброс ведущего интерфейса записи AXI 
        rcm_nand_set( chip, NAND_REG_sw_rstn_r, 0x1 );                // программный сброс ведущего интерфейса чтения AXI, автоматически обнуляется
        if( rcm_nand_wait_ready( chip, STAT_REG_CONT_READY ) ) return -EIO;
        rcm_nand_set_timings( chip, timings, freq/1000000 );
        return 0;
}

static void rcm_nand_prepare_dma_read( struct rcm_nand_chip* chip, size_t bytes ) { // чтение ИЗ флэш
        rcm_nand_set( chip, NAND_REG_awlen_max, 0 );//0xF );          // AXI burst size рекомендуемое значение: 0xF(16*4) (таблица 1.845)
        rcm_nand_set( chip, NAND_REG_msb_lsbw, 0x1 );                 // 1-big_endian,0-little endian
        rcm_nand_set( chip, NAND_REG_start_dma_w, chip->dma_handle );
        rcm_nand_set( chip, NAND_REG_end_dma_w, chip->dma_handle+bytes-1 );
        rcm_nand_set( chip, NAND_REG_cntrl_dma_w,                     // 28..5-размер сегмента-1, 31-генерация адресов
                      START_ADDR_GEN | ( (bytes-1) << SEGM_SIZE_SHIFT ) );
}

static void rcm_nand_prepare_dma_write( struct rcm_nand_chip* chip, size_t bytes ) { // запись В флэш
        rcm_nand_set( chip, NAND_REG_axi_arlen, 0 );//0xF );          // максимальное значение ARLEN для транзакций по ведущему интерфейсу чтения AXI,рекомендуемое значение 0xF. 
        rcm_nand_set( chip, NAND_REG_msb_lsbr, 0x1 );                 // 1-big_endian,0-little endian
        rcm_nand_set( chip, NAND_REG_start_dma_r, chip->dma_handle );
        rcm_nand_set( chip, NAND_REG_end_dma_r, chip->dma_handle+bytes-1 );
        rcm_nand_set( chip, NAND_REG_cntrl_dma_r,                     // 28..5-размер сегмента-1, 31-генерация адресов
                      START_ADDR_GEN | ( (bytes-1) << SEGM_SIZE_SHIFT ) );
}

static int rcm_nand_core_reset( struct rcm_nand_chip* chip, uint32_t chip_select ) {
        if( rcm_nand_wait_ready( chip, STAT_REG_CONT_READY ) )
                return -EIO;

        INIT_COMPLETION( chip )
        chip->init_flag = 1;
        chip->state = MNAND_RESET;

        rcm_nand_set( chip, NAND_REG_irq_mask_nand, IRQ_RESET ); // IRQ_RCM_NAND_MASK
        rcm_nand_set( chip, NAND_REG_command, CMD_REG_RESET );
        rcm_nand_update_control( chip,
                                 chip_select,           // микросхема
                                 0,                     // код размера страницы
                                 0,                     // размер запасной области/количество считываемых байтов
                                 1,                     // начать операцию сразу
                                 0,                     // количество адресных циклов
                                 0,                     // режим коррекции ошибок
                                 0,                     // защита от записи
                                 0,                     // разрешение оценки кодов коррекции
                                 0 );                   // первая отправляемая во флэш команда

        WAIT_FOR_COMPLETION_IO( chip )
        rcm_nand_set( chip, NAND_REG_irq_mask_nand, 0 );
        if( chip->state != MNAND_IDLE ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_reset: uncomplete interrupt(%08x)\n", chip->status_irq )
                PRINT_INT_STATUS
                chip->state = MNAND_IDLE;
                return -EIO;
        }
        return 0; 
} 

static int rcm_nand_core_erase( struct rcm_nand_chip* chip, loff_t off ) {
        int cs;
        if( rcm_nand_wait_ready( chip, STAT_REG_CONT_READY | STAT_REG_NAND_READY ) )
                return -EIO;

        cs = rcm_nand_chip_offset( chip, &off );

        //NAND_DBG_PRINT_INF( "rcm_nand_core_erase: cs=%u,off=0x%08llX\n", cs, off )

        INIT_COMPLETION( chip )
        chip->state = MNAND_ERASE;
 
        rcm_nand_set( chip, NAND_REG_irq_mask_nand, IRQ_ERASE ); // IRQ_RCM_NAND_MASK
        rcm_nand_set( chip, NAND_REG_command, CMD_REG_ERASE_BLOCK );
        rcm_nand_set( chip, NAND_REG_col_addr, 0 );                                         // адрес столбца
        rcm_nand_set( chip, NAND_REG_row_addr_read_d, (off >> chip->mtd.writesize_shift) ); // адрес строки и адрес блока
                                                                                            // (off >> chip.mtd.erasesize_shift) << (ffs(chip.mtd.erasesize)) );
        rcm_nand_update_control( chip,
                                 cs,                            // микросхема
                                 0,                             // код размера страницы
                                 chip->mtd.oobsize,             // размер запасной области/количество считываемых байтов
                                 1,                             // начать операцию сразу
                                 5,                             // количество адресных циклов
                                 NAND_ECC_MODE,                 // режим коррекции ошибок
                                 0,                             // защита от записи
                                 ENABLE_ECC_OOB,                // разрешение оценки кодов коррекции
                                 FLASH_CMD_BLOCK_ERASE );       // первая отправляемая во флэш команда
        WAIT_FOR_COMPLETION_IO( chip )
        rcm_nand_set( chip, NAND_REG_irq_mask_nand, 0 );
        if( chip->state != MNAND_IDLE ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_erase: uncomplete interrupt(%08x)\n", chip->status_irq )
                PRINT_INT_STATUS
                chip->state = MNAND_IDLE;
        }
        return chip->err ? -EIO : 0;
} 

static void rcm_nand_core_read_id( struct rcm_nand_chip* chip, uint32_t chip_select, size_t bytes) {
         //if( rcm_nand_wait_ready( chip, STAT_REG_CONT_READY | STAT_REG_NAND_READY ) )
         //        return -EIO;

        rcm_nand_prepare_dma_read( chip, bytes );
        INIT_COMPLETION( chip )
        chip->state = MNAND_READID;

        rcm_nand_set( chip, NAND_REG_irq_mask_nand, IRQ_READID ); // IRQ_RCM_NAND_MASK
        rcm_nand_set( chip, NAND_REG_command, CMD_REG_READ_ID );
        rcm_nand_set( chip, NAND_REG_col_addr, 0x00 );          // адрес для считывания идентификационного номера

        rcm_nand_update_control( chip,
                                 chip_select,                   // микросхема
                                 0,                             // код размера страницы
                                 bytes,                         // размер запасной области/количество считываемых байтов
                                 1,                             // начать операцию сразу
                                 1,                             // количество адресных циклов
                                 0,                             // режим коррекции ошибок
                                 0,                             // защита от записи
                                 0,                             // разрешение оценки кодов коррекции
                                 FLASH_CMD_READ_ID );           // первая отправляемая во флэш команда
        WAIT_FOR_COMPLETION_IO( chip )
        rcm_nand_set( chip, NAND_REG_irq_mask_nand, 0 );
        if( chip->state != MNAND_IDLE ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_read_id: uncomplete interrupt(%08x)\n", chip->status_irq )
                PRINT_INT_STATUS
                chip->state = MNAND_IDLE;
        }
}
 
static int rcm_nand_core_read( struct rcm_nand_chip* chip, loff_t off ) { 
        int cs;
        //loff_t page;

        if( rcm_nand_wait_ready( chip, STAT_REG_CONT_READY | STAT_REG_NAND_READY ) )
                return -EIO;

        memset( chip->dma_area, 0xff, chip->mtd.writesize + chip->mtd.oobsize );
        cs = rcm_nand_chip_offset( chip, &off ); 
        //page = off & (~(chip->mtd.writesize-1)); 
        //NAND_DBG_PRINT_INF( "rcm_nand_core_read: cs=%u,off=%08llX,page=%08llX\n", cs, off, page )
 
        rcm_nand_prepare_dma_read( chip, chip->mtd.writesize + chip->mtd.oobsize );
        INIT_COMPLETION( chip )
        chip->state = MNAND_READ;
        rcm_nand_set( chip, NAND_REG_irq_mask_nand, IRQ_READ_FINISH ); // IRQ_RCM_NAND_MASK
        rcm_nand_set( chip, NAND_REG_command, CMD_REG_READ );                                   // чтение (из flash)
        rcm_nand_set( chip, NAND_REG_col_addr, 0 );                                             // адрес столбца
        rcm_nand_set( chip, NAND_REG_row_addr_read_d, (off >> chip->mtd.writesize_shift) );     // адрес строки и адрес блока

        rcm_nand_update_control( chip,
                                 cs,                                                            // микросхема
                                 rcm_nand_pagesize_code( chip->mtd.writesize ),                 // код размера страницы
                                 RDWR_OOB_SIZE,                                                 // размер запасной области
                                 1,                                                             // начать операцию сразу
                                 5,                                                             // 5 байтов (циклов) адреса в текущей команде
                                 NAND_ECC_MODE,                                                 // режим ecc
                                 1,                                                             // hw write protect enabled (почему?)
                                 ENABLE_ECC_OOB,                                                // 0-запретить проверку коррекции ошибок
                                 FLASH_CMD_PAGE_READ );                                         // команда для flash
        WAIT_FOR_COMPLETION_IO( chip )
        rcm_nand_set( chip, NAND_REG_irq_mask_nand, 0 );
        if( chip->state != MNAND_IDLE ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_read: uncomplete interrupt(%08x)\n", chip->status_irq )
                PRINT_INT_STATUS
                chip->state = MNAND_IDLE;
        }
        //PRINT_BUF_256( ((uint8_t*)chip->dma_area), 0 )
#if ( NAND_ECC_MODE == NAND_ECC_MODE_4BITS )
        if( chip->empty ) {                                                                     // читаемая страница чистая,но корректор пытается исправить ошибки
                memset( chip->dma_area, 0xff, chip->mtd.writesize + chip->mtd.oobsize );        // уберем это
                //NAND_DBG_PRINT_WRN( "rcm_nand_core_read: page is empty\n" )
        }
#endif
        return chip->err ? -EIO : 0;
} 

#ifdef NAND_DBG_READ_CHECK
static int rcm_nand_core_read_with_check( struct rcm_nand_chip* chip, loff_t from ) {
        void* buf[2] = { NULL, NULL };
        int err = -ENOMEM, nrd, nok =-1;
        uint32_t ncmp = chip->mtd.writesize + chip->mtd.oobsize;

        buf[0] = kmalloc( DMA_SIZE, GFP_KERNEL );
        buf[1] = kmalloc( DMA_SIZE, GFP_KERNEL );

        if( ( buf[0] == NULL ) || ( buf[1] == NULL ) )
                goto ret;

        for( nrd = 0; nrd < READ_RETRY_CNT; nrd++ ) {
                if( ( err = rcm_nand_core_read( chip, from ) ) == 0 ) {
                        memcpy( buf[1], chip->dma_area, ncmp );
                        break;
                }
        }
        if( err != 0 ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_read_with_check: line %u\n", __LINE__ )
                goto ret;
        }

        for( nrd = 0; nrd < READ_RETRY_CNT; nrd++ ) {
                if( ( err = rcm_nand_core_read( chip, from ) ) == 0 ) {
                        memcpy( buf[nrd&1], chip->dma_area, ncmp );
                        if( ( nok = memcmp( buf[0], buf[1], ncmp ) ) == 0 ) break;
                }
        }

        if( err != 0 ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_read_with_check: line %u\n", __LINE__ )
        }
        else if( nok ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_read_with_check: line %u\n", __LINE__ )
        }
ret:
        if( buf[0] ) kfree( buf[0] );
        if( buf[1] ) kfree( buf[1] );
        return err ? err : nok ? -EIO : 0;
}
#endif // NAND_DBG_READ_CHECK

static int rcm_nand_core_write( struct rcm_nand_chip* chip, loff_t off ) { 
        int cs;

        if( rcm_nand_wait_ready( chip, STAT_REG_CONT_READY | STAT_REG_NAND_READY ) )
                return -EIO;

#if ( NAND_ECC_MODE == NAND_ECC_MODE_4BITS )
        if( rcm_nand_is_buffer_ff( (uint8_t*)chip->dma_area, chip->mtd.writesize + chip->mtd.oobsize - ECC_OOB_CTRL_USED ) ) {
                NAND_DBG_PRINT_WRN( "rcm_nand_core_write: write page as empty\n" )              // это сделано для того,чтобы запись образа фс с чистыми страницами работал
                return 0;                                                                       // тут надо бы проверить, что страница действительно пуста
        }
#endif

        cs = rcm_nand_chip_offset( chip, &off );
        //NAND_DBG_PRINT_INF( "rcm_nand_core_write: cs=%u,off=0x%08llX,status=%08x\n", cs, off, status )
        rcm_nand_prepare_dma_write( chip, chip->mtd.writesize + chip->mtd.oobsize - ECC_OOB_CTRL_USED ); 
        INIT_COMPLETION( chip )
        chip->state = MNAND_WRITE;

        rcm_nand_set( chip, NAND_REG_irq_mask_nand, IRQ_PROGRAM_FINISH ); // IRQ_RCM_NAND_MASK
        rcm_nand_set( chip, NAND_REG_cdb_buffer_type, 1 );
        rcm_nand_set( chip, NAND_REG_command, CMD_REG_PROGRAM );                                // запись (в flash)
        rcm_nand_set( chip, NAND_REG_col_addr, 0 );                                             // адрес столбца
        rcm_nand_set( chip, NAND_REG_row_addr_read_d, (off >> chip->mtd.writesize_shift) );     // адрес строки и адрес блока
        rcm_nand_update_control( chip,
                                 cs,                                                            // микросхема
                                 rcm_nand_pagesize_code( chip->mtd.writesize ),                 // код размера страницы
                                 RDWR_OOB_SIZE,                                                 // размер запасной области
                                 1,                                                             // начать операцию сразу
                                 5,                                                             // 5 байтов (циклов) адреса в текущей команде
                                 NAND_ECC_MODE,                                                 // режим ecc (пока без коррекции)
                                 0,                                                             // 1-hw write protect enabled (почему?)
                                 ENABLE_ECC_OOB,                                                // 0-запретить проверку коррекции ошибок
                                 FLASH_CMD_PROGRAM_PAGE );                                      // команда для flash
        mb();                                 
        rcm_nand_set( chip, NAND_REG_cdb_buffer_enable, 0x1 );
        WAIT_FOR_COMPLETION_IO( chip )
        rcm_nand_set( chip, NAND_REG_irq_mask_nand, 0 );
        if( chip->state != MNAND_IDLE ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_write: uncomplete interrupt(%08x)\n", chip->status_irq )
                PRINT_INT_STATUS
                chip->state = MNAND_IDLE;
        }
        //PRINT_BUF_256( ((uint8_t*)chip->dma_area), 0 )
        return chip->err ? -EIO : 0;
}

#ifdef NAND_DBG_WRITE_CHECK
static int rcm_nand_core_write_with_check( struct rcm_nand_chip* chip, loff_t to ) {
        uint8_t* buf[2] = { NULL, NULL };
        int err = -ENOMEM, nwr, ok = 0, i;
        uint32_t ncmp = chip->mtd.writesize + chip->mtd.oobsize - ECC_OOB_CTRL_USED;

        buf[0] = kmalloc( DMA_SIZE, GFP_KERNEL );
        buf[1] = kmalloc( DMA_SIZE, GFP_KERNEL );

        if( ( buf[0] == NULL ) || ( buf[1] == NULL ) )
                goto ret;

        memcpy( buf[0], chip->dma_area, ncmp );

        for( nwr = 0; nwr < READ_RETRY_CNT; nwr++ ) {
                if( ( err = rcm_nand_core_write( chip, to ) ) != 0 ) {
                        NAND_DBG_PRINT_ERR( "rcm_nand_core_write_with_check: line %u\n", __LINE__ )
                        goto ret;
                }

                if( ( err = rcm_nand_core_read_with_check( chip, to ) ) == 0 ) {
                        memcpy( buf[1], chip->dma_area, ncmp );
                        for( i=0; i<ncmp; i++ ) {
                            if( buf[0][i] != buf[1][i] ) {
                                    NAND_DBG_PRINT_ERR( "%u: %02x:%02x\n", i, buf[0][i], buf[1][i] );
                                    break;
                             }
                        }
                        if( i == ncmp ) {
                                ok = 1;
                                goto ret;
                        }
                }
        }

        if( err != 0 ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_write_with_check: line %u\n", __LINE__ )
        }
        else if( !ok ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_write_with_check: line %u\n", __LINE__ )
        }
ret:
        if( buf[0] ) kfree( buf[0] );
        if( buf[1] ) kfree( buf[1] );
        return err ? err : ok ? 0 : -EIO;
}
#endif // NAND_DBG_WRITE_CHECK

static int rcm_nand_reset( struct rcm_nand_chip* chip ) {
        int err = rcm_nand_core_reset( chip, 0 );
        if( err == 0 )
                err = rcm_nand_core_reset( chip, 1 );
        return err;
}

static int rcm_nand_read_id( struct rcm_nand_chip* chip, int cs ) { 
        struct nand_flash_dev* type = 0; 
        int i; 
        //char* vendor = "unknown";
 
        for (i=0; i<READ_RETRY_CNT; i++) {
                rcm_nand_core_read_id( chip, cs, 256 );      // 2
                //PRINT_BUF_256( ((uint8_t*)(chip->dma_area)),0 )
                if (*(((uint8_t*)chip->dma_area)+1)!=0x00) 
                break; 
        } 
 
        if (i==READ_RETRY_CNT) { 
                NAND_DBG_PRINT_ERR("rcm_nand_read_id: bad chip id or no chip at CS=%d\n", cs); 
                return -ENODEV; 
        } 

        for (i = 0; rcm_nand_manuf_ids[i].name != NULL; i++) {     // Lookup the flash vendor
                if (((uint8_t*)chip->dma_area)[0] == rcm_nand_manuf_ids[i].id) { 
                        //vendor =  rcm_nand_manuf_ids[i].name; 
                        break; 
                } 
        }
 
        for (i = 0; rcm_nand_flash_ids[i].name != NULL; i++) {     // Lookup the flash id
                if (((uint8_t*)chip->dma_area)[1] == rcm_nand_flash_ids[i].dev_id && 
                        rcm_nand_flash_ids[i].mfr_id == 0) { 
                        type = &rcm_nand_flash_ids[i]; 
                        break; 
                } 
        } 
  
        if (!type) {
                NAND_DBG_PRINT_ERR("rcm_nand_read_id: bad type\n")
                return -ENODEV;
        }

#define NBUG(a,b) \
        { if (cs && (a!=b)) goto inconsistent; }                // чипы должны быть одинаковыми
  
        if(!type->pagesize) { 
                uint8_t extid; 
                u_long tmp;
                //rcm_nand_core_read_id( chip, cs, 5 );
                extid = ((uint8_t*)chip->dma_area)[3]; 
                //NAND_DBG_PRINT_INF( "rcm_nand_read_id: flash ext_id 0x%02X\n", extid)
                tmp = 1024 << (extid & 0x3); 
                NBUG(chip->mtd.writesize, tmp); 
                chip->mtd.writesize = tmp; 
                extid >>= 2;
                tmp = (8 << (extid & 0x01)) * (chip->mtd.writesize >> 9); 
                NBUG(chip->mtd.oobsize, tmp); 
                chip->mtd.oobsize = tmp; 
                extid >>= 2; 
                tmp = (64 * 1024) << (extid & 0x03); 
                NBUG(chip->mtd.erasesize, tmp); 
                chip->mtd.erasesize = tmp;
        } else { 
                chip->mtd.erasesize = type->erasesize; 
                chip->mtd.writesize = type->pagesize; 
                chip->mtd.oobsize = chip->mtd.writesize / 32; 
        } 

        chip->mtd.writebufsize = chip->mtd.writesize; 
        chip->mtd.size += (uint64_t) type->chipsize << 20; 
        chip->chip_size[cs] = (uint64_t) type->chipsize << 20;
        //chip->mtd.ecc_strength = type->ecc.strength_ds;
        //chip->mtd.ecc_step_size = type->ecc.step_ds;
/*
        NAND_DBG_PRINT_INF( "rcm_nand_read_id: %s,CS=%d,%s,chipsize=%u,size=%llu,writesize=%u,oobsize=%u,erasesize=%u\n", 
                            chip->mtd.name,
                            cs,
                            vendor,
                            type->chipsize,                             // размер одной микросхемы
                            chip->mtd.size,                             // полный размер памяти в байтах
                            chip->mtd.writesize,                        // размер страницы в байтах
                            chip->mtd.oobsize,                          // размер дополнительной области в байтах
                            chip->mtd.erasesize )                       // размер блока в байтах (число страниц в блоке * размер страницы)
*/
        //NAND_DBG_PRINT_INF( "rcm_nand_read_id: ecc_strength=%u,ecc_step_size=%u\n",
        //                    chip->mtd.ecc_strength,
        //                    chip->mtd.ecc_step_size )
        return 0; 
inconsistent:
        NAND_DBG_PRINT_ERR("rcm_nand_read_id: chip configurations differ,error\n")
        return -ENODEV; 
} 

static int rcm_nand_read_id2( struct rcm_nand_chip* chip ) {
        int i, err;
        for( i=0; i<READ_RETRY_CNT; i++ ) {
                err = rcm_nand_read_id( chip, 0 );
                if( err == 0 )
                        err = rcm_nand_read_id( chip, 1 );
                if( err == 0 ) break;
        }
        return err;
}

static int rcm_nand_erase( struct mtd_info* mtd, struct erase_info* instr ) { 
        int err;
        struct rcm_nand_chip* chip = (struct rcm_nand_chip*)mtd->priv;

        NAND_DBG_PRINT_INF( "rcm_nand_erase: addr=0x%08llX, len=%lld\n", instr->addr, instr->len )

        if( ( instr->addr & (chip->mtd.erasesize-1) ) ||
              instr->addr >= chip->mtd.size || 
              instr->addr + instr->len > chip->mtd.size  || 
              instr->len != chip->mtd.erasesize ) {
                        //NAND_DBG_PRINT_INF( "rcm_nand_erase: bad parameters addr=0x%08llx,len=%08llx\n", instr->addr, instr->len )
                        return -EINVAL;
        }
        DOWN( &chip->mutex )
        err = rcm_nand_core_erase( chip, instr->addr); 
        UP( &chip->mutex )

        if( err ) {
                instr->fail_addr = instr->addr; 
        }
 
        NAND_DBG_PRINT_INF( "rcm_nand_erase: err=%d,addr=0x%08llx,fail_addr=0x%08llx\n", err, instr->addr, instr->fail_addr )
        return err; 
}

static int rcm_nand_write_oob( struct mtd_info* mtd, loff_t to, struct mtd_oob_ops* ops ) {
        uint8_t* data = ops->datbuf;
        uint8_t* dataend = data ? data + ops->len : 0;
        uint8_t* oob = ops->oobbuf;
        uint8_t* oobend = oob ? oob + ops->ooblen : 0;
        int err;
        struct rcm_nand_chip* chip = (struct rcm_nand_chip*)mtd->priv;

#if !WRITE_ALIGNED
        size_t shift;
        size_t bytes;
#endif // WRITE_ALIGNED

        NAND_DBG_PRINT_INF( "rcm_nand_write_oob: to=0x%08llX,ops.mode=%d,ops.len=0x%X,ops.ooblen=%d,ops.ooboffs=0x%08X\n",
                             to, ops->mode, ops->len, ops->ooblen, ops->ooboffs )

        ops->retlen = 0;
        ops->oobretlen = 0;
#if WRITE_ALIGNED
        if( ( ( ( to & (chip->mtd.writesize-1) ) != 0 ) ) || ( ( (dataend - data)  & (chip->mtd.writesize-1) ) != 0 ) ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_write_oob: writing non page aligned data to=0x%08llx,len=0x%08x\n", to, ops->len )
                return -EINVAL;
        }
#endif // WRITE_ALIGNED
        if( to >= chip->mtd.size ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_write_oob: to>chipsize(to=0x%08llx)\n", to )
                return -EINVAL;
        }

        if( ops->ooboffs > mtd->oobsize ) { 
                NAND_DBG_PRINT_ERR( "rcm_nand_write_oob: oob>mtd->oobsize(ooboffs=0x%08X)\n", ops->ooboffs ) 
                return -EINVAL; 
        }

        err = DOWN_KILLABLE( &chip->mutex )

        if( 0 == err ) { 
                for(; (data !=dataend) || (oob !=oobend); ) {
                        memset( chip->dma_area, 0xFF, chip->mtd.writesize + chip->mtd.oobsize );
                        //err = rcm_nand_core_read_with_check( to );
                        //if( err ) break;

                        if( data != dataend ) {
                        #if WRITE_ALIGNED
                                memcpy( chip->dma_area, data, chip->mtd.writesize ); 
                                data += chip->mtd.writesize;
                        #else // !WRITE_ALIGNED                          
                                shift = (to & (mtd->writesize -1));
                                bytes = min_t( size_t, dataend-data, mtd->writesize-shift );
                                memcpy( chip->dma_area + shift, data, bytes ); 
                                data += bytes;
                        #endif
                        } 

                        if( oob != oobend ) { 
                                switch( ops->mode ) {
                                default:
                                case MTD_OPS_PLACE_OOB:                 // OOB data are placed at the given offset (default)
                                        
                                        err =  -EINVAL;
                                        break;                          // часть OOB используется корректором ошибок контроллера
                                case MTD_OPS_RAW:                       // OOB data are transferred as-is, with no error correction,this mode implies MTD_OPS_PLACE_OOB
                                case MTD_OPS_AUTO_OOB:                  // OOB data are automatically placed at the free areas which are defined by the internal ecclayout
                                        if( ( ops->ooblen > chip->mtd.oobavail ) || ( ops->ooboffs + ops->ooblen > chip->mtd.oobavail ) ) { // доступная область у нас с 0 смещения
                                                err = -EINVAL;
                                        }
                                        else {
                                                //PRINT_BUF( "rcm_nand_write_oob", oob, ops->ooblen )
                                                memcpy( chip->dma_area + chip->mtd.writesize + ops->ooboffs, oob, ops->ooblen ); 
                                                oob = oobend;
                                        }
                                        break;
                                }
                        } 
                        if( err ) {
                                NAND_DBG_PRINT_ERR( "rcm_nand_write_oob: bad parameters for write oob mode=%u len=%u offs=%u\n", ops->mode, ops->ooblen, ops->ooboffs )
                                break;
                        }

                        //memcpy( (uint8_t*)chip->dma_area+chip->mtd.writesize, oob_test_data, 64 );
#ifdef NAND_DBG_WRITE_CHECK
                        err = rcm_nand_core_write_with_check( chip, to );
#else
                        err = rcm_nand_core_write( chip, to );
#endif
                        if( err )
                                break;
                        #if WRITE_ALIGNED
                                to += chip->mtd.writesize;
                        #else // !WRITE_ALIGNED
                                to &= ~((1 << chip->mtd.writesize_shift)-1);
                                to += chip->mtd.writesize;
                        #endif
                }
                UP( &chip->mutex )
        }

        if( err ) { 
                NAND_DBG_PRINT_ERR( "rcm_nand_write_oob: returned error at 0x%08llx\n", to )
                return err; 
        } 
        ops->retlen = ops->len; 
        ops->oobretlen = ops->ooblen;

        return err; 
}

static int rcm_nand_read_oob( struct mtd_info* mtd, loff_t from, struct mtd_oob_ops* ops ) {
        size_t shift;
        size_t bytes;
        uint8_t* data = ops->datbuf;
        uint8_t* oob = ops->oobbuf;
        int err;
        uint8_t* dataend = data + ops->len;
        uint8_t* oobend = oob ? oob + ops->ooblen : 0;
        struct rcm_nand_chip* chip = (struct rcm_nand_chip*)mtd->priv;

        NAND_DBG_PRINT_INF( "rcm_nand_read_oob: from=0x%08llX ops.mode=%d ops.len=%d ops.ooblen=%d ops ooboffs=0x%08X data=%p\n",
                            from, ops->mode, ops->len, ops->ooblen, ops->ooboffs, data )
        if( ops->len != 0 && data == 0 ) { 
                ops->len = 0;
                dataend = 0;
        }    
        ops->retlen = 0;
        ops->oobretlen = 0;
        chip->rd_page_cnt = 0;

        err = DOWN_KILLABLE( &chip->mutex )

        if( 0 == err ) {
                for(;;) {
#ifdef NAND_DBG_READ_CHECK
                        err = rcm_nand_core_read_with_check( chip, from );
#else
                        err = rcm_nand_core_read( chip, from );
#endif
                        //PRINT_BUF_512( ((uint8_t*)chip->dma_area) )
                        //PRINT_BUF_64(( (uint8_t*)chip->dma_area+mtd->writesize), 0 )
                        if( err != 0 )
                                 break; 
                        if( data != dataend ) { 
                                shift = ( from & (mtd->writesize -1) );
                                bytes = min_t( size_t, dataend-data, mtd->writesize-shift );
                                //NAND_DBG_PRINT_INF( "rcm_nand_read_oob: data=%px,dataend=%px,len=0x%X,shift=0x%X,bytes=0x%X\n", data, dataend, ops->len, shift, bytes )
                                memcpy( data, chip->dma_area + shift, bytes ); 
                                data += bytes;
                         }

                         if( oob != oobend ) {
                                switch( ops->mode ) {
                                default:
                                case MTD_OPS_PLACE_OOB:                 // OOB data are placed at the given offset (default)
                                        err =  -EINVAL;
                                        break;                          // часть OOB используется корректором ошибок контроллера
                                case MTD_OPS_AUTO_OOB:                  // OOB data are automatically placed at the free areas which are defined by the internal ecclayout
                                case MTD_OPS_RAW:                       // OOB data are transferred as-is, with no error correction,this mode implies MTD_OPS_PLACE_OOB
                                        if( ( ops->ooblen > chip->mtd.oobavail ) || ( ops->ooboffs + ops->ooblen > chip->mtd.oobavail ) ) { // доступная область у нас с 0 смещения
                                                err = -EINVAL;
                                        }
                                        else {
                                                //PRINT_BUF( "rcm_nand_read_oob", oob, ops->ooblen )
                                                memcpy( oob, chip->dma_area + chip->mtd.writesize + ops->ooboffs, ops->ooblen ); 
                                                oob = oobend;
                                        }
                                        break;
                                }
                        }
                        if( err ) {
                                NAND_DBG_PRINT_ERR( "rcm_nand_read_oob: bad parameters for read oob mode=%u len=%u off=%u\n", ops->mode, ops->ooblen, ops->ooboffs )
                                break;
                        }

                        if( ( data == dataend ) &&  ( oob == oobend ) )
                                break;

                        from &= ~((1 << mtd->writesize_shift)-1);
                        from += mtd->writesize;
                }
                UP( &chip->mutex )
        }

        if( err ) { // -EUCLEAN
                NAND_DBG_PRINT_ERR( "rcm_nand_read_oob: returned error at 0x%08llx\n", from )
                return err; 
        } 
        ops->retlen = ops->len; 
        ops->oobretlen = ops->ooblen;
 
        return err; 
}

static int rcm_nand_write(struct mtd_info* mtd, loff_t off, size_t len, size_t* retlen, const u_char* buf) { 
        struct mtd_oob_ops ops = { 
                .datbuf  = (uint8_t*)buf, 
                .len = len
        };
        int err = -EINVAL;
        if( buf ) {
                err = rcm_nand_write_oob( mtd, off, &ops ); 
                *retlen = ops.retlen;
        }
         else {
                *retlen = 0;
        }
        //NAND_DBG_PRINT_INF( "rcm_nand_write: off=0x%08llX,len=%zu,err=%d,retlen=%zu\n", off, len, err, *retlen )
        return err; 
}

static int rcm_nand_read( struct mtd_info* mtd, loff_t off, size_t len, size_t* retlen, u_char* buf ) { 
        struct mtd_oob_ops ops = {
                .datbuf = buf,
                .len = len,
        };
        int err = -EINVAL; 
        if( buf ) {
                 err = rcm_nand_read_oob( mtd, off, &ops );
                *retlen = ops.retlen;
        }
        else {
                *retlen = 0;
        }
        //NAND_DBG_PRINT_INF( "rcm_nand_read: off=0x%08llX,len=%zu,err=%d,retlen=%zu\n", off, len, err, *retlen )
        return err;
}

static int rcm_nand_lsif0_config( const struct device_node *of_node ) {
        void* lsif0_io;
#ifndef __UBOOT__
        struct device_node* tmp_of_node;
        struct resource res;
        if( ( tmp_of_node = of_parse_phandle( of_node, "config", 0 ) ) == NULL ) {
                NAND_DBG_PRINT_ERR( "of_parse_phandle(config): error\n" )
                return -ENOENT;
        }
        if( of_address_to_resource( tmp_of_node, 0, &res ) != 0 ) {
		NAND_DBG_PRINT_ERR( "of_address_to_resource(config): error\n" )
                return -ENODEV;
        }
	if( ( lsif0_io = ioremap(res.start, resource_size(&res) ) ) == NULL ) {
                NAND_DBG_PRINT_ERR( "ioremap(config): error\n" )
		return -ENOMEM;
        }
#else
        uint32_t config_reg[2];
        int ret;
        if( ( ret = of_property_read_u32_array( of_node, "config-reg", config_reg, 2 ) ) != 0 ) {
                NAND_DBG_PRINT_ERR( "of_property_read_u32_array: config-reg read error\n" )
                return ret;
        }
        lsif0_io = (void*)config_reg[0];
        IOWRITE32( 0x00000002, lsif0_io + EXT_MEM_MUX_MODE );
#endif
        IOWRITE32( 0x00000001, lsif0_io + NAND_RADDR_EXTEND );
        IOWRITE32( 0x00000001, lsif0_io + NAND_WADDR_EXTEND );
#ifndef __UBOOT__
        iounmap( lsif0_io );
#endif
        return 0;
}

static int rcm_nand_get_timings( const struct device_node *of_node, uint32_t* timings, uint32_t* freq ) {
        struct device_node* tmp_of_node;
        int ret;
#ifndef __UBOOT__
        if( ( tmp_of_node = of_parse_phandle( of_node, "clocks", 0 ) ) == NULL ) {
                NAND_DBG_PRINT_ERR( "of_parse_phandle(clocks): error\n" )
                return -ENOENT;
        }
#else
        tmp_of_node = (struct device_node*)of_node;
#endif // 
        if( ( ret = of_property_read_u32( tmp_of_node, "clock-frequency", freq ) ) != 0 ) {
                NAND_DBG_PRINT_ERR( "of_property_read_u32(%s): clock-frequency read error\n", tmp_of_node->name )
                return ret;
        }
        if( ( ret = of_property_read_u32_array( of_node, "timings", timings, 30 ) ) != 0 ) {
                NAND_DBG_PRINT_ERR( "of_property_read_u32_array: timings read error\n" )
                return ret;
        }
        return 0;
}

#ifndef __UBOOT__
static struct of_device_id of_platform_nand_table[];
typedef struct platform_device rcm_nand_device;
#else
typedef struct udevice rcm_nand_device;
#endif

static int rcm_nand_probe( rcm_nand_device* ofdev ) {
        struct device_node *of_node;
        uint32_t freq, timings[30];
        int err;
        struct rcm_nand_chip* chip;

#ifndef __UBOOT__
        struct resource res;
        const char *part_probes[] = { "ofpart"/*"cmdlinepart"*/, NULL, };
        const struct of_device_id *match;

        of_node = ofdev->dev.of_node;

        chip = (struct rcm_nand_chip*)kmalloc(sizeof( struct rcm_nand_chip ), GFP_KERNEL);
        if( !chip )
                return -ENOMEM;
        memset( chip, 0, sizeof( struct rcm_nand_chip ) );
        platform_set_drvdata( ofdev, chip );

        match = of_match_device( of_platform_nand_table, &ofdev->dev );
        if( !match ) {
                dev_err( &ofdev->dev, "of_match_device: failed\n" );
                return -EINVAL;
        }
#else // __UBOOT__
        uint32_t reg[2];

        of_node = (struct device_node*)ofnode_to_np( ofdev->node );

        chip = (struct rcm_nand_chip*)calloc( 1, sizeof( struct rcm_nand_chip ) );
        if( !chip )
                return -ENOMEM;
        ofdev->priv = chip;
#endif // __UBOOT__
        chip->dev = ofdev;

#ifndef __UBOOT__
        if( of_address_to_resource( of_node, 0, &res ) != 0 ) {
		dev_err( &ofdev->dev, "of_address_to_resource: failed\n" );
                return -ENODEV;
        }
	if( ( chip->io = ioremap( res.start, resource_size(&res) ) ) == NULL ) {
                dev_err( &ofdev->dev, "ioremap: error\n" );
		return -ENOMEM;
        }
#else
        if( ( err = of_property_read_u32_array( of_node, "reg", reg, 2 ) ) != 0 ) {
                NAND_DBG_PRINT_ERR( "of_property_read_u32_array: reg read error\n" )
                return err;
        }
        chip->io = (void*)reg[0];
#endif
        if( ( err = rcm_nand_lsif0_config( of_node ) ) != 0 ) {
                dev_err( &ofdev->dev, "lsif0 config: failed\n" );
                goto error_with_unmap;
        }

#ifdef COMPLETE_INTERRUPT
        chip->irq = irq_of_parse_and_map( of_node, 0 );
        //NAND_DBG_PRINT_INF( "rcm_nand_probe: irq_of_parse_and_map: return %u\n", chip->irq )
        if( ( err = request_irq( chip->irq, rcm_nand_interrupt_handler, IRQF_SHARED, DRIVER_NAME, chip ) ) ) {
                dev_err( &ofdev->dev, "failed to set handler on irq %d,err=%d\n", chip->irq, err );
                goto error_with_unmap;
        }
#endif // COMPLETE_INTERRUPT

#ifndef __UBOOT__
        chip->dev->dev.coherent_dma_mask = DMA_BIT_MASK( 32 );
        chip->dma_area = dma_alloc_coherent( &chip->dev->dev, DMA_SIZE, &chip->dma_handle, GFP_KERNEL | GFP_DMA );
#else
        chip->dma_handle = (dmaaddr_t)memalign( 512/*ARCH_DMA_MINALIGN*/, DMA_SIZE );
        chip->dma_area = (void*)(uint32_t)(chip->dma_handle);  //???types?
#endif

        if( !chip->dma_area ) {
                dev_err( &ofdev->dev, "failed to request DMA area\n" );
                err = -ENOMEM;
                goto error_with_free_irq; 
        }
        memset( chip->dma_area, 0xFF, DMA_SIZE );

        INIT_COMPLETION( chip )
        SEMA_INIT( &chip ); 
        DOWN( &chip->mutex )

        if( ( err = rcm_nand_get_timings( of_node, timings, &freq ) ) != 0 ) {
                dev_err( &ofdev->dev, "rcm_nand_get_timings failed\n" );
                goto error_with_free_mem;
        }

        if( ( err = rcm_nand_hw_init( chip, timings, freq ) ) != 0 ) {
                dev_err( &ofdev->dev, "rcm_nand_hw_init failed,err=%d\n", err );
                goto error_with_free_mem;
        }

        dev_info( &ofdev->dev, "found NAND controller id=%08x,version=%08x\n", chip->ctrl_id, chip->ctrl_ver );

        if( ( err = rcm_nand_reset( chip ) ) != 0 ) {
                dev_err( &ofdev->dev, "rcm_nand_reset failed,err=%d\n", err );
                goto error_with_free_mem;
        }

        chip->mtd.name = DRIVER_NAME;
        chip->mtd.size = 0;
        chip->chip_size[0] = 0;
        chip->chip_size[1] = 0;

        if( ( err = rcm_nand_read_id2( chip ) ) != 0 ) {
                dev_err( &ofdev->dev, "rcm_nand_read_id2 failed,err=%d\n", err );
                goto error_with_free_mem;
        }

        UP( &chip->mutex )

        chip->mtd.owner = THIS_MODULE;
        chip->mtd.type = MTD_NANDFLASH;
        chip->mtd.flags = MTD_WRITEABLE;
        chip->mtd._erase = rcm_nand_erase;
        chip->mtd._read = rcm_nand_read;
        chip->mtd._write = rcm_nand_write;
        chip->mtd._read_oob = rcm_nand_read_oob;
        chip->mtd._write_oob = rcm_nand_write_oob;
        //chip->mtd.dev.parent = &ofdev->dev;???
         chip->mtd.oobavail = chip->mtd.oobsize - ECC_OOB_CTRL_USED;

        if( is_power_of_2( chip->mtd.writesize ) ) chip->mtd.writesize_shift = ffs(chip->mtd.writesize)-1;
        if( is_power_of_2( chip->mtd.erasesize ) ) chip->mtd.erasesize_shift = ffs(chip->mtd.erasesize)-1;
        chip->mtd.writesize_mask = (1<<chip->mtd.writesize_shift)-1;
        chip->mtd.erasesize_mask = (1<<chip->mtd.erasesize_shift)-1;

#if( NAND_ECC_MODE == NAND_ECC_MODE_4BITS )
        chip->mtd.oobavail -= 4;      // используем байты 0..15,16..19 читаются как ff
#endif
        chip->mtd.priv = chip;
        dev_info( &ofdev->dev, "detected %llu bytes NAND memory,page size %u bytes,user oob size %u bytes\n", chip->mtd.size, chip->mtd.writesize, chip->mtd.oobavail );

#ifndef __UBOOT__
        chip->mtd.dev = ofdev->dev;     // device
        err = mtd_device_parse_register( &chip->mtd, part_probes, 0, NULL, 0 );
#else
        chip->mtd.dev = ofdev;          // udevice*
        err = nand_register( 0, &chip->mtd );
#endif
        if( err ) {
                dev_err( &ofdev->dev, "failed add mtd device,err: %d\n", err );
                goto error_with_free_mem;
        }
        return 0;

error_with_free_mem:
#ifndef __UBOOT__
        dma_free_coherent( &chip->dev->dev, DMA_SIZE, chip->dma_area, chip->dma_handle );
#else
        free( chip->dma_area );
#endif
error_with_free_irq:
#ifdef COMPLETE_INTERRUPT
        free_irq( chip->irq, chip );
#endif
        chip->irq = 0;
error_with_unmap:
        iounmap( chip->io );
        kfree( chip );
        return err;
}

#ifndef __UBOOT__

static int rcm_nand_remove( struct platform_device* ofdev ) { 
        struct rcm_nand_chip* chip = platform_get_drvdata( ofdev );
        //NAND_DBG_PRINT_INF( "rcm_nand_remove chip=%px\n", chip );
        mtd_device_unregister( &chip->mtd );
        dma_free_coherent( &chip->dev->dev, DMA_SIZE, chip->dma_area, chip->dma_handle );
#ifdef COMPLETE_INTERRUPT
        free_irq( chip->irq, chip );
#endif
        iounmap( chip->io );
        kfree( chip );
        return 0; 
}

static struct of_device_id of_platform_nand_table[] = {
        { .compatible = "rcm,nand" }, 
        { /* end of list */ }
}; 

static struct platform_driver of_platform_rcm_nand_driver = {
        .driver = { 
                .name = DRIVER_NAME, 
                .owner = THIS_MODULE, 
                .of_match_table = of_platform_nand_table,
        }, 
        .probe = rcm_nand_probe,
        .remove = rcm_nand_remove 
}; 

module_platform_driver(of_platform_rcm_nand_driver); 

#else // __UBOOT__

static const struct udevice_id rcm_nand_ids[] = {
        { .compatible = "rcm,nand" },
        { /* end of list */ }
};

U_BOOT_DRIVER(rcm_nand) = {
        .name = DRIVER_NAME,
        .id = UCLASS_MTD,
        .of_match = rcm_nand_ids,
        .probe = rcm_nand_probe
};

void board_nand_init( void ) {
        //NAND_DBG_PRINT_INF( "%s\n", __FUNCTION__ );
        struct udevice *dev;
        int ret;

        ret = uclass_get_device_by_driver( UCLASS_MTD,
                                           DM_GET_DRIVER(rcm_nand),
                                           &dev );
        if( ret && ret != -ENODEV ) {
                NAND_DBG_PRINT_ERR( "Failed to initialize RCM NAND controller,error=%d)\n", ret );
        }
}

#endif // __UBOOT__

MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("Alexey Spirkov <alexeis@astrosoft.ru>");
MODULE_DESCRIPTION("RCM SoC NAND controller driver");

#else // CONFIG_SPL_BUILD

#define WRLSIF0(D,R) iowrite32(SWAP_BYTES(D),(void*)(LSIF0_CTRL_BASE+R))
#define WRNAND(D,R) iowrite32(SWAP_BYTES(D),(void*)(NAND_CTRL_BASE+R))
#define RDNAND(R) SWAP_BYTES(ioread32(((void*)(NAND_CTRL_BASE+R))))
#define CTRL( CHIP_SELECT, PAGE_SIZE, OOB_SIZE, OP_BEGIN, NUM_ADR_BYTES, ECC_MODE, HW_PROTECT, OOB_ECC, COMMAND )       \
                ( ( OP_BEGIN << CTRL_REG_OP_BEGIN_SHIFT ) |                                                             \
                ( PAGE_SIZE << CTRL_REG_PAGE_SIZE_SHIFT ) |                                                             \
                ( NUM_ADR_BYTES << CTRL_REG_NUM_ADR_BYTES_SHIFT ) |                                                     \
                ( ECC_MODE << CTRL_REG_ECC_MODE_SHIFT ) |                                                               \
                ( CHIP_SELECT << CTRL_REG_CE_SHIFT ) |                                                                  \
                ( HW_PROTECT << CTRL_REG_HWP_SHIFT ) |                                                                  \
                ( OOB_ECC << CTRL_REG_OOB_ECC_SHIFT ) |                                                                 \
                ( OOB_SIZE << CTRL_REG_OOB_SIZE_SHIFT ) |                                                               \
                ( COMMAND << CTRL_REG_FCMD_SHIFT ) )
#define AFSEL_INIT                                                                                                      \
        IOWRITE32( 0x000000FF, (void*)MGPIO3_GPIOAFSEL );                                                               \
        IOWRITE32( 0x00000003, (void*)MGPIO4_GPIOAFSEL );                                                               \
        IOWRITE32( 0x000000F0, (void*)MGPIO7_GPIOAFSEL );                                                               \
        IOWRITE32( 0x00000083, (void*)MGPIO8_GPIOAFSEL );

#define CHIPSIZE        (chip->size)                    // 0x8000000
#define OOBSIZE         (chip->oob_size)                // 64
#define PAGESIZE        (chip->write_size)              // 0x800
#define RDDMACNT        (PAGESIZE+OOBSIZE)
#define PAGEMASK        (chip-writesize-1)              // 0x7ff
#define PAGESIZESHIFT   (ffs(chip->write_size)-1)       // 11
#if ( NAND_ECC_MODE == NAND_ECC_MODE_NO_ECC )
        #define OOBSIZE_CTRL OOBSIZE
#elif ( NAND_ECC_MODE == NAND_ECC_MODE_4BITS )
        #define OOBSIZE_CTRL (OOBSIZE-1)
#else
        #error "Unsupportded ECC mode"
#endif

struct rcm_spl_nand_chip {
        uint8_t man_id;
        uint8_t dev_id;
        uint64_t size;
        uint32_t write_size;
        uint32_t oob_size;
        uint32_t erase_size;
};

struct rcm_spl_nand_chips {
        uint64_t full_size;
        struct rcm_spl_nand_chip chip[2];
};

void nand_init( void ) {
        uint32_t n, m;
        printf( "%s: start\n", __FUNCTION__ );

        WRLSIF0( 2, EXT_MEM_MUX_MODE );
        WRLSIF0( 1, NAND_RADDR_EXTEND );
        WRLSIF0( 1, NAND_WADDR_EXTEND );
        if( ( n = RDNAND( NAND_REG_id ) ) != RCM_NAND_CTRL_ID ) {
                printf( "%s: bad_id(%08x)\n", __FUNCTION__, n );
                return;
        }
        AFSEL_INIT

        WRNAND( 0, NAND_REG_cntrl_sw_rst );
        WRNAND( 0, NAND_REG_sw_rst );
        WRNAND( 1, NAND_REG_sw_rst );
        WRNAND( 1, NAND_REG_sw_rstn_r );

        for( n = 0; n < NAND_READY_TIMEOUT; n++ ) {
                if( RDNAND( NAND_REG_status ) & STAT_REG_CONT_READY ) break;
        }
        if( n == NAND_READY_TIMEOUT ) {
                printf( "%s: not ready)\n", __FUNCTION__ );
                return;
        }

        WRNAND( 0x03060315, NAND_REG_timing_0 );
        WRNAND( 0x03080603, NAND_REG_timing_1 );
        WRNAND( 0x06040b05, NAND_REG_timing_2 );
        WRNAND( 0x0003030b, NAND_REG_timing_3 );
        WRNAND( 0x040b0201, NAND_REG_timing_4 );
        WRNAND( 0x06011501, NAND_REG_timing_5 );
        WRNAND( 0x0b0b0d05, NAND_REG_timing_6 );
        WRNAND( 0x00151505, NAND_REG_timing_7 );

        for( m = 0; m < 2; m++ ) {
                WRNAND( IRQ_RESET, NAND_REG_irq_mask_nand );
                WRNAND( CMD_REG_RESET, NAND_REG_command );
                WRNAND( CTRL( m, 0, 0, 1, 0, 0, 0, 0, 0 ), NAND_REG_control );
                for( n = 0; n < NAND_READY_IRQ_TIMEOUT; n++ ) {
                        if( RDNAND( NAND_REG_irq_status ) & IRQ_RESET ) break;
                }
                if( n == NAND_READY_IRQ_TIMEOUT ) {
                        printf( "%s(%08x): reset failed\n", __FUNCTION__,  RDNAND( NAND_REG_status ) );
                        return;
                }
        }
        printf( "%s: succesfull(%08x)\n", __FUNCTION__, RDNAND( NAND_REG_status ) );
}

static int nand_spl_wait_irq( uint32_t mask ) {
        int n;
        uint32_t st;
        for( n = 0; n < NAND_READY_IRQ_TIMEOUT; n++ ) {
                st = RDNAND( NAND_REG_irq_status );
                if( st & mask )
                {
                        if( st & ( IRQ_AXIW_ERROR | IRQ_AXIR_ERROR ) ) {
                                printf( "%s: axi error\n", __FUNCTION__ );
                                return -1;
                        }
                        if( ( st & ( IRQ_UNCORR_ERROR0 | IRQ_UNCORR_ERROR1 ) ) && !( st & IRQ_STATUS_EMPTY ) ) {
                                printf( "%s: uncorrect error\n", __FUNCTION__ );
                                return -1;
                        }
                        return 0;
                }
        }
        printf( "%s: read failed\n", __FUNCTION__ );
        return -1;
}

static void nand_spl_init_dma( uint8_t* dst, uint32_t size ) {
        memset( dst, 0xFF, size );
        WRNAND( 0, NAND_REG_awlen_max );
        WRNAND( 1, NAND_REG_msb_lsbw );
        WRNAND( (uint32_t)dst, NAND_REG_start_dma_w );
        WRNAND( (uint32_t)dst+size-1, NAND_REG_end_dma_w );
        WRNAND( START_ADDR_GEN | ( (size-1) << SEGM_SIZE_SHIFT ), NAND_REG_cntrl_dma_w );
}

static int nand_spl_read_param( uint32_t cs, struct rcm_spl_nand_chip* chip, uint8_t* dst ) {
        struct nand_flash_dev* type = NULL; 
        int i;
        nand_spl_init_dma( dst, 256 );
        WRNAND( IRQ_READID, NAND_REG_irq_mask_nand );
        WRNAND( CMD_REG_READ_ID, NAND_REG_command );
        WRNAND( 0, NAND_REG_col_addr ); 
        WRNAND( 0, NAND_REG_row_addr_read_d );
        WRNAND( CTRL( cs, 0, 256, 1, 1, 0, 0, 0, FLASH_CMD_READ_ID ), NAND_REG_control );
        if( nand_spl_wait_irq( IRQ_READID ) == 0 ) {
                chip->man_id = dst[0];
                chip->dev_id = dst[1];
                chip->write_size = 1024 << (dst[3] & 0x3);
                chip->oob_size =  (8 << (dst[3] & 0x01)) * (chip->write_size >> 9); 
                chip->erase_size =  (64 * 1024) << (dst[3] & 0x03);

                for (i = 0; rcm_nand_flash_ids[i].name != NULL; i++) {
                        if( ((uint8_t*)dst)[1] == rcm_nand_flash_ids[i].dev_id && rcm_nand_flash_ids[i].mfr_id == 0 ) { 
                                type = &rcm_nand_flash_ids[i]; 
                                break;
                        } 
                }
                if( type == NULL ) {
                        printf( "%s: chips type not found\n", __FUNCTION__ );
                        return -1;
                }
                chip->size = (uint64_t)type->chipsize << 20;
                return 0;
        }
        return -1;
}

static int nand_spl_read_page( uint32_t offs, uint32_t size, void* dst, const struct rcm_spl_nand_chip* chip ) {
        uint32_t cs = ( offs >= CHIPSIZE ) ? 1 : 0;
        nand_spl_init_dma( dst, size );
        WRNAND( IRQ_READ_FINISH, NAND_REG_irq_mask_nand );
        WRNAND( CMD_REG_READ, NAND_REG_command );
        WRNAND( 0, NAND_REG_col_addr ); 
        WRNAND( (offs >> PAGESIZESHIFT), NAND_REG_row_addr_read_d );
        WRNAND( CTRL( cs, 2, OOBSIZE_CTRL, 1, 5, NAND_ECC_MODE, 1, ENABLE_ECC_OOB, FLASH_CMD_PAGE_READ ), NAND_REG_control );
        return nand_spl_wait_irq( IRQ_READ_FINISH );
}

static int nand_spl_read_page_with_check( uint32_t offs, uint32_t size, void* dst, const struct rcm_spl_nand_chip* chip ) {
        int i;
        void* p[2];
        
        p[0] = dst, p[1] = dst+RDDMACNT;
        for( i=0; i<READ_RETRY_CNT; i++ ) {
                if( !nand_spl_read_page( offs, RDDMACNT, p[0], chip ) ) break;

        }
        if( i == READ_RETRY_CNT )
                return -1;

        for( i=1; i<=READ_RETRY_CNT; i++ ) {
                if( !nand_spl_read_page( offs, RDDMACNT, p[i&1], chip ) && !memcmp( p[0], p[1], RDDMACNT ) )
                        return 0;
        }
        return -1;
}

static int nand_spl_init_chips_param( struct rcm_spl_nand_chips* chips, void* dma_buf ) {
        int i;

        for( i=0; i<READ_RETRY_CNT; i++ ) {
                if( !nand_spl_read_param( 0, &chips->chip[0], dma_buf ) &&
                    !nand_spl_read_param( 1, &chips->chip[1], dma_buf ) ) break;
        }

        if( i == READ_RETRY_CNT ) {
                printf( "%s: parameters read error\n", __FUNCTION__ );
                return -1;
        }

        if( memcmp( &chips->chip[0], &chips->chip[1], sizeof( struct rcm_spl_nand_chip ) ) ) {
                printf( "%s: chis is different\n", __FUNCTION__ );
                return -1;
        }
        chips->full_size = chips->chip[0].size + chips->chip[1].size;
        printf( "%s: fs=%llu,dev_id=%02x,sz=%llu,ws=%u,os=%u,es=%u\n", __FUNCTION__,
                chips->full_size, 
                chips->chip[0].dev_id,
                chips->chip[0].size,
                chips->chip[0].write_size,
                chips->chip[0].oob_size,
                chips->chip[0].erase_size );
        return 0;
}

//#define NAND_LOAD_OFF_SPL
//#define NAND_TEST_SPL

int nand_spl_load_image( uint32_t offs, unsigned int size, void *dst ) {
        int err;
        struct rcm_spl_nand_chips chips = { 0 };
        const struct rcm_spl_nand_chip* chip = &chips.chip[0];
        printf( "%s: start offs=%08x,size=%08x,dst=%08x\n", __FUNCTION__, offs, size, (uint32_t)dst );

        if( ( err = nand_spl_init_chips_param( &chips, dst ) ) != 0 )
                return err;

#ifdef NAND_TEST_SPL
        offs = 0;
#endif
        while(1) {
#ifdef NAND_DBG_READ_CHECK
                err = nand_spl_read_page_with_check( offs, RDDMACNT, dst, &chips.chip[0] );
#else
                err = nand_spl_read_page( offs, RDDMACNT, dst, &chips.chip[0] );
#endif
                if( err ) {
                        printf( "%s: error offs=%08x\n", __FUNCTION__, offs );
                        return err;
                }
#ifndef NAND_TEST_SPL
                if( size <= PAGESIZE ) {
                        printf( "%s: succesfull\n", __FUNCTION__ );
                        #ifdef NAND_LOAD_OFF_SPL
                                return -1;
                        #else
                                return 0;
                        #endif
                }
                size -= PAGESIZE;
                offs += PAGESIZE;
                dst += PAGESIZE;
#else
                offs += PAGESIZE, offs %= chips.full_size;
                mb();
                if( !( offs & 0x1ffff ) ) printf( "%s: ok offs=%08x\n", __FUNCTION__, offs );
#endif
        }
}

void nand_deselect( void ) {
        printf( "%s\n", __FUNCTION__ );
}

#endif // CONFIG_SPL_BUILD
