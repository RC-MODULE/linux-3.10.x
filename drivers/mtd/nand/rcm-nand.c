#include <linux/types.h>
#include <linux/module.h> 
#include <linux/gfp.h>
#include <linux/init.h> 
#include <linux/platform_device.h> 
#include <linux/io.h> 
#include <linux/interrupt.h> 
#include <linux/regmap.h>
#include <linux/mtd/mtd.h> 
#include <linux/mtd/nand.h> 
#include <linux/mtd/partitions.h>
#include <linux/mtd/rawnand.h>
#include <linux/mfd/syscon.h>
#include <linux/mtd/physmap.h>
#include <linux/semaphore.h> 
#include <linux/completion.h> 
#include <linux/dma-mapping.h> 
#include <linux/delay.h> 
#include <linux/of_address.h> 
#include <linux/of_irq.h> 
#include <linux/of_platform.h> 
#include <linux/delay.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/unistd.h>
#include <linux/crc32.h>
#include "rcm-nandids.h"

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

// LSIF0_CTRL
#define NAND_RADDR_EXTEND               0x24            // Расширение адреса для 36-разрядной адресации канала NAND памяти:
#define NAND_WADDR_EXTEND               0x28            // Биты [3:0] - соответствуют 4 старшим разряда адреса
// команды ONFI FLASH (см.стандарт)
#define FLASH_CMD_PAGE_READ             0x00            // ONFI: Read
#define FLASH_CMD_BLOCK_ERASE           0x60            // ONFI: Block Erase
#define FLASH_CMD_PROGRAM_PAGE          0x80            // ONFI: Page Program
#define FLASH_CMD_READ_ID               0x90            // ONFI: Read ID
#define FLASH_CMD_NAND_ONFI             0xEC            // ONFI: Read Parameter Page

#define DRIVER_NAME "rcm-nand" 

#define NAND_DBG_READ_CHECK                             // чтение до 2-х одинаковых буферов
#define READ_RETRY_CNT                  5               // число попыток чтения
#define NAND_DBG_PRINTK

#if defined NAND_DBG_PRINTK
    #define NAND_DBG_PRINT_INF(...) printk("RCM_NAND[INF] " __VA_ARGS__);
    #define NAND_DBG_PRINT_WRN(...) printk("RCM_NAND[WRN] " __VA_ARGS__);
    #define NAND_DBG_PRINT_ERR(...) printk("RCM_NAND[ERR] " __VA_ARGS__);
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

const const uint8_t oob_test_data[64] = {
        0xD6, 0x39, 0xDF, 0xF6, 0xBA, 0xBF, 0x53, 0xA6, 0x46, 0xA7, 0x5A, 0x85, 0xD2, 0x35, 0x2B, 0xF6,
        0xC1, 0x79, 0x51, 0x38, 0x76, 0x32, 0x91, 0xE8, 0xAB, 0x8A, 0xDB, 0x22, 0x86, 0x58, 0x4F, 0x01,
        0x68, 0x5C, 0x92, 0x35, 0x80, 0x1E, 0x97, 0x89, 0xD5, 0x7D, 0x4E, 0xC0, 0x63, 0x28, 0x57, 0xB1,
        0xCD, 0x35, 0x7E, 0xC4, 0xB5, 0x8B, 0xA6, 0xAF, 0x2D, 0xF3, 0xBE, 0x1E, 0x56, 0xAF, 0x99, 0x40 };

enum {
    MNAND_IDLE = 0, 
    MNAND_WRITE, 
    MNAND_READ, 
    MNAND_ERASE, 
    MNAND_READID, 
    MNAND_RESET 
}; 

struct rcm_nand_chip {
        struct platform_device* dev;
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
        int err;
        int empty;
        uint64_t chip_size[2];
        struct completion cmpl;
        struct semaphore mutex;
}; 

static void rcm_nand_set( struct rcm_nand_chip* chip, uint32_t addr, uint32_t val ) {
        iowrite32( val, chip->io + addr ); 
}

static uint32_t rcm_nand_get( struct rcm_nand_chip* chip, uint32_t addr ) { 
        return ioread32( chip->io + addr ); 
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

int rcm_nand_chip_offset( struct rcm_nand_chip* chip, loff_t* size ) {
        int cs = 0; 
        if( *size >= chip->chip_size[0] ) {
                cs++;
                *size -= chip->chip_size[0];
        }
        return cs; 
}

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

        chip->empty = (chip->status_irq & IRQ_STATUS_EMPTY ) ? 1 : 0;
        chip->err = ( (chip->status_irq & ( IRQ_AXIW_ERROR | IRQ_AXIR_ERROR ) ) ||
                      ( !chip->empty && (chip->status_irq & ( IRQ_UNCORR_ERROR0 | IRQ_UNCORR_ERROR1 ) ) ) ) ? 1 : 0;

        if( chip->err ) {
                PRINT_INT_STATUS
        }
        complete_all( &chip->cmpl );
        return IRQ_HANDLED; 
} 

void rcm_nand_update_control( struct rcm_nand_chip* chip,
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
        //union {
        //        struct { uint32_t fl_cmd : 8, oob_size : 11, oob_en : 1, wr_prot : 1, res : 1, ce_num : 1, ecc_mode : 2, addr_bytes : 3, page_size : 3, op_begin : 1; };
        //        uint32_t dw;
        //} rd_reg;
        rcm_nand_set( chip, NAND_REG_control, wr_reg );
        //rd_reg.dw = rcm_nand_get( chip, NAND_REG_control );
        // NAND_DBG_PRINT_INF( "ctrl_rd: OB=%u,PS=%u,AB=%u,EM=%u,CE=%u,RS=%u,WP=%u,OE=%u,OS=%u,FC=%u\n",
        //                     rd_reg.op_begin, rd_reg.page_size, rd_reg.addr_bytes, rd_reg.ecc_mode, rd_reg.ce_num,
        //                    rd_reg.res, rd_reg.wr_prot, rd_reg.oob_en, rd_reg.oob_size, rd_reg.fl_cmd );
}

static int rcm_nand_wait_ready( struct rcm_nand_chip* chip ) {
        int ret = -1;
        uint32_t status, timeout = NAND_READY_TIMEOUT;

        while( --timeout ) {
                status = rcm_nand_get( chip, NAND_REG_status );
                //if( ( status & 0x0000007F ) !=  0x00000060 ) {
                //        NAND_DBG_PRINT_ERR( "rcm_nand_wait_ready: flash status=%08x\n", status )
                //}
                if( status & ( STAT_REG_CONT_READY | STAT_REG_NAND_READY ) ) {
                        ret = 0;
                        break;
                }
        }
        //NAND_DBG_PRINT_INF( "rcm_nand_wait_ready: ret=%d,status=%08x,timeout=%u\n", ret, status, timeout )
        return ret;
}

static void rcm_nand_set_timing( struct rcm_nand_chip* chip, uint32_t offset, const uint32_t* timings, unsigned cnt, uint32_t freq ) {
        unsigned i;
        uint32_t timing_reg_wr = 0, timing_reg_rd, n = 0;

        for( i=0; i<cnt; i++ ) {
                timing_reg_wr |= ( RCM_NAND_TIMING_TO_CLOCKS( timings[i], freq ) << n );
                n += 8;
        }
        rcm_nand_set( chip, offset, timing_reg_wr );
        timing_reg_rd = rcm_nand_get( chip, offset );
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
        uint32_t ctrl_id = rcm_nand_get( chip, NAND_REG_id ),         // 0x444E414E
                 ctrl_ver = rcm_nand_get( chip, NAND_REG_version );   // 0x000008B4
        NAND_DBG_PRINT_INF( "rcm_nand_init: id=%08x,version=%08x,ecc mode=%d\n", ctrl_id, ctrl_ver, NAND_ECC_MODE )
        rcm_nand_set( chip, NAND_REG_cntrl_sw_rst, 0x0);              // запись 0 в данный регистр инициирует программный сброс контроллера
        rcm_nand_set( chip, NAND_REG_sw_rst, 0x0 );                   // запись 0,затем 1 в данный регистр инициирует
        rcm_nand_set( chip, NAND_REG_sw_rst, 0x1 );                   // программный сброс ведущего интерфейса записи AXI 
        rcm_nand_set( chip, NAND_REG_sw_rstn_r, 0x1 );                // программный сброс ведущего интерфейса чтения AXI, автоматически обнуляется
        if( rcm_nand_wait_ready( chip ) ) return -EIO;
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
        if( rcm_nand_wait_ready( chip ) )
                return -EIO;
    
        init_completion( &chip->cmpl ); 
        chip->init_flag = 1;
        chip->state = MNAND_RESET;

        rcm_nand_set( chip, NAND_REG_irq_mask_nand, IRQ_RESET ); // IRQ_RCM_NAND_MASK
        rcm_nand_set( chip, NAND_REG_command, CMD_REG_RESET );
        rcm_nand_update_control( chip,
                                 chip_select,           // микросхема
                                 0,                     // код размера страницы
                                 256,                   // размер запасной области/количество считываемых байтов
                                 1,                     // начать операцию сразу
                                 0,                     // количество адресных циклов
                                 0,                     // режим коррекции ошибок
                                 0,                     // защита от записи
                                 0,                     // разрешение оценки кодов коррекции
                                 0 );                   // первая отправляемая во флэш команда

        wait_for_completion_io( &chip->cmpl );
        rcm_nand_set( chip, NAND_REG_irq_mask_nand, 0 );
        if( chip->state != MNAND_IDLE ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_reset: uncomplete interrupt(%08x)\n", chip->status_irq )
                PRINT_INT_STATUS
                chip->state = MNAND_IDLE;
        }
        return 0; 
} 

static int rcm_nand_core_erase( struct rcm_nand_chip* chip, loff_t off ) {
        int cs;
        if( rcm_nand_wait_ready( chip ) )
                return -EIO;

        cs = rcm_nand_chip_offset( chip, &off );

        NAND_DBG_PRINT_INF( "rcm_nand_core_erase: cs=%u,off=0x%08llX\n", cs, off )

        init_completion(&chip->cmpl);
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
        wait_for_completion_io( &chip->cmpl );
        rcm_nand_set( chip, NAND_REG_irq_mask_nand, 0 );
        if( chip->state != MNAND_IDLE ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_erase: uncomplete interrupt(%08x)\n", chip->status_irq )
                PRINT_INT_STATUS
                chip->state = MNAND_IDLE;
        }
        return /*( chip->status & STAT_REG_NAND_FAIL )*/ chip->err ? -EIO : 0;
} 

static void rcm_nand_core_read_id( struct rcm_nand_chip* chip, uint32_t chip_select, size_t bytes) {
         //if( rcm_nand_wait_ready( chip ) )
                //return -EIO;
    
        rcm_nand_prepare_dma_read( chip, bytes );
        init_completion( &chip->cmpl ); 
        chip->state = MNAND_READID;

        rcm_nand_set( chip, NAND_REG_irq_mask_nand, IRQ_READID ); // IRQ_RCM_NAND_MASK
        rcm_nand_set( chip, NAND_REG_command, CMD_REG_READ_ID );
        rcm_nand_set( chip, NAND_REG_col_addr, 0x00 );       // адрес для считывания идентификационного номера

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

        wait_for_completion_io( &chip->cmpl );
        rcm_nand_set( chip, NAND_REG_irq_mask_nand, 0 );
        if( chip->state != MNAND_IDLE ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_read_id: uncomplete interrupt(%08x)\n", chip->status_irq )
                PRINT_INT_STATUS
                chip->state = MNAND_IDLE;
        }
}
 
static int rcm_nand_core_read( struct rcm_nand_chip* chip, loff_t off ) { 
        int cs;
        loff_t page;

        if( rcm_nand_wait_ready( chip ) )
                return -EIO;

        memset( chip->dma_area, 0xff, chip->mtd.writesize + chip->mtd.oobsize );
        cs = rcm_nand_chip_offset( chip, &off ); 
        page = off & (~(chip->mtd.writesize-1)); 
        //NAND_DBG_PRINT_INF( "rcm_nand_core_read: cs=%u,off=%08llX,page=%08llX\n", cs, off, page )
 
        rcm_nand_prepare_dma_read( chip, chip->mtd.writesize + chip->mtd.oobsize );
        init_completion( &chip->cmpl ); 
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
        wait_for_completion_io( &chip->cmpl );
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
        }
#endif
        return /*( chip->status & STAT_REG_NAND_FAIL )*/ chip->err ? -EIO : 0;
} 

static int rcm_nand_core_read_with_check( struct rcm_nand_chip* chip, loff_t from ) {
        void* buf[2] = { NULL, NULL };
        int err = -ENOMEM, nrd, nok =-1;

        buf[0] = kmalloc( DMA_SIZE, GFP_KERNEL  );
        buf[1] = kmalloc( DMA_SIZE, GFP_KERNEL  );

        if( ( buf[0] == NULL ) || ( buf[1] == NULL ) )
                goto ret;

        for( nrd = 0; nrd < READ_RETRY_CNT; nrd++ ) {
                if( ( err = rcm_nand_core_read( chip, from ) ) == 0 ) {
                        memcpy( buf[1], chip->dma_area, chip->mtd.writesize + chip->mtd.oobsize );
                        break;
                }
        }
        if( err != 0 )
                goto ret;

        for( nrd = 0; nrd < READ_RETRY_CNT; nrd++ ) {
                if( ( err = rcm_nand_core_read( chip, from ) ) == 0 ) {
                        memcpy( buf[nrd&1], chip->dma_area, chip->mtd.writesize + chip->mtd.oobsize );
                        if( ( nok = memcmp( buf[0], buf[1], chip->mtd.writesize + chip->mtd.oobsize ) ) == 0 ) break;
                }
        }
ret:
        if( buf[0] ) kfree( buf[0] );
        if( buf[1] ) kfree( buf[1] );
        return err ? err : nok ? -EIO : 0;
}

static int rcm_nand_core_write( struct rcm_nand_chip* chip, loff_t off ) { 
        int cs;
       
        if( rcm_nand_wait_ready( chip ) )
                return -EIO;

        cs = rcm_nand_chip_offset( chip, &off );
        //NAND_DBG_PRINT_INF( "rcm_nand_core_write: cs=%u,off=0x%08llX,status=%08x\n", cs, off, status )
        rcm_nand_prepare_dma_write( chip, chip->mtd.writesize + chip->mtd.oobsize - ECC_OOB_CTRL_USED ); 
        init_completion( &chip->cmpl ); 
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
        rcm_nand_set( chip, NAND_REG_cdb_buffer_enable, 0x1 );
        wait_for_completion_io( &chip->cmpl );
        rcm_nand_set( chip, NAND_REG_irq_mask_nand, 0 );
        if( chip->state != MNAND_IDLE ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_core_write: uncomplete interrupt(%08x)\n", chip->status_irq )
                PRINT_INT_STATUS
                chip->state = MNAND_IDLE;
        }
        //PRINT_BUF_256( ((uint8_t*)chip->dma_area), 0 )
        return /*( chip->status & STAT_REG_NAND_FAIL )*/ chip->err ? -EIO : 0;
} 
 
static int rcm_nand_read_id( struct rcm_nand_chip* chip, int cs ) { 
        struct nand_flash_dev* type = 0; 
        int i; 
        int retries = 5; 
        char* vendor = "unknown";
 
        for (i=0; i<retries; i++) {                             // hw weirdness: sometimes after an unlucky reset or incomplete powerdown a bad id may be read. 
                rcm_nand_core_read_id( chip, cs, 256 );      // 2
                //PRINT_BUF_256( ((uint8_t*)(chip->dma_area)),0 )
                if (*(((uint8_t*)chip->dma_area)+1)!=0x00) 
                break; 
        } 
 
        if (i==retries) { 
                NAND_DBG_PRINT_ERR("rcm_nand_read_id: bad chip id or no chip at CS=%d\n", cs); 
                return -ENODEV; 
        } 

        for (i = 0; nand_manuf_ids[i].name != NULL; i++) {     // Lookup the flash vendor
                if (((uint8_t*)chip->dma_area)[0] == nand_manuf_ids[i].id) { 
                        vendor =  nand_manuf_ids[i].name; 
                        break; 
                } 
        }
 
        for (i = 0; nand_flash_ids[i].name != NULL; i++) {     // Lookup the flash id
                if (((uint8_t*)chip->dma_area)[1] == nand_flash_ids[i].dev_id && 
                        nand_flash_ids[i].mfr_id == 0) { 
                        type = &nand_flash_ids[i]; 
                        break; 
                } 
        } 
  
        if (!type)
                return -ENODEV;

#define NBUG(a,b) \
        { if (cs && (a!=b)) goto inconsistent; } // We're not likely work if erase/write/oob sizes on different CS differ
  
        if(!type->pagesize) { 
                uint8_t extid; 
                u_long tmp;
                rcm_nand_core_read_id( chip, cs, 5 );
                extid = ((uint8_t*)chip->dma_area)[3]; 
                NAND_DBG_PRINT_INF( "rcm_nand_read_id: flash ext_id 0x%02X\n", extid)
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

        NAND_DBG_PRINT_INF( "rcm_nand_read_id: %s,CS=%d,%s,chipsize=%u,size=%llu,writesize=%u,oobsize=%u,erasesize=%u\n", 
                            chip->mtd.name,
                            cs,
                            vendor,
                            type->chipsize,                             // размер одной микросхемы
                            chip->mtd.size,                             // полный размер памяти в байтах
                            chip->mtd.writesize,                        // размер страницы в байтах
                            chip->mtd.oobsize,                          // размер дополнительной области в байтах
                            chip->mtd.erasesize )                       // размер блока в байтах (число страниц в блоке * размер страницы)

        //NAND_DBG_PRINT_INF( "rcm_nand_read_id: ecc_strength=%u,ecc_step_size=%u\n",
        //                    chip->mtd.ecc_strength,
        //                    chip->mtd.ecc_step_size )
        return 0; 
inconsistent: 
        NAND_DBG_PRINT_INF("rcm_nand_read_id: chip configurations differ, ignoring CS1\n"); 
        return -EIO; 
} 

static int rcm_nand_erase( struct mtd_info* mtd, struct erase_info* instr ) { 
        int err;
        struct rcm_nand_chip* chip = (struct rcm_nand_chip*)mtd->priv;

        NAND_DBG_PRINT_INF( "rcm_nand_erase: addr=0x%08llX, len=%lld\n", instr->addr, instr->len )

        if( ( instr->addr & (chip->mtd.erasesize-1) ) ||
              instr->addr >= chip->mtd.size || 
              instr->addr + instr->len > chip->mtd.size  || 
              instr->len != chip->mtd.erasesize ) {
                        NAND_DBG_PRINT_INF( "rcm_nand_erase: bad parameters addr=0x%08llx,len=%08llx\n", instr->addr, instr->len )
                        return -EINVAL;
        }
        down( &chip->mutex ); 
        err = rcm_nand_core_erase( chip, instr->addr); 
        up( &chip->mutex ); 

        if( err ) {
                instr->fail_addr = instr->addr; 
        }
 
        NAND_DBG_PRINT_INF( "rcm_nand_erase: err=%d,addr=0x%08llx,fail_addr=0x%08llx\n", err, instr->addr, instr->fail_addr )
        return err; 
}

static int rcm_nand_read( struct mtd_info* mtd, loff_t off, size_t len, size_t* retlen, u_char* buf );

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

        NAND_DBG_PRINT_INF( "rcm_nand_write_oob: to=0x%08llX,ops.mode=%d,ops.len=0x%X,ops.ooblen=%d,ops.ooboffs=0x%08X,crc=%08x\n",
                             to, ops->mode, ops->len, ops->ooblen, ops->ooboffs, crc32_be( ~0, data, ops->len ) )

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

        err = down_killable( &chip->mutex ); 

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
                                case MTD_OPS_RAW:                       // OOB data are transferred as-is, with no error correction,this mode implies MTD_OPS_PLACE_OOB
                                        NAND_DBG_PRINT_ERR( "rcm_nand_write_oob: bad parameters for write oob mode=%u\n", ops->mode )
                                        err =  -EINVAL;
                                        break;                          // часть OOB используется корректором ошибок контроллера
                                case MTD_OPS_AUTO_OOB:                  // OOB data are automatically placed at the free areas which are defined by the internal ecclayout
                                        if( ( ops->ooblen > chip->mtd.oobavail ) || ( ops->ooboffs + ops->ooblen > chip->mtd.oobavail ) ) { // доступная область у нас с 0 смещения
                                                NAND_DBG_PRINT_ERR( "rcm_nand_write_oob: bad parameters for write oob len=%u off=%u avl=%u\n",
                                                                    ops->ooblen, ops->ooboffs, chip->mtd.oobavail )
                                                err = -EINVAL;
                                        }
                                        else {
                                                PRINT_BUF( "rcm_nand_write_oob", oob, ops->ooblen )
                                                memcpy( chip->dma_area + chip->mtd.writesize + ops->ooboffs, oob, ops->ooblen ); 
                                                oob = oobend;
                                        }
                                        break;
                                }
                        } 
                        if( err )
                                break;

                        //memcpy( (uint8_t*)chip->dma_area+chip->mtd.writesize, oob_test_data, 64 );

                        err = rcm_nand_core_write( chip, to );
                        if( err )
                                break;
                        #if WRITE_ALIGNED
                                to += chip->mtd.writesize;
                        #else // !WRITE_ALIGNED
                                to &= ~((1 << chip->mtd.writesize_shift)-1);
                                to += chip->mtd.writesize;
                        #endif
                }
                up( &chip->mutex );
        }

        if( err ) { 
                NAND_DBG_PRINT_ERR( "rcm_nand_write_oob: core write returned error at 0x%08llx\n", to ) 
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

        //NAND_DBG_PRINT_INF( "rcm_nand_read_oob: from=0x%08llX ops.mode=%d ops.len=%d ops.ooblen=%d ops ooboffs=0x%08X data=%p\n",
        //                    from, ops->mode, ops->len, ops->ooblen, ops->ooboffs, data );
        if( ops->len != 0 && data == 0 ) { 
                ops->len = 0;
                dataend = 0;
        }      
        ops->retlen = 0;
        ops->oobretlen = 0;
        chip->rd_page_cnt = 0;

        err = down_killable( &chip->mutex );

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
                                case MTD_OPS_RAW:                       // OOB data are transferred as-is, with no error correction,this mode implies MTD_OPS_PLACE_OOB
                                        NAND_DBG_PRINT_ERR( "rcm_nand_read_oob: bad parameters for write oob mode=%u\n", ops->mode )
                                        err =  -EINVAL;
                                        break;                          // часть OOB используется корректором ошибок контроллера
                                case MTD_OPS_AUTO_OOB:                  // OOB data are automatically placed at the free areas which are defined by the internal ecclayout
                                        if( ( ops->ooblen > chip->mtd.oobavail ) || ( ops->ooboffs + ops->ooblen > chip->mtd.oobavail ) ) { // доступная область у нас с 0 смещения
                                                NAND_DBG_PRINT_ERR( "rcm_nand_read_oob: bad parameters for write oob len=%u off=%u avl=%u\n", ops->ooblen, ops->ooboffs, chip->mtd.oobavail )
                                                err = -EINVAL;
                                        }
                                        else {
                                                PRINT_BUF( "rcm_nand_read_oob", oob, ops->ooblen )
                                                memcpy( oob, chip->dma_area + chip->mtd.writesize + ops->ooboffs, ops->ooblen ); 
                                                oob = oobend;
                                        }
                                        break;
                                }
                        }
                        if( err )
                                break;

                        if( ( data == dataend ) &&  ( oob == oobend ) )
                                break;

                        from &= ~((1 << mtd->writesize_shift)-1);
                        from += mtd->writesize;
                }
                up( &chip->mutex );
        }

        if( err==0 ) { // -EUCLEAN 
                ops->retlen = ops->len; 
                ops->oobretlen = ops->ooblen; 
        } 
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
        NAND_DBG_PRINT_INF( "rcm_nand_write: off=0x%08llX,len=%zu,err=%d,retlen=%zu,crc=%08x\n", off, len, err, *retlen, crc32_be( ~0, buf, len ) )
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
        NAND_DBG_PRINT_INF( "rcm_nand_read: off=0x%08llX,len=%zu,err=%d,retlen=%zu,crc=%08x\n", off, len, err, *retlen, crc32_be( ~0, buf, len ) )
        //PRINT_BUF_16( buf, 0 )
        return err;
}

static int rcm_nand_lsif0_config( struct device_node *of_node ) {
        struct device_node* tmp_of_node;
   	struct regmap* lsif0_config;
        int ret;

        if( ( tmp_of_node = of_parse_phandle( of_node, "config", 0 ) ) == NULL ) {
                NAND_DBG_PRINT_ERR( "of_parse_phandle(config): error\n" )
                return -ENOENT;
        }
        lsif0_config = syscon_node_to_regmap( tmp_of_node );

        if( ( ret = regmap_write( lsif0_config, NAND_RADDR_EXTEND, 1 ) ) != 0 ) {
                NAND_DBG_PRINT_ERR( "regmap_write(NAND_RADDR_EXTEND): error %d\n", ret )
                return ret;
        }

        if( ( ret = regmap_write( lsif0_config, NAND_WADDR_EXTEND, 1 ) ) != 0 ) {
                NAND_DBG_PRINT_ERR( "regmap_write(NAND_WADDR_EXTEND): error %d\n", ret )
                return ret;
        }
        return 0;
}

static int rcm_nand_get_timings( struct device_node *of_node, uint32_t* timings, uint32_t* freq ) {
        struct device_node* tmp_of_node;
        int ret;

        if( ( tmp_of_node = of_parse_phandle( of_node, "clocks", 0 ) ) == NULL ) {
                NAND_DBG_PRINT_ERR( "of_parse_phandle(clocks): error\n" )
                return -ENOENT;
        }
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

static struct of_device_id of_platform_nand_table[];

static int rcm_nand_probe( struct platform_device* ofdev ) {
        const struct of_device_id *match;
        struct device_node *of_node = ofdev->dev.of_node;
        const char *part_probes[] = { "cmdlinepart", NULL, };
        uint32_t freq, timings[30];
        int err;
        struct rcm_nand_chip* chip;

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

        chip->dev = ofdev;
        chip->io = of_iomap( of_node, 0 );
        if ( WARN_ON( !chip->io ) ) {
                dev_err( &ofdev->dev, "of_iomap: failed\n" );
                return -EIO;
        }

        if( ( err = rcm_nand_lsif0_config( of_node ) ) != 0 ) {
                dev_err( &ofdev->dev, "lsif0 config: failed\n" );
                goto error_with_unmap;
        }

        chip->irq = irq_of_parse_and_map( of_node, 0 );
        //NAND_DBG_PRINT_INF( "rcm_nand_probe: irq_of_parse_and_map: return %u\n", chip->irq )
        if( ( err = request_irq( chip->irq, rcm_nand_interrupt_handler, IRQF_SHARED, DRIVER_NAME, chip ) ) ) {
                dev_err( &ofdev->dev, "failed to set handler on irq %d,err=%d\n", chip->irq, err );
                goto error_with_unmap;
        }

        chip->dev->dev.coherent_dma_mask = DMA_BIT_MASK( 32 );
        chip->dma_area = dma_alloc_coherent( &chip->dev->dev, DMA_SIZE, &chip->dma_handle, GFP_KERNEL | GFP_DMA );
        if( !chip->dma_area ) {
                dev_err( &ofdev->dev, "failed to request DMA area\n" );
                err = -ENOMEM;
                goto error_with_free_irq; 
        }
        memset( chip->dma_area, 0xFF, DMA_SIZE );

        init_completion( &chip->cmpl );
        sema_init( &chip->mutex, 1 ); 
        down( &chip->mutex );

        if( ( err = rcm_nand_get_timings( of_node, timings, &freq ) ) != 0 ) {
                dev_err( &ofdev->dev, "rcm_nand_get_timings failed\n" );
                goto error_with_free_dma;
        }

        if( ( err = rcm_nand_hw_init( chip, timings, freq ) ) != 0 ) {
                dev_err( &ofdev->dev, "rcm_nand_hw_init failed\n" );
                goto error_with_free_dma;
        }

        rcm_nand_core_reset( chip, 0 );
        rcm_nand_core_reset( chip, 1 );

        chip->mtd.name = DRIVER_NAME;
        chip->mtd.size = 0;
        chip->chip_size[0] = 0;
        chip->chip_size[1] = 0;

        err = rcm_nand_read_id( chip, 0 );
        err = rcm_nand_read_id( chip, 1 );

        up( &chip->mutex );

        chip->mtd.owner = THIS_MODULE;
        chip->mtd.type = MTD_NANDFLASH;
        chip->mtd.flags = MTD_WRITEABLE;
        chip->mtd._erase = rcm_nand_erase;
        chip->mtd._read = rcm_nand_read;
        chip->mtd._write = rcm_nand_write;
        chip->mtd._read_oob = rcm_nand_read_oob;
        chip->mtd._write_oob = rcm_nand_write_oob;
        chip->mtd.dev.parent = &ofdev->dev;
        chip->mtd.oobavail = chip->mtd.oobsize - ECC_OOB_CTRL_USED;

#if( NAND_ECC_MODE == NAND_ECC_MODE_4BITS )
        chip->mtd.oobavail -= 4;      // байты 16..19 прочитать не удалось
#endif
        chip->mtd.priv = chip;
        dev_info( &ofdev->dev, "detected %llu bytes NAND memory,user oob size %u bytes\n", chip->mtd.size, chip->mtd.oobavail );

        err = mtd_device_parse_register( &chip->mtd, part_probes, 0, NULL, 0 );
        if( err ) {
                dev_err( &ofdev->dev, "failed add mtd device,err: %d\n", err );
                goto error_with_free_dma;
        }
        return 0;

error_with_free_dma: 
        dma_free_coherent( &chip->dev->dev, DMA_SIZE, chip->dma_area, chip->dma_handle );
error_with_free_irq: 
        free_irq( chip->irq, chip );
        chip->irq = 0;
error_with_unmap:
        iounmap( chip->io );
        return err;
}

static int rcm_nand_remove( struct platform_device* ofdev ) { 
        struct rcm_nand_chip* chip = platform_get_drvdata( ofdev );
        NAND_DBG_PRINT_INF( "rcm_nand_remove chip=%px\n", chip );
        mtd_device_unregister( &chip->mtd ); 
        dma_free_coherent( &chip->dev->dev, DMA_SIZE, chip->dma_area, chip->dma_handle );
        free_irq( chip->irq, chip );
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

MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("Shalyt Vladimir Vladimir.Shalyt@astrosoft.ru"); 
MODULE_DESCRIPTION("RCM SoC NAND controller driver"); 
