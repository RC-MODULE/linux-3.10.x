#include <linux/types.h>
#include <linux/module.h> 
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

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
//#include <linux/mtd/mnand.h>
#include <linux/unistd.h>
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
#define STATUS_REG_NAND_FAIL            0x00000100      // возникновение некорректируемой ошибки
#define STATUS_REG_CONT_READY           0x00000200      // контроллер NAND готов к приему команды
#define STATUS_REG_NAND_READY           0x00000400      // флэш-память готова к выполнению операций
// NAND_REG_control
#define OP_BEGIN_SHIFT                  0               // 0-флаг начала операции (1-начать)
#define PAGE_SIZE_SHIFT                 1               // 3..1-размер страницы (0-512..7-65536)
#define NUM_ADR_BYTES_SHIFT             4               // 6..4-количество байтов (циклов) адреса в текущей команде
#define ECC_MODE_SHIFT                  7               // 8..7-режим коррекции ошибок
#define CE_SHIFT                        9               // 9-номер кристалла для текущей операции
#define	HWP_SHIFT                       11              // 11-аппаратная защита от записи(0-запись разрешена?)
#define OOB_ECC_SHIFT                   12              // 12-разрешение оценки кодов коррекции ошибок для данных в поле OOB(рекомендуется 1)
#define OOB_SIZE_SHIFT                  13              // 23..13-команда
#define	FCMD_SHIFT                      24              // 31..24-первая команда для отправки во флэш память
// NAND_REG_command (переименовать нормально надо)
#define CMD_CONTROL_READ                0x01            // чтение NAND
#define CMD_CONTROL_READ_ID             0x03            // чтение идентификатора
#define CMD_PROGRAM_TO_NAND	        0x04            // программирование NAND
#define CMD_ERASE_BLOCK		        0x06            // стирание блока
#define CMD_CONTROL_RESET               0x07            // сброс флэш памяти NAND
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
//#define IRQ_AXI_WRITE                  (1 << 6)  // ошибка? nand_impl.c
#define IRQ_AXIW_ERROR                  (1 << 6)        // прерывание при возникновении ошибочной транзакции по интерфейсу записи AXI (неверное значение BRESP)
//???????????????????
#define IRQ_LAST_ADR_AXIW               (1 << 7)        // прерывание по достижению последнего адреса ведущего интерфейса записи AXI
#define IRQ_LAST_SEGM_AXIW              (1 << 8)        // прерывание по достижению последнего сегмента адреса ведущего интерфейса записи AXI
#define IRQ_AXIR_ERROR                  (1 << 9)        // прерывание при возникновении ошибочной транзакции по шине чтения AXI(неверное значение RRESP)
#define IRQ_LAST_ADR_AXIR               (1 << 10)       // прерывание по достижению последнего адреса ведущего интерфейса чтения AXI
#define IRQ_LAST_SEGM_AXIR              (1 << 11)       // прерывание по достижению последнего сегмента адреса ведущего интерфейса чтения
#define IRQ_UNCORR_ERROR1               (1 << 12)       // прерывание при возникновении некорректируемой ошибки
#define IRQ_STATUS_EMPTY                (1 << 13)       // состояние «читаемая страница пуста»
#define IRQ_RCM_NAND_MASK               0x1FFF
// разное
#define NAND_DEFAULT_CHIP               0
// таймауты
#define NAND_READY_TIMEOUT              10000
// выравнивание на размер страницы
#define NAND_PAGE_ALIGNED(x) (((x) & (g_chip.mtd.writesize-1)) ==0) 
// макросы для расчета таймингов
#define RCM_NAND_DIV_ROUND_UP(x, y) (1 + (((x) - 1) / (y)))
#define RCM_NAND_TIMING_TO_CLOCKS(t,f) ((RCM_NAND_DIV_ROUND_UP((t+1)*f, 1000))  & 0xff)
// режим коррекции ошибок
#define NAND_ECC_MODE_UNDEFINED         (-1)
#define NAND_ECC_MODE_NO_ECC            0
#define NAND_ECC_MODE_4BITS             1
#define NAND_ECC_MODE_24BITS            2
// LSIF0_CTRL
#define NAND_RADDR_EXTEND 0x24 // Расширение адреса для 36-разрядной адресации канала NAND памяти:
#define NAND_WADDR_EXTEND 0x28 // Биты [3:0] - соответствуют 4 старшим разряда адреса
// команды ONFI FLASH (см.стандарт)
#define FLASH_CMD_PAGE_READ             0x00        // ONFI: Read
#define FLASH_CMD_BLOCK_ERASE           0x60        // ONFI: Block Erase
#define FLASH_CMD_PROGRAM_PAGE          0x80        // ONFI: Page Program
#define FLASH_CMD_READ_ID               0x90        // ONFI: Read ID
#define FLASH_CMD_NAND_ONFI             0xEC        // ONFI: Read Parameter Page

#define DRIVER_NAME "rcm-nand" 

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

#define MNAND_DECLARE_PARAM(name, def) \
    static int g_mnand_##name = def; \
    module_param_named(name, g_mnand_##name, int, 0);

    MNAND_DECLARE_PARAM(debug, 0);

#define TRACE(level, format, ... ) \
    do { \
        if(g_mnand_debug) { \
        printk(level DRIVER_NAME "/%s:%d: " format, __func__, __LINE__, ##__VA_ARGS__); \
        } \
    } while(0); 

    struct mem_resource { 
        struct resource* res; 
        void __iomem* io; 
    }; 

//FixMe: Dirty Hack 
static volatile int init_flag=0; 

enum {
    MNAND_IDLE = 0, 
    MNAND_WRITE, 
    MNAND_READ, 
    MNAND_ERASE, 
    MNAND_READID, 
    MNAND_RESET 
}; 

struct mnand_chip {
    struct platform_device* dev;
    struct mem_resource regs;
    int irq;

    uint32_t state; 

    dma_addr_t dma_handle; 
    size_t dma_size; 
    void* dma_area; 

    uint64_t active_page; 
    struct mtd_info mtd; 

    uint32_t status1; 
    uint32_t status2; 

    uint32_t ecc_corrected; 
    uint32_t ecc_failed; 
    uint64_t chip_size[2]; 
} static g_chip; 


struct write_info { 
    uint32_t address; 
    uint32_t value; 
    }; 

struct write_info g_write_buffer[1024]; 
int g_write_pos = 0; 

struct page_log { 
    uint32_t address; 
    uint8_t data[2048+64]; 
};

struct page_log g_write_page_log[100]; 
int g_write_page_pos = 0; 

struct page_log g_read_page_log[100]; 
int g_read_page_pos = 0; 

#define MNAND_STATUS_ECC_ERROR 0x100 
#define MNAND_ECC_BLOCKSIZE 256 
#define MNAND_ECC_BYTESPERBLOCK 3 

struct nand_ecclayout {
    __u32 eccbytes;
    __u32 eccpos[32];
    struct nand_oobfree oobfree[1];                     // 8?
    __u32 oobavail;
};

static struct nand_ecclayout g_ecclayout = {            // struct mtd_info' has no member named 'ecclayout'
    .eccbytes = 24,                                     // используем глобальную переменную
    .eccpos = { 
        1, 2, 3, 4, 5, 6, 7, 8, 
        9, 10, 11, 12, 13, 14, 15, 16, 
        17, 18, 19, 20, 21, 22, 23, 24 
    }, 
    .oobfree = { 
        { 
        .offset = 25, 
        .length = 39 
        } 
    }, 
    .oobavail = 39 
};

static DEFINE_SEMAPHORE(g_mutex); 
static DECLARE_COMPLETION(g_completion); 

void rcm_nand_set(uint32_t addr, uint32_t val) 
{
    struct write_info wi = { addr, val }; 

    if(g_write_pos == 1024)
        g_write_pos = 0; 

    g_write_buffer[g_write_pos++] = wi; 

    //TRACE(KERN_DEBUG, "set(0x%08X, 0x%08X)\n", addr, val); 
    iowrite32(val, g_chip.regs.io + addr); 
}

uint32_t rcm_nand_get(uint32_t addr) 
{ 
    uint32_t r = ioread32(g_chip.regs.io + addr); 
    //TRACE(KERN_DEBUG, "0x%08X = get(0x%08X)\n", r, addr); 
    return r; 
}

static uint32_t rcm_nand_pagesize_code( uint32_t size )
{
    switch( size )
    {
    case 512: return 0;
    case 1024: return 1;
    case 2048: return 2;
    case 4096: return 3;
    case 8192: return 4;
    case 16384: return 5;
    case 32768: return 6;
    case 65536: return 7;
    default:
        NAND_DBG_PRINT_ERR( "get_pagesize_code: code size error (%u)\n", size )
        return 0;
    }
}

static inline char* rcm_nand_get_read_ecc(size_t i) 
{ 
    return g_chip.dma_area + g_chip.mtd.writesize + g_ecclayout.eccpos[MNAND_ECC_BYTESPERBLOCK * i]; 
} 

static inline char* rcm_nand_get_calculated_ecc(size_t i) 
{ 
    static int data; 
    char* cdata = (char*)&data; 
    char c; 
    data = rcm_nand_get(0xD8+i*sizeof(uint32_t)); 
    c = cdata[0]; 
    cdata[0] = cdata[2]; 
    cdata[2] = c; 
    return cdata; 
}

//void mnand_cs(int cs)  
//{
//    uint32_t tmp; 
//    BUG_ON(cs>1); 
//    tmp = rcm_nand_get(0x4); 
//    tmp &= ~(1 << 26); 
//    tmp |= cs << 26; 
//    rcm_nand_set(0x4, tmp); 
//}

int rcm_nand_chip_offset( loff_t* size )
{
        int cs = 0; 
        if( *size >= g_chip.chip_size[0] ) {
                cs++;
                *size -= g_chip.chip_size[0];
        }
        return cs; 
}

void mnand_reset_grabber(void) 
{ 
/*  rcm_nand_set(0x1008,0x8); 
    rcm_nand_set(0x100C, 1); 
    rcm_nand_set(0x100C, 0); 

    rcm_nand_set(0x2008, 0xffffffff); 
    rcm_nand_set(0x200C, 1); 
    rcm_nand_set(0x200C, 0); 

    rcm_nand_set(0x3008, 0xffffffff); 
    rcm_nand_set(0x300C, 1); 
    rcm_nand_set(0x300C, 0); 

    rcm_nand_set(0x400C, 1); 
    rcm_nand_set(0x400C, 0); 

    rcm_nand_set(0x500C, 1); 
    rcm_nand_set(0x500C, 0); 

    rcm_nand_set(0x800C, 1); 
    rcm_nand_set(0x800C, 0); 
    rcm_nand_set(0x8008, 0xffffffff);*/ 
}

static irqreturn_t rcm_nand_interrupt_handler(int irq, void* data) 
{ 
    if( init_flag == 0 ) {
        NAND_DBG_PRINT_ERR( "rcm_nand_interrupt_handler: init_flag=0\n" ) 
        return IRQ_NONE;
    }

    g_chip.status1 = rcm_nand_get( NAND_REG_irq_status );

    if( !( g_chip.status1 & IRQ_RCM_NAND_MASK ) )
        return IRQ_NONE;

    switch( g_chip.state ) {
        case MNAND_READ:
            if( g_chip.status1 & IRQ_READ_FINISH ) {
                NAND_DBG_PRINT_INF( "rcm_nand_interrupt_handler: IRQ_READ_FINISH\n" )
                g_chip.state = MNAND_IDLE;
            }
            break;
        case MNAND_READID: 
            if( g_chip.status1 & IRQ_READID ) {
                NAND_DBG_PRINT_INF( "rcm_nand_interrupt_handler: IRQ_READID\n" )
                g_chip.state = MNAND_IDLE; 
            }
            break; 
        case MNAND_WRITE: 
            if( g_chip.status1 & IRQ_PROGRAM_FINISH ) {
                NAND_DBG_PRINT_INF( "rcm_nand_interrupt_handler: IRQ_PROGRAM_FINISH\n" )
                g_chip.state = MNAND_IDLE; 
            } 
            break; 
        case MNAND_ERASE: 
            if( g_chip.status1 & IRQ_ERASE ) {
                NAND_DBG_PRINT_INF( "rcm_nand_interrupt_handler: IRQ_ERASE\n" )
                g_chip.state = MNAND_IDLE; 
            }
            break; 
        case MNAND_RESET:
            if( g_chip.status1 & IRQ_RESET ) {
                NAND_DBG_PRINT_INF( "rcm_nand_interrupt_handler: IRQ_RESET\n" )
                g_chip.state = MNAND_IDLE; 
            }
            break;
        default:
            NAND_DBG_PRINT_ERR( "rcm_nand_interrupt_handler: interrupt, but idle state\n" )
            return IRQ_NONE; 
    } 

    if( g_chip.state == MNAND_IDLE ) {
        complete_all(&g_completion);
    }
    else {  // пока так, сюда попадем при ошибках шины и некорректируемых
        NAND_DBG_PRINT_ERR( " rcm_nand_interrupt_handler: error in state %u\n", g_chip.state )
        NAND_DBG_PRINT_ERR( "   IRQ_READ_FINISH=%u\n", (bool)( g_chip.status1 & IRQ_READ_FINISH ) )
        NAND_DBG_PRINT_ERR( "        IRQ_READID=%u\n", (bool)( g_chip.status1 & IRQ_READID ) )
        NAND_DBG_PRINT_ERR( "IRQ_PROGRAM_FINISH=%u\n", (bool)( g_chip.status1 & IRQ_PROGRAM_FINISH ) )
        NAND_DBG_PRINT_ERR( "         IRQ_ERASE=%u\n", (bool)( g_chip.status1 & IRQ_ERASE ) )
        NAND_DBG_PRINT_ERR( "         IRQ_RESET=%u\n", (bool)( g_chip.status1 & IRQ_RESET ) )
        NAND_DBG_PRINT_ERR( " IRQ_UNCORR_ERROR0=%u\n", (bool)( g_chip.status1 & IRQ_UNCORR_ERROR0 ) )
        NAND_DBG_PRINT_ERR( "    IRQ_AXIW_ERROR=%u\n", (bool)( g_chip.status1 & IRQ_AXIW_ERROR ) )
        NAND_DBG_PRINT_ERR( " IRQ_LAST_ADR_AXIW=%u\n", (bool)( g_chip.status1 & IRQ_LAST_ADR_AXIW ) )
        NAND_DBG_PRINT_ERR( "IRQ_LAST_SEGM_AXIW=%u\n", (bool)( g_chip.status1 & IRQ_LAST_SEGM_AXIW ) )
        NAND_DBG_PRINT_ERR( "    IRQ_AXIR_ERROR=%u\n", (bool)( g_chip.status1 & IRQ_AXIR_ERROR ) )
        NAND_DBG_PRINT_ERR( " IRQ_LAST_ADR_AXIR=%u\n", (bool)( g_chip.status1 & IRQ_LAST_ADR_AXIR ) )
        NAND_DBG_PRINT_ERR( "IRQ_LAST_SEGM_AXIR=%u\n", (bool)( g_chip.status1 & IRQ_LAST_SEGM_AXIR ) )
        NAND_DBG_PRINT_ERR( " IRQ_UNCORR_ERROR1=%u\n", (bool)( g_chip.status1 & IRQ_UNCORR_ERROR1 ) )
        NAND_DBG_PRINT_ERR( "  IRQ_STATUS_EMPTY=%u\n", (bool)( g_chip.status1 & IRQ_STATUS_EMPTY ) )
        // ?complete_all(&g_completion);
    }
    return IRQ_HANDLED; 
} 

void rcm_nand_update_control( uint32_t chip_select,
                              uint32_t page_size,
                              uint32_t oob_size,                // команды в отношении страницы – размер OOB,остальные команды–количество читаемых байтов данных
                              uint32_t op_begin,
                              uint32_t num_adr_bytes, 
                              uint32_t ecc_mode, 
                              uint32_t hw_w_protect,
                              uint32_t oob_ecc,
                              uint32_t command )
{
    uint32_t wr_reg = ( op_begin << OP_BEGIN_SHIFT ) |
                      ( page_size << PAGE_SIZE_SHIFT ) |
                      ( num_adr_bytes << NUM_ADR_BYTES_SHIFT ) |
                      ( ecc_mode << ECC_MODE_SHIFT ) |
                      ( chip_select << CE_SHIFT ) |
                      ( hw_w_protect << HWP_SHIFT ) |
                      ( oob_ecc << OOB_ECC_SHIFT ) |
                      ( oob_size << OOB_SIZE_SHIFT ) |
                      ( command << FCMD_SHIFT ), rd_reg;
    rcm_nand_set( NAND_REG_control, wr_reg );
    rd_reg = rcm_nand_get( NAND_REG_control );
    NAND_DBG_PRINT_INF( "rcm_nand_update_control: control wr=%08x,rd=%08x\n", wr_reg, rd_reg )
}

static int rcm_nand_wait_ready( void )
{
    int ret = -1;
    uint32_t status, timeout = NAND_READY_TIMEOUT;

    while( --timeout )
    {
        status = rcm_nand_get( NAND_REG_status );
        //if( status & STATUS_REG_NAND_FAIL )
        //    break;
        if( status & ( STATUS_REG_CONT_READY | STATUS_REG_NAND_READY ) ) {
           ret = 0;
           break;
        }
    }
    NAND_DBG_PRINT_INF( "rcm_nand_wait_ready: return %d,status=%08x,timeout=%u\n", ret, status, timeout )
    return ret;
}

static void rcm_nand_set_timing( uint32_t offset, const uint32_t* timings, unsigned cnt, uint32_t freq )
{
    unsigned i;
    uint32_t timing_reg_wr = 0, timing_reg_rd, n = 0;
    for( i=0; i<cnt; i++ )
    {
        timing_reg_wr |= ( RCM_NAND_TIMING_TO_CLOCKS( timings[i], freq ) << n );
        n += 8;
    }
    rcm_nand_set( offset, timing_reg_wr );
    timing_reg_rd = rcm_nand_get( offset );
    NAND_DBG_PRINT_INF( "rcm_nand_set_timing: reg=%02x,wr=%08x,rd=%08x(%u,%u,%u,%u)\n",
                        offset, timing_reg_wr, timing_reg_rd,
                        (timing_reg_rd>>24)&0xff, (timing_reg_rd>>16)&0xff, (timing_reg_rd>>8)&0xff, (timing_reg_rd>>0)&0xff )
}

static void rcm_nand_set_timings( uint32_t* timings, uint32_t freq )
{
    NAND_DBG_PRINT_INF( "rcm_nand_set_timings: freq=%u\n", freq )
    rcm_nand_set_timing( NAND_REG_timing_0, timings+0, 4, freq );         // tADL_min, tALH_min, tALS_min, tCH_min
    rcm_nand_set_timing( NAND_REG_timing_1, timings+4, 4, freq );         // tCLH_min, tCLS_min, tCS_min, tDH_min
    rcm_nand_set_timing( NAND_REG_timing_2, timings+8, 4, freq );         // tDS_min, tWC_min, tWH_min, tWP_min
    rcm_nand_set_timing( NAND_REG_timing_3, timings+12, 3, freq );        // tWW_min, tAR_min, tCLR_min
    rcm_nand_set_timing( NAND_REG_timing_4, timings+15, 4, freq );        // tCOH_min, tIR_min, tRC_min, tREH_min
    rcm_nand_set_timing( NAND_REG_timing_5, timings+19, 4, freq );        // tRHOH_min, tRHW_min, tRLOH_min, tRP_min
    timings[25] += 2;
    rcm_nand_set_timing( NAND_REG_timing_6, timings+23, 4, freq );        // tRR_min, tWHR_min, tCEA_max+2, tCHZ_max
    timings[27] += 2;
    timings[29] += 2;
    rcm_nand_set_timing( NAND_REG_timing_7, timings+27, 3, freq );        // tREA_max+2, tRHZ_max, tWB_max+2
}

int rcm_nand_hw_init( uint32_t* timings, uint32_t freq ) 
{
    uint32_t ctrl_id = rcm_nand_get( NAND_REG_id ),                 // 0x444E414E
             ctrl_ver = rcm_nand_get( NAND_REG_version );           // 0x000008B4

    NAND_DBG_PRINT_INF( "rcm_nand_init: id=%08x,version=%08x\n", ctrl_id, ctrl_ver )

    rcm_nand_set( NAND_REG_cntrl_sw_rst, 0x0);                      // запись 0 в данный регистр инициирует программный сброс контроллера
    rcm_nand_set( NAND_REG_sw_rst, 0x0 );                           // запись 0,затем 1 в данный регистр инициирует
    rcm_nand_set( NAND_REG_sw_rst, 0x1 );                           // программный сброс ведущего интерфейса записи AXI 
    rcm_nand_set( NAND_REG_sw_rstn_r, 0x1 );                        // программный сброс ведущего интерфейса чтения AXI, автоматически обнуляется

    if( rcm_nand_wait_ready() ) return -EIO;

    rcm_nand_set_timings( timings, freq/1000000 );

    return 0;
}

static void rcm_nand_prepare_dma_read( size_t bytes )                   // чтение ИЗ флэш
{
    rcm_nand_set( NAND_REG_awlen_max, 0xF );                            // AXI burst size рекомендуемое значение: 0xF(16*4) (таблица 1.845)
    rcm_nand_set( NAND_REG_msb_lsbw, 0x1 );                             // 1-big_endian,0-little endian
    rcm_nand_set( NAND_REG_start_dma_w, g_chip.dma_handle );
    rcm_nand_set( NAND_REG_end_dma_w, g_chip.dma_handle+bytes-1 );
    rcm_nand_set( NAND_REG_cntrl_dma_w,
                  START_ADDR_GEN | ( (bytes-1) << SEGM_SIZE_SHIFT ) );  // 28..5-размер сегмента-1, 31-генерация адресов
}

static void rcm_nand_prepare_dma_write( size_t bytes )                  // запись В флэш
{
    rcm_nand_set( NAND_REG_axi_arlen, 0xF );                            // максимальное значение ARLEN для транзакций по ведущему интерфейсу чтения AXI,рекомендуемое значение 0xF. 
    rcm_nand_set( NAND_REG_msb_lsbr, 0x1 );                             // 1-big_endian,0-little endian
    rcm_nand_set( NAND_REG_start_dma_r, g_chip.dma_handle );
    rcm_nand_set( NAND_REG_end_dma_r, g_chip.dma_handle+bytes-1 );
    rcm_nand_set( NAND_REG_cntrl_dma_r,
                  START_ADDR_GEN | ( (bytes-1) << SEGM_SIZE_SHIFT ) );  // 28..5-размер сегмента-1, 31-генерация адресов
}

int mnand_ready(void) 
{ 
    return (rcm_nand_get(0) & 0x200) != 0; 
}

static bool rcm_nand_wait_int( u32 mask )
{
    bool ret = false;
    u32 irq_status;
    u32 timeout = NAND_READY_TIMEOUT;
    while( --timeout )
    {
        irq_status = rcm_nand_get(  NAND_REG_irq_status );
        if( irq_status & mask )
        {
            ret = true;
            break;
        }
    }

    NAND_DBG_PRINT_INF( "rcm_nand_wait_int %s (irq_status=%08x,timeout=%u)\n", ret ? "Ok" : "Error", irq_status, timeout )
    NAND_DBG_PRINT_INF( "   IRQ_READ_FINISH=%u\n", (bool)( irq_status & IRQ_READ_FINISH ) )
    NAND_DBG_PRINT_INF( "        IRQ_READID=%u\n", (bool)( irq_status & IRQ_READID ) )
    NAND_DBG_PRINT_INF( "IRQ_PROGRAM_FINISH=%u\n", (bool)( irq_status & IRQ_PROGRAM_FINISH ) )
    NAND_DBG_PRINT_INF( "         IRQ_ERASE=%u\n", (bool)( irq_status & IRQ_ERASE ) )
    NAND_DBG_PRINT_INF( "         IRQ_RESET=%u\n", (bool)( irq_status & IRQ_RESET ) )
    NAND_DBG_PRINT_INF( " IRQ_UNCORR_ERROR0=%u\n", (bool)( irq_status & IRQ_UNCORR_ERROR0 ) )
    NAND_DBG_PRINT_INF( "    IRQ_AXIW_ERROR=%u\n", (bool)( irq_status & IRQ_AXIW_ERROR ) )
    NAND_DBG_PRINT_INF( " IRQ_LAST_ADR_AXIW=%u\n", (bool)( irq_status & IRQ_LAST_ADR_AXIW ) )
    NAND_DBG_PRINT_INF( "IRQ_LAST_SEGM_AXIW=%u\n", (bool)( irq_status & IRQ_LAST_SEGM_AXIW ) )
    NAND_DBG_PRINT_INF( "    IRQ_AXIR_ERROR=%u\n", (bool)( irq_status & IRQ_AXIR_ERROR ) )
    NAND_DBG_PRINT_INF( " IRQ_LAST_ADR_AXIR=%u\n", (bool)( irq_status & IRQ_LAST_ADR_AXIR ) )
    NAND_DBG_PRINT_INF( "IRQ_LAST_SEGM_AXIR=%u\n", (bool)( irq_status & IRQ_LAST_SEGM_AXIR ) )
    NAND_DBG_PRINT_INF( " IRQ_UNCORR_ERROR1=%u\n", (bool)( irq_status & IRQ_UNCORR_ERROR1 ) )
    NAND_DBG_PRINT_INF( "  IRQ_STATUS_EMPTY=%u\n", (bool)( irq_status & IRQ_STATUS_EMPTY ) )
    return ret;
}

static int rcm_nand_core_reset( uint32_t chip_select ) 
{
        if( rcm_nand_wait_ready() )
                return -EIO;
    
        init_completion(&g_completion); 
        init_flag=1;
        g_chip.state = MNAND_RESET;

        rcm_nand_set( NAND_REG_irq_mask_nand, IRQ_RESET );
        rcm_nand_set( NAND_REG_command, CMD_CONTROL_RESET );
        rcm_nand_update_control( chip_select,           // микросхема
                                 0,                     // код размера страницы
                                 256,                   // размер запасной области/количество считываемых байтов
                                 1,                     // начать операцию сразу
                                 0,                     // количество адресных циклов
                                 NAND_ECC_MODE_NO_ECC,  // режим коррекции ошибок
                                 0,                     // защита от записи
                                 0,                     // разрешение оценки кодов коррекции
                                 0 );                   // первая отправляемая во флэш команда

        wait_for_completion_io(&g_completion);
        g_chip.state = MNAND_IDLE;
        return 0; 
} 

static int mnand_core_erase(loff_t off) 
{
        int cs;
        uint32_t status;
        //BUG_ON(g_chip.state != MNAND_IDLE); 
        //BUG_ON(!mnand_ready()); 
        if( rcm_nand_wait_ready() )
                return -EIO;

        cs = rcm_nand_chip_offset( &off );
    
        mnand_reset_grabber(); 

        init_completion(&g_completion);
        g_chip.state = MNAND_ERASE;
 
        rcm_nand_set( NAND_REG_irq_mask_nand, IRQ_ERASE );
        rcm_nand_set( NAND_REG_command, CMD_ERASE_BLOCK );
        rcm_nand_set( NAND_REG_col_addr, 0 );                                                   // адрес столбца
        rcm_nand_set( NAND_REG_row_addr_read_d,
                      (off >> g_chip.mtd.erasesize_shift) << (ffs(g_chip.mtd.erasesize)) );     // адрес строки и адрес блока
        rcm_nand_update_control( cs,                                                            // микросхема
                                 0,                                                             // код размера страницы
                                 g_chip.mtd.oobsize,                                            // размер запасной области
                                 1,                                                             // начать операцию сразу
                                 5,                                                             // 5 байтов (циклов) адреса в текущей команде
                                 NAND_ECC_MODE_NO_ECC,                                          // режим ecc (пока без коррекции)
                                 0,                                                             // hw write protect enabled (почему?)
                                 0,                                                             // запретить проверку коррекции ошибок
                                 FLASH_CMD_BLOCK_ERASE );                                       // команда для flash
        wait_for_completion_io( &g_completion );
        //BUG_ON(!mnand_ready());
        g_chip.state = MNAND_IDLE; 
        status = rcm_nand_get( NAND_REG_status );
        NAND_DBG_PRINT_INF( "mnand_core_erase: completion status %08x\n", status )
        return ( status & STATUS_REG_NAND_FAIL ) ? -EIO : 0;
} 

static void mnand_core_read_id( uint32_t chip_select, size_t bytes) 
{
         //if( rcm_nand_wait_ready() )
                //return -EIO;
    
        rcm_nand_prepare_dma_read( bytes );
        init_completion(&g_completion); 
        g_chip.state = MNAND_READID;

        rcm_nand_set( NAND_REG_irq_mask_nand, IRQ_READID );
        rcm_nand_set( NAND_REG_command, CMD_CONTROL_READ_ID );
        rcm_nand_set( NAND_REG_col_addr, 0x00 );        // адрес для считывания идентификационного номера
        rcm_nand_update_control( chip_select,           // микросхема
                                 0,                     // код размера страницы
                                 bytes,                 // размер запасной области/количество считываемых байтов
                                 1,                     // начать операцию сразу
                                 1,                     // количество адресных циклов
                                 NAND_ECC_MODE_NO_ECC,  // режим коррекции ошибок
                                 0,                     // защита от записи
                                 0,                     // разрешение оценки кодов коррекции
                                 FLASH_CMD_READ_ID );   // первая отправляемая во флэш команда

        wait_for_completion_io(&g_completion); 
        g_chip.state = MNAND_IDLE;
}

/* 
* invparity is a 256 byte table that contains the odd parity 
* for each byte. So if the number of bits in a byte is even, 
* the array element is 1, and when the number of bits is odd 
* the array eleemnt is 0. 
 */ 
static const char invparity[256] = { 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1 
};

static const char addressbits[256] = {
    0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 
    0x02, 0x02, 0x03, 0x03, 0x02, 0x02, 0x03, 0x03, 
    0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 
    0x02, 0x02, 0x03, 0x03, 0x02, 0x02, 0x03, 0x03, 
    0x04, 0x04, 0x05, 0x05, 0x04, 0x04, 0x05, 0x05, 
    0x06, 0x06, 0x07, 0x07, 0x06, 0x06, 0x07, 0x07, 
    0x04, 0x04, 0x05, 0x05, 0x04, 0x04, 0x05, 0x05, 
    0x06, 0x06, 0x07, 0x07, 0x06, 0x06, 0x07, 0x07, 
    0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 
    0x02, 0x02, 0x03, 0x03, 0x02, 0x02, 0x03, 0x03, 
    0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 
    0x02, 0x02, 0x03, 0x03, 0x02, 0x02, 0x03, 0x03, 
    0x04, 0x04, 0x05, 0x05, 0x04, 0x04, 0x05, 0x05, 
    0x06, 0x06, 0x07, 0x07, 0x06, 0x06, 0x07, 0x07, 
    0x04, 0x04, 0x05, 0x05, 0x04, 0x04, 0x05, 0x05, 
    0x06, 0x06, 0x07, 0x07, 0x06, 0x06, 0x07, 0x07, 
    0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09, 
    0x0a, 0x0a, 0x0b, 0x0b, 0x0a, 0x0a, 0x0b, 0x0b, 
    0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09, 
    0x0a, 0x0a, 0x0b, 0x0b, 0x0a, 0x0a, 0x0b, 0x0b, 
    0x0c, 0x0c, 0x0d, 0x0d, 0x0c, 0x0c, 0x0d, 0x0d, 
    0x0e, 0x0e, 0x0f, 0x0f, 0x0e, 0x0e, 0x0f, 0x0f, 
    0x0c, 0x0c, 0x0d, 0x0d, 0x0c, 0x0c, 0x0d, 0x0d, 
    0x0e, 0x0e, 0x0f, 0x0f, 0x0e, 0x0e, 0x0f, 0x0f, 
    0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09, 
    0x0a, 0x0a, 0x0b, 0x0b, 0x0a, 0x0a, 0x0b, 0x0b, 
    0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09, 
    0x0a, 0x0a, 0x0b, 0x0b, 0x0a, 0x0a, 0x0b, 0x0b, 
    0x0c, 0x0c, 0x0d, 0x0d, 0x0c, 0x0c, 0x0d, 0x0d, 
    0x0e, 0x0e, 0x0f, 0x0f, 0x0e, 0x0e, 0x0f, 0x0f, 
    0x0c, 0x0c, 0x0d, 0x0d, 0x0c, 0x0c, 0x0d, 0x0d, 
    0x0e, 0x0e, 0x0f, 0x0f, 0x0e, 0x0e, 0x0f, 0x0f 
}; 

static const char bitsperbyte[256] = { 
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 
    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8, 
}; 

static int mnand_correct_data(struct mtd_info *mtd, unsigned char *buf, 
                          unsigned char *read_ecc, unsigned char *calc_ecc) 
{ 
    unsigned char b0, b1, b2; 
    unsigned char byte_addr, bit_addr; 
    /* 256 or 512 bytes/ecc  */ 
    const uint32_t eccsize_mult = 1; 
 
/* 
    * b0 to b2 indicate which bit is faulty (if any) 
    * we might need the xor result  more than once, 
    * so keep them in a local var 
    */ 
    #ifdef CONFIG_MTD_NAND_ECC_SMC 
        b0 = read_ecc[0] ^ calc_ecc[0]; 
        b1 = read_ecc[1] ^ calc_ecc[1]; 
    #else 
        b0 = read_ecc[1] ^ calc_ecc[1]; 
        b1 = read_ecc[0] ^ calc_ecc[0]; 
    #endif 
        b2 = read_ecc[2] ^ calc_ecc[2]; 
 
 /* check if there are any bitfaults */ 

    /* repeated if statements are slightly more efficient than switch ... */ 
    /* ordered in order of likelihood */ 
    if ((b0 | b1 | b2) == 0) 
        return 0;	/* no error */ 

    if ((((b0 ^ (b0 >> 1)) & 0x55) == 0x55) && 
        (((b1 ^ (b1 >> 1)) & 0x55) == 0x55) && 
        ((eccsize_mult == 1 && ((b2 ^ (b2 >> 1)) & 0x54) == 0x54) || 
         (eccsize_mult == 2 && ((b2 ^ (b2 >> 1)) & 0x55) == 0x55))) { 
                 /* single bit error */ 
                 /* 
                  * rp17/rp15/13/11/9/7/5/3/1 indicate which byte is the faulty 
                  * byte, cp 5/3/1 indicate the faulty bit. 
                  * A lookup table (called addressbits) is used to filter 
                  * the bits from the byte they are in. 
                  * A marginal optimisation is possible by having three 
                  * different lookup tables. 
                  * One as we have now (for b0), one for b2 
                  * (that would avoid the >> 1), and one for b1 (with all values 
                  * << 4). However it was felt that introducing two more tables 
                  * hardly justify the gain. 
                  * 
                  * The b2 shift is there to get rid of the lowest two bits. 
                  * We could also do addressbits[b2] >> 1 but for the 
                  * performace it does not make any difference 
                  */ 
                 if (eccsize_mult == 1) 
                         byte_addr = (addressbits[b1] << 4) + addressbits[b0]; 
                 else 
                         byte_addr = (addressbits[b2 & 0x3] << 8) + 
                                     (addressbits[b1] << 4) + addressbits[b0]; 
                    bit_addr = addressbits[b2 >> 2]; 
                  /* flip the bit */ 
                    buf[byte_addr] ^= (1 << bit_addr); 
                    return 1; 
         } 
/* count nr of bits; use table lookup, faster than calculating it */ 
        if ((bitsperbyte[b0] + bitsperbyte[b1] + bitsperbyte[b2]) == 1) 
            return 1;	/* error in ecc data; no action needed */ 
        printk(KERN_ERR "uncorrectable error : \n"); 
        return -1; 
}

int mnand_calculate_ecc(struct mtd_info *mtd, const unsigned char *buf, unsigned char *code) 
{
    int i; 
    const uint32_t *bp = (uint32_t *)buf; 
    /* 256 or 512 bytes/ecc  */ 
    const uint32_t eccsize_mult = 1; 

    uint32_t cur;		/* current value in buffer */ 
    /* rp0..rp15..rp17 are the various accumulated parities (per byte) */ 
    uint32_t rp0, rp1, rp2, rp3, rp4, rp5, rp6, rp7; 
    uint32_t rp8, rp9, rp10, rp11, rp12, rp13, rp14, rp15, rp16; 
    uint32_t uninitialized_var(rp17);	/* to make compiler happy */ 
    uint32_t par;		/* the cumulative parity for all data */ 
    uint32_t tmppar;	/* the cumulative parity for this iteration; 
                           for rp12, rp14 and rp16 at the end of the 
                           loop */ 

    par = 0; 
    rp4 = 0; 
    rp6 = 0; 
    rp8 = 0; 
    rp10 = 0; 
    rp12 = 0; 
    rp14 = 0; 
    rp16 = 0; 

    /* 
     * The loop is unrolled a number of times; 
     * This avoids if statements to decide on which rp value to update 
     * Also we process the data by longwords. 
     * Note: passing unaligned data might give a performance penalty. 
     * It is assumed that the buffers are aligned. 
     * tmppar is the cumulative sum of this iteration. 
     * needed for calculating rp12, rp14, rp16 and par 
     * also used as a performance improvement for rp6, rp8 and rp10 
     */ 
    for (i = 0; i < eccsize_mult << 2; i++) { 
            cur = *bp++; 
            tmppar = cur; 
            rp4 ^= cur; 
            cur = *bp++; 
            tmppar ^= cur; 
            rp6 ^= tmppar; 
            cur = *bp++; 
            tmppar ^= cur; 
            rp4 ^= cur; 
            cur = *bp++; 
            tmppar ^= cur; 
            rp8 ^= tmppar; 

            cur = *bp++; 
            tmppar ^= cur; 
            rp4 ^= cur; 
            rp6 ^= cur; 
            cur = *bp++; 
            tmppar ^= cur; 
            rp6 ^= cur; 
            cur = *bp++; 
            tmppar ^= cur; 
            rp4 ^= cur; 
            cur = *bp++; 
            tmppar ^= cur; 
            rp10 ^= tmppar; 

            cur = *bp++; 
            tmppar ^= cur; 
            rp4 ^= cur; 
            rp6 ^= cur; 
            rp8 ^= cur; 
            cur = *bp++; 
            tmppar ^= cur; 
            rp6 ^= cur; 
            rp8 ^= cur; 
            cur = *bp++; 
            tmppar ^= cur; 
            rp4 ^= cur; 
            rp8 ^= cur; 
            cur = *bp++; 
            tmppar ^= cur; 
            rp8 ^= cur; 

            cur = *bp++; 
            tmppar ^= cur; 
            rp4 ^= cur; 
            rp6 ^= cur; 
            cur = *bp++; 
            tmppar ^= cur; 
            rp6 ^= cur; 
            cur = *bp++; 
            tmppar ^= cur; 
            rp4 ^= cur; 
            cur = *bp++; 
            tmppar ^= cur; 

            par ^= tmppar; 
            if ((i & 0x1) == 0) 
                    rp12 ^= tmppar; 
            if ((i & 0x2) == 0) 
                    rp14 ^= tmppar; 
            if (eccsize_mult == 2 && (i & 0x4) == 0) 
                    rp16 ^= tmppar; 
}
/* 
          * handle the fact that we use longword operations 
          * we'll bring rp4..rp14..rp16 back to single byte entities by 
          * shifting and xoring first fold the upper and lower 16 bits, 
          * then the upper and lower 8 bits. 
          */ 
         rp4 ^= (rp4 >> 16); 
         rp4 ^= (rp4 >> 8); 
         rp4 &= 0xff; 
         rp6 ^= (rp6 >> 16); 
         rp6 ^= (rp6 >> 8); 
         rp6 &= 0xff; 
         rp8 ^= (rp8 >> 16); 
         rp8 ^= (rp8 >> 8); 
         rp8 &= 0xff; 
         rp10 ^= (rp10 >> 16); 
         rp10 ^= (rp10 >> 8); 
         rp10 &= 0xff; 
         rp12 ^= (rp12 >> 16); 
         rp12 ^= (rp12 >> 8); 
         rp12 &= 0xff; 
         rp14 ^= (rp14 >> 16); 
         rp14 ^= (rp14 >> 8); 
         rp14 &= 0xff; 
         if (eccsize_mult == 2) { 
                 rp16 ^= (rp16 >> 16); 
                 rp16 ^= (rp16 >> 8); 
                 rp16 &= 0xff; 
         } 
 
 
         /* 
          * we also need to calculate the row parity for rp0..rp3 
          * This is present in par, because par is now 
          * rp3 rp3 rp2 rp2 in little endian and 
          * rp2 rp2 rp3 rp3 in big endian 
          * as well as 
          * rp1 rp0 rp1 rp0 in little endian and 
          * rp0 rp1 rp0 rp1 in big endian 
          * First calculate rp2 and rp3 
          */ 
 #ifdef __BIG_ENDIAN 
         rp2 = (par >> 16); 
         rp2 ^= (rp2 >> 8); 
         rp2 &= 0xff; 
         rp3 = par & 0xffff; 
         rp3 ^= (rp3 >> 8); 
         rp3 &= 0xff; 
 #else 
         rp3 = (par >> 16); 
         rp3 ^= (rp3 >> 8); 
         rp3 &= 0xff; 
         rp2 = par & 0xffff; 
         rp2 ^= (rp2 >> 8); 
         rp2 &= 0xff; 
 #endif 
 
 
         /* reduce par to 16 bits then calculate rp1 and rp0 */ 
         par ^= (par >> 16); 
 #ifdef __BIG_ENDIAN 
         rp0 = (par >> 8) & 0xff; 
         rp1 = (par & 0xff); 
 #else 
         rp1 = (par >> 8) & 0xff; 
         rp0 = (par & 0xff); 
 #endif 
 
 
         /* finally reduce par to 8 bits */ 
         par ^= (par >> 8); 
         par &= 0xff; 
 
 
         /* 
          * and calculate rp5..rp15..rp17 
          * note that par = rp4 ^ rp5 and due to the commutative property 
          * of the ^ operator we can say: 
          * rp5 = (par ^ rp4); 
          * The & 0xff seems superfluous, but benchmarking learned that 
          * leaving it out gives slightly worse results. No idea why, probably 
          * it has to do with the way the pipeline in pentium is organized. 
          */ 
         rp5 = (par ^ rp4) & 0xff; 
         rp7 = (par ^ rp6) & 0xff; 
         rp9 = (par ^ rp8) & 0xff; 
         rp11 = (par ^ rp10) & 0xff; 
         rp13 = (par ^ rp12) & 0xff; 
         rp15 = (par ^ rp14) & 0xff; 
         if (eccsize_mult == 2) 
                 rp17 = (par ^ rp16) & 0xff; 
 
 
         /* 
          * Finally calculate the ecc bits. 
          * Again here it might seem that there are performance optimisations 
          * possible, but benchmarks showed that on the system this is developed 
          * the code below is the fastest 
          */ 
 #ifdef CONFIG_MTD_NAND_ECC_SMC 
         code[0] = 
                 (invparity[rp7] << 7) | 
                 (invparity[rp6] << 6) | 
                 (invparity[rp5] << 5) | 
                 (invparity[rp4] << 4) | 
                 (invparity[rp3] << 3) | 
                 (invparity[rp2] << 2) | 
                 (invparity[rp1] << 1) | 
                 (invparity[rp0]); 
         code[1] = 
                 (invparity[rp15] << 7) | 
                 (invparity[rp14] << 6) | 
                 (invparity[rp13] << 5) | 
                 (invparity[rp12] << 4) | 
                 (invparity[rp11] << 3) | 
                 (invparity[rp10] << 2) | 
                 (invparity[rp9] << 1)  | 
                 (invparity[rp8]); 
 #else 
         code[1] = 
                 (invparity[rp7] << 7) | 
                 (invparity[rp6] << 6) | 
                 (invparity[rp5] << 5) | 
                 (invparity[rp4] << 4) | 
                 (invparity[rp3] << 3) | 
                 (invparity[rp2] << 2) | 
                 (invparity[rp1] << 1) | 
                 (invparity[rp0]); 
         code[0] = 
                 (invparity[rp15] << 7) | 
                 (invparity[rp14] << 6) | 
                 (invparity[rp13] << 5) | 
                 (invparity[rp12] << 4) | 
                 (invparity[rp11] << 3) | 
                 (invparity[rp10] << 2) | 
                 (invparity[rp9] << 1)  | 
                 (invparity[rp8]); 
 #endif 
         if (eccsize_mult == 1) 
                 code[2] = 
                         (invparity[par & 0xf0] << 7) | 
                         (invparity[par & 0x0f] << 6) | 
                         (invparity[par & 0xcc] << 5) | 
                         (invparity[par & 0x33] << 4) | 
                         (invparity[par & 0xaa] << 3) | 
                         (invparity[par & 0x55] << 2) | 
                         3; 
         else 
                 code[2] = 
                         (invparity[par & 0xf0] << 7) | 
                         (invparity[par & 0x0f] << 6) | 
                         (invparity[par & 0xcc] << 5) | 
                         (invparity[par & 0x33] << 4) | 
                         (invparity[par & 0xaa] << 3) | 
                         (invparity[par & 0x55] << 2) | 
                         (invparity[rp17] << 1) | 
                         (invparity[rp16] << 0); 
         return 0; 
 } 
 
 
 static int mnand_core_read(loff_t off) 
 { 
        int cs;
        loff_t page;
        uint32_t status;
        //size_t corrected = 0; 
        //size_t failed = 0; 

        if( rcm_nand_wait_ready() )
                return -EIO;

        status = rcm_nand_get( NAND_REG_status ); 
        cs = rcm_nand_chip_offset( &off ); 
        page = off & (~(g_chip.mtd.writesize-1)); 
 
        //BUG_ON(g_chip.state != MNAND_IDLE); 
        //BUG_ON(!mnand_ready()); 
        //mnand_reset_grabber(); 
 
        NAND_DBG_PRINT_INF( "mnand_core_read: cs=%u,off=%08llX,page=%08llX,status=%08x\n", cs, off, page, status )
 
        if( g_chip.active_page != page ) { 
                g_chip.active_page = -1;
                rcm_nand_prepare_dma_read( g_chip.mtd.writesize + g_chip.mtd.oobsize );
                init_completion(&g_completion); 
                g_chip.state = MNAND_READ; 
                rcm_nand_set( NAND_REG_irq_mask_nand, IRQ_READ_FINISH );
                rcm_nand_set( NAND_REG_command, CMD_CONTROL_READ );                                     // чтение (из flash)
                rcm_nand_set( NAND_REG_col_addr, 0 );                                                   // адрес столбца
                rcm_nand_set( NAND_REG_row_addr_read_d, (off >> g_chip.mtd.writesize_shift) );          // адрес строки и адрес блока
                rcm_nand_update_control( cs,                                                            // микросхема
                                         rcm_nand_pagesize_code( g_chip.mtd.writesize ),                // код размера страницы
                                         g_chip.mtd.oobsize,                                            // размер запасной области
                                         1,                                                             // начать операцию сразу
                                         5,                                                             // 5 байтов (циклов) адреса в текущей команде
                                         NAND_ECC_MODE_4BITS,/*NAND_ECC_MODE_NO_ECC,*/                  // режим ecc (пока без коррекции)
                                         1,                                                             // hw write protect enabled (почему?)
                                         0,                                                             // запретить проверку коррекции ошибок
                                         FLASH_CMD_PAGE_READ );                                         // команда для flash
                wait_for_completion_io( &g_completion );
                PRINT_BUF_256( ((uint8_t*)g_chip.dma_area), 0 )
                 //BUG_ON(!mnand_ready());
                g_chip.state = MNAND_IDLE; 
                status = rcm_nand_get( NAND_REG_status );
                NAND_DBG_PRINT_INF( "mnand_core_read: completion status %08x\n", status )
                return ( status & STATUS_REG_NAND_FAIL ) ? -EIO : 0;
#if 0
                 g_read_page_log[g_read_page_pos].address = page; 
                 memcpy(g_read_page_log[g_read_page_pos].data, g_chip.dma_area, 2048+64); 
                 ++g_read_page_pos; 
                 if(g_read_page_pos == sizeof(g_read_page_log) / sizeof(g_read_page_log[0])) 
                         g_read_page_pos = 0; 
 
 
                 if(/*rcm_nand_get(0) & MNAND_STATUS_ECC_ERROR*/1) { 
                         size_t i; 
                         /*Check if we had a bad block*/ 
                         if(*((char*)(g_chip.dma_area + g_chip.mtd.writesize)) == 0xFF) { 
                                 for(i = 0; i < g_chip.mtd.writesize/MNAND_ECC_BLOCKSIZE; ++i) { 
                                         char ecc[3]; 
                                         mnand_calculate_ecc(&g_chip.mtd, g_chip.dma_area + i * MNAND_ECC_BLOCKSIZE, ecc); 
                                         if(memcmp(rcm_nand_get_calculated_ecc(i),ecc,3)!=0) { 
                                                 printk("\n\necc mismatch %02X %02X %02X - %02x %02X %02X \n\n", 
                                                        ecc[0], ecc[1], ecc[2], 
                                                        rcm_nand_get_calculated_ecc(i)[0], 
                                                        rcm_nand_get_calculated_ecc(i)[1], 
                                                        rcm_nand_get_calculated_ecc(i)[2]); 
 
 
                                                 BUG_ON(1); 
                                         } 
                                         if(memcmp(rcm_nand_get_read_ecc(i), rcm_nand_get_calculated_ecc(i), MNAND_ECC_BYTESPERBLOCK)) { 
                                                 int stat = mnand_correct_data(&g_chip.mtd, 
                                                                               g_chip.dma_area + i * MNAND_ECC_BLOCKSIZE, 
                                                                               rcm_nand_get_read_ecc(i), 
                                                                               rcm_nand_get_calculated_ecc(i)); 
                                                 if( stat < 0 ) { 
                                                         ++failed; 
                                                 } else { 
                                                         ++corrected; 
                                                 } 
                                         } 
                                 } 
                         } 
                 } 
 
 
                 g_chip.ecc_corrected += corrected; 
                 g_chip.ecc_failed += failed; 
 
 
                 /*if(!failed) 
                   g_chip.active_page = page;*/ 
#endif
         } 
         return 0;

 } 
 
 
 static int mnand_core_write( loff_t off ) 
 { 
        int cs;
        uint32_t status;
        
        if( rcm_nand_wait_ready() )
                return -EIO;

        status = rcm_nand_get( NAND_REG_status );
//       BUG_ON(!mnand_ready()); 
//       BUG_ON(g_chip.state != MNAND_IDLE); 
        NAND_DBG_PRINT_INF( "mnand_core_write: off=0x%08llX,status=%08x\n", off, status )
        cs = rcm_nand_chip_offset( &off ); 
        g_chip.active_page = -1;
        rcm_nand_prepare_dma_write( g_chip.mtd.writesize  + g_chip.mtd.oobsize ); 
        init_completion(&g_completion); 
        g_chip.state = MNAND_WRITE;

        rcm_nand_set( NAND_REG_irq_mask_nand, IRQ_PROGRAM_FINISH );
        rcm_nand_set( NAND_REG_cdb_buffer_type, 1 );
        rcm_nand_set( NAND_REG_command, CMD_PROGRAM_TO_NAND );                                  // запись (в flash)
        rcm_nand_set( NAND_REG_col_addr, 0 );                                                   // адрес столбца
        rcm_nand_set( NAND_REG_row_addr_read_d, (off >> g_chip.mtd.writesize_shift) );          // адрес строки и адрес блока
        rcm_nand_update_control( cs,                                                            // микросхема
                                 rcm_nand_pagesize_code( g_chip.mtd.writesize ),                // код размера страницы
                                 g_chip.mtd.oobsize,                                            // размер запасной области
                                 1,                                                             // начать операцию сразу
                                 5,                                                             // 5 байтов (циклов) адреса в текущей команде
                                 NAND_ECC_MODE_4BITS,/*NAND_ECC_MODE_NO_ECC*/                   // режим ecc (пока без коррекции)
                                 0,                                                             // hw write protect enabled (почему?)
                                 0,                                                             // запретить проверку коррекции ошибок
                                 FLASH_CMD_PROGRAM_PAGE );                                      // команда для flash
        rcm_nand_set( NAND_REG_cdb_buffer_enable, 0x1 );
        wait_for_completion_io(&g_completion);
        PRINT_BUF_256( ((uint8_t*)g_chip.dma_area), 0 )
/*
         g_write_page_log[g_write_page_pos].address = off; 
         memcpy(g_write_page_log[g_write_page_pos].data, g_chip.dma_area, 2048+64); 
         ++g_write_page_pos; 
         if((g_write_page_pos) == sizeof(g_write_page_log)/sizeof(g_write_page_log[0])) 
                 g_write_page_pos = 0; 
*/
//         rcm_nand_set(0x08, 0x27); 
//         rcm_nand_set(0x0C, (u32)((off >> g_chip.mtd.writesize_shift) << 12)); 
//         BUG_ON(rcm_nand_get(0x0C) != (u32)((off >> g_chip.mtd.writesize_shift) << 12)); 
//        BUG_ON(!mnand_ready());

        g_chip.state = MNAND_IDLE;
        status = rcm_nand_get( NAND_REG_status );
        NAND_DBG_PRINT_INF( "mnand_core_write: completion status %08x\n", status )
        return ( status & STATUS_REG_NAND_FAIL ) ? -EIO : 0;
 } 
 
 
 static int rcm_nand_read_id(int cs) 
 { 
         struct nand_flash_dev* type = 0; 
         int i; 
         int retries=5; 
         char* vendor = "unknown"; 
         // mnand_cs(cs); 
 
 
         /* hw weirdness: sometimes after an unlucky reset or incomplete 
          * powerdown a bad id may be read. 
          */ 
 
 
         for (i=0; i<retries; i++) { 
                mnand_core_read_id(cs,256)/*2)*/;
                //PRINT_BUF_256( ((uint8_t*)(g_chip.dma_area)),0 )
                 if (*(((uint8_t*)g_chip.dma_area)+1)!=0x00) 
                         break; 
         } 
 
 
         if (i==retries) { 
                 NAND_DBG_PRINT_ERR("rcm_nand_read_id: bad chip id or no chip at CS=%d\n", cs); 
                 return -ENODEV; 
         } 
 
 
         /* Lookup the flash vendor */ 
         for (i = 0; nand_manuf_ids[i].name != NULL; i++) { 
                 if (((uint8_t*)g_chip.dma_area)[0] == nand_manuf_ids[i].id) { 
                         vendor =  nand_manuf_ids[i].name; 
                         break; 
                 } 
         }


         /* Lookup the flash id */ 
         for (i = 0; nand_flash_ids[i].name != NULL; i++) { 
                 if (((uint8_t*)g_chip.dma_area)[1] == nand_flash_ids[i].dev_id && 
                         nand_flash_ids[i].mfr_id == 0) { 
                         type = &nand_flash_ids[i]; 
                         break; 
                 } 
         } 
  
         if (!type)
                 return -ENODEV;


 /* We're not likely work if erase/write/oob sizes on different CS 
  *  differ. 
  */ 

 #define NBUG(a,b) \
        { if (cs && (a!=b)) goto inconsistent; } 
 
 
         if(!type->pagesize) { 
                 uint8_t extid; 
                 u_long tmp; 
 
 
                 mnand_core_read_id(cs,5); 
 
 
                 extid = ((uint8_t*)g_chip.dma_area)[3]; 
                 NAND_DBG_PRINT_INF( "rcm_nand_read_id: flash ext_id 0x%02X\n", extid)
 
 
                 tmp = 1024 << (extid & 0x3); 
                 NBUG(g_chip.mtd.writesize, tmp); 
                 g_chip.mtd.writesize = tmp; 
 
 
                 extid >>= 2; 
 
 
                 tmp = (8 << (extid & 0x01)) * (g_chip.mtd.writesize >> 9); 
                 NBUG(g_chip.mtd.oobsize, tmp); 
                 g_chip.mtd.oobsize = tmp; 
 
 
                 extid >>= 2; 
                 tmp = (64 * 1024) << (extid & 0x03); 
                 NBUG(g_chip.mtd.erasesize, tmp); 
                 g_chip.mtd.erasesize = tmp; 
 
 
         } else { 
                 g_chip.mtd.erasesize = type->erasesize; 
                 g_chip.mtd.writesize = type->pagesize; 
                 g_chip.mtd.oobsize = g_chip.mtd.writesize / 32; 
         } 
 
 
        g_chip.mtd.writebufsize = g_chip.mtd.writesize; 
        g_chip.mtd.size += (uint64_t) type->chipsize << 20; 
        g_chip.chip_size[cs] = (uint64_t) type->chipsize << 20;

        //g_chip.mtd.ecc_strength = type->ecc.strength_ds;
        //g_chip.mtd.ecc_step_size = type->ecc.step_ds;

        NAND_DBG_PRINT_INF( "rcm_nand_read_id: %s,CS=%d,%s,size=%u,writesize=%u,oobsize=%u,erasesize=%u\n", 
                            g_chip.mtd.name,
                            cs,
                            vendor,
                            type->chipsize,
                            g_chip.mtd.writesize, 
                            g_chip.mtd.oobsize,
                            g_chip.mtd.erasesize )

        //NAND_DBG_PRINT_INF( "rcm_nand_read_id: ecc_strength=%u,ecc_step_size=%u\n",
        //                    g_chip.mtd.ecc_strength,
        //                    g_chip.mtd.ecc_step_size )
        return 0; 
 
 
 inconsistent: 
         NAND_DBG_PRINT_INF("rcm_nand_read_id: chip configurations differ, ignoring CS1\n"); 
         return -EIO; 
 } 
 
 
 static int mnand_erase(struct mtd_info* mtd, struct erase_info* instr) 
 { 
         int err; 

        // TRACE(KERN_DEBUG, "addr=0x%08llX, len=%lld\n", instr->addr, instr->len); 
        NAND_DBG_PRINT_INF( "mnand_erase: addr=0x%08llX, len=%lld\n", instr->addr, instr->len )

         if(instr->addr >= g_chip.mtd.size || 
             instr->addr + instr->len > g_chip.mtd.size  || 
             instr->len != g_chip.mtd.erasesize) {
                        NAND_DBG_PRINT_INF( "mnand_erase: bad parameters addr=0x%08llx,len=%08llx\n", instr->addr, instr->len )
                        return -EINVAL;
             }
 
 
         down(&g_mutex); 
 
 
         err = mnand_core_erase(instr->addr); 
 
 
         up(&g_mutex); 
 
 
         if(err) {  
                 //instr->state = MTD_ERASE_FAILED; 
                 instr->fail_addr = instr->addr; 
         } else { 
                 //instr->state = MTD_ERASE_DONE; 
          } 
 
        NAND_DBG_PRINT_INF( "mnand_erase: err=%d,addr=0x%08llx\n", err, instr->addr )
         //mtd_erase_callback(instr);  // implicit declaration of function 'mtd_erase_callback
         return err; 
 } 

 static uint8_t* mnand_fill_oob(uint8_t *oob, struct mtd_oob_ops *ops) 
 { 
         size_t len = ops->ooblen; 
 
 
         switch(ops->mode) { 
         case MTD_OPS_PLACE_OOB: 
         case MTD_OPS_RAW: 
                 memcpy(g_chip.dma_area + g_chip.mtd.writesize + ops->ooboffs, oob, len); 
                 return oob + len; 
         case MTD_OPS_AUTO_OOB: { 
                 struct nand_oobfree *free = g_ecclayout.oobfree;       // g_chip.mtd.ecclayout->oobfree; 
                 uint32_t boffs = 0, woffs = ops->ooboffs; 
                 size_t bytes = 0; 
 
 
                 for(; free->length && len; free++, len -= bytes) { 
                         /* Write request not from offset 0 ? */ 
                         if (unlikely(woffs)) { 
                                 if (woffs >= free->length) { 
                                         woffs -= free->length; 
                                         continue; 
                                 } 
                                 boffs = free->offset + woffs; 
                                 bytes = min_t(size_t, len, 
                                               (free->length - woffs)); 
                                 woffs = 0; 
                         } else { 
                                 bytes = min_t(size_t, len, free->length); 
                                 boffs = free->offset; 
                         } 
                         memcpy(g_chip.dma_area + g_chip.mtd.writesize + boffs, oob, bytes); 
                         oob += bytes; 
                 } 
                 return oob; 
         } 
         default: 
                 BUG(); 
         } 
         return NULL; 
 } 
 
 
 static uint8_t* mnand_transfer_oob(uint8_t *oob, struct mtd_oob_ops *ops, size_t len) 
 { 
         switch(ops->mode) { 
 
 
         case MTD_OPS_PLACE_OOB: 
         case MTD_OPS_RAW: 
                 TRACE(KERN_DEBUG, "raw transfer\n"); 
                 memcpy(oob, g_chip.dma_area + g_chip.mtd.writesize + ops->ooboffs, len); 
                 return oob + len; 
 
 
         case MTD_OPS_AUTO_OOB: { 
                 struct nand_oobfree *free = g_ecclayout.oobfree; // g_chip.mtd.ecclayout->oobfree; 
                 uint32_t boffs = 0, roffs = ops->ooboffs; 
                 size_t bytes = 0; 
 
 
                 for(; free->length && len; free++, len -= bytes) { 
                         /* Read request not from offset 0 ? */ 
                         if (unlikely(roffs)) { 
                                 if (roffs >= free->length) { 
                                         roffs -= free->length; 
                                         continue; 
                                 } 
                                 boffs = free->offset + roffs; 
                                 bytes = min_t(size_t, len, 
                                               (free->length - roffs)); 
                                 roffs = 0; 
                         } else { 
                                 bytes = min_t(size_t, len, free->length); 
                                 boffs = free->offset; 
                         } 
                         memcpy(oob, g_chip.dma_area + g_chip.mtd.writesize + boffs, bytes); 
                         oob += bytes; 
                 } 
                 return oob; 
         } 
         default: 
                 BUG(); 
         } 
         return NULL; 
 } 

static int mnand_write_oob( struct mtd_info* mtd, loff_t to, struct mtd_oob_ops* ops )
{
        uint8_t* data = ops->datbuf;
        uint8_t* dataend = data ? data + ops->len : 0;
        uint8_t* oob = ops->oobbuf;
        uint8_t* oobend = oob ? oob + ops->ooblen : 0;
        int err; 

        NAND_DBG_PRINT_INF( "mnand_write_oob: to=0x%08llX, ops.mode=%d, ops.len=%d, ops.ooblen=%d ops.ooboffs=0x%08X\n",
                            to, ops->mode, ops->len, ops->ooblen, ops->ooboffs);

         ops->retlen = 0;
         ops->oobretlen = 0;
 
         if( to >= g_chip.mtd.size || !NAND_PAGE_ALIGNED(to) || !NAND_PAGE_ALIGNED(dataend - data) ) {
                NAND_DBG_PRINT_ERR( "mnand_write_oob: writing non page aligned data to 0x%08llx 0x%08x\n", to, ops->len )
                return -EINVAL;
         }

         if( ops->ooboffs > mtd->oobsize ) { 
                NAND_DBG_PRINT_ERR( "mnand_write_oob: oob > mtd->oobsize\n" ) 
                return -EINVAL; 
         } 

         err = down_killable( &g_mutex ); 

         if( 0 == err ) { 
                for(; (data !=dataend) || (oob !=oobend); ) { 
                        memset(g_chip.dma_area, 0xFF, g_chip.mtd.writesize + g_chip.mtd.oobsize); 

                        if( data != dataend ) { 
                                memcpy( g_chip.dma_area, data, g_chip.mtd.writesize ); 
                                data += g_chip.mtd.writesize; 
                        } 
/*
                         if(oob != oobend) { 
                                 oob = mnand_fill_oob(oob, ops); 
                                 if(!oob) { 
                                         err = -EINVAL; 
                                         printk(KERN_ERR "oob einval\n"); 
                                         break; 
                                 } 
                         } 
*/
                         err = mnand_core_write(to); 
                         if( err )
                                break; 
                         to += g_chip.mtd.writesize; 
                 } 
                 up( &g_mutex ); 
         } 

        if( err ) { 
                NAND_DBG_PRINT_ERR( "mnand_write_oob: core write returned error at 0x%08llx\n", to ) 
                return err; 
        } 

         ops->retlen = ops->len; 
         ops->oobretlen = ops->ooblen; 
         return err; 
 } 
 
 
static int mnand_read_oob( struct mtd_info* mtd, loff_t from, struct mtd_oob_ops* ops )
{ 
        uint8_t* data = ops->datbuf;
        //uint8_t* oob = ops->oobbuf;
        int err;
        uint8_t* dataend = data + ops->len;
        //uint8_t* oobend = oob ? oob + ops->ooblen : 0;
/*
        TRACE( KERN_DEBUG,
               "from=0x%08llX, ops.mode=%d, ops.len=%d, ops.ooblen=%d ops.ooboffs=0x%08X data=%p\n",
               from, ops->mode, ops->len, ops->ooblen, ops->ooboffs, data);
        TRACE(KERN_DEBUG, "oob %p, oobend %p\n", oob, oobend);
*/

         if( ops->len != 0 && data == 0 ) { 
                 ops->len = 0;
                 dataend = 0;
         } 

         ops->retlen = 0;
         ops->oobretlen = 0;

         err = down_killable(&g_mutex);

         if(0 == err) {
                for(;;) {
                        err = mnand_core_read( from );
                        //PRINT_BUF_512( ((uint8_t*)g_chip.dma_area) )
                        if(err != 0) { 
                                 break; 
                        } 
                        if( data != dataend )
                        { 
                                size_t shift = (from & (mtd->writesize -1));
                                size_t bytes = min_t(size_t,dataend-data, mtd->writesize-shift);

                                NAND_DBG_PRINT_INF( "mnand_read_oob: data=%p,dataend=%p,shift=0x%X,bytes=0x%X\n", data, dataend, shift, bytes )
                                memcpy( data, g_chip.dma_area + shift, bytes ); 
                                data += bytes;
                         } 
/*
                         if(oob != oobend) {
                                 TRACE(KERN_DEBUG, "oob %p, oobend %p\n", oob, oobend);
                                 oob = mnand_transfer_oob(oob, ops, oobend-oob);
                                 if(!oob) {
                                         err = -EINVAL;
                                         break;
                                 }
                         }
*/
                         if( data == dataend )//&& oob == oobend)
                                 break;

                         from &= ~((1 << mtd->writesize_shift)-1);
                         from += mtd->writesize;
                 } 
 /*
                 if(data) { 
                         if(g_chip.mtd.ecc_stats.failed != g_chip.ecc_failed) { 
                                 g_chip.mtd.ecc_stats.failed = g_chip.ecc_failed; 
                                 g_chip.mtd.ecc_stats.corrected = g_chip.ecc_corrected; 
                                 err = -EBADMSG; 
                         } else if(g_chip.mtd.ecc_stats.corrected != g_chip.ecc_corrected) { 
                                 g_chip.mtd.ecc_stats.failed = g_chip.ecc_failed; 
                                 g_chip.mtd.ecc_stats.corrected = g_chip.ecc_corrected; 
                                 err = -EUCLEAN; 
                         } 
                 } 
 */
                  up( &g_mutex );
        } 

        if( err==0 || err == -EUCLEAN ) { 
                ops->retlen = ops->len; 
                ops->oobretlen = ops->ooblen; 
        } 
        return err; 
 } 
 
 
 static int mnand_isbad(struct mtd_info* mtd, loff_t off) 
 { 
         uint8_t f=0; 
         int err; 
         struct mtd_oob_ops ops = { 
                 .mode = MTD_OPS_RAW, 
                 .ooblen = 1, 
                 .oobbuf = &f, 
                 .ooboffs = 0 
         }; 
 
 
         TRACE(KERN_DEBUG, "off 0x%08llX\n", off); 
 
 
         err = mnand_read_oob(mtd, off, &ops); 
 
 
         TRACE(KERN_DEBUG,"err %d, f=%0x02X\n", err, f); 
 
 
         return (err == 0 && f == 0xFF) ? 0 : 1; 
 } 
 
 
 static int mnand_markbad(struct mtd_info* mtd, loff_t off) 
 { 
         uint8_t f=0; 
         struct mtd_oob_ops ops = { 
                 .mode = MTD_OPS_RAW, 
                 .ooblen = 1, 
                 .oobbuf = &f, 
                 .ooboffs = 0 
         }; 
 
 
         TRACE(KERN_DEBUG, "off 0x%08llX\n", off); 
 
 
         return mnand_write_oob(mtd, off, &ops); 
 } 
 
 
 static int mnand_write(struct mtd_info* mtd, loff_t off, size_t len, size_t* retlen, const u_char* buf) 
 { 
         struct mtd_oob_ops ops = { 
                 .datbuf  = (uint8_t*)buf, 
                 .len = len 
         };
         int err = -EINVAL;
         if( buf ) {
                err = mnand_write_oob( mtd, off, &ops ); 
                *retlen = ops.retlen;
         }
         else {
                *retlen = 0;
         }
         NAND_DBG_PRINT_INF( "mnand_write: off=0x%08llX,len=%zu,err=%d,retlen=%zu\n", off, len, err, *retlen )
         return err; 
 } 

 
 static int mnand_read(struct mtd_info* mtd, loff_t off, size_t len, size_t* retlen, u_char* buf)
 { 
        struct mtd_oob_ops ops = {
                .datbuf = buf,
                .len = len,
        };
        int err = -EINVAL; 
        if( buf ) {
                 err = mnand_read_oob( mtd, off, &ops );
                *retlen = ops.retlen;
        }
        else {
                *retlen = 0;
        }
        NAND_DBG_PRINT_INF( "mnand_read: off=0x%08llX,len=%zu,err=%d,retlen=%zu\n", off, len, err, *retlen )
        return err;
 } 

static int rcm_nand_lsif0_config( struct device_node *of_node )
{
    struct device_node* tmp_of_node;
   	struct regmap* lsif0_config;
    int ret;
    //uint32_t reg[2] = { 0 };

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

    //if( !regmap_read( lsif0_config, NAND_RADDR_EXTEND, &reg[0] ) && 
    //    !regmap_read( lsif0_config, NAND_WADDR_EXTEND, &reg[1] ) ) {
    //    NAND_DBG_PRINT_INF( "lsif0_config: RADDR %08x,WADDR %08x", reg[0], reg[1] )
    //}
    return 0;
}

static int rcm_nand_get_timings( struct device_node *of_node, uint32_t* timings, uint32_t* freq )
{
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

static int rcm_nand_probe(struct platform_device* ofdev) 
{
        struct erase_info instr = { 0, 1, 0 };
        const struct of_device_id *match;
        struct device_node *of_node = ofdev->dev.of_node;
        const char *part_probes[] = { "cmdlinepart", NULL, };
        uint32_t freq, timings[30];
        int err;

        match = of_match_device( of_platform_nand_table, &ofdev->dev );
        if( !match ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_probe: of_match_device: failed\n" )
                return -EINVAL;
        }

        memset( &g_chip, 0, sizeof(g_chip) );

        g_chip.dev = ofdev;
        g_chip.active_page = -1;
        g_chip.regs.io = of_iomap( of_node, 0 );
        if ( WARN_ON( !g_chip.regs.io ) ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_probe: of_iomap: failed\n" )
                return -EIO;
        }

        if( ( err = rcm_nand_lsif0_config( of_node ) ) != 0 ) {
                return -EIO;
        }

        g_chip.irq = irq_of_parse_and_map( of_node, 0 );
        NAND_DBG_PRINT_INF( "rcm_nand_probe: irq_of_parse_and_map: return %u\n", g_chip.irq )
        if( ( err = request_irq( g_chip.irq, rcm_nand_interrupt_handler, IRQF_SHARED, DRIVER_NAME, &g_chip ) ) ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_probe: failed to set handler on irq #%d (code: %d)\n", g_chip.irq, err );
                goto error_irq;
        }

        g_chip.dev->dev.coherent_dma_mask = DMA_BIT_MASK( 32 );
        g_chip.dma_area = dma_alloc_coherent( &g_chip.dev->dev, 4096, &g_chip.dma_handle, GFP_KERNEL | GFP_DMA );
        if( !g_chip.dma_area ) {
                NAND_DBG_PRINT_ERR( "rcm_nand_probe: failed to request DMA area\n" )
                err = -ENOMEM;
                goto error_dma; 
        }
        NAND_DBG_PRINT_INF( "dma_alloc_coherent: area=%px,hanlde=%08x\n", g_chip.dma_area, (uint32_t)g_chip.dma_handle )
        memset( g_chip.dma_area, 0xFF, 4096 );

        sema_init( &g_mutex, 1 ); 
        down( &g_mutex );

        if( ( err = rcm_nand_get_timings( of_node, timings, &freq ) ) != 0 ) {
                goto error_dma;
        }

        if( ( err = rcm_nand_hw_init( timings, freq ) ) != 0 ) {
                goto error_dma;
        }

        // mnand_reset_grabber(); 
        rcm_nand_core_reset( 0 );
        rcm_nand_core_reset( 1 );

        g_chip.mtd.name = DRIVER_NAME;
        g_chip.mtd.size = 0;
        g_chip.chip_size[0] = 0;
        g_chip.chip_size[1] = 0;

        err = rcm_nand_read_id(0);
        err = rcm_nand_read_id(1);

        up( &g_mutex );

        g_chip.mtd.owner = THIS_MODULE;
        g_chip.mtd.type = MTD_NANDFLASH;
        g_chip.mtd.flags = MTD_WRITEABLE;
        g_chip.mtd._erase = mnand_erase;
        g_chip.mtd._read = mnand_read;
        g_chip.mtd._write = mnand_write;
        g_chip.mtd._block_isbad = mnand_isbad;
        g_chip.mtd._block_markbad = mnand_markbad;
        g_chip.mtd._read_oob = mnand_read_oob;
        g_chip.mtd._write_oob = mnand_write_oob;
        //g_chip.mtd.ecclayout = &g_ecclayout;
        g_chip.mtd.dev.parent = &ofdev->dev;

        NAND_DBG_PRINT_INF( "rcm_nand_probe: detected %llu bytes of NAND memory\n", g_chip.mtd.size )

        instr.len = g_chip.mtd.erasesize;
        mnand_erase( &g_chip.mtd, &instr );

        err = mtd_device_parse_register( &g_chip.mtd, part_probes, 0, NULL, 0 );
        if( err ) { 
                NAND_DBG_PRINT_ERR( "rcm_nand_probe: failed add mtd device (code: %d)\n", err );
                goto error_dma;
        } 
        return 0;
  
 error_dma: 
         dma_free_coherent(&g_chip.dev->dev, 4096, g_chip.dma_area, g_chip.dma_handle);
 error_irq: 
         free_irq(g_chip.irq, &g_chip);
         g_chip.irq = 0;
         return err;
}

static int rcm_nand_remove( struct platform_device* ofdev )
{ 
        struct mem_resource* ret = &g_chip.regs;
        mtd_device_unregister( &g_chip.mtd ); 
        dma_free_coherent( &g_chip.dev->dev, 4096, g_chip.dma_area, g_chip.dma_handle );
        free_irq( g_chip.irq, &g_chip );
        iounmap( ret->io );
        NAND_DBG_PRINT_INF( "rcm_nand_remove: releasing mem region(0x%08X,0x%08X)\n",
                            (uint32_t)ret->res->start,
                            (uint32_t)(ret->res->end - ret->res->start+1) )
        release_mem_region( ret->res->start, ret->res->end - ret->res->start + 1);
        return 0; 
}


static struct of_device_id of_platform_nand_table[] = {
        { .compatible = "rcm,nand" }, 
        { /* end of list */ }, 
}; 


static struct platform_driver of_platform_mnand_driver = {
        .driver = { 
                .name = DRIVER_NAME, 
                .owner = THIS_MODULE, 
                .of_match_table = of_platform_nand_table,
        }, 
        .probe = rcm_nand_probe,
        .remove = rcm_nand_remove 
}; 


module_platform_driver(of_platform_mnand_driver); 


MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("Shalyt Vladimir Vladimir.Shalyt@astrosoft.ru"); 
MODULE_DESCRIPTION("RCM SoC NAND controller driver"); 

