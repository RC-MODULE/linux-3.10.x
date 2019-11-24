#include <linux/types.h>
#include <linux/compiler_types.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/dmaengine.h>
#include <linux/mtd/cfi.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/export.h>
#include <linux/irqreturn.h>

#define DRIVER_NAME "rcm-nand"              // hisi504_nand

#define NAND_DBG_PRINT

#ifdef NAND_DBG_PRINT
    #define NAND_DBG_PRINT_INF(...) printk("RCM_NAND[INFO] " __VA_ARGS__);
    #define NAND_DBG_PRINT_WRN(...) printk("RCM_NAND[WARN] " __VA_ARGS__);
    #define NAND_DBG_PRINT_ERR(...) printk("RCM_NAND[ERR] " __VA_ARGS__);
#else
    #define NAND_DBG_PRINT_INF(...) while(0);
    #define NAND_DBG_PRINT_WRN(...) while(0);
    #define NAND_DBG_PRINT_ERR(...) while(0);
#endif

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
// NAND_REG_control-сдвиг битов
#define OP_BEGIN_SHIFT                  0           // 0-флаг начала операции (1-начать)
#define PAGE_SIZE_SHIFT                 1           // 3..1-размер страницы (0-512..7-65536)
#define NUM_ADR_BYTES_SHIFT             4           // 6..4-количество байтов (циклов) адреса в текущей команде
#define ECC_MODE_SHIFT                  7           // 8..7-режим коррекции ошибок
#define CE_SHIFT                        9           // 9-номер кристалла для текущей операции
#define	HWP_SHIFT                       11          // 11-аппаратная защита от записи(0-запись разрешена?)
#define OOB_ECC_SHIFT                   12          // 12-разрешение оценки кодов коррекции ошибок для данных в поле OOB(рекомендуется 1)
#define OOB_SIZE_SHIFT                  13          // 23..13-команда
#define	FCMD_SHIFT                      24          // 31..24-первая команда для отправки во флэш память
// NAND_REG_cntrl_dma_w-сдвиг битов
#define SEGM_SIZE_SHIFT                 5           // 28..5-размер сегмента (байты–1),по достижению конца сгенерируется прерывания
#define START_ADDR_GEN                  (1<<31)     // 31-после задания значения начать генерировать адреса с начального по конечный
// NAND_REG_irq_mask_nand-биты (Таблица 1.829)
#define IRQ_READ_FINISH                 (1 << 0)    // прерывание по завершении выполнения команды чтения
#define IRQ_READID                      (1 << 1)    // прерывание по завершении выполнения команды считывания идентификационного номера
#define IRQ_PROGRAM_FINISH              (1 << 2)    // прерывание по завершении выполнения команды программирования
#define IRQ_ERASE                       (1 << 3)    // прерывание по завершении выполнения команды стирания блока
#define IRQ_RESET                       (1 << 4)    // прерывание по завершении выполнения команды сброса флэш-памяти NAND
#define IRQ_UNCORR_ERROR                (1 << 5)    // прерывание при возникновении некорректируемой ошибки
//#define IRQ_AXI_WRITE                  (1 << 6)  // ошибка? nand_impl.c
#define IRQ_AXIW_ERROR                  (1 << 6)    // прерывание при возникновении ошибочной транзакции по интерфейсу записи AXI (неверное значение BRESP)
//???????????????????
#define IRQ_LAST_ADR_AXIW               (1 << 7)    // прерывание по достижению последнего адреса ведущего интерфейса записи AXI
#define IRQ_LAST_SEGM_AXIW              (1 << 8)    // прерывание по достижению последнего сегмента адреса ведущего интерфейса записи AXI
#define IRQ_AXIR_ERROR                  (1 << 9)    // прерывание при возникновении ошибочной транзакции по шине чтения AXI(неверное значение RRESP)
#define IRQ_LAST_ADR_AXIR               (1 << 10)   // прерывание по достижению последнего адреса ведущего интерфейса чтения AXI
#define IRQ_LAST_SEGM_AXIR              (1 << 11)   // прерывание по достижению последнего сегмента адреса ведущего интерфейса чтения
// NAND_REG_command - команды
#define CMD_CONTROL_READ                    0x01        // чтение NAND
#define CMD_CONTROL_READ_ID                 0x03        // чтение идентификатора
#define CMD_PROGRAM_TO_NAND	                0x04        // программирование NAND
#define CMD_ERASE_BLOCK		                0x06        // стирание блока
#define CMD_CONTROL_RESET                   0x07        // сброс флэш памяти NAND
// команды ONFI FLASH (см.стандарт)
#define CMD_FLASH_PAGE_READ                 0x00
#define CMD_FLASH_BLOCK_ERASE               0x60
#define CMD_FLASH_PROGRAM_PAGE              0x80
#define CMD_FLASH_READ_ID                   0x90
#define CMD_NAND_ONFI                       0xEC        // Read Parameter Page
// NAND_REG_status-биты
#define STATUS_CONT_READY                   0x00000200  // статус: 1-контроллер NAND готов к приему команды
#define STATUS_NAND_READY                   0x00000400  // статус: 1-флэш-память готова к выполнению операций
// разное
#define NAND_TIMEOUT                        10000
#define NAND_RESET_TIMEOUT                  8000
#define NAND_ONFI_SIZE                      128
#define NAND_PAGE_SIZE_BASE                 512
#define NAND_DEFAULT_CHIP                   0
// идентификаторы
#define READ_ID_JEDEC_ADDR                  0x00        // стандартизацией технологий компьютерной памяти занимается комитет JEDEC, образованный при ассоциации EIA
#define READ_ID_ONFI_ADDR                   0x20        // но существует сторонняя организация – Open NAND Flash Interface Working Group (ONFI) – деятельность которой сосредоточена на разработке открытых стандартов низкоуровневых интерфейсов
// макросы для расчета таймингов
#define NAND_DIV_ROUND_UP(x, y) (1 + (((x) - 1) / (y)))
#define NAND_TIMING_TO_CLOCKS(t,f) ((NAND_DIV_ROUND_UP((t+1)*f, 1000))  & 0xff)
// тайминги по умолчанию
#define  TADL_MIN_DEF   200
#define  TALH_MIN_DEF   20
#define  TALS_MIN_DEF   50
#define  TCH_MIN_DEF    20
#define  TCLH_MIN_DEF   20
#define  TCLS_MIN_DEF   50
#define  TCS_MIN_DEF    70
#define  TDH_MIN_DEF    20
#define  TDS_MIN_DEF    40
#define  TWC_MIN_DEF    100
#define  TWH_MIN_DEF    30
#define  TWP_MIN_DEF    50
#define  TWW_MIN_DEF    100
#define  TAR_MIN_DEF    25
#define  TCLR_MIN_DEF   20
#define  TCOH_MIN_DEF   0
#define  TIR_MIN_DEF    10
#define  TRC_MIN_DEF    100
#define  TREH_MIN_DEF   30
#define  TRHOH_MIN_DEF  0
#define  TRHW_MIN_DEF   200
#define  TRLOH_MIN_DEF  0
#define  TRP_MIN_DEF    50
#define  TRR_MIN_DEF    40
#define  TWHR_MIN_DEF   120
#define  TCEA_MAX_DEF   100
#define  TCHZ_MAX_DEF   100
#define  TREA_MAX_DEF   40
#define  TRHZ_MAX_DEF   200
#define  TWB_MAX_DEF    200
// режим коррекции ошибок
#define NAND_ECC_MODE_UNDEFINED (-1)
#define NAND_ECC_MODE_NO_ECC    0
#define NAND_ECC_MODE_4BITS     1
#define NAND_ECC_MODE_24BITS    2


// уточнить CMD address
//#define READ_ID_JEDEC_ADDR	0x00
//#define READ_ID_ONFI_ADDR	0x20

//#define NAND_IRQ_RESET                      0x00000010  // завершение выполнения команды сброса
//#define NAND_IRQ_END_READ                   0x00000001  // завершение выполнения команды чтения
//#define NAND_IRQ_END_READ_ID                0x00000002  // завершение выполнения команды считывания идентификационного номера
//#define NAND_IRQ_LAST_BUF_ADDR              0x00000080  // последний адрес состояния ведущего интерфейса записи AXI
//#define NAND_IRQ_LAST_ADDR_AXI_W            0x00000100  // последний сегмент адреса состояния ведущего интерфейса записи AXI 

//#define NAND_DMA_CONT_READ                  0x80000000  // 0xC00001E0 //size of buf's seg  = 16 -1 = 15 [25:5],   [31:30] - init bits


//#define NAND_CI_CELLTYPE_MSK                0xC rawnand.h
//#define NAND_MFR_SAMSUNG                    0xEC
//#define NAND_MFR_AMD                        0x01

//#define NAND_CONTROL_START                  0x01        // start
//#define NAND_CONTROL_C_B_ADDR_5             0x5

struct nand_info{
    u32 page_size_code;                 // код размера страницы
    u32 pagesize;                       // размер страницы в байтах
    u32 ecc_mode;                       // режим коррекции ошибок
    u32 sparesize;                      // дополнительная область для корректирующих кодов и метаданных
    u32 blocksize;                      // страниц на блок
    bool oob_ecc_enabled;               // разрешение сохранения/проверки кодов ecc for OOB
	u32 oob_size;                       // размер OOB
	u8 manufacturer[12];
    u32 timings[30];
};

struct rcm_nand_host
{
    struct device* dev;
    void* iobase;
    struct mtd_info mtd_info;           // структура для регистрации
    struct nand_info nand_info;         // параметры микросхемы памяти
};

static u32 lsif_reg_readl( void *base, u32 offset )
{
    struct rcm_nand_host* rcm_nand_host = (struct rcm_nand_host*)base;
    return readl( rcm_nand_host->iobase + offset );
}

static void lsif_reg_writel( void *base, u32 offset, u32 val )
{
    struct rcm_nand_host* rcm_nand_host = (struct rcm_nand_host*)base;
    writel( val, rcm_nand_host->iobase + offset );
}

static bool rcm_nand_wait_int( void* base, u32 mask )
{
    u32 timeout = NAND_TIMEOUT;
    while( --timeout )
    {
        if( lsif_reg_readl( base, NAND_REG_irq_status )  & mask ) return true;
    }
    return false;
}

static inline bool rcm_nand_is_ready( void* base )
{
    return lsif_reg_readl( base,
                           NAND_REG_status ) & ( STATUS_CONT_READY | STATUS_NAND_READY ) ? true : false;
}

static bool rcm_nand_wait_ready( void* base )
{
    u32 timeout = NAND_TIMEOUT;
    while( --timeout )
    {
        if( rcm_nand_is_ready( base ) ) return true;
    }
    return false;
}

inline void rcm_nand_update_control( void* base,
                                     u32 page_size,
                                     u32 oob_size,
                                     u32 op_begin,
                                     u32 num_adr_bytes, 
                                     u32 ecc_mode, 
                                     u32 hw_w_protect,
                                     u32 oob_ecc,
                                     u32 command )
{
    lsif_reg_writel( base,
                     NAND_REG_control,
                    ( ( op_begin << OP_BEGIN_SHIFT ) |
                      ( page_size << PAGE_SIZE_SHIFT ) |
                      ( num_adr_bytes << NUM_ADR_BYTES_SHIFT ) |
                      ( ecc_mode << ECC_MODE_SHIFT ) |
                      ( NAND_DEFAULT_CHIP << CE_SHIFT ) |
                      ( hw_w_protect << HWP_SHIFT ) |
                      ( oob_ecc << OOB_ECC_SHIFT ) |
                      ( oob_size << OOB_SIZE_SHIFT ) |
                      ( command << FCMD_SHIFT ) ) );
}

static bool rcm_nand_flash_reset( void* base )
{
    lsif_reg_writel( base, NAND_REG_irq_mask_nand, IRQ_RESET );
    lsif_reg_writel( base, NAND_REG_command, CMD_CONTROL_RESET );
    rcm_nand_update_control( base,
                             0,                                     // размер страницы
                             0,                                     // размер запасной области
                             1,                                     // начать операцию сразу
                             0,                                     // количество адресных циклов
                             NAND_ECC_MODE_NO_ECC,                  // режим коррекции ошибок
                             0,                                     // защита от записи
                             0,                                     // разрешение оценки кодов коррекции
                             0 );                                   // первая отправляемая  во флэш команда
    if( !rcm_nand_wait_int( base, IRQ_RESET ) ) return false;
    return rcm_nand_wait_ready( base );
}

static void rcm_nand_dma_config( void* base, u32 start_addr, u32 end_addr, u32 segment_size, int write )
{
    lsif_reg_writel( base, NAND_REG_awlen_max, 0xF );               // рекомендуемое значение: 0xF (таблица 1.845)
    lsif_reg_writel( base, NAND_REG_msb_lsbw, 0x0 );                // порядок следования байтов ведущего интерфейса записи AXI (1-big_endian,0-little endian)
    if( write )
    {                                                               // запись
        lsif_reg_writel( base, NAND_REG_start_dma_w, (u32)start_addr );
        lsif_reg_writel( base, NAND_REG_end_dma_w, end_addr );
        lsif_reg_writel( base,                                      // 28..5-размер сегмента-1, 31-генерация адресов
                         NAND_REG_cntrl_dma_w,
                         START_ADDR_GEN | ( segment_size << SEGM_SIZE_SHIFT ) );
	}
    else
    {                                                               // чтение
        lsif_reg_writel( base, NAND_REG_start_dma_r, (u32)start_addr );
        lsif_reg_writel( base, NAND_REG_end_dma_r, end_addr );
        lsif_reg_writel( base,                                      // 28..5-размер сегмента-1, 30-чтение регистра current_dma_r, 31-генерация адресов
                         NAND_REG_cntrl_dma_r,
                         START_ADDR_GEN | ( segment_size << SEGM_SIZE_SHIFT ) );
    }
}

static bool rcm_nand_readid( void* base, u8 cmd_addr, u8* buffer )  // 4-х байтовый идентификатор JEDECID/ONFI
{                                                                   // cmd_addr равен 0x00 для JEDEC, 0x20 для ONFI
    rcm_nand_dma_config( base, (u32)buffer, (u32)buffer + 256, 64, true );
    lsif_reg_writel( base, NAND_REG_irq_mask_nand, IRQ_READID );
    lsif_reg_writel( base, NAND_REG_command, CMD_FLASH_READ_ID );
    lsif_reg_writel( base, NAND_REG_col_addr, cmd_addr );           // адрес для считывания идентификационного номера
    rcm_nand_update_control( base,
                             0,                                     // размер страницы
                             0,                                     // размер запасной области
                             1,                                     // начать операцию сразу
                             1,                                     // количество адресных циклов
                             NAND_ECC_MODE_NO_ECC,                  // режим коррекции ошибок
                             0,                                     // защита от записи
                             0,                                     // разрешение оценки кодов коррекции
                             CMD_CONTROL_READ_ID );                 // первая отправляемая во флэш команда
    return rcm_nand_wait_int( base, IRQ_READID );
}

static bool rcm_nand_read_page( void* base, struct nand_info* dev_info, u16 column_address, u16 row_address, u8 block_address, u32* buffer )
{
    u8 ecc_size_per_512 = ( dev_info->ecc_mode == NAND_ECC_MODE_24BITS ) ? 21 :
                          ( dev_info->ecc_mode == NAND_ECC_MODE_4BITS ) ? 7 : 0;            // размер ecc области на базовую страницу
    u32 ecc_size =	ecc_size_per_512 * (dev_info->pagesize/ 512);                           // полный размер ecc области

    if( !rcm_nand_wait_ready( base ) ) return false;
    lsif_reg_writel( base, NAND_REG_irq_mask_nand, IRQ_LAST_ADR_AXIW );                     // было IRQ_AXI_WRITE(/tests/src/lib/nand),видимо,ошибка
    rcm_nand_dma_config( base,
                         (u32)buffer,                                                       // стартовый адрес
                         (u32)buffer + dev_info->pagesize + dev_info->sparesize - ecc_size, // конечный адрес
                         0,                                                                 // размер сегмента-1,почему 0?
                         true );                                                            // запись (в DMA буфер)
    lsif_reg_writel( base, NAND_REG_command, CMD_CONTROL_READ );                            // чтение (из flash)
    lsif_reg_writel( base, NAND_REG_col_addr, column_address );                             // адрес столбца
    lsif_reg_writel( base,
                     NAND_REG_row_addr_read_d,
                     (block_address << 16) | (row_address << 0) );                          // адреса блока и ряда
    rcm_nand_update_control( base,
                             dev_info->page_size_code,                                      // размер страницы
                             dev_info->oob_size,                                            // размер запасной области
                             1,                                                             // начать операцию сразу
                             5,                                                             // 5 байтов (циклов) адреса в текущей команде
                             dev_info->ecc_mode,                                            // режим ecc
                             1,                                                             // hw write protect enabled (почему?)
                             dev_info->oob_ecc_enabled,                                     // oob_ecc enabled
                             CMD_FLASH_PAGE_READ );                                         // команда для flash
    return rcm_nand_wait_int( base, IRQ_LAST_ADR_AXIW );
}

static bool rcm_nand_program_page( void* base, struct nand_info* dev_info, u16 column_address, u16 row_address, u8 block_address, u32* buffer)
{
    u8 ecc_size_per_512 = ( dev_info->ecc_mode == NAND_ECC_MODE_24BITS ) ? 21 :
                          ( dev_info->ecc_mode == NAND_ECC_MODE_4BITS ) ? 7 : 0;            // размер ecc области на базовую страницу
    u32 ecc_size =	ecc_size_per_512 * (dev_info->pagesize/ 512);                           // полный размер ecc области

    if( !rcm_nand_wait_ready( base ) ) return false;
    lsif_reg_writel( base, NAND_REG_irq_mask_nand, IRQ_PROGRAM_FINISH );
    lsif_reg_writel( base, NAND_REG_cdb_buffer_type, 1 );                                   // при обработке последнего адреса производится остановка ведущего интерфейса чтения AXI
    rcm_nand_dma_config( base,
                         (u32)buffer,                                                       // стартовый адрес
                         (u32)buffer + dev_info->pagesize + dev_info->sparesize - ecc_size, // конечный адрес
                         0,                                                                 // размер сегмента-1,почему 0?
                         false );                                                           // чтение (из DMA буфера)
    lsif_reg_writel( base, NAND_REG_command,CMD_PROGRAM_TO_NAND );                          // программирование (flash)
    lsif_reg_writel( base, NAND_REG_col_addr, column_address );                             // адрес столбца
   	lsif_reg_writel( base,
                     NAND_REG_row_addr_read_d,
                     (block_address << 16) | (row_address << 0) );                          // адреса блока и ряда
    rcm_nand_update_control( base,
                             dev_info->page_size_code,                                      // размер страницы
                             dev_info->oob_size,                                            // размер запасной области
                             1,                                                             // начать операцию сразу
                             5,                                                             // 5 байтов (циклов) адреса в текущей команде
                             dev_info->ecc_mode,                                            // режим ecc
                             0,                                                             // no write protect at write
                             dev_info->oob_ecc_enabled,                                     // oob_ecc enabled
                             CMD_FLASH_PROGRAM_PAGE );                                      // команда для flash
    return rcm_nand_wait_int( base, IRQ_PROGRAM_FINISH );
}

static bool rcm_nand_erase_block( void* base, struct nand_info* dev_info, u32 block_address )
{
    if( !rcm_nand_wait_ready( base ) ) return false;
    lsif_reg_writel( base, NAND_REG_irq_mask_nand, IRQ_ERASE );
    lsif_reg_writel( base, NAND_REG_command, CMD_ERASE_BLOCK );                             // задаем код команды контроллеру
    lsif_reg_writel( base, NAND_REG_row_addr_read_d, block_address );                       // адрес строки + адрес блока
    rcm_nand_update_control( base,
                             0x0,                                                           // размер страницы
                             dev_info->oob_size,                                            // размер запасной области
                             1,                                                             // начать операцию сразу
                             5,                                                             // 5 байтов (циклов) адреса в текущей команде
                             NAND_ECC_MODE_NO_ECC,                                          // режим ecc
                             0,                                                             // disable wprotect
                             0,                                                             // disable ecc for OOB
                             CMD_FLASH_BLOCK_ERASE );                                       // было 0x0 в файле nand_impl,ошибка?
    return rcm_nand_wait_int( base, IRQ_ERASE );
}

static void rcm_nand_set_timing( void* base, u32 offset, u32* timings, unsigned cnt, u32 freq )
{
    unsigned i;
    u32 timing_reg = 0, n = 0;
    for( i=0; i<cnt; i++ )
    {
        timing_reg |= ( NAND_TIMING_TO_CLOCKS( timings[i], freq ) << n );
        n += 8;
    }
    lsif_reg_writel( base, offset, timing_reg );
    NAND_DBG_PRINT_INF( "rcm_nand_set_timing: reg=%02x,data=%08x", offset, timing_reg )
}

static void rcm_nand_set_timings( void* base, u32* timings, u32 freq )
{
    rcm_nand_set_timing( base, NAND_REG_timing_0, timings+0, 4, freq );         // tADL_min, tALH_min, tALS_min, tCH_min
    rcm_nand_set_timing( base, NAND_REG_timing_1, timings+4, 4, freq );         // tCLH_min, tCLS_min, tCS_min, tDH_min
    rcm_nand_set_timing( base, NAND_REG_timing_2, timings+8, 4, freq );         // tDS_min, tWC_min, tWH_min, tWP_min
    rcm_nand_set_timing( base, NAND_REG_timing_3, timings+12, 3, freq );        // tWW_min, tAR_min, tCLR_min
    rcm_nand_set_timing( base, NAND_REG_timing_4, timings+15, 4, freq );        // tCOH_min, tIR_min, tRC_min, tREH_min
    rcm_nand_set_timing( base, NAND_REG_timing_5, timings+19, 4, freq );        // tRHOH_min, tRHW_min, tRLOH_min, tRP_min
    timings[25] += 2;
    rcm_nand_set_timing( base, NAND_REG_timing_6, timings+23, 4, freq );        // tRR_min, tWHR_min, tCEA_max+2, tCHZ_max
    timings[27] += 2;
    timings[29] += 2;
    rcm_nand_set_timing( base, NAND_REG_timing_7, timings+27, 3, freq );        // tREA_max+2, tRHZ_max, tWB_max+2
}

static u32 get_pagesize_code( u32 size )
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
        NAND_DBG_PRINT_ERR( "get_pagesize_code: code size error (%u)", size )
        return 0;
    }
}

static bool rcm_nand_soft_reset( void* base )
{
    lsif_reg_writel( base, NAND_REG_cntrl_sw_rst, 0x0);                  // запись 0 в данный регистр инициирует программный сброс контроллера
    lsif_reg_writel( base, NAND_REG_sw_rst, 0x0 );                       // запись 0,затем 1 в данный регистр инициирует
    lsif_reg_writel( base, NAND_REG_sw_rst, 0x1 );                       // программный сброс ведущего интерфейса записи AXI 
    lsif_reg_writel( base, NAND_REG_sw_rstn_r, 0x1 );                    // программный сброс ведущего интерфейса чтения AXI, автоматически обнуляется
    return rcm_nand_wait_ready( base );
}

static bool rcm_nand_get_dev_info( void* base, struct nand_info * dev_info )
{
    u8 nand_onfi[256];                                                  // ONFI buffer

    if( rcm_nand_readid( base, READ_ID_ONFI_ADDR, nand_onfi ) == false )
    {
        NAND_DBG_PRINT_ERR( "rcm_nand_get_dev_info: cannot read nand onfi id\n" )
        return false;
    }

    if( nand_onfi[0] == 'O' && nand_onfi[1] == 'N' && nand_onfi[2] == 'F' && nand_onfi[3] == 'I' )
    {
        dev_info->pagesize = ( nand_onfi[83] << 24 | nand_onfi[82] << 16 | nand_onfi[81] << 8 | nand_onfi[80] );    // размер страницы
        dev_info->page_size_code = get_pagesize_code( dev_info->pagesize );                                         // код размера страницы
        dev_info->blocksize = ( nand_onfi[95] << 24 | nand_onfi[94] << 16 | nand_onfi[93] << 8 | nand_onfi[92] );   // размер блока
        dev_info->sparesize = ( nand_onfi[85] << 8 | nand_onfi[84] );                                               // размер дополнительной области
        dev_info->ecc_mode = ( nand_onfi[112] == 24 ) ? NAND_ECC_MODE_24BITS :                                      // режим коррекции ошибок
                             ( nand_onfi[112] == 4 ) ? NAND_ECC_MODE_4BITS :
                             ( nand_onfi[112] == 0 ) ? NAND_ECC_MODE_NO_ECC : NAND_ECC_MODE_UNDEFINED;
        dev_info->manufacturer[0] = nand_onfi[32];
        dev_info->manufacturer[1] = nand_onfi[33];
        dev_info->manufacturer[2] = nand_onfi[34];
        dev_info->manufacturer[3] = nand_onfi[35];
        dev_info->manufacturer[4] = nand_onfi[36];
        dev_info->manufacturer[5] = nand_onfi[37];
        dev_info->manufacturer[6] = nand_onfi[38];
        dev_info->manufacturer[7] = nand_onfi[39];
        dev_info->manufacturer[8] = nand_onfi[40];
        dev_info->manufacturer[9] = nand_onfi[41];
        dev_info->manufacturer[10] = nand_onfi[42];
        dev_info->manufacturer[11] = nand_onfi[43];

        NAND_DBG_PRINT_INF( "Flash: pagesize=%u,page_size_code=%u,blocksize=%u,sparesize=%u,ecc_mode=%u",
                            dev_info->pagesize,
                            dev_info->page_size_code,
                            dev_info->blocksize,
                            dev_info->sparesize,
                            dev_info->ecc_mode )
        NAND_DBG_PRINT_INF( "Flash: manufacturer=%c%c%c%c%c%c%c%c%c%c%c%c",
                            dev_info->manufacturer[0], dev_info->manufacturer[1], dev_info->manufacturer[2], dev_info->manufacturer[3],
                            dev_info->manufacturer[4], dev_info->manufacturer[5], dev_info->manufacturer[6], dev_info->manufacturer[7],
                            dev_info->manufacturer[8], dev_info->manufacturer[9], dev_info->manufacturer[10], dev_info->manufacturer[11] );
    }
    else
    {
        NAND_DBG_PRINT_ERR( "rcm_nand_get_dev_info: non onfi id")
        return false;
    }
    dev_info->oob_ecc_enabled = true;                               // non-onfi параметры...вопрос
    dev_info->oob_size = 0;                                         // Filesystem and other...
    return true;
}

int rcm_nand_init( void* base, struct nand_info * dev_info, u32* timings, u32 freq )
{
    if( !rcm_nand_soft_reset( base ) )              // сброс контроллера
    {
        NAND_DBG_PRINT_ERR( "rcm_nand_soft_reset: ERROR" )
        return EIO;
    }
    rcm_nand_set_timings( base, timings, freq );
    if( !rcm_nand_flash_reset( base ) )             // сброс микросхемы
    {
        NAND_DBG_PRINT_ERR( "rcm_nand_flash_reset: ERROR");
        return EIO;
    }
    if( !rcm_nand_wait_ready( base ) )              // ожидание готовности
    {
		NAND_DBG_PRINT_ERR( "rcm_nand_wait_ready: NAND not ready" );
        return EIO;
    }
    if( !rcm_nand_get_dev_info( base, dev_info ) )  // чтение параметров микросхемы
    {
        NAND_DBG_PRINT_ERR( "nand_get_dev_info: ERROR" );
        return EIO;
    }
    return 0;
}

int rcm_nand_read( struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf )
{   // предполагается, что loff_t это смещение относительно начала памяти микросхемы
/*
    struct rcm_nand_host* rcm_nand_host = (struct rcm_nand_host*)mtd->priv;
    struct nand_info* nand = &rcm_nand_host->nand_info;

    unsigned start_page = 1;
    unsigned num_page = 1;
    unsigned cur_page = 0;
    unsigned block;

    while( nand->pagesize * start_page <= from ) start_page++;
    while( nand->pagesize * num_page < len ) num_page++;

 
    for ( cur_page = start_page; cur_page < start_page + num_page; cur_page++ )
    {
        if( !rcm_nand_read_page( rcm_nand_host,
                                 nand,
                                 0,
                                 cur_page,
                                 0,
                                 buf ) ) return -1;
        *retlen += nand->pagesize;
    }
    return 0;*/
}

int rcm_nand_write( struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf )
{
    struct rcm_nand_host* rcm_nand_host = (struct rcm_nand_host*)mtd->priv;
    if( rcm_nand_host ) return 0;
    return 0;
}

int rcm_nand_erase( struct mtd_info *mtd, struct erase_info *instr )
{
    struct rcm_nand_host* rcm_nand_host = (struct rcm_nand_host*)mtd->priv;
    if( rcm_nand_host ) return 0;
    return 0;
}

static int rcm_nand_probe( struct platform_device *pdev )
{
    int freq = 1000;        // где???????????????/
    int ret;
    struct device* dev = &pdev->dev;
	struct rcm_nand_host* host;
    struct nand_info* nand;                     // параметры микросхемы
	struct mtd_info* mtd;                       // регистрационная структура
	struct resource* iobase;                    // запрошенный ресурс для управления

    host = kzalloc( sizeof( struct rcm_nand_host ), GFP_KERNEL );
    if( !host )
    {
        NAND_DBG_PRINT_ERR( "rcm_nand_probe: failed to allocate device structure.\n" )
        return -ENOMEM;
    }
    host->dev = dev;
    mtd = &host->mtd_info;
    mtd->priv = host;
    nand = &host->nand_info;

    platform_set_drvdata( pdev, host );
//
    iobase = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    host->iobase = devm_ioremap_resource( dev, iobase );
    if( IS_ERR( host->iobase ) )
    {
        return PTR_ERR( host->iobase );
    }

    ret = of_property_read_u32_array( pdev->dev.of_node,   // ассоциированный узел дерева
                                      "timings",
                                      nand->timings,
                                      30 );
    if( ret )
    {
        NAND_DBG_PRINT_ERR( "of_property_read_u32_array: timigs read error" )
        return ret;
    }

    ret = rcm_nand_init( host, nand, nand->timings, freq );         // todo потом...оптимально как-то
    if( ret )
    {
        NAND_DBG_PRINT_ERR( "rcm_controller_setup failed\n" )
        kfree( host );
        return ret;
    }

    mtd->type = MTD_NANDFLASH;
    mtd->flags = MTD_CAP_NANDFLASH;
    mtd->size = nand->blocksize * ( nand->pagesize + nand->oob_size );  // полный размер (с учетом OOB????)
    mtd->erasesize = nand->blocksize * nand->pagesize;                  // стирание блоками
    mtd->writebufsize = mtd->writesize =  nand->pagesize;               // запись страницами
    mtd->oobsize = nand->oob_size;                                      // размер Out-of-band (OOB) области
    mtd->_erase = rcm_nand_erase;
    mtd->_read = rcm_nand_read;
    mtd->_write = rcm_nand_write;
    mtd->name = DRIVER_NAME;
    mtd->dev.parent = &pdev->dev;

    ret = mtd_device_register( mtd, NULL, 0 );
    if( ret )
    {
        NAND_DBG_PRINT_ERR(  "mtd_device_register failed(%d)\n", ret )
        kfree( host );
        return ret;
	}
	return 0;
}

static int rcm_nand_remove( struct platform_device *pdev )
{
    NAND_DBG_PRINT_INF( "rcm_nand_remove" )
    return 0;
}

//-------------------------------------------------------------

static const struct of_device_id rcm_nand_match[] = {                                // типы поддерживаемых устройств
	{ .compatible = "rcm,nand" },
	{},
};
MODULE_DEVICE_TABLE(of, rcm_nand_match);

static struct platform_driver rcm_nand_driver = {
	.probe = rcm_nand_probe,
	.remove = rcm_nand_remove,
	.driver =
		{
			.name = DRIVER_NAME,
            .owner = THIS_MODULE,
			.of_match_table = rcm_nand_match,
		},
};

module_platform_driver(rcm_nand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vladimir Shalyt <Vladimir.Shalyt@astrosoft.ru>");
MODULE_DESCRIPTION("RCM SoC NAND controller driver");


