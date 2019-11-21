#include <linux/err.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/cfi.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#define NAND_DEBUG

#ifdef NAND_DEBUG
    #define NAND_DBG_PRINT(...) printk("RCM_NAND TRACE" __VA_ARGS__);
#else
    #define NAND_DBG_PRINT(...) while(0);
#endif


#define     NAND_REG_status                 0x00
#define     NAND_REG_control                0x04
#define     NAND_REG_command                0x08
#define     NAND_REG_col_addr               0x0c
#define     NAND_REG_row_addr_read_d        0x10
#define     NAND_REG_irq_mask_nand          0x14
#define     NAND_REG_cdb_buffer_enable      0x18
#define     NAND_REG_cdb_buffer_type        0x1c
#define     NAND_REG_cdb_ch0_buffer_status  0x20
#define     NAND_REG_axi_arlen              0x24
#define     NAND_REG_sw_rstn_r              0x28
#define     NAND_REG_reserved_2c            0x2c
#define     NAND_REG_axi_if_err_addr        0x30
#define     NAND_REG_axi_if_status          0x34
#define     NAND_REG_msb_lsbr               0x38
#define     NAND_REG_reserved_3c            0x3c
#define     NAND_REG_start_dma_r            0x40
#define     NAND_REG_end_dma_r              0x44
#define     NAND_REG_current_dma_r          0x48
#define     NAND_REG_cntrl_dma_r            0x4c
#define     NAND_REG_cntrl_sw_rst           0x50
#define     NAND_REG_id                     0x54
#define     NAND_REG_version                0x58
#define     NAND_REG_reserved_5c            0x5c
#define     NAND_REG_reserved_60            0x60
#define     NAND_REG_reserved_64            0x64
#define     NAND_REG_reserved_68            0x68
#define     NAND_REG_awlen_max              0x6c
#define     NAND_REG_sw_rst                 0x70
#define     NAND_REG_msb_lsbw               0x74
#define     NAND_REG_respw_status           0x78
#define     NAND_REG_status_full            0x7c
#define     NAND_REG_reserved_80            0x80
#define     NAND_REG_reserved_84            0x84
#define     NAND_REG_start_dma_w            0x88
#define     NAND_REG_end_dma_w              0x8c
#define     NAND_REG_current_dma_w          0x90
#define     NAND_REG_cntrl_dma_w            0x94
#define     NAND_REG_reserved_9c            0x9c
#define     NAND_REG_reserved_a0            0xa0
#define     NAND_REG_reserved_a4            0xa4
#define     NAND_REG_reserved_a8            0xa8
#define     NAND_REG_reserved_ac            0xac
#define     NAND_REG_reserved_b0            0xb0
#define     NAND_REG_irq_status             0xb4
#define     NAND_REG_timing_0               0xb8
#define     NAND_REG_timing_1               0xbc
#define     NAND_REG_timing_2               0xc0
#define     NAND_REG_timing_3               0xc4
#define     NAND_REG_timing_4               0xc8
#define     NAND_REG_timing_5               0xcc
#define     NAND_REG_timing_6               0xd0
#define     NAND_REG_timing_7               0xd4

#define CMD_CONTROL_RESET                   0x07        // reset
#define CMD_CONTROL_READ                    0x01        // read
#define CMD_CONTROL_READ_ID                 0x03        // read ID
#define CMD_NAND_ONFI                       0xEC        // read ONFI
#define CMD_NAND_ID                         0x90        // read ID

#define NAND_IRQ_RESET                      0x00000010  // завершение выполнения команды сброса
#define NAND_IRQ_END_READ                   0x00000001  // завершение выполнения команды чтения
#define NAND_IRQ_END_READ_ID                0x00000002  // завершение выполнения команды считывания идентификационного номера
#define NAND_IRQ_LAST_BUF_ADDR              0x00000080  // последний адрес состояния ведущего интерфейса записи AXI
#define NAND_IRQ_LAST_ADDR_AXI_W            0x00000100  // последний сегмент адреса состояния ведущего интерфейса записи AXI 

#define STATUS_CONT_READY                   0x00000200  // статус: 1-контроллер NAND готов к приему команды
#define STATUS_NAND_READY                   0x00000400  // статус: 1-флэш-память готова к выполнению операций

#define NAND_DMA_CONT_READ                  0x80000000  // 0xC00001E0 //size of buf's seg  = 16 -1 = 15 [25:5],   [31:30] - init bits
#define NAND_TIMEOUT                        10000
#define NAND_RESET_TIMEOUT                  8000
#define NAND_ONFI_SIZE                      128
#define NAND_PAGE_SIZE_BASE                 512

#define NAND_CI_CELLTYPE_MSK                0xC
#define NAND_MFR_SAMSUNG                    0xEC
#define NAND_MFR_AMD                        0x01

#define NAND_CONTROL_START                  0x01        // start
#define NAND_CONTROL_C_B_ADDR_5             0x5

// расчет таймингов
#define NAND_DIV_ROUND_UP(x, y) (1 + (((x) - 1) / (y)))
#define NAND_TIMING_TO_CLOCKS(t,f) ((NAND_DIV_ROUND_UP((t+1)*f, 1000))  & 0xff)
// тайминги
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

struct nand_flash_info              // параметры модуля памяти
{
    u32 id_m;                       // manufacturer identifier
    u32 id_dev;                     // nand_flash identifier
    u32 pagesize;                   // page size in B
    u32 blocksize;                  // size of block in pages
    u32 sparesize;                  // size of spare data (extra data in each page used for ECC) in B
    u32 mode_ecc;                   // режим коррекции ошибок
};

static const struct nand_flash_info flash_id_map[] = {
    //id_m  id_dev  pagesize blocksize  sparesize mode_ecc
    { 0x01, 0x73,   512,    1024,       16,         1},
    { 0x01, 0x75,   512,    1024,       16,         1},
    { 0x01, 0x76,   512,    1024,       16,         1},
    { 0x20, 0x35,   512,    32,         16,         1},
    { 0x20, 0x36,   512,    32,         16,         1},
    { 0x20, 0x39,   512,    32,         16,         1},
    { 0x20, 0x73,   512,    32,         16,         1},
    { 0x20, 0x75,   512,    32,         16,         1},
    { 0x20, 0x76,   512,    32,         16,         1},
    { 0x20, 0x78,   512,    32,         16,         1},
    { 0x20, 0x79,   512,    32,         16,         1},
    { 0x98, 0x73,   512,    256,        16,         1},
    { 0xAD, 0x55,   512,    256,        16,         1},
    { 0x20, 0xAC,   2048,   32,         64,         1},
    { 0x20, 0xB1,   2048,   32,         64,         1},
    { 0x20, 0xBC,   2048,   32,         64,         1},
    { 0x2C, 0xDC,   2048,   32,         64,         1},
    { 0xAD, 0xDA,   2048,   64,         64,         1},
    { 0xAD, 0xF1,   2048,   32,         64,         1},
    { 0xEC, 0xAA,   2048,   32,         64,         1},
    { 0xEC, 0xD3,   2048,   64,         64,         1},
    { 0xEC, 0xDA,   2048,   32,         64,         1},
    { 0xEC, 0xDC,   2048,   32,         64,         1},
    { 0x98, 0xD3,   4096,   16,         128,        1},
    { 0xAD, 0xD5,   4096,   16,         128,        1},
    { 0xAD, 0xD5,   4096,   16,         128,        1},
    { 0, 0, 0, 0, 0, 0 }
};

//typedef u32 (*reg_readl_fn)(void *rcm_mtd, u32 offset);
//typedef void (*reg_writel_read_fn)(void *rcm_mtd, u32 offset, u32 val);

struct rcm_mtd {
	void *regs;
	// u32 high_addr;
	// reg_readl_fn readl;
	// reg_writel_read_fn writel;
};

static u32 lsif_reg_readl(void *base, u32 offset)
{
	struct rcm_mtd *rcm_mtd = (struct rcm_mtd *)base;
	return readl(rcm_mtd->regs + offset);
}

static void lsif_reg_writel(void *base, u32 offset, u32 val)
{
	struct rcm_mtd *rcm_mtd = (struct rcm_mtd *)base;
	writel(val, rcm_mtd->regs + offset);
}

static void configure_dma(void* base, u32* addr, int len)
{
    NAND_DBG_PRINT("configure_dma: configure_dma: addr=%08x,len=%u", (u32)addr, len)
    lsif_reg_writel(base, NAND_REG_awlen_max, 0x7);                                         // размер axi транзакции
    lsif_reg_writel(base, NAND_REG_msb_lsbw, 0x1);                                          // big_endian
    lsif_reg_writel(base, NAND_REG_start_dma_w, (u32)addr);                                 // dma начальный адрес
    lsif_reg_writel(base, NAND_REG_end_dma_w, (u32)addr+len-1);                             // dma конечный адрес
    lsif_reg_writel(base, NAND_REG_current_dma_w, (u32)addr);                               // dma текущий адрес - стартовый
    lsif_reg_writel(base, NAND_REG_cntrl_dma_w, NAND_DMA_CONT_READ | ((len -1) << 5));      // генерация адресов + размер сегмента-1
}

static int is_ready(void* base)                                                             // ждем готовности либо одной либо второй?
{
    unsigned t = 0;
    while( !( lsif_reg_readl(base, NAND_REG_status) & (STATUS_CONT_READY | STATUS_NAND_READY) ) && (t < NAND_TIMEOUT))
        t++;
    if (t==NAND_TIMEOUT)
        return EIO;
    return 0;
}

static int reset(void* base)
{
    int ret;
    lsif_reg_writel(base, NAND_REG_cntrl_sw_rst, 0x0);                  // Запись 0 в данный регистр инициирует программный сброс контроллера
    lsif_reg_writel(base, NAND_REG_sw_rst, 0x0);                        // Запись 0,затем 1 в данный регистр инициирует
    lsif_reg_writel(base, NAND_REG_sw_rst, 0x1);                        // программный сброс ведущего интерфейса записи AXI 
    lsif_reg_writel(base, NAND_REG_sw_rstn_r, 0x1);                     // программный сброс ведущего интерфейса чтения AXI, автоматически обнуляется
    ret = is_ready(base);                                               // ждем готовности
    NAND_DBG_PRINT("reset: return %d\n", ret )
    return ret;
}

static void set_timing(void* base, u32 offset, u32* timings, unsigned cnt, u32 freq)
{
    unsigned i;
    u32 timing_reg = 0, n = 0;
    for (i=0; i<cnt; i++)
    {
        timing_reg |= (NAND_TIMING_TO_CLOCKS(timings[i],freq)<<n);
        n += 8;
    }
    lsif_reg_writel(base, offset, timing_reg);
    NAND_DBG_PRINT("set_timing: reg=%02x,data=%08x", offset, timing_reg )
}

static void set_timings(void* base, u32* timings, u32 freq)
{
    set_timing(base, NAND_REG_timing_0, timings+0, 4, freq);       // tADL_min, tALH_min, tALS_min, tCH_min
    set_timing(base, NAND_REG_timing_1, timings+4, 4, freq);       // tCLH_min, tCLS_min, tCS_min, tDH_min
    set_timing(base, NAND_REG_timing_2, timings+8, 4, freq);       // tDS_min, tWC_min, tWH_min, tWP_min
    set_timing(base, NAND_REG_timing_3, timings+12, 3, freq);      // tWW_min, tAR_min, tCLR_min
    set_timing(base, NAND_REG_timing_4, timings+15, 4, freq);      // tCOH_min, tIR_min, tRC_min, tREH_min
    set_timing(base, NAND_REG_timing_5, timings+19, 4, freq);      // tRHOH_min, tRHW_min, tRLOH_min, tRP_min
    timings[25] += 2;
    set_timing(base, NAND_REG_timing_6, timings+23, 4, freq);      // tRR_min, tWHR_min, tCEA_max+2, tCHZ_max
    timings[27] += 2;
    timings[29] += 2;
    set_timing(base, NAND_REG_timing_7, timings+27, 3, freq);      // tREA_max+2, tRHZ_max, tWB_max+2
}

int read_nand(void* base,
              u32* buf_addr,
              u32 len,
              u32 start_page,
              struct nand_flash_info const* nand_info,
              u32 cmd,
              u32 col_addr,
              u32 nand_cmd)
{
    int ret;
    unsigned cur_page = 0;
    unsigned num_page = 1;
    unsigned page_size_code = 0;
    int dma_wite_size = 0;
    u32* dma_write_addr = 0;
    u32 size_of_ecc = 0;
    unsigned i;
    unsigned t = 0;

    if(nand_info->pagesize > 0)
    {
        while(nand_info->pagesize*num_page < len)                                           // нужное число страниц
            num_page++;

        i = nand_info->pagesize / NAND_PAGE_SIZE_BASE;
        while(i)
        {
            page_size_code++;
            i >>= 1;
        }
        page_size_code--;
    }
    size_of_ecc = (nand_info->pagesize / 512) * ((nand_info->mode_ecc == 1) ? 7 : 21 );     // 7 or 21 bytes per 512 bytes of nand pages

    for (cur_page=start_page; cur_page < start_page+num_page; cur_page++)
    {
        ret = is_ready(base);
        if (ret) return ret;

        dma_write_addr = (u32*)((u32)buf_addr + (cur_page-start_page)*(nand_info->pagesize ));

        if(nand_info->pagesize != 0)
            dma_wite_size = nand_info->pagesize + nand_info->sparesize - size_of_ecc;
        else
            dma_wite_size = len;

        configure_dma(base, dma_write_addr, dma_wite_size);

        lsif_reg_writel(base, NAND_REG_command, cmd);
        lsif_reg_writel(base, NAND_REG_col_addr, col_addr);
        lsif_reg_writel(base, NAND_REG_row_addr_read_d, cur_page);
        lsif_reg_writel(base, NAND_REG_control, (nand_cmd << 24) |                                                      // первая команда, подлежащая отправке во флэшпамять
                                                (((nand_info->pagesize != 0) ? (nand_info->sparesize ) : len) << 13) |  // 23:13: размер поля OOB/количество байтов данных
                                                (((nand_info->pagesize != 0) ? nand_info->mode_ecc : 0x0) << 7) |       // 8..7: режим коррекции ошибок
                                                (((nand_info->pagesize != 0) ? NAND_CONTROL_C_B_ADDR_5 : 0x1) << 4) |   // 6..4: количество байтов (циклов) адреса в текущей команд
                                                (((nand_info->pagesize != 0) ? page_size_code : 0x0) << 1) |            // 3..1: размер страницы, байты
                                                NAND_CONTROL_START);                                                    // 0: немедленное выполнение команды

        while( !( lsif_reg_readl(base, NAND_REG_irq_status) & ((nand_cmd == CMD_NAND_ID) ? NAND_IRQ_END_READ_ID : NAND_IRQ_LAST_BUF_ADDR ) ) && (t < NAND_TIMEOUT))
            t++;

        if (t==NAND_TIMEOUT)
        {
            ret = EIO;
            break;
        }
    }
    NAND_DBG_PRINT("read_nand: return %d", ret)
    return ret;
}

int get_info(void* base, struct nand_flash_info* nand_info)
{
    int ret;
    char id_buf[8] = {0};
    char onfi_buf[NAND_ONFI_SIZE] = {0};
    struct nand_flash_info const* id = flash_id_map;
    int busw = 0;
    int i = 0;

    nand_info->id_m = 0;
    nand_info->id_dev = 0;
    nand_info->blocksize = 0;
    nand_info->pagesize = 0;
    nand_info->mode_ecc = 0;
    nand_info->sparesize = 0;

    ret = read_nand(base,(u32*)onfi_buf, 4, 0, nand_info, CMD_CONTROL_READ_ID, 0x20, CMD_NAND_ID);  // читаем идетификатор

    if(ret)
    {
        NAND_DBG_PRINT("get_info: can't read chip id (%d)", ret)
        return ret;
    }

    if( onfi_buf[0] == 'O' && onfi_buf[1] == 'N' && onfi_buf[2] == 'F' && onfi_buf[3] == 'I' )
    {
        ret = read_nand(base, (u32*)onfi_buf, NAND_ONFI_SIZE, 0, nand_info, CMD_CONTROL_READ, 0x00, CMD_NAND_ONFI);

        if(ret)
        {
            NAND_DBG_PRINT("get_info: сan't read ONFI parameters (%d)", ret);
            return ret;
        }

        busw = onfi_buf[6] & 0x01;
        nand_info->pagesize = (onfi_buf[83] << 24 | onfi_buf[82] << 16 | onfi_buf[81] << 8 | onfi_buf[80]);
        nand_info->blocksize = (onfi_buf[95] << 24 | onfi_buf[94] << 16 | onfi_buf[93] << 8 | onfi_buf[92]);
        nand_info->sparesize = (onfi_buf[85] << 8 | onfi_buf[84]);
        nand_info->mode_ecc = ((onfi_buf[112] > 4) ? 2 : 1);
    }
    else // не ONFI
    {
        ret = read_nand(base, (u32*)id_buf, 8, 0, nand_info, CMD_CONTROL_READ_ID, 0x0, CMD_NAND_ID);

        if(ret)
        {
            NAND_DBG_PRINT("get_info: can't read chip id (%d)", ret)
            return ret;
        }
// ищем в таблице идентификатор
        NAND_DBG_PRINT("get_info: looking for chip id (%02x,%02x)", id_buf[0], id_buf[1])
        while( (id->id_m != 0) && ((id->id_dev != id_buf[1]) || (id->id_m != id_buf[0])) )
            ++id;

        if(!id->pagesize) // not found in table
        {
             // The 3rd id byte holds MLC / multichip data
            int cellinfo = id_buf[2];
             // The 4th id byte is the important one
             int extid = id_buf[3];

             /*
              * Field definitions are in the following datasheets:
              * Old style (4,5 byte ID): Samsung K9GAG08U0M (p.32)
              * New style   (6 byte ID): Samsung K9GBG08U0M (p.40)
              *
              * Check for wraparound + Samsung ID + nonzero 6th byte
              * to decide what to do.
              */
             if (id_buf[0] == id_buf[6] &&
                 id_buf[1] == id_buf[7] &&
                 id_buf[0] == NAND_MFR_SAMSUNG &&
                 (cellinfo & NAND_CI_CELLTYPE_MSK) &&
                 id_buf[5] != 0x00
                )
             {
                 nand_info->pagesize = 2048 << (extid & 0x03);
                 extid >>= 2;
                 switch (extid & 0x03)
                 {
                     case 1:
                        nand_info->sparesize = 128;
                         break;
                     case 2:
                        nand_info->sparesize = 218;
                         break;
                     case 3:
                        nand_info->sparesize = 400;
                         break;
                     default:
                        nand_info->sparesize = 436;
                         break;
                 }
                 extid >>= 2;
                 nand_info->blocksize = (128 * 1024) << (((extid >> 1) & 0x04) | (extid & 0x03));
                 busw = 0;
             } else
             {
                 nand_info->pagesize = 1024 << (extid & 0x3);
                 extid >>= 2;

                 nand_info->sparesize = (8 << (extid & 0x01)) * (nand_info->pagesize >> 9);
                 extid >>= 2;

                 nand_info->blocksize = (64 * 1024) << (extid & 0x03);
                 extid >>= 2;

                 busw = (extid & 0x01) ? 1 : 0;
             }

             do
             {
                 i++;
                 nand_info->blocksize -= nand_info->pagesize;
             }
             while(nand_info->blocksize);
             nand_info->blocksize = i;

             nand_info->mode_ecc = (nand_info->pagesize > 8*NAND_PAGE_SIZE_BASE) ? 0x2 : 0x1;
        }
        else // Old devices have chip data hardcoded in table
        {
            nand_info->pagesize = id->pagesize;
            nand_info->blocksize = id->blocksize;
            nand_info->sparesize =  id->pagesize/32;
            nand_info->mode_ecc =  id->mode_ecc;
            busw = 0;

            /*
             * Check for Spansion/AMD ID + repeating 5th, 6th byte since
             * some Spansion chips have erasesize that conflicts with size
             * listed in nand_ids table.
             * Data sheet (5 byte ID): Spansion S30ML-P ORNAND (p.39)
             */
            if (id_buf[0] == NAND_MFR_AMD &&
                id_buf[4] != 0x00 &&
                id_buf[5] == 0x00 &&
                id_buf[6] == 0x00 &&
                id_buf[7] == 0x00 &&
                nand_info->pagesize == 512
                )
            {
                nand_info->blocksize = 128 * 1024;
                nand_info->blocksize <<= ((id_buf[3] & 0x03) << 1);
            }
        }
    }

    if(busw)
    {
        NAND_DBG_PRINT("get_info: BUSWIDTH 16 is not supported by controller")
        return ENODEV;
    }

    NAND_DBG_PRINT("get_info: blocksize=%u,id_dev=%08x,id_m=%08x,mode_ecc=%u,pagesize=%u,sparesize=%u",
                   nand_info->blocksize,
                   nand_info->id_dev,
                   nand_info->id_m,
                   nand_info->mode_ecc,
                   nand_info->pagesize,
                   nand_info->sparesize)
    return 0;
}

static int rcm_controller_setup(struct platform_device *pdev)                       // собственно setup контроллера
{
    int ret;
    unsigned t = 0;
    #define freq 1000
    u32 timings[30];
    struct nand_flash_info nand_info;

	struct rcm_mtd *rcm_mtd = platform_get_drvdata(pdev);

    if (!reset(rcm_mtd))                                                            // формируем сброс контроллера
        return -EIO;

    ret = of_property_read_u32_array(pdev->dev.of_node, "timings", timings, 30);    // считываем тайминги из devtree
    if (ret)
        return ret;

    set_timings(rcm_mtd, timings, freq);                                           // устанавливаем тайминги в контроллере

    lsif_reg_writel(rcm_mtd, NAND_REG_command, CMD_CONTROL_RESET);                  // сброс флэш-памяти NAND
    lsif_reg_writel(rcm_mtd, NAND_REG_command, NAND_CONTROL_START);                 // считывание с NAND

    t = 0;
    while (!(lsif_reg_readl(rcm_mtd, NAND_REG_irq_status) & NAND_IRQ_RESET) &&      // ожидание сброса flash
            (t++ < NAND_RESET_TIMEOUT));

    if (t < NAND_RESET_TIMEOUT)
        ret = get_info(rcm_mtd, &nand_info);
    else
        ret = -EIO;
    NAND_DBG_PRINT("rcm_controller_setup: return %d", ret)
    return ret;
}

static int rcm_mtd_probe(struct platform_device *pdev)                              // инициализация драйвера
{

    int ret;
    struct rcm_mtd *rcm_mtd;
    struct resource *ctrl;

    rcm_mtd = devm_kzalloc(&pdev->dev, sizeof(struct rcm_mtd), GFP_KERNEL);         // память под приватные данные
    if (!rcm_mtd)
        return -ENOMEM;

    platform_set_drvdata(pdev, rcm_mtd);	                                        // конфиг указателя на них

    ctrl = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    if (!ctrl)
    {
	    dev_err(&pdev->dev, "failed to get control resource\n");
		return -ENOENT;
	}
	
    rcm_mtd->regs = devm_ioremap_resource(&pdev->dev, ctrl);                        // проверяет ресурс, запрашивает регион памяти, проецирует его на физические адреса устройства
    if (IS_ERR(rcm_mtd->regs))
    		return PTR_ERR(rcm_mtd->regs);

	ret = rcm_controller_setup(pdev);                                               // инициалируем устройство
    if (ret)
    {
		dev_err(&pdev->dev, "hw setup failed\n");
		return ret;
	}

	dev_info(&pdev->dev, "registered\n");
	return 0;
}


static int rcm_mtd_remove(struct platform_device *pdev)                             // удаление 
{
	return 0;
}

static const struct of_device_id rcm_mtd_match[] = {                                // типы поддерживаемых устройств
	{ .compatible = "rcm,nand" },
	{},
};
MODULE_DEVICE_TABLE(of, rcm_mtd_match);

static struct platform_driver rcm_mtd_driver = {
	.probe = rcm_mtd_probe,
	.remove = rcm_mtd_remove,
	.driver =
		{
			.name = "rcm-nand",
			.of_match_table = rcm_mtd_match,
		},
};

module_platform_driver(rcm_mtd_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vladimir Shalyt <Vladimir.Shalyt@astrosoft.ru>");
MODULE_DESCRIPTION("RCM SoC NAND controller driver");


