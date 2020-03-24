#ifndef DEFINES_H
#define DEFINES_H

#include <linux/ioctl.h>
#include <linux/watchdog.h>

#define SCREEN_WIDTH     500
#define SCREEN_HIGH      500

// ****************************************************************************
//
//								MAP REGISTERS
//
// ****************************************************************************

	// CONTROL
	#define ADDR_ID_REG           0x000 // address of register id_reg          
	#define ADDR_ENABLE           0x004 // address of register enable_o        
	#define ADDR_PR_RESET         0x008 // address of register pr_reset_o   
	#define ADDR_REC_ENABLE       0x00c // address of register rec_enable_o 
	// STATUS
	#define ADDR_ACTIVE_FRAME     0x100 // address of register active_frame_i  
	#define ADDR_FRAME_SIZE       0x104 // address of register frame_size_i 
	#define ADDR_FRAME_PARAM      0x108 // address of register frame_param_i  
	#define ADDR_DMA0_STATUS      0x10c // address of register dma1_status_i   
	#define ADDR_DMA1_STATUS      0x110 // address of register dma2_status_i   
	#define ADDR_DMA2_STATUS      0x114 // address of register dma3_status_i   
	#define ADDR_DMA_ERROR        0x118 // address of register dma_error 
	// INTERRUPT
	#define ADDR_INT_STATUS       0x200 // address of register int_status_i    
	#define ADDR_INT_MASK         0x204 // address of register int_mask_o      
	#define ADDR_TEST_INT         0x208 // address of register test_int_o      
	#define ADDR_INTERRUPTION     0x20c // address of register interruption_i  
	// CONVERSION
	#define ADDR_GAM_ENABLE       0x300 // address of register gam_enable_o    
	#define ADDR_CONV_ENABLE      0x304 // address of register conv_enable_o   
	#define ADDR_C_0_0            0x308 // address of register c_1_0_o         
	#define ADDR_C_0_1            0x30c // address of register c_1_1_o         
	#define ADDR_C_0_2            0x310 // address of register c_1_2_o         
	#define ADDR_C_0_3            0x314 // address of register c_1_3_o         
	#define ADDR_C_1_0            0x318 // address of register c_2_0_o         
	#define ADDR_C_1_1            0x31c // address of register c_2_1_o         
	#define ADDR_C_1_2            0x320 // address of register c_2_2_o         
	#define ADDR_C_1_3            0x324 // address of register c_2_3_o         
	#define ADDR_C_2_0            0x328 // address of register c_3_0_o         
	#define ADDR_C_2_1            0x32c // address of register c_3_1_o         
	#define ADDR_C_2_2            0x330 // address of register c_3_2_o         
	#define ADDR_C_2_3            0x334 // address of register c_3_3_o 
	#define ADDR_CH0_RANGE        0x338 // address of register ch1_range 
	#define ADDR_CH1_RANGE        0x33c // address of register ch2_range 
	#define ADDR_CH2_RANGE        0x340 // address of register ch3_range 
	// DMA
	#define ADDR_BASE_SW_ENA      0x400 // address of register base_sw_ena_o   
	#define ADDR_DMA0_ADDR0       0x404 // address of register dma1_addr1_o    
	#define ADDR_DMA0_ADDR1       0x408 // address of register dma1_addr2_o    
	#define ADDR_DMA1_ADDR0       0x40c // address of register dma2_addr1_o    
	#define ADDR_DMA1_ADDR1       0x410 // address of register dma2_addr2_o    
	#define ADDR_DMA2_ADDR0       0x414 // address of register dma3_addr1_o    
	#define ADDR_DMA2_ADDR1       0x418 // address of register dma3_addr2_o
	#define ADDR_Y_SIZE           0x41c // address of register frame size for Y
	#define ADDR_C_SIZE           0x420 // address of register frame size for C
	#define ADDR_FULL_LINE_SIZE   0x424 // address of register full_line_size
	#define ADDR_MODE             0x428 // address of register mode      
	#define ADDR_LOCATION_DATA    0x42c // address of register location_data   
	#define ADDR_TRANSPARENCY     0x430 // address of register transparency_o  
	#define ADDR_BASE_POINT       0x434 // address of register base_point_o 
	#define ADDR_DMA_ID           0x438 // address of register dma_id 
	#define ADDR_AXI_PARAM        0x43c // address of register axi_param     
	// GAMMA
	#define BASE_ADDR_TABLE_0     0xd00 // address of table gamma-correction
	#define BASE_ADDR_TABLE_1     0xe00 // address of table gamma-correction
	#define BASE_ADDR_TABLE_2     0xf00 // address of table gamma-correction

// ****************************************************************************
//
//								INTERRUPT
//
// ************************************************************************

	#define CH2_OVERFLOW		31	// Разрешение генерации сигнала прерывания по событию: запись в полный буфер второго канала
	#define CH1_OVERFLOW        28	// Разрешение генерации сигнала прерывания по событию: запись в полный буфер первого канала
	#define CH0_OVERFLOW        25	// Разрешение генерации сигнала прерывания по событию: запись в полный буфер нулевого канала
	#define RECEIVE_END         18	// Разрешение генерации сигнала прерывания по событию: окончание захвата кадра
	#define INT_VSYNC           16	// Разрешение генерации сигнала прерывания по событию: фронт сигнала развёртки вертикальной синхронизации
	#define INT_HSYNC           15	// Разрешение генерации сигнала прерывания по событию: фронт сигнала развёртки горизонтальной синхронизации
	#define INT_ST_ERROR        14	// Разрешение генерации сигнала прерывания по событию: несовпадение количества шин, по которым передаются видеоданные, в соседних кадрах
	#define INT_VS_ERROR        13	// Разрешение генерации сигнала прерывания по событию: несовпадение количества линий в соседних кадрах
	#define INT_HS_ERROR        12	// Разрешение генерации сигнала прерывания по событию: несовпадение количество точек в соседних линиях
	#define INT_DONE            11	// Разрешение генерации сигнала прерывания по событию: окончание распознавания развёртки
	#define GRB_OFF             8	// Разрешение генерации сигнала прерывания по событию: состояние активности устройства захвата видеоизображения
	#define END_WRITE           7	// Разрешение генерации сигнала прерывания по событию: окончание записи кадра в память
	#define INT_UNC_ERR         6	// Разрешение генерации сигнала прерывания по событию: неисправимая ошибка синхрокода EAV и SAV
	#define INT_REPL            5	// Разрешение генерации сигнала прерывания по событию: исправление ошибки синхрокода EAV и SAV
	#define INT_ERR_CODE        4	// Разрешение генерации сигнала прерывания по событию: ошибка синхрокода EAV и SAV
	#define DMA2_ERROR          3	// Разрешение генерации сигнала прерывания по событию: при записи в память второго канала ответ с AXI «ошибка»
	#define DMA1_ERROR          2	// Разрешение генерации сигнала прерывания по событию: при записи в память первого канала ответ с AXI «ошибка»
	#define DMA0_ERROR          1	// Разрешение генерации сигнала прерывания по событию: при записи в память нулевого канала ответ с AXI «ошибка»
	#define TEST_INT            0	// Разрешение генерации сигнала прерывания по событию: тестовый запрос прерывания в активном состоянии

	#define EXTERNAL			0
	#define INTERNAL			1
	
	#define OFF					0
	#define ON					1
	
	#define YCBCR				0
	#define RGB					1
	
	#define YCBCR422			0
	#define YCBCR444			1
	#define RGB888				2
	
	#define HD					0
	#define SD					1
	
	#define SERIAL				0
	#define PARALLEL			1

	#define VIDIOC_SET_GAMMA		_IOWR('v', BASE_VIDIOC_PRIVATE + 0, struct grb_gamma)
	#define VIDIOC_G_PARAMS			_IOWR('v', BASE_VIDIOC_PRIVATE + 1, struct grb_parameters)
	#define VIDIOC_S_PARAMS			_IOWR('v', BASE_VIDIOC_PRIVATE + 2, struct grb_parameters)
	#define VIDIOC_AUTO_DETECT		_IOWR('v', BASE_VIDIOC_PRIVATE + 3, struct grb_parameters)
	#define VIDIOC_SET_COLOR_CONV	_IOWR('v', BASE_VIDIOC_PRIVATE + 4, struct grb_gamma)

	#define C_WHITE   "\033[1;29;40m"
	#define C_RED	  "\033[1;31;40m"
	#define C_GREEN   "\033[1;32;40m"
	#define C_YELLOW  "\033[1;33;40m"
	#define C_BLUE	  "\033[1;34;40m"
	#define C_CRIMSON "\033[1;35;40m"			// малиновый
	#define C_CYAN	  "\033[1;36;40m"			// голубой
	#define C_GREY	  "\033[1:37:40m"
	#define C_CLEAR   "\033[1;0m"

	#define DEVICE_NAME "grb_v2"
	#define DRIVER_NAME "grb_driver"
	#define GRB_DEVID 0xec176627

	#define PHYS_TO_DMA(A) ((A)|0x40000000)
	#define U16x2_TO_U32(H,L) (((H)<<16)|(L))
	#define U8x4_TO_U32(U0,U1,U2,U3) (((u32)U0<<0)+((u32)U1<<8)+((u32)U2<<16)+((u32)U3<<24))

	struct grb_parameters {
		u32 sync;
		u32 std_in;
		u32 v_if;
		u32 d_format;
		u32 std_out;
		u32 alpha;
	};

	struct input_format {
		u32 color;
		u32 color_std;
		u32	format_din;
	};

	struct output_format {
		u32 color;
		u32 color_std;
		u32 format_dout;
		u32 y_hor_size, y_ver_size, y_full_size;
		u32 c_hor_size, c_ver_size, c_full_size;
	};

	struct grb_gamma {
		u32 active_gamma;
		int table_Y_G[256];
		int table_C_R[256];
		int table_C_B[256];
	};

	struct coef_conv {
		u32	coef[3][4];
		u32	range[3];
	};

	static const struct coef_conv YCBCR_TO_RGB_HD;
	static const struct coef_conv YCBCR_TO_RGB_SD;
	static const struct coef_conv RGB_TO_YCBCR_HD;
	static const struct coef_conv RGB_TO_YCBCR_SD;
	static const struct coef_conv YCBCR_HD_TO_SD;
	static const struct coef_conv YCBCR_SD_TO_HD;

#endif