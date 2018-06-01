#ifndef __PLAT_RMSDIO
#define __PLAT_RMSDIO


struct rmsdio_platform_data 
{
	int gpio_card_detect; //card detect
	int gpio_write_protect; // r/w gpio
	int clock; //Incoming clock rate
};


#endif /* PLAT_RMSDIO */
