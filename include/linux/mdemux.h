#ifndef __LINUX_MDEMUX_H
#define __LINUX_MDEMUX_H

struct mdemux_platform_data {
	int i2c_bus_id;
	int i2c_dvbs_demodulator_addr;
	int i2c_dvbs_tuner_addr;
	int i2c_dvbs_lnb_addr;
};

#define DECLARE_PLDATA_GETTER(fld,def) \
static int g_##fld = -1; \
module_param_named(fld, g_##fld, int, 0); \
inline static int get_##fld(struct mdemux_platform_data* pldata) \
{ \
	if(g_##fld >= 0) return g_##fld; \
	if(pldata && pldata->fld >= 0) return pldata->fld; \
	return def; \
}

#endif

