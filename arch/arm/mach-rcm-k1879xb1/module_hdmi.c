#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/of_platform.h>

#include "module_hdmi.h"

#define DRIVER_NAME "module_hdmi"
#define DEVICE_NAME "module_hdmi"

#define STATE_INIT	(1 << 0)
#define STATE_SD	(1 << 1)

struct module_hdmi_data {
	struct i2c_adapter *adapter;

	unsigned short addr_0, addr_1;

	unsigned int current_state, wanted_state;

	wait_queue_head_t wq;
	struct task_struct *thread;
	bool force_dvi;
};

static struct module_hdmi_data *g_point = NULL;


void module_hdmi_video_setup_sd(void)
{
	if(NULL == g_point) {
		pr_err("module_hdmi: g_point is NULL\n");
		return;
	}

	g_point->wanted_state |= STATE_SD;
	wake_up_interruptible(&g_point->wq);
}

void module_hdmi_video_setup_hd(void)
{
	if(NULL == g_point) {
		pr_err("module_hdmi: g_point is NULL\n");
		return;
	}

	g_point->wanted_state &= ~STATE_SD;
	wake_up_interruptible(&g_point->wq);
}


static int module_hdmi_write_u8(struct i2c_adapter *adap,
		u8 dev, u8 reg, u8 val)
{
	char buf[2] = {reg, val};
	int ret;
	struct i2c_msg msg = {
		.addr = dev,
		.flags = 0,
		.len = 2,
		.buf = buf
	};

	if ((ret = i2c_transfer(adap, &msg, 1)) < 0)
		pr_err("module_hdmi: i2c_transfer() failed\n");

	return ret;
}
/*
static int module_hdmi_read_u8(struct i2c_adapter *adap,
		u8 dev, u8 reg, u8 *val)
{
	struct i2c_msg msg[2] = {
		{
			.addr = dev,
			.flags = 0,
			.buf = &reg,
			.len = 1
		},
		{
			.addr = dev,
			.flags = I2C_M_RD,
			.buf = val,
			.len = sizeof(val)
		}
	};
	int ret;

	if ((ret = i2c_transfer(adap, msg, ARRAY_SIZE(msg))) < 0)
		pr_err("module_hdmi: i2c_transfer() failed\n");

	return ret;
}
*/
static int module_hdmi_read_u16(struct i2c_adapter *adap,
		u8 dev, u8 reg, u16 *val)
{
	struct i2c_msg msg[2] = {
		{
			.addr = dev,
			.flags = 0,
			.buf = &reg,
			.len = 1
		},
		{
			.addr = dev,
			.flags = I2C_M_RD,
			.buf = (u8 *)val,
			.len = sizeof(val),
		}
	};
	int ret;

	if ((ret = i2c_transfer(adap, msg, ARRAY_SIZE(msg))) < 0)
		pr_err("module_hdmi: i2c_transfer() failed\n");

	return ret;
}


static void module_hdmi_do_init(struct module_hdmi_data *data)
{
	struct i2c_adapter *adap = data->adapter;

	module_hdmi_write_u8(adap, data->addr_0, 0x08, 0x37);

	if(data->force_dvi) {
		//DVI
		module_hdmi_write_u8(adap, data->addr_1, 0x2f, 0x20);
	} else {
		// HDMI
		module_hdmi_write_u8(adap, data->addr_1, 0x2f, 0x21);
	}


	module_hdmi_write_u8(adap, data->addr_0, 0x3e, 0x03);
	module_hdmi_write_u8(adap, data->addr_0, 0x4a, 0x0c);
	module_hdmi_write_u8(adap, data->addr_0, 0x48, 0x10);
	module_hdmi_write_u8(adap, data->addr_1, 0x3d, 0x0f);
	module_hdmi_write_u8(adap, data->addr_0, 0x33, 0x00);

	module_hdmi_write_u8(adap, data->addr_1, 0x01, 0x02);
	module_hdmi_write_u8(adap, data->addr_1, 0x02, 0x03);
	module_hdmi_write_u8(adap, data->addr_1, 0x03, 0x00);
	module_hdmi_write_u8(adap, data->addr_1, 0x04, 0x18);
	module_hdmi_write_u8(adap, data->addr_1, 0x05, 0x00);
	module_hdmi_write_u8(adap, data->addr_1, 0x22, 0x22);
	module_hdmi_write_u8(adap, data->addr_1, 0x24, 0x02);
	module_hdmi_write_u8(adap, data->addr_1, 0x1d, 0x40);
	module_hdmi_write_u8(adap, data->addr_1, 0x14, 0x11);

	data->current_state |= STATE_INIT;
}

static void module_hdmi_do_setup_sd(struct module_hdmi_data *data)
{
	struct i2c_adapter *adap = data->adapter;

	module_hdmi_write_u8(adap, data->addr_0, 0x4a, 0x1e);

	data->current_state |= STATE_SD;
}

static void module_hdmi_do_setup_hd(struct module_hdmi_data *data)
{
	struct i2c_adapter *adap = data->adapter;

	module_hdmi_write_u8(adap, data->addr_0, 0x4a, 0x0c);

	data->current_state &= ~STATE_SD;
}

static int thread_change_state(void *p)
{
	struct module_hdmi_data *data = p;

	while (1) {
		wait_event_interruptible(data->wq,
				(data->current_state != data->wanted_state) ||
				kthread_should_stop());
		if (kthread_should_stop())
			break;

		if ((data->current_state & STATE_INIT) == 0 &&
		    (data->wanted_state & STATE_INIT) != 0)
			module_hdmi_do_init(data);

		if ((data->current_state & STATE_SD) == 0 &&
		    (data->wanted_state & STATE_SD) != 0)
			module_hdmi_do_setup_sd(data);

		if ((data->current_state & STATE_SD) != 0 &&
		    (data->wanted_state & STATE_SD) == 0)
			module_hdmi_do_setup_hd(data);
	}

	return 0;
}

static int module_hdmi_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct module_hdmi_data *data;

	//bool* force_dvi_ptr;

	u16 did;
	int ret;

	pr_info("probing RCM HDMI\n");

	ret = module_hdmi_read_u16(adapter, client->addr, 0x02, &did);
	if (ret < 0) {
		pr_err("module_hdmi: failed to read Device ID register\n");
		return ret;
	}
	if (did != 0x9132) {
		pr_err("module_hdmi: unexpected Device ID value (0x%02x)", did);
		return -ENODEV;
	}
	pr_info("module_hdmi: Device ID: 0x%04x\n", did);

	data = devm_kzalloc(&client->dev,
			  sizeof(struct module_hdmi_data), GFP_KERNEL);
	if (!data) {
		pr_err("module_hdmi_data alloc failed\n");
		return -ENOMEM;
	}

	data->adapter = adapter;
	data->addr_0 = client->addr;
	data->addr_1 = client->addr | 0x4;

	i2c_set_clientdata(client, data);

	init_waitqueue_head(&data->wq);
	data->wanted_state |= STATE_INIT;
    data->force_dvi = of_property_read_bool(client->dev.of_node, "force-dvi");

	data->thread = kthread_run(thread_change_state, (void *) data,
			"module_hdmi");

	if (IS_ERR(data->thread)) {
		pr_err("module_hdmi: failed to create communication thread\n");
		ret = PTR_ERR(data->thread);
		goto kthread_run_failed;
	}

	/* save point to global */
	g_point = data;

	pr_debug("module_hdmi: probe complete. g_point is %p\n", g_point);

	return 0;

kthread_run_failed:
	pr_err("module_hdmi: error bail out\n");
	i2c_set_clientdata(client, 0);
	return ret;
}

static int module_hdmi_remove(struct i2c_client *client)
{
	struct module_hdmi_data *data = i2c_get_clientdata(client);

	kthread_stop(data->thread);
	return 0;
}

static const struct i2c_device_id module_hdmi_id_table[] = {
	{ DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, module_hdmi_id_table);

static const struct of_device_id module_hdmi_dt_ids[] = {
	{ .compatible = "rcm,hdmi", },
	{ }
};
MODULE_DEVICE_TABLE(of, module_hdmi_dt_ids);

static struct i2c_driver module_hdmi_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(module_hdmi_dt_ids),
	},
	.probe = module_hdmi_probe,
	.remove = module_hdmi_remove,
	.id_table = module_hdmi_id_table
};

module_i2c_driver(module_hdmi_driver);

