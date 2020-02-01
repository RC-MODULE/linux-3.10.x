/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <dev@alsp.net>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/of.h>

#include "pseudo_spi_serial.h"

static int pseudo_spi_serial_duplex = 0;
module_param(pseudo_spi_serial_duplex, int, 0);
MODULE_PARM_DESC(pseudo_spi_serial_duplex,
				 "Enable/disable serial pseudo spi duplex mode. Set 1 to enable.");

static int pseudo_spi_serial_use_dma = 1;
module_param(pseudo_spi_serial_use_dma, int, 0);
MODULE_PARM_DESC(pseudo_spi_serial_use_dma,
				 "Using DMA in serial pseudo spi driver. Set 1 to use DMA.");

#define PSEUDO_SPI_SERIAL_FIFO_SIZE 	2048
#define PSEUDO_SPI_SERIAL_READ_SIZE 	8

#define DELAY_TIME						(HZ / 50)

static struct pseudo_spi_serial_data* pseudo_spi_serial_table[PSEUDO_SPI_SERIAL_TTY_MINORS];

static volatile unsigned long pseudo_spi_serial_mask;

struct tty_driver* pseudo_spi_serial_tty_driver;

extern void pseudo_spi_console_init(struct pseudo_spi_serial_data* data);
extern void pseudo_spi_console_exit(void);

static void pseudo_spi_serial_timer(struct timer_list*	timer)
{
	struct pseudo_spi_serial_data* data = from_timer(data, timer, timer);

	tasklet_schedule(&data->io_tasklet);

	mod_timer(&data->timer, jiffies + DELAY_TIME);
}

static int pseudo_spi_serial_open(struct tty_struct* tty, struct file* file)
{
	struct pseudo_spi_serial_data* data;
	int index;

	index = tty->index;
	data = pseudo_spi_serial_table[index];
	
	if (data == NULL)
	{
		pr_err("%s: No ttySPI%d device.\n", __func__, index);
		return -ENODEV;
	}

	return tty_port_open(&data->tty_port, tty, file);
}

static int pseudo_spi_serial_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct pseudo_spi_serial_data* data =
		container_of(port, struct pseudo_spi_serial_data, tty_port);
	unsigned long flags;

	if (data == NULL)
	{
		pr_err("%s: No device.\n", __func__);
		return -ENODEV;
	}

	spin_lock_irqsave(&data->lock, flags);

	/* save our structure within the tty structure */
	tty->driver_data = data;
	data->tty = tty;

	timer_setup(&data->timer, pseudo_spi_serial_timer, 0);
	mod_timer(&data->timer, jiffies + DELAY_TIME);

	kfifo_reset(&data->tx_fifo);

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static void pseudo_spi_serial_close(struct tty_struct *tty, struct file *file)
{
	struct pseudo_spi_serial_data* data = tty->driver_data;

	if (data)
	{
		tty_port_close(&data->tty_port, tty, file);
	}
}

static void pseudo_spi_serial_port_shutdown(struct tty_port *port)
{
	struct pseudo_spi_serial_data* data =
		container_of(port, struct pseudo_spi_serial_data, tty_port);
	unsigned long flags;

	if (data)
	{
		spin_lock_irqsave(&data->lock, flags);

		del_timer(&data->timer);
		tasklet_kill(&data->io_tasklet);

		spin_unlock_irqrestore(&data->lock, flags);
	}
}

int pseudo_spi_serial_do_write(struct pseudo_spi_serial_data* data,
							   const unsigned char *buffer, int count)
{
	int retval = -EINVAL;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	retval = kfifo_in(&data->tx_fifo, buffer, count);

	if (retval > 0)
	{
		tasklet_schedule(&data->io_tasklet);
	}

	spin_unlock_irqrestore(&data->lock, flags);

	return retval;
}

int pseudo_spi_serial_write(struct tty_struct *tty, const unsigned char *buffer, int count)
{
	struct pseudo_spi_serial_data* data = tty->driver_data;

	if (!data)
	{
		pr_err("%s: No device.\n", __func__);
		return -ENODEV;
	}
	
	return pseudo_spi_serial_do_write(data, buffer, count);
}

static int pseudo_spi_serial_write_room(struct tty_struct *tty) 
{
	struct pseudo_spi_serial_data* data = tty->driver_data;
	int room = -EINVAL;
	unsigned long flags;

	if (!data)
	{
		pr_err("%s: No device.\n", __func__);
		return -ENODEV;
	}

	spin_lock_irqsave(&data->lock, flags);
	
	room = PSEUDO_SPI_SERIAL_FIFO_SIZE - kfifo_len(&data->tx_fifo);

	spin_unlock_irqrestore(&data->lock, flags);

	return room;
}

static void pseudo_spi_serial_hangup(struct tty_struct *tty)
{
	struct pseudo_spi_serial_data* data = tty->driver_data;
	
	if (data)
	{
		tty_port_hangup(&data->tty_port);
	}
}

static int pseudo_spi_serial_chars_in_buffer(struct tty_struct *tty) 
{
	struct pseudo_spi_serial_data* data = tty->driver_data;
	int cnt_chars = -EINVAL;
	unsigned long flags;

	if (!data)
	{
		pr_err("%s: No device.\n", __func__);
		return -ENODEV;
	}

	spin_lock_irqsave(&data->lock, flags);
	
	cnt_chars = kfifo_len(&data->tx_fifo);

	spin_unlock_irqrestore(&data->lock, flags);

	return cnt_chars;
}

static int pseudo_spi_serial_get_id(void)
{
	int i = 0;

	while ((i < PSEUDO_SPI_SERIAL_TTY_MINORS) && (test_and_set_bit(i, &pseudo_spi_serial_mask)))
	{
		++i;
	}

	return i;
}

static void pseudo_spi_serial_release_id(int id)
{
	clear_bit(id, &pseudo_spi_serial_mask);
}

static void pseudo_spi_serial_io_completed(void* context)
{
	struct pseudo_spi_serial_data* data = (struct pseudo_spi_serial_data*)context;
	struct spi_transfer* transfer;
	int i;
	int push = 0;
	unsigned long flags;

	if (data->spi_msg.status != 0)
	{
		pr_err("%s: SPI IO transfer error: %d\n", __func__, data->spi_msg.status);
	}
	else
	{
		transfer = list_last_entry(&data->spi_msg.transfers, struct spi_transfer, transfer_list);

		for (i = 0; i < transfer->len; i+=2)
		{
			if (data->rx_buffer[i] == 0x01)
			{
				pr_debug("Received char(%d): 0x%02X\n", (i / 2), (unsigned)data->rx_buffer[i + 1]);
				if (data->tty)
				{
					tty_insert_flip_char(data->tty->port, data->rx_buffer[i + 1], TTY_NORMAL);
				}
				push = 1;
			}
		}

		if ((push) && (data->tty))
		{
			tty_flip_buffer_push(data->tty->port);
		}
	}

	spin_lock_irqsave(&data->lock, flags);

	data->spi_msg.context = NULL;

	if ((!kfifo_is_empty(&data->tx_fifo)) || (push))
	{
		tasklet_schedule(&data->io_tasklet);
	}

	spin_unlock_irqrestore(&data->lock, flags);
}

static void pseudo_spi_serial_io(unsigned long context)
{
	struct pseudo_spi_serial_data* data = (struct pseudo_spi_serial_data*)context;
	int retval;
	int i;
	unsigned tx_count;

	if (data->spi_msg.context)
	{
		return;
	}

	tx_count = kfifo_out(&data->tx_fifo, data->tx_buffer, PSEUDO_SPI_SERIAL_FIFO_SIZE);

	spi_message_init(&data->spi_msg);
	data->spi_msg.context  = data;
	data->spi_msg.complete = pseudo_spi_serial_io_completed;

	memset(&data->spi_xfer, 0, sizeof(data->spi_xfer));

	data->spi_xfer.len    = PSEUDO_SPI_SERIAL_READ_SIZE * 2;
	data->spi_xfer.rx_buf = data->rx_buffer;

	if (tx_count > 0)
	{
		if (data->tty)
		{
			tty_wakeup(data->tty);
		}

		for (i = tx_count - 1; i >= 0; --i)
		{
			data->tx_buffer[i * 2 + 1] = data->tx_buffer[i];
			data->tx_buffer[i * 2]     = 0x01;
		}

		if (pseudo_spi_serial_duplex)
		{
			data->spi_xfer.len = max(data->spi_xfer.len, tx_count * 2);
		}
		else
		{
			data->spi_xfer.len += tx_count * 2;
		}

		if (data->spi_xfer.len > tx_count * 2)
		{
			memset(&data->tx_buffer[tx_count * 2], 0, data->spi_xfer.len - (tx_count * 2));
		}
		
		data->spi_xfer.tx_buf = data->tx_buffer;
	}

	if (pseudo_spi_serial_use_dma)
	{
		data->spi_msg.is_dma_mapped = 1;
		data->spi_xfer.tx_dma = data->tx_dma;
		data->spi_xfer.rx_dma = data->rx_dma;
	}

	spi_message_add_tail(&data->spi_xfer, &data->spi_msg);

	retval = spi_async(data->spi, &data->spi_msg);

	if (retval)
	{
		data->spi_msg.context = NULL;
		pr_err("%s: spi_async() failed: %d\n", __func__, retval);
	}
}

static const struct tty_port_operations tty_port_ops = {
	.activate = pseudo_spi_serial_port_activate,
	.shutdown = pseudo_spi_serial_port_shutdown,
};

static int pseudo_spi_serial_probe(struct spi_device *spi)
{
	struct pseudo_spi_serial_data* data;
	int ret = 0;
	int id = 0;

	ret = spi_setup(spi);
	if (ret < 0)
	{
		dev_dbg(&spi->dev, "needs SPI mode %02x, %d KHz; %d\n",
				spi->mode, spi->max_speed_hz / 1000, ret);
		return ret;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
	{
		dev_err(&spi->dev, "failed to allocate memory for device data.\n" );
		return -ENOMEM;
	}

	if (of_find_property(spi->dev.of_node, "serial_pseudo_spi,duplex", NULL))
	{
		pseudo_spi_serial_duplex = 1;
	}

	if (of_find_property(spi->dev.of_node, "serial_pseudo_spi,use_dma", NULL))
	{
		pseudo_spi_serial_use_dma = 1;
	}

	data->spi = spi;

	spin_lock_init(&data->lock);

	ret = dma_set_coherent_mask(&spi->dev, DMA_BIT_MASK(32));

	if (ret)
	{
		dev_err(&spi->dev, "Failed to set DMA-mask");
		goto err_free;
	}

	if (pseudo_spi_serial_duplex)
		data->size_buffer = PSEUDO_SPI_SERIAL_FIFO_SIZE * 2;
	else
		data->size_buffer = PSEUDO_SPI_SERIAL_FIFO_SIZE * 2 + PSEUDO_SPI_SERIAL_READ_SIZE * 2;

	data->tx_buffer = dma_alloc_coherent(&spi->dev, data->size_buffer, &data->tx_dma, GFP_KERNEL);

	if (!data->tx_buffer)
	{
		dev_err(&spi->dev, "DMA-TX buffer allocation failed");
		ret = -ENOMEM;
		goto err_free;
	}

	data->rx_buffer = dma_alloc_coherent(&spi->dev, data->size_buffer, &data->rx_dma, GFP_KERNEL);

	if (!data->rx_buffer)
	{
		dev_err(&spi->dev, "DMA-RX buffer allocation failed");
		ret = -ENOMEM;
		goto err_free;
	}

	if (kfifo_alloc(&data->tx_fifo, PSEUDO_SPI_SERIAL_FIFO_SIZE, GFP_KERNEL))
	{
		dev_err(&spi->dev, "failed to allocate memory for TX-fifo.\n" );
		ret = -ENOMEM;
		goto err_free;
	}

	tasklet_init(&data->io_tasklet, pseudo_spi_serial_io, (unsigned long)data);

	dev_set_drvdata(&spi->dev, data);

	id = pseudo_spi_serial_get_id();

	if (id >= PSEUDO_SPI_SERIAL_TTY_MINORS)
	{
		dev_err(&spi->dev, "device count exceed.\n");
		ret = -EBUSY;
		goto err_fifo_free;
	}

	pseudo_spi_serial_table[id] = data;

	tty_port_init(&data->tty_port);
	data->tty_port.ops = &tty_port_ops;

	data->tty_dev = tty_port_register_device(&data->tty_port, pseudo_spi_serial_tty_driver,
											 id, &spi->dev);
	if (IS_ERR(data->tty_dev))
	{
		dev_dbg(&spi->dev, "%s: registering tty device failed", __func__);
		ret = PTR_ERR(data->tty_dev);
		goto err_port;
	}

	if (id == 0)
	{
		pseudo_spi_console_init(data);
	}

	return 0;

err_port:
	if (data->tty_dev)
	{
		tty_unregister_device(pseudo_spi_serial_tty_driver, id);
	}
	tty_port_destroy(&data->tty_port);

err_fifo_free:
	kfifo_free(&data->tx_fifo);
	dev_set_drvdata(&spi->dev, 0);
err_free:
	if (data->tx_buffer)
	{
		dma_free_coherent(&spi->dev, data->size_buffer, data->tx_buffer, data->tx_dma);
	}
	if (data->rx_buffer)
	{
		dma_free_coherent(&spi->dev, data->size_buffer, data->rx_buffer, data->tx_dma);
	}
	kfree(data);

	return ret;
}


static int pseudo_spi_serial_remove(struct spi_device *spi)
{
	int id;
	struct pseudo_spi_serial_data* data = dev_get_drvdata(&spi->dev);

	del_timer(&data->timer);

	dev_set_drvdata(&spi->dev, 0);

	for (id = 0; id < PSEUDO_SPI_SERIAL_TTY_MINORS; ++id)
	{
		if (pseudo_spi_serial_table[id] == data)
		{
			pseudo_spi_serial_table[id] = 0;
			break;
		}
	}

	if (id == 0)
	{
		pseudo_spi_console_exit();
	}

	pseudo_spi_serial_release_id(id);

	tasklet_kill(&data->io_tasklet);

	if (data->tty_dev)
	{
		tty_unregister_device(pseudo_spi_serial_tty_driver, id);
	}
	tty_port_destroy(&data->tty_port);

	kfifo_free(&data->tx_fifo);
	dma_free_coherent(&spi->dev, data->size_buffer, data->tx_buffer, data->tx_dma);
	dma_free_coherent(&spi->dev, data->size_buffer, data->rx_buffer, data->tx_dma);
	kfree(data);

	return 0;
}

static const struct tty_operations serial_ops = {
	.open =				pseudo_spi_serial_open,
	.close =			pseudo_spi_serial_close,
	.write =			pseudo_spi_serial_write,
	.write_room =		pseudo_spi_serial_write_room,
	.hangup = 			pseudo_spi_serial_hangup,
	.chars_in_buffer = 	pseudo_spi_serial_chars_in_buffer
};

static int __init pseudo_spi_serial_init(void)
{
	int result;

	pseudo_spi_serial_tty_driver = alloc_tty_driver(PSEUDO_SPI_SERIAL_TTY_MINORS);
	if (!pseudo_spi_serial_tty_driver)
	{
		pr_err("%s: Failed to allocate tty driver.\n", __func__);
		return -ENOMEM;
	}

	pseudo_spi_serial_tty_driver->owner = THIS_MODULE;
	pseudo_spi_serial_tty_driver->driver_name = "spiserial";
	pseudo_spi_serial_tty_driver->name = "ttySPI";
	pseudo_spi_serial_tty_driver->major = PSEUDO_SPI_SERIAL_TTY_MAJOR;
	pseudo_spi_serial_tty_driver->minor_start = 0;
	pseudo_spi_serial_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	pseudo_spi_serial_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	pseudo_spi_serial_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	pseudo_spi_serial_tty_driver->init_termios = tty_std_termios;
	pseudo_spi_serial_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	pseudo_spi_serial_tty_driver->init_termios.c_ispeed = 9600;
	pseudo_spi_serial_tty_driver->init_termios.c_ospeed = 9600;

	tty_set_operations(pseudo_spi_serial_tty_driver, &serial_ops);

	result = tty_register_driver(pseudo_spi_serial_tty_driver);
	if (result)
	{
		pr_err("%s: tty_register_driver failed\n", __func__);
		put_tty_driver(pseudo_spi_serial_tty_driver);
		return result;
	}

	return result;
}

static void __exit pseudo_spi_serial_exit(void)
{
	tty_unregister_driver(pseudo_spi_serial_tty_driver);
}

static const struct of_device_id pseudo_spi_serial_of_match_table[] = {
	{ .compatible = "rcm,serial_pseudo_spi", },
	{},
};
MODULE_DEVICE_TABLE(of, pseudo_spi_serial_of_match_table);

static struct spi_driver pseudo_spi_serial_driver = {
	.driver = {
		.name 			= "pseudo_spi_serial",
		.of_match_table = pseudo_spi_serial_of_match_table,
	},
	.probe =	pseudo_spi_serial_probe,
	.remove =	pseudo_spi_serial_remove,
};

module_spi_driver(pseudo_spi_serial_driver);

MODULE_AUTHOR("Alexey Spirkov");
MODULE_DESCRIPTION("Serial pseudo SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:serial_pseudo_spi");

module_init(pseudo_spi_serial_init);
module_exit(pseudo_spi_serial_exit);
