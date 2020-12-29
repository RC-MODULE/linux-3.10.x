// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2020 Alexey Spirkov <dev@alsp.net>
 *  2020 Vladimir Shalyt <Vladimir.Shalyt@mir.dev>
 *  - DMA support
 */
//#define DEBUG

#include "rcm_muart.h"

#define MUART_MDMA_RX_TIMEOUT 16 // in bit intervals

#define MUART_DMA_BUFF_SIZE UART_XMIT_SIZE

#define MUART_DMA_RX_TIMEOUT 200

#define MUART_CTRL_DUM_i 15

static int muart_dma_push_rx_data(struct uart_port* port, char* buf, int len)
{
	unsigned long flags;
	int copied = 0;
	int res = 0;
	struct tty_port* tty_port = &port->state->port;

	while (len) {
		spin_lock_irqsave(&port->lock, flags);
		copied = tty_insert_flip_string(tty_port, buf, len);
		tty_flip_buffer_push(tty_port);
		spin_unlock_irqrestore(&port->lock, flags);
		if (!copied)
			break;
		len -= copied;
		res += copied;
	}
	return res;
}

static void muart_rx_dma_callback(void *param)
{
	struct muart_dma_buff *buff = param;
	struct muart_dma* dma = buff->dma;
	struct muart_port *uart = dma->chan->private;
	unsigned long flags;
	enum dma_status status;
	struct dma_tx_state state;

	spin_lock_irqsave(&dma->lock, flags);

	status = dmaengine_tx_status(dma->chan, buff->cookie, &state);

	if (status != DMA_COMPLETE) {
		dev_warn(uart->port.dev, "RX DMA error (cookie = 0x%08X).\n",
		         buff->cookie);
	} else {
		dev_dbg(uart->port.dev, 
		        "RX DMA operation finished successfully "
		        "(cookie = 0x%08X, residue = %u).\n", 
		        buff->cookie, state.residue);
	}

	buff->error = (status != DMA_COMPLETE);
	buff->residue = state.residue;
	complete_all(&buff->dma_completion);

	spin_unlock_irqrestore(&dma->lock, flags);
}

static int muart_rx_start(struct muart_dma* dma)
{
	struct muart_port *uart = dma->chan->private;
	unsigned long flags;
	struct muart_dma_buff *buff;
	struct dma_async_tx_descriptor *desc;
	int cnt = 0;

	spin_lock_irqsave(&dma->lock, flags);

	while (true) {
		if (dma->cnt_buffs == MUART_DMA_CNT_BUFFS_PER_CHAN)
			break;

		buff = &dma->buffs[dma->next_buff];

		sg_dma_address(&buff->sg) = 
#ifdef CONFIG_BASIS_PLATFORM
			dma->ep_addr + dma->next_buff * MUART_DMA_BUFF_SIZE;
#else
			dma->dma_addr + dma->next_buff * MUART_DMA_BUFF_SIZE;
#endif
		sg_dma_len(&buff->sg) = MUART_DMA_BUFF_SIZE;

//		memset(dma->buff + dma->next_buff * MUART_DMA_BUFF_SIZE,
//		       0xCC, MUART_DMA_BUFF_SIZE);

		dev_dbg(uart->port.dev,
		        "RX DMA operation ready (%u bytes).\n",
		        sg_dma_len(&buff->sg));

		desc = dmaengine_prep_slave_sg(dma->chan, &buff->sg, 1,
		                               DMA_DEV_TO_MEM,
		                               DMA_PREP_INTERRUPT);

		if (!desc) {
			dev_warn(uart->port.dev,
			         "Failed prepare DMA RX-operation.\n");
			break;
		}

		reinit_completion(&buff->dma_completion);

		desc->callback = muart_rx_dma_callback;
		desc->callback_param = buff;

		dev_dbg(uart->port.dev,
		        "RX DMA operation prepared (%u bytes).\n", 
		        sg_dma_len(&buff->sg));

		buff->cookie = dmaengine_submit(desc);
		dma_async_issue_pending(dma->chan); 

		dev_dbg(uart->port.dev,
		        "RX DMA operation submitted "
		        "(%u bytes, cookie = 0x%08X).\n", 
		        sg_dma_len(&buff->sg), buff->cookie);

		dma->next_buff = (dma->next_buff + 1) % 
		                 MUART_DMA_CNT_BUFFS_PER_CHAN;
		++dma->cnt_buffs;

		++cnt;
	}

	spin_unlock_irqrestore(&dma->lock, flags);

	return cnt;
}

static bool muart_rx_wait(struct muart_dma* dma)
{
	unsigned long flags;
	struct muart_dma_buff *buff = NULL;

	spin_lock_irqsave(&dma->lock, flags);

	if (dma->cnt_buffs > 0) 
		buff = &dma->buffs[dma->first_buff];

	spin_unlock_irqrestore(&dma->lock, flags);

	if (!buff)
		return false;

	if (!wait_for_completion_timeout(&buff->dma_completion,
	                                 msecs_to_jiffies(MUART_DMA_RX_TIMEOUT)))
		return false;

	return true;
}

static int muart_rx_process(struct muart_dma* dma)
{
	struct muart_port *uart = dma->chan->private;
	unsigned long flags;
	struct muart_dma_buff *buff;
	int processed = 0;
	int pushed;
	void *ptr;

	spin_lock_irqsave(&dma->lock, flags);

	while (true) {
		if (dma->cnt_buffs == 0)
			break;

		buff = &dma->buffs[dma->first_buff];
		ptr = dma->buff + dma->first_buff * MUART_DMA_BUFF_SIZE;

		if (!try_wait_for_completion(&buff->dma_completion))
			break;

		if (buff->error) {
			dev_warn(uart->port.dev,
			         "RX DMA error (cookie = 0x%08X).\n",
			         buff->cookie);
		} else {
			pushed = muart_dma_push_rx_data(&uart->port, ptr,
			                                sg_dma_len(&buff->sg) -
			                                buff->residue);
			if (pushed < sg_dma_len(&buff->sg) - buff->residue)
				dev_err(uart->port.dev,
				        "Failed to push RX-data"
				        "(received - %lld, pushed - %u).\n",
				        (u64)(sg_dma_len(&buff->sg) - buff->residue),
				        pushed);
		}

		dma->first_buff = (dma->first_buff + 1) % 
		                  MUART_DMA_CNT_BUFFS_PER_CHAN;
		--dma->cnt_buffs;
		++processed;
	}

	spin_unlock_irqrestore(&dma->lock, flags);

	return processed;
}

static int muart_dma_rx(void* context)
{
	struct muart_dma* dma = context;

	while (!kthread_should_stop()) {
		muart_rx_start(dma);

		if (muart_rx_wait(dma))
			muart_rx_process(dma);
	}

	return 0;
}

void muart_dma_stop_rx(struct muart_port *uart)
{
	if (uart->rx.thread) {
		kthread_stop(uart->rx.thread);
		uart->rx.thread = NULL;
	}
}

static void muart_tx_dma_callback(void *param)
{
	struct muart_dma_buff *buff = param;
	struct muart_dma* dma = buff->dma;
	struct muart_port *uart = dma->chan->private;
	unsigned long flags;

	spin_lock_irqsave(&dma->lock, flags);
	
	do {
		if (dma->cnt_buffs == 0) {
			dev_warn(uart->port.dev, 
			         "Unexpected TX DMA callback: "
			         "no submitted transfers.\n");
			break;
		}

		if (buff != &dma->buffs[dma->first_buff])
		{
			dev_warn(uart->port.dev,
			         "Unexpected TX DMA callback: "
			         "unexpected buffer completed.\n");
			break;
		}

		if (dma_async_is_tx_complete(dma->chan, buff->cookie,
		                             NULL, NULL) != DMA_COMPLETE) {
			dev_warn(uart->port.dev, 
			         "TX DMA error (cookie = 0x%08X).\n",
			         buff->cookie);
		} else {
			dev_dbg(uart->port.dev, 
			        "TX DMA operation finished successfully "
			        "(%u bytes, cookie = 0x%08X).\n", 
			        sg_dma_len(&buff->sg), buff->cookie);
		}

		dma->first_buff = (dma->first_buff + 1) % 
		                  MUART_DMA_CNT_BUFFS_PER_CHAN;
		--dma->cnt_buffs;
	} while (0);

	spin_unlock_irqrestore(&dma->lock, flags);

	spin_lock_irqsave(&uart->port.lock, flags);

	if ((!uart_circ_empty(&uart->port.state->xmit)) && 
	    (!uart_tx_stopped(&uart->port)))
		muart_dma_tx_start(uart);

	spin_unlock_irqrestore(&uart->port.lock, flags);
}

bool muart_dma_tx_start(struct muart_port *uart)
{
	struct muart_dma* dma = &uart->tx;
	struct circ_buf *xmit = &uart->port.state->xmit;
	unsigned count = 0;
	unsigned cnt_to_end;
	unsigned long flags;
	struct muart_dma_buff *buff;
	void* ptr;
	struct dma_async_tx_descriptor *desc;

	if (uart_circ_empty(xmit) || uart_tx_stopped(&uart->port))
		return true;

	spin_lock_irqsave(&dma->lock, flags);

	if (dma->cnt_buffs == MUART_DMA_CNT_BUFFS_PER_CHAN) {
		dev_dbg(uart->port.dev, "No DMA buffers for TX operation.\n");
		spin_unlock_irqrestore(&dma->lock, flags);
		return false;
	}

	buff = &dma->buffs[dma->next_buff];
	ptr = dma->buff + dma->next_buff * MUART_DMA_BUFF_SIZE;

	while (1) {
		cnt_to_end = CIRC_CNT_TO_END(xmit->head, xmit->tail,
		                             UART_XMIT_SIZE);
		if (!cnt_to_end)
			break;

		BUG_ON(count + cnt_to_end > MUART_DMA_BUFF_SIZE);

		memcpy(ptr, &xmit->buf[xmit->tail], cnt_to_end);

		ptr += cnt_to_end;

		xmit->tail = (xmit->tail + cnt_to_end) & (UART_XMIT_SIZE - 1);
		count += cnt_to_end;
	}

	sg_dma_address(&buff->sg) = 
#ifdef CONFIG_BASIS_PLATFORM
		dma->ep_addr + dma->next_buff * MUART_DMA_BUFF_SIZE;
#else
		dma->dma_addr + dma->next_buff * MUART_DMA_BUFF_SIZE;
#endif
	sg_dma_len(&buff->sg) = count;

	desc = dmaengine_prep_slave_sg(dma->chan, &buff->sg, 1, DMA_MEM_TO_DEV, 
	                               DMA_PREP_INTERRUPT);

	if (!desc) {
		dev_err(uart->port.dev, "Failed prepare DMA TX-operation.\n");
		spin_unlock_irqrestore(&dma->lock, flags);
		return false;
	}

	desc->callback = muart_tx_dma_callback;
	desc->callback_param = buff;

	dev_dbg(uart->port.dev, "TX DMA operation prepared (%u bytes).\n", 
	        sg_dma_len(&buff->sg));

	buff->cookie = dmaengine_submit(desc);
	dma_async_issue_pending(dma->chan); 

	dev_dbg(uart->port.dev,
	        "TX DMA operation submitted (%u bytes, cookie = 0x%08X).\n", 
	        sg_dma_len(&buff->sg), buff->cookie);

	dma->next_buff = (dma->next_buff + 1) % MUART_DMA_CNT_BUFFS_PER_CHAN;
	++dma->cnt_buffs;

	spin_unlock_irqrestore(&dma->lock, flags);

	uart_write_wakeup(&uart->port);

	return true;
}

#ifdef CONFIG_BASIS_PLATFORM
struct dma_chan *muart_find_by_chan_id(struct dma_device *dev, int chan_id)
{
	struct dma_chan *chan, *candidate = NULL;

	list_for_each_entry(chan, &dev->channels, device_node)
		if (chan->chan_id == chan_id) {
			candidate = chan;
			break;
		}

	if (!candidate)
		return NULL;

	return dma_get_slave_channel(candidate);
}
#endif

static void muart_release_dma(struct muart_port *uart)
{
	int i;

	for (i = 0; i < 2; ++i) {
		struct muart_dma* dma =  (i == 0) ? &uart->tx : &uart->rx;

		if (!dma->chan)
			continue;
		dma_release_channel(dma->chan);
		dma->chan = NULL;
	}
}

static int muart_request_dma(struct muart_port *uart)
{
	int ret = 0;
	int i;
#ifdef CONFIG_BASIS_PLATFORM
	struct dma_device *dma_dev;
	struct basis_device *device;

	device = basis_device_find(uart->dma_dev);
	if (!device) {
		dev_err(uart->port.dev, "Failed to found BASIS-device \"%s\"\n",
		        uart->dma_dev);
		return -ENODEV;
	}

	if (!device->priv) {
		dev_err(uart->port.dev,
		        "BASIS-device \"%s\" is not DMA-Device\n",
		        uart->dma_dev);
		return -ENODEV;
	}

	dma_dev = device->priv;
#endif

	for (i = 0; i < 2; ++i) {
#ifdef CONFIG_BASIS_PLATFORM
		u32 ch_num = (i == 0) ? uart->tx_ch_num : uart->rx_ch_num;
#else
		char *ch_name = (i == 0) ? "tx0" : "rx0";
#endif
		struct muart_dma* dma =  (i == 0) ? &uart->tx : &uart->rx;

		dma->next_buff  = 0;
		dma->first_buff = 0;
		dma->cnt_buffs  = 0;

#ifdef CONFIG_BASIS_PLATFORM
		dma->chan = muart_find_by_chan_id(dma_dev, ch_num);
#else
		dma->chan = dma_request_chan(uart->port.dev, ch_name);
#endif
		if ((!dma->chan) || (IS_ERR(dma->chan))) {
			dev_err(uart->port.dev,
			        "No DMA %s channel specified.\n",
			        (i == 0) ? "TX" : "RX");
			dma->chan = NULL;
			ret = -ENODEV;
			break;
		}

		dma->chan->private = uart;
	}

	if (ret != 0)
		muart_release_dma(uart);

	return ret;
}

int muart_dma_startup(struct muart_port *uart)
{
	struct muart_regs *regs = (struct muart_regs *)uart->port.membase;
	int err;

	err = muart_request_dma(uart);
	if (err)
		return err;

	writel(MUART_MDMA_RX_TIMEOUT, &regs->rxtimeout);

	uart->rx.thread = kthread_run(muart_dma_rx, &uart->rx, "muart_dma_rx");

	if (IS_ERR(uart->rx.thread)) {
		dev_err(uart->port.dev, "Failed to start RX-thread.\n");
		err = PTR_ERR(uart->rx.thread);
		muart_release_dma(uart);
		return err;
	};

	return 0;
}

void muart_dma_shutdown(struct muart_port *uart)
{
	muart_dma_stop_rx(uart);
	muart_release_dma(uart);
}

void muart_free_dma(struct muart_port *uart)
{
	int i;

	for (i = 0; i < 2; ++i) {
		struct muart_dma* dma =  (i == 0) ? &uart->tx : &uart->rx;

		if (dma->buff) {
#ifdef CONFIG_BASIS_PLATFORM
			basis_device_dma_free_coherent(
				uart->port.dev,
				MUART_DMA_CNT_BUFFS_PER_CHAN * 
				MUART_DMA_BUFF_SIZE, 
				dma->buff, dma->dma_addr, dma->ep_addr);
#else
			dma_free_coherent(uart->port.dev, 
			                  MUART_DMA_CNT_BUFFS_PER_CHAN * 
			                  MUART_DMA_BUFF_SIZE, 
			                  dma->buff, dma->dma_addr);
#endif
			dma->buff = NULL;
		}
	}
}

int muart_alloc_dma(struct muart_port *uart)
{
	int ret = 0;
	int i;
	int j;

	for (i = 0; i < 2; ++i) {
		struct muart_dma* dma =  (i == 0) ? &uart->tx : &uart->rx;

		spin_lock_init(&dma->lock);

		dma->buff = 
#ifdef CONFIG_BASIS_PLATFORM
			basis_device_dma_alloc_coherent(
				uart->port.dev,
				MUART_DMA_CNT_BUFFS_PER_CHAN * 
				MUART_DMA_BUFF_SIZE,
				&dma->dma_addr, &dma->ep_addr, GFP_KERNEL);
#else
			dma_alloc_coherent(uart->port.dev,
			                   MUART_DMA_CNT_BUFFS_PER_CHAN * 
			                   MUART_DMA_BUFF_SIZE,
			                   &dma->dma_addr, GFP_KERNEL);
#endif
		if (!dma->buff) {
			dev_err(uart->port.dev,
			        "Failed to allocate DMA-buffer.\n");
			ret = -ENOMEM;
			break;
		}

		for (j = 0; j < MUART_DMA_CNT_BUFFS_PER_CHAN; ++j) {
			dma->buffs[j].dma = dma;
			sg_init_table(&dma->buffs[j].sg, 1);
			init_completion(&dma->buffs[j].dma_completion);
		}
	}

	if (ret != 0)
		muart_free_dma(uart);

	return ret;
}

