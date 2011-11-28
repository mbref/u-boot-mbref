/*
 * Xilinx xps_ll_temac ethernet driver for u-boot
 *
 * FIFO interface
 *
 * Copyright (C) 2008 - 2011 Michal Simek <monstr@monstr.eu>
 * Copyright (C) 2008 - 2011 PetaLogix
 *
 * Copyright (C) 2011 Stephan Linz <linz@li-pro.net>
 *
 * Based on Yoshio Kashiwagi kashiwagi@co-nss.co.jp driver
 * Copyright (C) 2008 Nissin Systems Co.,Ltd.
 * March 2008 created
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <config.h>
#include <common.h>
#include <net.h>

#include <asm/types.h>
#include <asm/io.h>

#include "xilinx_ll_temac.h"

#ifdef DEBUG
static void debugll(struct eth_device *dev, int count)
{
	struct ll_temac *ll_temac = dev->priv;
	struct fifo_ctrl *fifo_ctrl = (void *)ll_temac->ctrladdr;

	printf("FIFO: %d isr 0x%08x, ier 0x%08x, rdfr 0x%08x, "
		"rdfo 0x%08x rlr 0x%08x\n", count,
		in_be32(&fifo_ctrl->isr), in_be32(&fifo_ctrl->ier),
		in_be32(&fifo_ctrl->rdfr), in_be32(&fifo_ctrl->rdfo),
		in_be32(&fifo_ctrl->rlf));
}
#endif

int ll_temac_reset_fifo(struct eth_device *dev)
{
	struct ll_temac *ll_temac = dev->priv;
	struct fifo_ctrl *fifo_ctrl = (void *)ll_temac->ctrladdr;

	out_be32(&fifo_ctrl->tdfr, LL_FIFO_TDFR_KEY);
	out_be32(&fifo_ctrl->rdfr, LL_FIFO_RDFR_KEY);
	out_be32(&fifo_ctrl->isr, ~0UL);
	out_be32(&fifo_ctrl->ier, 0);

	return 0;
}

int ll_temac_recv_fifo(struct eth_device *dev)
{
	struct ll_temac *ll_temac = dev->priv;
	struct fifo_ctrl *fifo_ctrl = (void *)ll_temac->ctrladdr;
	uchar *rx_bp = ll_temac->rx_bp;
	u32 *buf = (u32 *)rx_bp;
	u32 i, len = 0;

	if (in_be32(&fifo_ctrl->isr) & LL_FIFO_ISR_RC) {

		/* reset isr */
		out_be32(&fifo_ctrl->isr, ~0UL);

		/* while (fifo_ctrl->isr); */
		len = in_be32(&fifo_ctrl->rlf) & LL_FIFO_RLF_MASK;

		for (i = 0; i < len; i += 4)
			*buf++ = in_be32(&fifo_ctrl->rdfd);

#ifdef DEBUG
		debugll(dev, 1);
#endif

		NetReceive((uchar *)rx_bp, len);
	}

	return len;
}

int ll_temac_send_fifo(struct eth_device *dev,
			volatile void *buffer, int length)
{
	struct ll_temac *ll_temac = dev->priv;
	struct fifo_ctrl *fifo_ctrl = (void *)ll_temac->ctrladdr;
	u32 *buf = (u32 *)buffer;
	u32 i;

	for (i = 0; i < length; i += 4)
		out_be32(&fifo_ctrl->tdfd, *buf++);

	out_be32(&fifo_ctrl->tlf, length);
	return 0;
}
