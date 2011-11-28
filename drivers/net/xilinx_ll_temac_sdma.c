/*
 * Xilinx xps_ll_temac ethernet driver for u-boot
 *
 * SDMA interface
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

#if defined(CONFIG_XILINX_440) || defined(CONFIG_XILINX_405)

/*
 * Indirect DCR access operations mi{ft}dcr_xilinx() espacialy
 * for Xilinx PowerPC implementations on FPGA.
 *
 * FIXME: This part should go up to arch/powerpc -- but where?
 */
#include <asm/processor.h>
#define XILINX_INDIRECT_DCR_ADDRESS_REG	0
#define XILINX_INDIRECT_DCR_ACCESS_REG	1
inline unsigned mifdcr_xilinx(const unsigned dcrn)
{
	mtdcr(XILINX_INDIRECT_DCR_ADDRESS_REG, dcrn);
	return mfdcr(XILINX_INDIRECT_DCR_ACCESS_REG);
}
inline void mitdcr_xilinx(const unsigned dcrn, int val)
{
	mtdcr(XILINX_INDIRECT_DCR_ADDRESS_REG, dcrn);
	mtdcr(XILINX_INDIRECT_DCR_ACCESS_REG, val);
}

/* Xilinx Device Control Register (DCR) in/out accessors */
inline unsigned ll_temac_xldcr_in32(phys_addr_t addr)
{
	return mifdcr_xilinx((const unsigned)addr);
}
inline void ll_temac_xldcr_out32(phys_addr_t addr, unsigned value)
{
	mitdcr_xilinx((const unsigned)addr, value);
}

void ll_temac_collect_xldcr_sdma_reg_addr(struct eth_device *dev)
{
	struct ll_temac *ll_temac = dev->priv;
	phys_addr_t dmac_ctrl = ll_temac->ctrladdr;
	phys_addr_t *ra = ll_temac->sdma_reg_addr;

	ra[TX_NXTDESC_PTR]   = dmac_ctrl + TX_NXTDESC_PTR;
	ra[TX_CURBUF_ADDR]   = dmac_ctrl + TX_CURBUF_ADDR;
	ra[TX_CURBUF_LENGTH] = dmac_ctrl + TX_CURBUF_LENGTH;
	ra[TX_CURDESC_PTR]   = dmac_ctrl + TX_CURDESC_PTR;
	ra[TX_TAILDESC_PTR]  = dmac_ctrl + TX_TAILDESC_PTR;
	ra[TX_CHNL_CTRL]     = dmac_ctrl + TX_CHNL_CTRL;
	ra[TX_IRQ_REG]       = dmac_ctrl + TX_IRQ_REG;
	ra[TX_CHNL_STS]      = dmac_ctrl + TX_CHNL_STS;
	ra[RX_NXTDESC_PTR]   = dmac_ctrl + RX_NXTDESC_PTR;
	ra[RX_CURBUF_ADDR]   = dmac_ctrl + RX_CURBUF_ADDR;
	ra[RX_CURBUF_LENGTH] = dmac_ctrl + RX_CURBUF_LENGTH;
	ra[RX_CURDESC_PTR]   = dmac_ctrl + RX_CURDESC_PTR;
	ra[RX_TAILDESC_PTR]  = dmac_ctrl + RX_TAILDESC_PTR;
	ra[RX_CHNL_CTRL]     = dmac_ctrl + RX_CHNL_CTRL;
	ra[RX_IRQ_REG]       = dmac_ctrl + RX_IRQ_REG;
	ra[RX_CHNL_STS]      = dmac_ctrl + RX_CHNL_STS;
	ra[DMA_CONTROL_REG]  = dmac_ctrl + DMA_CONTROL_REG;
}

#endif /* CONFIG_XILINX_440 || ONFIG_XILINX_405 */

/* Xilinx Processor Local Bus (PLB) in/out accessors */
inline unsigned ll_temac_xlplb_in32(phys_addr_t addr)
{
	return in_be32((void *)addr);
}
inline void ll_temac_xlplb_out32(phys_addr_t addr, unsigned value)
{
	out_be32((void *)addr, value);
}

/* collect all register addresses for Xilinx PLB in/out accessors */
void ll_temac_collect_xlplb_sdma_reg_addr(struct eth_device *dev)
{
	struct ll_temac *ll_temac = dev->priv;
	struct sdma_ctrl *sdma_ctrl = (void *)ll_temac->ctrladdr;
	phys_addr_t *ra = ll_temac->sdma_reg_addr;

	ra[TX_NXTDESC_PTR]   = (phys_addr_t)&sdma_ctrl->tx_nxtdesc_ptr;
	ra[TX_CURBUF_ADDR]   = (phys_addr_t)&sdma_ctrl->tx_curbuf_addr;
	ra[TX_CURBUF_LENGTH] = (phys_addr_t)&sdma_ctrl->tx_curbuf_length;
	ra[TX_CURDESC_PTR]   = (phys_addr_t)&sdma_ctrl->tx_curdesc_ptr;
	ra[TX_TAILDESC_PTR]  = (phys_addr_t)&sdma_ctrl->tx_taildesc_ptr;
	ra[TX_CHNL_CTRL]     = (phys_addr_t)&sdma_ctrl->tx_chnl_ctrl;
	ra[TX_IRQ_REG]       = (phys_addr_t)&sdma_ctrl->tx_irq_reg;
	ra[TX_CHNL_STS]      = (phys_addr_t)&sdma_ctrl->tx_chnl_sts;
	ra[RX_NXTDESC_PTR]   = (phys_addr_t)&sdma_ctrl->rx_nxtdesc_ptr;
	ra[RX_CURBUF_ADDR]   = (phys_addr_t)&sdma_ctrl->rx_curbuf_addr;
	ra[RX_CURBUF_LENGTH] = (phys_addr_t)&sdma_ctrl->rx_curbuf_length;
	ra[RX_CURDESC_PTR]   = (phys_addr_t)&sdma_ctrl->rx_curdesc_ptr;
	ra[RX_TAILDESC_PTR]  = (phys_addr_t)&sdma_ctrl->rx_taildesc_ptr;
	ra[RX_CHNL_CTRL]     = (phys_addr_t)&sdma_ctrl->rx_chnl_ctrl;
	ra[RX_IRQ_REG]       = (phys_addr_t)&sdma_ctrl->rx_irq_reg;
	ra[RX_CHNL_STS]      = (phys_addr_t)&sdma_ctrl->rx_chnl_sts;
	ra[DMA_CONTROL_REG]  = (phys_addr_t)&sdma_ctrl->dma_control_reg;
}

/* Check for TX and RX channel errors. */
static inline int ll_temac_sdma_error(struct eth_device *dev)
{
	int err;
	struct ll_temac *ll_temac = dev->priv;
	phys_addr_t *ra = ll_temac->sdma_reg_addr;

	err = ll_temac->in32(ra[TX_CHNL_STS]) & CHNL_STS_ERROR;
	err |= ll_temac->in32(ra[TX_CHNL_STS]) & CHNL_STS_ERROR;

	return err;
}

int ll_temac_init_sdma(struct eth_device *dev)
{
	struct ll_temac *ll_temac = dev->priv;
	struct cdmac_bd *rx_dp = ll_temac->rx_dp;
	struct cdmac_bd *tx_dp = ll_temac->tx_dp;
	phys_addr_t *ra = ll_temac->sdma_reg_addr;

	memset(tx_dp, 0, sizeof(*tx_dp));
	memset(rx_dp, 0, sizeof(*rx_dp));

	/* from LL TEMAC (Rx) */
	rx_dp->phys_buf_p = ll_temac->rx_bp;

	rx_dp->next_p = rx_dp;
	rx_dp->buf_len = PKTSIZE_ALIGN;
	flush_cache((u32)rx_dp, sizeof(*tx_dp));
	flush_cache((u32)rx_dp->phys_buf_p, PKTSIZE_ALIGN);

	/* setup first fd */
	ll_temac->out32(ra[RX_CURDESC_PTR], (int)rx_dp);
	ll_temac->out32(ra[RX_TAILDESC_PTR], (int)rx_dp);
	ll_temac->out32(ra[RX_NXTDESC_PTR], (int)rx_dp);

	/* to LL TEMAC (Tx) */
	tx_dp->phys_buf_p = ll_temac->tx_bp;
	tx_dp->next_p = tx_dp;

	flush_cache((u32)tx_dp, sizeof(*tx_dp));
	ll_temac->out32(ra[TX_CURDESC_PTR], (int)tx_dp);

	return 0;
}

int ll_temac_halt_sdma(struct eth_device *dev)
{
	unsigned timeout = 50;	/* 1usec * 50 = 50usec */
	struct ll_temac *ll_temac = dev->priv;
	phys_addr_t *ra = ll_temac->sdma_reg_addr;

	/*
	 * Soft reset the DMA
	 *
	 * Quote from MPMC documentation: Writing a 1 to this field
	 * forces the DMA engine to shutdown and reset itself. After
	 * setting this bit, software must poll it until the bit is
	 * cleared by the DMA. This indicates that the reset process
	 * is done and the pipeline has been flushed.
	 */
	ll_temac->out32(ra[DMA_CONTROL_REG], DMA_CONTROL_RESET);
	while (timeout && (ll_temac->in32(ra[DMA_CONTROL_REG])
					& DMA_CONTROL_RESET)) {
		timeout--;
		udelay(1);
	}

	if (!timeout) {
		printf("%s: Timeout\n", __func__);
		return -1;
	}

	return 0;
}

int ll_temac_reset_sdma(struct eth_device *dev)
{
	u32 r;
	struct ll_temac *ll_temac = dev->priv;
	phys_addr_t *ra = ll_temac->sdma_reg_addr;

	/* Soft reset the DMA.  */
	if (ll_temac_halt_sdma(dev))
		return -1;

	/* Now clear the interrupts.  */
	r = ll_temac->in32(ra[TX_CHNL_CTRL]);
	r &= ~CHNL_CTRL_IRQ_MASK;
	ll_temac->out32(ra[TX_CHNL_CTRL], r);

	r = ll_temac->in32(ra[RX_CHNL_CTRL]);
	r &= ~CHNL_CTRL_IRQ_MASK;
	ll_temac->out32(ra[RX_CHNL_CTRL], r);

	/* Now ACK pending IRQs.  */
	ll_temac->out32(ra[TX_IRQ_REG], IRQ_REG_IRQ_MASK);
	ll_temac->out32(ra[RX_IRQ_REG], IRQ_REG_IRQ_MASK);

	/* Set tail-ptr mode, disable errors for both channels.  */
	ll_temac->out32(ra[DMA_CONTROL_REG],
			/* Enable use of tail pointer register */
			DMA_CONTROL_TPE |
			/* Disable error when 2 or 4 bit coalesce cnt overfl */
			DMA_CONTROL_RXOCEID |
			/* Disable error when 2 or 4 bit coalesce cnt overfl */
			DMA_CONTROL_TXOCEID);

	return 0;
}

int ll_temac_recv_sdma(struct eth_device *dev)
{
	int length;
	struct ll_temac *ll_temac = dev->priv;
	struct cdmac_bd *rx_dp = ll_temac->rx_dp;
	phys_addr_t *ra = ll_temac->sdma_reg_addr;

	if (ll_temac_sdma_error(dev)) {

		if (ll_temac_reset_sdma(dev))
			return -1;

		ll_temac_init_sdma(dev);
	}

	flush_cache((u32)rx_dp, sizeof(*rx_dp));

	if (!(rx_dp->sca.stctrl & CDMAC_BD_STCTRL_COMPLETED))
		return 0;

	/*
	 * Read out the packet info and start the DMA
	 * onto the second buffer to enable the ethernet rx
	 * path to run in parallel with sw processing
	 * packets.
	 */
	length = rx_dp->sca.app[4] & CDMAC_BD_APP4_RXBYTECNT_MASK;
	if (length > 0)
		NetReceive(rx_dp->phys_buf_p, length);

	/* flip the buffer and re-enable the DMA.  */
	flush_cache((u32)rx_dp->phys_buf_p, length);

	rx_dp->buf_len = PKTSIZE_ALIGN;
	rx_dp->sca.stctrl = 0;
	rx_dp->sca.app[4] = 0;

	flush_cache((u32)rx_dp, sizeof(*rx_dp));
	ll_temac->out32(ra[RX_TAILDESC_PTR], (int)rx_dp);

	return length;
}

int ll_temac_send_sdma(struct eth_device *dev, volatile void *buffer,
							int length)
{
	unsigned timeout = 50;	/* 1usec * 50 = 50usec */
	struct ll_temac *ll_temac = dev->priv;
	struct cdmac_bd *tx_dp = ll_temac->tx_dp;
	phys_addr_t *ra = ll_temac->sdma_reg_addr;

	if (ll_temac_sdma_error(dev)) {

		if (ll_temac_reset_sdma(dev))
			return -1;

		ll_temac_init_sdma(dev);
	}

	memcpy(ll_temac->tx_bp, (void *)buffer, length);
	flush_cache((u32)ll_temac->tx_bp, length);

	tx_dp->sca.stctrl = CDMAC_BD_STCTRL_SOP | CDMAC_BD_STCTRL_EOP |
			CDMAC_BD_STCTRL_STOP_ON_END;
	tx_dp->buf_len = length;
	flush_cache((u32)tx_dp, sizeof(*tx_dp));

	/* DMA start */
	ll_temac->out32(ra[TX_CURDESC_PTR], (int)tx_dp);
	ll_temac->out32(ra[TX_TAILDESC_PTR], (int)tx_dp);

	do {
		flush_cache((u32)tx_dp, sizeof(*tx_dp));
		udelay(1);
	} while (timeout-- && !(tx_dp->sca.stctrl & CDMAC_BD_STCTRL_COMPLETED));

	if (!timeout) {
		printf("%s: Timeout\n", __func__);
		return -1;
	}

	return 0;
}
