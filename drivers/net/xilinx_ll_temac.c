/*
 * Xilinx xps_ll_temac ethernet driver for u-boot
 *
 * supports SDMA or FIFO access
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
#include <malloc.h>
#include <asm/io.h>
#include <phy.h>
#include <miiphy.h>

#include "xilinx_ll_temac.h"

#if !defined(CONFIG_MII) || !defined(CONFIG_CMD_MII)
# error "LL_TEMAC requires MII -- missing CONFIG_MII or CONFIG_CMD_MII"
#endif

#if !defined(CONFIG_PHYLIB)
# error "LL_TEMAC requires PHYLIB -- missing CONFIG_PHYLIB"
#endif

#if !defined(CONFIG_PHY_ADDR)
#define CONFIG_PHY_ADDR -1
#endif

/*
 * Prior to PHY access, the MDIO clock must be setup. This driver will set a
 * safe default that should work with PLB bus speeds of up to 150 MHz and keep
 * the MDIO clock below 2.5 MHz. If the user wishes faster access to the PHY
 * then the clock divisor can be set to a different value by setting the
 * correct bus speed value with CONFIG_XILINX_LL_TEMAC_CLK.
 */
#if !defined(CONFIG_XILINX_LL_TEMAC_CLK)
#define MDIO_CLOCK_DIV		MC_CLKDIV_10(150000000)
#else
#define MDIO_CLOCK_DIV		MC_CLKDIV_25(CONFIG_XILINX_LL_TEMAC_CLK)
#endif

/* Use MII register 1 (MII status register) to detect PHY */
#define PHY_DETECT_REG		1

/*
 * Mask used to verify certain PHY features (or register contents)
 * in the register above:
 *  0x1000: 10Mbps full duplex support
 *  0x0800: 10Mbps half duplex support
 *  0x0008: Auto-negotiation support
 */
#define PHY_DETECT_MASK		0x1808

/* Data buffer for LL TEMAC Rx and Tx direction */
static unsigned char rx_buffer[PKTSIZE_ALIGN] __attribute((aligned(DMAALIGN)));
static unsigned char tx_buffer[PKTSIZE_ALIGN] __attribute((aligned(DMAALIGN)));

/* CDMAC buffer descriptor for LL TEMAC Rx and Tx buffer handling */
static struct cdmac_bd rx_descr __attribute((aligned(DMAALIGN)));
static struct cdmac_bd tx_descr __attribute((aligned(DMAALIGN)));

/* Ethernet interface ready status */
static int check_status(struct temac_reg *regs, u32 mask)
{
	unsigned timeout = 50;	/* 1usec * 50 = 50usec */

	/*
	 * Quote from LL TEMAC documentation: The bits in the RDY
	 * register are asserted when there is no access in progress.
	 * When an access is in progress, a bit corresponding to the
	 * type of access is automatically de-asserted. The bit is
	 * automatically re-asserted when the access is complete.
	 */
	while (timeout && (!(in_be32(&regs->rdy) & mask))) {
		timeout--;
		udelay(1);
	}

	if (!timeout) {
		printf("%s: Timeout on 0x%08x @%p\n", __func__,
				mask, &regs->rdy);
		return 1;
	}

	return 0;
}

/*
 * Indirect MII PHY write via ll_temac.
 *
 * http://www.xilinx.com/support/documentation/ip_documentation/xps_ll_temac.pdf
 * page 67, Using the MII Management to Access PHY Registers
 */
static int phywrite(struct eth_device *dev, u8 phy_addr,
					u8 reg_addr, u16 phy_data)
{
	struct temac_reg *regs = (struct temac_reg *)dev->iobase;

	out_be32(&regs->lsw, (phy_data & LSW_REGDAT_MASK));
	out_be32(&regs->ctl, CTL_WEN | TEMAC_MIIMWD);

	out_be32(&regs->lsw,
		((phy_addr << LSW_PHYAD_POS) & LSW_PHYAD_MASK) |
		(reg_addr & LSW_REGAD_MASK));
	out_be32(&regs->ctl, CTL_WEN | TEMAC_MIIMAI);

	if (check_status(regs, RSE_MIIM_WR))
		return 1;

	return 0;
}

/*
 * Indirect MII PHY read via ll_temac.
 *
 * http://www.xilinx.com/support/documentation/ip_documentation/xps_ll_temac.pdf
 * page 67, Using the MII Management to Access PHY Registers
 */
static int phyread(struct eth_device *dev, u8 phy_addr,
					u8 reg_addr, u16 *value)
{
	struct temac_reg *regs = (struct temac_reg *)dev->iobase;

	out_be32(&regs->lsw,
		((phy_addr << LSW_PHYAD_POS) & LSW_PHYAD_MASK) |
		(reg_addr & LSW_REGAD_MASK));
	out_be32(&regs->ctl, TEMAC_MIIMAI);

	if (check_status(regs, RSE_MIIM_RR))
		return 1;

	*value = in_be32(&regs->lsw) & LSW_REGDAT_MASK;
	return 0;
}

#ifdef DEBUG
static inline void read_phy_reg(struct eth_device *dev, u8 phy_addr)
{
	int reg_addr, ret;
	unsigned short value;

	debug("phy%d   ", phy_addr);
	for (reg_addr = 0; reg_addr < 32; reg_addr++) {
		ret = phyread(dev, phy_addr, reg_addr, &value);
		debug("%d: 0x%04x ", reg_addr, value);
	}
	debug("\n");
}
#endif

/*
 * Indirect write to ll_temac.
 *
 * http://www.xilinx.com/support/documentation/ip_documentation/xps_ll_temac.pdf
 * page 23, second paragraph, The use of CTL0 register or CTL1 register
 */
static int indirect_set(struct eth_device *dev, u16 regn, u32 reg_data)
{
	struct temac_reg *regs = (struct temac_reg *)dev->iobase;

	out_be32(&regs->lsw, (reg_data & MLSW_MASK));
	out_be32(&regs->ctl, CTL_WEN | (regn & CTL_ADDR_MASK));

	if (check_status(regs, RSE_CFG_WR))
		return 0;

	return 1;
}

/*
 * Indirect read from ll_temac.
 *
 * http://www.xilinx.com/support/documentation/ip_documentation/xps_ll_temac.pdf
 * page 23, second paragraph, The use of CTL0 register or CTL1 register
 */
static int indirect_get(struct eth_device *dev, u16 regn, u32* reg_data)
{
	struct temac_reg *regs = (struct temac_reg *)dev->iobase;

	out_be32(&regs->ctl, (regn & CTL_ADDR_MASK));

	if (check_status(regs, RSE_CFG_RR))
		return 0;

	*reg_data = in_be32(&regs->lsw) & MLSW_MASK;
	return 1;
}

/* setting sub-controller and ll_temac to proper setting */
static int setup_ctrl(struct eth_device *dev)
{
	struct ll_temac *ll_temac = dev->priv;

	if (ll_temac->ctrlreset) {

		if (ll_temac->ctrlreset(dev))
			return 0;

	}

	if (ll_temac->ctrlinit) {

		if (ll_temac->ctrlinit(dev))
			return 0;

	}

	if (!indirect_set(dev, TEMAC_MC,
			MC_MDIOEN | (MDIO_CLOCK_DIV & MC_CLKDIV_MASK)))
		return 0;

	/* Promiscuous mode disable */
	if (!indirect_set(dev, TEMAC_AFM, 0))
		return 0;

	/* Enable Receiver - RX bit */
	if (!indirect_set(dev, TEMAC_RCW1, RCW1_RX))
		return 0;

	/* Enable Transmitter - TX bit */
	if (!indirect_set(dev, TEMAC_TC, TC_TX))
		return 0;

	return 1;
}

/* setting ll_temac and phy to proper setting */
static int setup_phy(struct eth_device *dev)
{
	unsigned short phyreg;
	unsigned int i, speed, emmc_reg, ret;
	struct ll_temac *ll_temac = dev->priv;
	struct phy_device *phydev;

	unsigned int supported = SUPPORTED_10baseT_Half |
			SUPPORTED_10baseT_Full |
			SUPPORTED_100baseT_Half |
			SUPPORTED_100baseT_Full |
			SUPPORTED_1000baseT_Half |
			SUPPORTED_1000baseT_Full;

	if (ll_temac->phyaddr == -1) {
		for (i = 31; i >= 0; i--) {
			ret = phyread(dev, i, PHY_DETECT_REG, &phyreg);
			if (!ret && (phyreg != 0xFFFF) &&
			((phyreg & PHY_DETECT_MASK) == PHY_DETECT_MASK)) {
				/* Found a valid PHY address */
				ll_temac->phyaddr = i;
				debug("phy %x result %x\n", i, phyreg);
				break;
			}
		}
	}

	/* interface - look at tsec */
	phydev = phy_connect(ll_temac->bus, ll_temac->phyaddr, dev, 0);

	phydev->supported &= supported;
	phydev->advertising = phydev->supported;
	ll_temac->phydev = phydev;
	phy_config(phydev);
	phy_startup(phydev);

	switch (phydev->speed) {
	case 1000:
		speed = EMMC_LSPD_1000;
		break;
	case 100:
		speed = EMMC_LSPD_100;
		break;
	case 10:
		speed = EMMC_LSPD_10;
		break;
	default:
		return 0;
	}

	if (!indirect_get(dev, TEMAC_EMMC, &emmc_reg))
		return 0;

	emmc_reg &= ~EMMC_LSPD_MASK;
	emmc_reg |= speed;

	if (!indirect_set(dev, TEMAC_EMMC, emmc_reg))
		return 0;

	return 1;
}

/* setup mac addr */
static int ll_temac_addr_setup(struct eth_device *dev)
{
	u32 val;

	/* set up unicast MAC address filter */
	val = ((dev->enetaddr[3] << 24) | (dev->enetaddr[2] << 16) |
			(dev->enetaddr[1] << 8) | (dev->enetaddr[0]));
	val &= UAW0_UADDR_MASK;

	if (!indirect_set(dev, TEMAC_UAW0, val))
		return 1;

	val = ((dev->enetaddr[5] << 8) | dev->enetaddr[4]);
	val &= UAW1_UADDR_MASK;

	if (!indirect_set(dev, TEMAC_UAW1, val))
		return 1;

	return 0;
}

/* halt device */
static void ll_temac_halt(struct eth_device *dev)
{
	struct ll_temac *ll_temac = dev->priv;

	/* Disable Receiver */
	indirect_set(dev, TEMAC_RCW0, 0);

	/* Disable Transmitter */
	indirect_set(dev, TEMAC_TC, 0);

	if (ll_temac->ctrlhalt)
		ll_temac->ctrlhalt(dev);
}

static int ll_temac_init(struct eth_device *dev, bd_t *bis)
{
#if DEBUG
	int i;
#endif

	printf("%s: Xilinx XPS LocalLink Tri-Mode Ether MAC #%d at 0x%08X.\n",
		dev->name, 0, dev->iobase);

	if (!setup_ctrl(dev))
		return -1;

#if DEBUG
	for (i = 0; i < 32; i++)
		read_phy_reg(dev, i);
#endif

	if (!setup_phy(dev)) {
		ll_temac_halt(dev);
		return -1;
	}

	return 0;
}

static int ll_temac_miiphy_read(const char *devname, unsigned char addr,
				unsigned char reg, unsigned short *value)
{
	int ret;
	struct eth_device *dev = eth_get_dev();

	ret = phyread(dev, addr, reg, value);
	debug("%s 0x%x, 0x%x, 0x%x\n", __func__, addr, reg, *value);
	return ret;
}

static int ll_temac_miiphy_write(const char *devname, unsigned char addr,
				unsigned char reg, unsigned short value)
{
	struct eth_device *dev = eth_get_dev();

	debug("%s 0x%x, 0x%x, 0x%x\n", __func__, addr, reg, value);
	return phywrite(dev, addr, reg, value);
}

/*
 * bis:		board information
 * base_addr:	LL TEMAC register bank
 * ctrl_addr:	LL TEMAC sub-controller register bank (FIFO or SDMA)
 * mode:	driver mode bit flags (see xilinx_ll_temac.h)
 */
int xilinx_ll_temac_initialize(bd_t *bis, unsigned long base_addr,
				int mode, unsigned long ctrl_addr)
{
	struct eth_device *dev;
	struct ll_temac *ll_temac;

	dev = calloc(1, sizeof(*dev));
	if (dev == NULL)
		return -1;

	dev->priv = calloc(1, sizeof(struct ll_temac));
	if (dev->priv == NULL) {
		free(dev);
		return -1;
	}

	ll_temac = dev->priv;

	sprintf(dev->name, "Xlltem.%lx", base_addr);

	dev->iobase = base_addr;
	ll_temac->ctrladdr = ctrl_addr;
	ll_temac->rx_bp = rx_buffer;
	ll_temac->tx_bp = tx_buffer;
	ll_temac->rx_dp = &rx_descr;
	ll_temac->tx_dp = &tx_descr;
	ll_temac->phyaddr = CONFIG_PHY_ADDR;

	dev->init = ll_temac_init;
	dev->halt = ll_temac_halt;
	dev->write_hwaddr = ll_temac_addr_setup;

	if (mode & M_SDMA) {
#if defined(CONFIG_XILINX_440) || defined(CONFIG_XILINX_405)
		if (mode & M_DCR) {
			ll_temac_collect_xldcr_sdma_reg_addr(dev);
			ll_temac->in32 = ll_temac_xldcr_in32;
			ll_temac->out32 = ll_temac_xldcr_out32;
		} else
#endif
		{
			ll_temac_collect_xlplb_sdma_reg_addr(dev);
			ll_temac->in32 = ll_temac_xlplb_in32;
			ll_temac->out32 = ll_temac_xlplb_out32;
		}
		ll_temac->ctrlinit = ll_temac_init_sdma;
		ll_temac->ctrlhalt = ll_temac_halt_sdma;
		ll_temac->ctrlreset = ll_temac_reset_sdma;
		dev->recv = ll_temac_recv_sdma;
		dev->send = ll_temac_send_sdma;
	} else {
		ll_temac->in32 = NULL;
		ll_temac->out32 = NULL;
		ll_temac->ctrlinit = NULL;
		ll_temac->ctrlhalt = NULL;
		ll_temac->ctrlreset = ll_temac_reset_fifo;
		dev->recv = ll_temac_recv_fifo;
		dev->send = ll_temac_send_fifo;
	}

	eth_register(dev);

	miiphy_register(dev->name, ll_temac_miiphy_read, ll_temac_miiphy_write);
	ll_temac->bus = miiphy_get_dev_by_name(dev->name);
	ll_temac->bus->reset = NULL;

	return 1;
}
