/*
 *
 * s2imac ethernet driver for u-boot
 *
 * Author: Roman Wagner rw@sensortoimage.de
 *
 * Copyright (C) 2010 Sensor to image GmbH
 * October 2010 created
 *
 * Copyright (C) 2011 Li-Pro.Net, Stephan Linz <linz@li-pro.net>
 * January 2011 code improvements and coding style corrections
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#include <config.h>
#include <common.h>
#include <net.h>
#include <malloc.h>
#include <asm/processor.h>
#include <asm/io.h>

#undef ETH_HALTING

/* MDIO register basis */
#define MDIO_BASE		(dev->iobase + 0x1000)

/* Tx/Rx buffer basis */
#define TXBUF			(dev->iobase + 0x4000)
#define RXBUF			(dev->iobase + 0x8000)

/* MAC registers definition */
#define RCW1			(dev->iobase + ((0x240) << 2))
#define TC			(dev->iobase + ((0x280) << 2))
#define EMMC			(dev->iobase + ((0x300) << 2))
#define MC			(dev->iobase + ((0x340) << 2))
#define UAW0			(dev->iobase + ((0x380) << 2))
#define UAW1			(dev->iobase + ((0x384) << 2))
#define AFM			(dev->iobase + ((0x390) << 2))

#define MDIO_ENABLE_MASK	0x40
#define MDIO_CLOCK_DIV_MASK	0x3F

/* direct registers definition */
#define GCSR			(dev->iobase + 0xC000)
#define CLK_FREQ		(dev->iobase + 0xC008)
#define TOCNT_DIV		(dev->iobase + 0xC014)
#define MDIO_ACC		(dev->iobase + 0xC028)
#define ETHSIZE  		(dev->iobase + 0xC02C)
#define MAC_HIGH 		(dev->iobase + 0xC030)
#define MAC_LOW  		(dev->iobase + 0xC034)
#define TX_LEN  		(dev->iobase + 0xC048)
#define RX_LEN   		(dev->iobase + 0xC04C)
#define MAC_RX_HIGH		(dev->iobase + 0xC090)
#define MAC_RX_LOW		(dev->iobase + 0xC094)

#define GCSR_CONNECTED		0x00000001
#define GCSR_RST_PHY		0x00000004

#define ETHER_MTU		1520

/* FIXME: use ETHER_MTU ??? */
static unsigned char rx_buffer[0x4000] __attribute ((aligned (32)));
volatile u32 *gige_txbuf;
volatile u32 *gige_rxbuf;

// ---- Read Ethernet PHY register ---------------------------------------------
u16 get_phy_reg (struct eth_device *dev, int phy_addr, int reg_addr)
{
	u32 ret;

	if (in_be32 ((u32 *) MDIO_ACC) & (1 << 16)) {
		/*
		 * Non-blocking mode
		 * (only this software thread waits for MDIO access)
		 */
		ret = in_be32 ((u32 *) (MDIO_BASE + ((phy_addr & 0x1F) << 7)
					+ ((reg_addr & 0x1F) << 2)));
		while (!(in_be32 ((u32 *) MDIO_ACC) & (1 << 17))) {
		};
		ret = in_be32 ((u32 *) MDIO_ACC);
	} else {
		/*
		 * Blocking mode
		 * (whole system waits for the bus transaction to finish)
		 */
		ret = in_be32 ((u32 *) (MDIO_BASE + ((phy_addr & 0x1F) << 7)
					+ ((reg_addr & 0x1F) << 2)));
	}

	return (unsigned short)(ret & 0xFFFF);
}

// ---- Write Ethernet PHY register --------------------------------------------

void set_phy_reg (struct eth_device *dev, int phy_addr, int reg_addr,
		  int reg_data)
{
	if (in_be32 ((u32 *) MDIO_ACC) & (1 << 16)) {
		/*
		 * Non-blocking mode
		 * (only this software thread waits for MDIO access)
		 */
		out_be32 ((u32 *) (MDIO_BASE + ((phy_addr & 0x1F) << 7)
				   + ((reg_addr & 0x1F) << 2)), reg_data);

		while (!(in_be32 ((u32 *) MDIO_ACC) & (1 << 17))) {
		};
	} else {
		/*
		 * Blocking mode
		 * (whole system waits for the bus transaction to finish)
		 */
		out_be32 ((u32 *) (MDIO_BASE + ((phy_addr & 0x1F) << 7)
				   + ((reg_addr & 0x1F) << 2)), reg_data);
	}
}

static int phy_addr = -1;
static int link = 0;

/* setting mac and phy to proper setting */
static int s2imac_phy_ctrl (struct eth_device *dev)
{
	int i, retries;
	unsigned int result;
	u32 cfg, rxtx, help;

	/* link is setup */
	if (link == 1)
		return 1;

	/* try out if have ever found the right phy? */
	if (phy_addr == -1) {
		puts ("Looking for phy ... ");
		for (i = 31; i >= 0; i--) {
			result = get_phy_reg (dev, i, 1);
			if ((result & 0x0ffff) != 0x0ffff) {
				debug ("%s: Phy Addr %x results %x\n",
				       dev->name, i, result);
				phy_addr = i;
				break;
			}
		}

		/* no success? -- wery bad */
		if (phy_addr == -1) {
			puts ("ERROR\n");
			return 0;
		}
		puts ("OK\n");
	}

	/* wait for link up */
	puts ("Waiting for link ... ");
	retries = 10;
	while (retries-- && ((get_phy_reg (dev, phy_addr, 1) & 0x24) != 0x24)) ;

	if (retries < 0) {
		puts ("ERROR\n");
		return 0;
	}
	puts ("OK\n");

	/* get PHY id */
	i = (get_phy_reg (dev, phy_addr, 2) << 16)
	    | get_phy_reg (dev, phy_addr, 3);
	debug ("%s: Phy ID 0x%x\n", dev->name, i);

	/*
	 * Marwell 88e1111 id - ml50x, ml605
	 */
	/* FIXME this part will be replaced by PHY lib */
	if (i == 0x1410cc2) {
		switch ((get_phy_reg (dev, phy_addr, 17)) & 0xE000) {
		case 0x0000:
			/* 10BASE-T, half-duplex */
			cfg = 0x00000000;
			rxtx = 0x14000000;
			puts ("10BASE-T/HD\n");
			break;
		case 0x2000:
			/* 10BASE-T, full-duplex */
			cfg = 0x00000000;
			rxtx = 0x10000000;
			puts ("10BASE-T/FD\n");
			break;
		case 0x4000:
			/* 100BASE-TX, half-duplex */
			cfg = 0x40000000;
			rxtx = 0x14000000;
			puts ("100BASE-T/HD\n");
			break;
		case 0x6000:
			/* 100BASE-TX, full-duplex */
			cfg = 0x40000000;
			rxtx = 0x10000000;
			puts ("100BASE-T/FD\n");
			break;
		case 0x8000:
			/* 1000BASE-T, half-duplex */
			cfg = 0x80000000;
			rxtx = 0x14000000;
			puts ("1000BASE-T/HD\n");
			break;
		case 0xA000:
			/* 1000BASE-T, full-duplex */
			cfg = 0x80000000;
			rxtx = 0x10000000;
			puts ("1000BASE-T/FD\n");
			break;
		default:
			puts ("Unsupported mode\n");
			link = 0;
			return 0;
		}

		link = 1;

		out_be32 ((u32 *) EMMC, cfg);
		/* Enable jumbo frames for Tx and Rx */
		out_be32 ((u32 *) TC, rxtx | 0x40000000);
		out_be32 ((u32 *) RCW1, rxtx | 0x40000000);

		help = in_be32 ((u32 *) GCSR);
		help |= GCSR_CONNECTED;
		out_be32 ((u32 *) GCSR, help);

		return 1;
	}

	puts ("Unsupported PHY\n");
	return 0;
}

/* setup mac addr */
static int s2imac_addr_setup (struct eth_device *dev)
{
	int val;
	u32 mac_l, mac_h;

	/* set up unicast MAC address filter */
	val = ((dev->enetaddr[3] << 24) | (dev->enetaddr[2] << 16) |
	       (dev->enetaddr[1] << 8) | (dev->enetaddr[0]));
	out_be32 ((u32 *) UAW0, val);
	val = (dev->enetaddr[5] << 8) | dev->enetaddr[4];
	out_be32 ((u32 *) UAW1, val);

	mac_h = (dev->enetaddr[0] << 8) | dev->enetaddr[1];
	mac_l = ((dev->enetaddr[2] << 24) | (dev->enetaddr[3] << 16) |
		 (dev->enetaddr[4] << 8) | (dev->enetaddr[5]));
	out_be32 ((u32 *) MAC_HIGH, mac_h);	/* set gige_mac_h register */
	out_be32 ((u32 *) MAC_LOW, mac_l);	/* set gige_mac_l register */
	out_be32 ((u32 *) MAC_RX_HIGH, mac_h);	/* set gige_rx_mac_h register */
	out_be32 ((u32 *) MAC_RX_LOW, mac_l);	/* set gige_rx_mac_l register */

	return 0;
}

/* halt device */
static void s2imac_halt (struct eth_device *dev)
{
	link = 0;

	/*
	 * TODO: The receiver and transmitter in current S2IMAC
	 *       implementation can not always be turned on.
	 */
	/* Disable Receiver
	   out_be32((u32 *)RCW1, 0x00000000); */
	/* Disable Transmitter
	   out_be32((u32 *)TC, 0x00000000); */
}

static int s2imac_init (struct eth_device *dev, bd_t * bis)
{
	static int first = 1;

	if (!first)
		return 0;
	first = 0;

	/* non-blocking PHY access enabled (if supported) */
	out_be32 ((u32 *) MDIO_ACC, 0x80000000);

	/* wait for end of PHY reset */
	while (in_be32 ((u32 *) GCSR) & GCSR_RST_PHY) {
	};

	/*
	 * Setup timers and clock generators
	 *   - timeout counter clock period 1 ms
	 *   - PHY MDC frequency (max 2.5 MHz according to IEEE Std 802.3-2002,
	 *     BCM5461A supports 12.5 MHz)
	 */
	out_be32 ((u32 *) CLK_FREQ, 62500000);
	out_be32 ((u32 *) TOCNT_DIV, (62500000 / 1000) - 1);
	out_be32 ((u32 *) MC, MDIO_ENABLE_MASK | (MDIO_CLOCK_DIV_MASK
						  & ((62500000 / (2 * 2500000))
						     - 1)));

	/* set up MAC address */
	s2imac_addr_setup (dev);

	/* Promiscuous mode disable */
	out_be32 ((u32 *) AFM, 0x00000000);
	/* Enable Receiver */
	out_be32 ((u32 *) RCW1, 0x10000000);
	/* Enable Transmitter */
	out_be32 ((u32 *) TC, 0x10000000);

	printf ("%s: Sensor to Image GigE Vision Ether MAC #%d at 0x%08X.\n",
		dev->name, 0, dev->iobase);

	s2imac_phy_ctrl (dev);
	return 1;
}

static int s2imac_send (struct eth_device *dev, volatile void *packet,
			int length)
{
	u16 *buf = (u16 *) packet, val, val1;
	u32 len, i;

	len = (length / 4) + 1;

	/* first 2 bytes of buffer not used */
	val = *buf++;
	gige_txbuf[0] = (u32) (((u32) 0x0000 << 16) | (u32) val);

	for (i = 1; i < len; i++) {
		val = *buf++;
		val1 = *buf++;
		gige_txbuf[i] = (u32) (((u32) val << 16) | ((u32) val1));
	}

	/* send reply */
	while (in_be32 ((u32 *) TX_LEN)) {
	};
	out_be32 ((u32 *) TX_LEN, length);

	return 0;
}

static int s2imac_recv (struct eth_device *dev)
{
	int len, i, l;
	u32 *phelp = (u32 *) rx_buffer;

	len = in_be32 ((u32 *) RX_LEN);

	l = (len / 4) + 1;
	if (len) {
		/* first 2 bytes of buffer not used */
		for (i = 0; i < l; i++)
			*phelp++ = ((gige_rxbuf[i] << 16)
				    | (gige_rxbuf[i + 1] >> 16));

		NetReceive (rx_buffer, len);
		out_be32 ((u32 *) RX_LEN, 0);
	}
	return 0;
}

int s2imac_initialize (bd_t * bis, int base_addr)
{
	struct eth_device *dev;

	dev = calloc (1, sizeof (*dev));
	if (dev == NULL)
		hang ();

	sprintf (dev->name, "S2IMAC");

	dev->iobase = base_addr;

	/* transmit and receive packet buffers */
	gige_txbuf = (volatile u32 *)TXBUF;
	gige_rxbuf = (volatile u32 *)RXBUF;

	dev->init = s2imac_init;
	dev->halt = s2imac_halt;
	dev->send = s2imac_send;
	dev->recv = s2imac_recv;

	eth_register (dev);

	return 0;
}
