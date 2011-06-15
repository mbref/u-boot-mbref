/*
 * (C) Copyright 2007-2008 Michal Simek
 *
 * Michal SIMEK <monstr@monstr.eu>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "../board/petalogix/ppc405-auto/xparameters.h"

/* cpu parameter */
#define CONFIG_405              1
#define CONFIG_4xx              1
#define CONFIG_XILINX_405       1

/* PPC-specific memory layout */
#define CONFIG_SYS_MONITOR_BASE TEXT_BASE
#define CONFIG_SYS_MONITOR_LEN          (192 * 1024)
#define CONFIG_SYS_MALLOC_LEN           (CONFIG_ENV_SIZE + 128 * 1024)

/*Stack*/
#define CONFIG_SYS_INIT_RAM_ADDR        0x800000/* Initial RAM address    */
#define CONFIG_SYS_INIT_RAM_END         0x2000  /* End of used area in RAM  */
#define CONFIG_SYS_GBL_DATA_SIZE        128     /* num bytes initial data   */
#define CONFIG_SYS_GBL_DATA_OFFSET      (CONFIG_SYS_INIT_RAM_END \
                                - CONFIG_SYS_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_OFFSET       CONFIG_SYS_GBL_DATA_OFFSET

/*Speed*/
#define CONFIG_SYS_CLK_FREQ     XPAR_CORE_CLOCK_FREQ_HZ

/* Common PPC-specific settings */
#define CONFIG_SYS_MEMTEST_START	0x00400000
					/* memtest works on           */
#define CONFIG_SYS_MEMTEST_END		0x00C00000
					/* 4 ... 12 MB in DRAM        */
#define CONFIG_SYS_EXTBDINFO		1
					/* Extended board_into (bd_t) */
#define CONFIG_SYS_HZ			1000
					/* decrementer freq: 1 ms ticks */
#define CONFIG_SYS_BOOTMAPSZ	(8 << 20)
				/* Initial Memory map for Linux */

/* Get PetaLinux platform common settings */
#include <configs/petalinux-auto-board.h>
#endif	/* __CONFIG_H */
