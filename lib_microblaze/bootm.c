/*
 * (C) Copyright 2007-2009 Michal Simek
 * (C) Copyright 2004 Atmark Techno, Inc.
 *
 * Michal  SIMEK <monstr@monstr.eu>
 * Yasushi SHOJI <yashi@atmark-techno.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <image.h>
#include <u-boot/zlib.h>
#include <asm/byteorder.h>

#if defined(CONFIG_OF_LIBFDT)
#include <fdt.h>
#include <libfdt.h>
#include <fdt_support.h>

#define DEBUG

/*static int boot_get_fdt (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],
		bootm_headers_t *images, char **of_flat_tree, ulong *of_size);
static image_header_t *image_get_fdt (ulong fdt_addr);*/
#endif

DECLARE_GLOBAL_DATA_PTR;

extern int do_reset (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);


void do_bootm_linux (int flag, int argc, char *argv[],
			bootm_headers_t *images)
{
	void	(*theKernel) (char *, ulong, ulong);
	char	*commandline = getenv ("bootargs");
	ulong	rd_data_start, rd_data_end;
	ulong	ep = 0;
	int	ret;

	char	*of_flat_tree = NULL;
#if defined(CONFIG_OF_LIBFDT)

	//ulong	of_size = 0;

	/* find flattened device tree */
	//ret = boot_get_fdt (argc, flag, argc, argv, images,
	//				&of_flat_tree, &of_size);
	//if (ret)
	//	goto error;

//	char **of_flat_tree = &images->ft_addr;
#endif

//#if 0
	/* find kernel entry point */
	if (images->legacy_hdr_valid) {
		ep = image_get_ep (&images->legacy_hdr_os_copy);
#if defined(CONFIG_FIT)
	} else if (images->fit_uname_os) {
		ret = fit_image_get_entry (images->fit_hdr_os,
				images->fit_noffset_os, &ep);
		if (ret) {
			puts ("Can't get entry point property!\n");
			goto error;
		}
#endif
	} else {
		puts ("Could not find kernel entry point!\n");
		goto error;
	}
//#endif
	theKernel = (void (*)(char *, ulong, ulong))images->ep;
	//printf("images->ep 0x%x, ep 0x%x\n", images->ep, ep = image_get_ep (&images->legacy_hdr_os_copy));

	/* find ramdisk */
	ret = boot_get_ramdisk (argc, argv, images, IH_ARCH_MICROBLAZE,
			&rd_data_start, &rd_data_end);
	if (ret)
		goto error;

	show_boot_progress (15);

//#if defined(CONFIG_OF_LIBFDT)
	if (!(ulong) of_flat_tree)
		of_flat_tree = (char *)simple_strtoul (argv[3], NULL, 16);
//#endif
	/*
	 * Linux Kernel Parameters (passing device tree):
	 * r5: pointer to command line
	 * r6: pointer to ramdisk
	 * r7: pointer to the fdt, followed by the board info data
	 */

//#ifdef DEBUG
	printf ("## Transferring control to Linux (at address %08lx), 0x%08lx"\
				" ramdisk 0x%08lx, FDT 0x%08lx...\n",
		(ulong) theKernel, ep, rd_data_start, (ulong) of_flat_tree);
//#endif
	//if (!images->autostart)
	//	return ;

#ifdef XILINX_USE_DCACHE
#ifdef XILINX_DCACHE_BYTE_SIZE
	flush_cache(0, XILINX_DCACHE_BYTE_SIZE);
#else
#warning please rebuild BSPs and update configuration
	flush_cache(0, 32768);
#endif
#endif

	theKernel (commandline, rd_data_start, (ulong) of_flat_tree);

	/* does not return */
	return;

error:
	printf("error\n");
//	if (images->autostart)
//		do_reset (cmdtp, flag, argc, argv);
	return;
}
