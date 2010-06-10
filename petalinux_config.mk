#
#  FILE:
#     petalinux_config.mk
#
#  DESCRIPTION:
#     Petalinux U-Boot build configuration file for Make context.
#
#  AUTHOR:
#      Benny Chen   <benny.chen@petalogix.com>
#
#  MODIFICATION:
#
#  LICENSING:
#      Copyright (c) 2006 PetaLogix. All rights reserved.
#
#   This program is free software; you can redistribute  it and/or modify it
#   under  the terms of  the GNU General Public License as published by the
#   Free Software Foundation;  either version 2 of the  License, or (at your
#   option) any later version.
#
#   THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
#   WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
#   NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
#   NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
#   USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#   ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
#   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#   You should have received a copy of the  GNU General Public License along
#   with this program; if not, write  to the Free Software Foundation, Inc.,
#   675 Mass Ave, Cambridge, MA 02139, USA.
#

# Generate lower case versions of Vendor and Product strings, for u-boot
#UBOOT_VENDOR=$(shell echo $(CONFIG_VENDOR) | tr '[\.\/\+A-Z]' '[___a-z]')
#UBOOT_PLATFORM=$(shell echo $(CONFIG_PRODUCT) | tr '[\.\/\+A-Z]' '[___a-z]')
# For now, we force petalogix/microblaze-auto combination for u-boot
UBOOT_VENDOR=petalogix
UBOOT_PLATFORM=microblaze-auto

UBOOTDIR = $(ROOTDIR)/u-boot
UBOOTIMG = u-boot-s
KERNEL_IMG = $(IMAGEDIR)/image.bin
UB_KERNEL_IMG = $(IMAGEDIR)/image.ub
MKIMAGE = $(UBOOTDIR)/tools/mkimage
UBOOT_BOARD_DIR=$(UBOOTDIR)/board/$(UBOOT_VENDOR)/$(UBOOT_PLATFORM)

UBOOT_SCRIPT_BOARD_TEMPLATE=$(UBOOTDIR)/include/configs/$(UBOOT_VENDOR)-$(UBOOT_PLATFORM).h.template
UBOOT_SCRIPT_BOARD=$(UBOOTDIR)/include/configs/$(UBOOT_VENDOR)-$(UBOOT_PLATFORM).h

# We need to crack some settings out of the user/vendor .config file
# This is a bit hacky, a more generic system would be nice

# Obtain Hardware Address Mappings
FLASH_START=$(shell grep FLASH_START $(UBOOT_BOARD_DIR)/xparameters.h \
		2>/dev/null | sed -r -e 's/[[:space:]]+/ /g' | cut -d ' ' -f 3)
FLASH_SIZE=$(shell grep FLASH_SIZE $(UBOOT_BOARD_DIR)/xparameters.h \
		2>/dev/null | sed -r -e 's/[[:space:]]+/ /g' | cut -d ' ' -f 3)
ERAM_START=$(shell grep CONFIG_KERNEL_BASE_ADDR $(LINUX_CONFIG) \
                2>/dev/null | cut -d "=" -f 2 | grep -v "^\#")
# Obtain U-Boot boot partition start address
FLASH_BOOT_START=$(shell grep CONFIG_FLASH_BOOT_START $(UBOOT_BOARD_DIR)/flash-partition.h 2>/dev/null | cut -f 3)
# U-Boot boot partition size
FLASH_BOOT_SIZE=$(shell grep CONFIG_FLASH_BOOT_SIZE $(UBOOT_BOARD_DIR)/flash-partition.h 2>/dev/null | cut -f 3)

# Get MTD partition table info - Not required any more
#MTDPARTS=$(shell grep CONFIG_SYSTEM_MTDPARTS $(CONFIG_CONFIG) \
                2>/dev/null | cut -d "=" -f 2 | grep -v "^\#" | tr -d '\"')
# Are we building u-boot?
BUILD_UBOOT=$(shell grep CONFIG_SYSTEM_BUILD_UBOOT $(CONFIG_CONFIG) \
                2>/dev/null | cut -d "=" -f 2 | grep -v "^\#")

USE_DHCP=$(shell grep "CONFIG_SYSTEM_USE_DHCP" $(CONFIG_CONFIG) \
		2</dev/null | cut -d "=" -f 2 | grep -v "^\#")
