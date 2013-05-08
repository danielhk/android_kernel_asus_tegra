# File: Makefile
#
# Copyright (C) 2008-2012, Marvell International Ltd.
#
# This software file (the "File") is distributed by Marvell International
# Ltd. under the terms of the GNU General Public License Version 2, June 1991
# (the "License").  You may use, redistribute and/or modify this File in
# accordance with the terms and conditions of the License, a copy of which
# is available by writing to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
# worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
#
# A copy of the GPL is available in file gpl-2.0.txt accompanying in this
# deliverables.
#
# THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
# IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
# ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
# this warranty disclaimer.

#############################################################################
# Configuration Options
#############################################################################

# Debug Option
# DEBUG LEVEL n/1/2:
# n: NO DEBUG
# 1: PRINTM(MSG,...), PRINTM(FATAL,...), PRINTM(WARN,...) and PRINTM(INFO,...)
# 2: All PRINTM()
CONFIG_DEBUG=1

#############################################################################
# Compiler Flags
#############################################################################

ccflags-y += -DFPNUM='"69"'

ifeq ($(CONFIG_64BIT), y)
	ccflags-y += -DMBT_64BIT
endif

ifeq ($(CONFIG_DEBUG),1)
	ccflags-y += -DDEBUG_LEVEL1
endif

ifeq ($(CONFIG_DEBUG),2)
	ccflags-y += -DDEBUG_LEVEL1
	ccflags-y += -DDEBUG_LEVEL2
endif

mbt8xxx-y := bt_main.o bt_proc.o mbt_char.o
mbt8xxx-y += bt_init.o
mbt8xxx-y += bt_sdiommc.o
obj-$(CONFIG_MBT_SD_CHAR) += mbt8xxx.o
mbt8xxx-objs := $(mbt8xxx-y)
