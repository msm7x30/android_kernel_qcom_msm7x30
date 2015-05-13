/* arch/arm/mach-msm/include/mach/memory.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2012, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* physical offset of RAM */
#define PLAT_PHYS_OFFSET UL(CONFIG_PHYS_OFFSET)

#define MAX_PHYSMEM_BITS 32
#define SECTION_SIZE_BITS 28


/* Certain configurations of MSM7x30 have multiple memory banks.
*  One or more of these banks can contain holes in the memory map as well.
*  These macros define appropriate conversion routines between the physical
*  and virtual address domains for supporting these configurations using
*  SPARSEMEM and a 3G/1G VM split.
*/

#if defined(CONFIG_ARCH_MSM7X30)

#define EBI0_PHYS_OFFSET PHYS_OFFSET
#define EBI0_PAGE_OFFSET PAGE_OFFSET
#define EBI0_SIZE 0x10000000

#ifndef __ASSEMBLY__

extern unsigned long ebi1_phys_offset;

#define EBI1_PHYS_OFFSET (ebi1_phys_offset)
#define EBI1_PAGE_OFFSET (EBI0_PAGE_OFFSET + EBI0_SIZE)

#if (defined(CONFIG_SPARSEMEM) && defined(CONFIG_VMSPLIT_3G))

#define __phys_to_virt(phys)				\
	((phys) >= EBI1_PHYS_OFFSET ?			\
	(phys) - EBI1_PHYS_OFFSET + EBI1_PAGE_OFFSET :	\
	(phys) - EBI0_PHYS_OFFSET + EBI0_PAGE_OFFSET)

#define __virt_to_phys(virt)				\
	((virt) >= EBI1_PAGE_OFFSET ?			\
	(virt) - EBI1_PAGE_OFFSET + EBI1_PHYS_OFFSET :	\
	(virt) - EBI0_PAGE_OFFSET + EBI0_PHYS_OFFSET)

#endif
#endif

#endif

#if defined CONFIG_ARCH_MSM_SCORPION
#define arch_has_speculative_dfetch()	1
#endif

#endif
