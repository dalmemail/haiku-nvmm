/*
 * Copyright 2012, Alex Smith, alex@alex-smith.me.uk.
 * Distributed under the terms of the MIT License.
 */
#ifndef KERNEL_ARCH_X86_PAGING_64BIT_PAGING_H
#define KERNEL_ARCH_X86_PAGING_64BIT_PAGING_H


#include <OS.h>


// PML5 entry bits.
#define X86_64_PML5E_PRESENT			(1LL << 0)
#define X86_64_PML5E_WRITABLE			(1LL << 1)
#define X86_64_PML5E_USER				(1LL << 2)
#define X86_64_PML5E_WRITE_THROUGH		(1LL << 3)
#define X86_64_PML5E_CACHING_DISABLED	(1LL << 4)
#define X86_64_PML5E_ACCESSED			(1LL << 5)
#define X86_64_PML5E_NOT_EXECUTABLE		(1LL << 63)
#define X86_64_PML5E_ADDRESS_MASK		0x000ffffffffff000L

// PML4 entry bits.
#define X86_64_PML4E_PRESENT			(1LL << 0)
#define X86_64_PML4E_WRITABLE			(1LL << 1)
#define X86_64_PML4E_USER				(1LL << 2)
#define X86_64_PML4E_WRITE_THROUGH		(1LL << 3)
#define X86_64_PML4E_CACHING_DISABLED	(1LL << 4)
#define X86_64_PML4E_ACCESSED			(1LL << 5)
#define X86_64_PML4E_NOT_EXECUTABLE		(1LL << 63)
#define X86_64_PML4E_ADDRESS_MASK		0x000ffffffffff000L

// PDPT entry bits.
#define X86_64_PDPTE_PRESENT			(1LL << 0)
#define X86_64_PDPTE_WRITABLE			(1LL << 1)
#define X86_64_PDPTE_USER				(1LL << 2)
#define X86_64_PDPTE_WRITE_THROUGH		(1LL << 3)
#define X86_64_PDPTE_CACHING_DISABLED	(1LL << 4)
#define X86_64_PDPTE_ACCESSED			(1LL << 5)
#define X86_64_PDPTE_DIRTY				(1LL << 6)
#define X86_64_PDPTE_LARGE_PAGE			(1LL << 7)
#define X86_64_PDPTE_GLOBAL				(1LL << 8)
#define X86_64_PDPTE_PAT				(1LL << 12)
#define X86_64_PDPTE_NOT_EXECUTABLE		(1LL << 63)
#define X86_64_PDPTE_ADDRESS_MASK		0x000ffffffffff000L

// Page directory entry bits.
#define X86_64_PDE_PRESENT				(1LL << 0)
#define X86_64_PDE_WRITABLE				(1LL << 1)
#define X86_64_PDE_USER					(1LL << 2)
#define X86_64_PDE_WRITE_THROUGH		(1LL << 3)
#define X86_64_PDE_CACHING_DISABLED		(1LL << 4)
#define X86_64_PDE_ACCESSED				(1LL << 5)
#define X86_64_PDE_DIRTY				(1LL << 6)
#define X86_64_PDE_LARGE_PAGE			(1LL << 7)
#define X86_64_PDE_GLOBAL				(1LL << 8)
#define X86_64_PDE_PAT					(1LL << 12)
#define X86_64_PDE_NOT_EXECUTABLE		(1LL << 63)
#define X86_64_PDE_ADDRESS_MASK			0x000ffffffffff000L

// Page table entry bits.
#define X86_64_PTE_PRESENT				(1LL << 0)
#define X86_64_PTE_WRITABLE				(1LL << 1)
#define X86_64_PTE_USER					(1LL << 2)
#define X86_64_PTE_WRITE_THROUGH		(1LL << 3)
#define X86_64_PTE_CACHING_DISABLED		(1LL << 4)
#define X86_64_PTE_ACCESSED				(1LL << 5)
#define X86_64_PTE_DIRTY				(1LL << 6)
#define X86_64_PTE_PAT					(1LL << 7)
#define X86_64_PTE_GLOBAL				(1LL << 8)
#define X86_64_PTE_NOT_EXECUTABLE		(1LL << 63)
#define X86_64_PTE_ADDRESS_MASK			0x000ffffffffff000L
#define X86_64_PTE_PROTECTION_MASK		(X86_64_PTE_NOT_EXECUTABLE	\
											| X86_64_PTE_WRITABLE	\
											| X86_64_PTE_USER)
#define X86_64_PTE_MEMORY_TYPE_MASK		(X86_64_PTE_WRITE_THROUGH \
											| X86_64_PTE_CACHING_DISABLED)


static const size_t k64BitPageTableRange = 0x200000L;
static const size_t k64BitPageDirectoryRange = 0x40000000L;
static const size_t k64BitPDPTRange = 0x8000000000L;
static const size_t k64BitPML4TRange = 0x1000000000000L;

static const size_t k64BitTableEntryCount = 512;


#define VADDR_TO_PML5E(va)	(((va) & 0x01fffffffffff000L) / k64BitPML4TRange)
#define VADDR_TO_PML4E(va)	(((va) % k64BitPML4TRange) / k64BitPDPTRange)
#define VADDR_TO_PDPTE(va)	(((va) % k64BitPDPTRange) / k64BitPageDirectoryRange)
#define VADDR_TO_PDE(va)	(((va) % k64BitPageDirectoryRange) / k64BitPageTableRange)
#define VADDR_TO_PTE(va)	(((va) % k64BitPageTableRange) / B_PAGE_SIZE)

// EPT PML4E bits
#define EPT_PML4E_PRESENT			(1LL << 0)
#define EPT_PML4E_WRITABLE			(1LL << 1)
#define EPT_PML4E_EXECUTABLE			(1LL << 2)
#define EPT_PML4E_ADDRESS_MASK			0x0000fffffffff000L

// EPT PDPTE bits
#define EPT_PDPTE_PRESENT			(1LL << 0)
#define EPT_PDPTE_WRITABLE			(1LL << 1)
#define EPT_PDPTE_EXECUTABLE			(1LL << 2)
#define EPT_PDPTE_ADDRESS_MASK			0x0000fffffffff000L

// EPT PDE bits
#define EPT_PDE_PRESENT				(1LL << 0)
#define EPT_PDE_WRITABLE			(1LL << 1)
#define EPT_PDE_EXECUTABLE			(1LL << 2)
#define EPT_PDE_ADDRESS_MASK			0x0000fffffffff000L

// EPT PTE bits.
#define EPT_PTE_PRESENT				(1LL << 0)
#define EPT_PTE_WRITABLE			(1LL << 1)
#define EPT_PTE_EXECUTABLE			(1LL << 2)
#define EPT_PTE_IGNORE_PAT			(1LL << 6)
#define EPT_PTE_ACCESSED			(1LL << 8)
#define EPT_PTE_DIRTY				(1LL << 9)
#define EPT_PTE_ADDRESS_MASK			0x0000fffffffff000L
// Memory types
#define EPT_PTE_CACHING_DISABLED		(0LL << 3)
#define EPT_PTE_WRITE_COMBINING			(1LL << 3)
#define EPT_PTE_WRITE_THROUGH			(4LL << 3)
#define EPT_PTE_WRITE_PROTECT			(5LL << 3)
#define EPT_PTE_WRITE_BACK			(6LL << 3)


#endif	// KERNEL_ARCH_X86_PAGING_64BIT_PAGING_H
