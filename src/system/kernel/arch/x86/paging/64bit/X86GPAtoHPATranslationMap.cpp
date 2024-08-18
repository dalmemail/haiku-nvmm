/*
 * Copyright 2012, Alex Smith, alex@alex-smith.me.uk
 * Copyright 2008-2011, Ingo Weinhold, ingo_weinhold@gmx.de.
 * Copyright 2002-2010, Axel Dörfler, axeld@pinc-software.de.
 * Distributed under the terms of the MIT License.
 *
 * Copyright 2001-2002, Travis Geiselbrecht. All rights reserved.
 * Distributed under the terms of the NewOS License.
 */


#include "paging/64bit/X86GPAtoHPATranslationMap.h"

#include <int.h>
#include <slab/Slab.h>
#include <thread.h>
#include <util/AutoLock.h>
#include <util/ThreadAutoLock.h>
#include <vm/vm_page.h>
#include <vm/VMAddressSpace.h>
#include <vm/VMCache.h>

#include "paging/64bit/EPTPagingMethod.h"
#include "paging/64bit/X86PagingStructures64Bit.h"
#include "paging/x86_physical_page_mapper.h"


//#define TRACE_X86_GPA_HPA_TRANSLATION_MAP
#ifdef TRACE_X86_GPA_HPA_TRANSLATION_MAP
#	define TRACE(x...) dprintf(x)
#else
#	define TRACE(x...) ;
#endif


// #pragma mark - X86GPAtoHPATranslationMap


X86GPAtoHPATranslationMap::X86GPAtoHPATranslationMap(bool la57)
	:
	fPagingStructures(NULL),
	fLA57(la57)
{
}


X86GPAtoHPATranslationMap::~X86GPAtoHPATranslationMap()
{
	TRACE("X86GPAtoHPATranslationMap::~X86GPAtoHPATranslationMap()\n");

	if (fPagingStructures == NULL)
		return;

	if (fPageMapper != NULL) {
		phys_addr_t address;
		vm_page* page;

		// Free all structures in the PMLTop.
		uint64* virtualPML4 = fPagingStructures->VirtualPMLTop();
		for (uint32 i = 0; i < 512; i++) {
			if ((virtualPML4[i] & X86_64_PML4E_PRESENT) == 0)
				continue;

			uint64* virtualPDPT = (uint64*)fPageMapper->GetPageTableAt(
				virtualPML4[i] & X86_64_PML4E_ADDRESS_MASK);
			for (uint32 j = 0; j < 512; j++) {
				if ((virtualPDPT[j] & X86_64_PDPTE_PRESENT) == 0)
					continue;

				uint64* virtualPageDir = (uint64*)fPageMapper->GetPageTableAt(
					virtualPDPT[j] & X86_64_PDPTE_ADDRESS_MASK);
				for (uint32 k = 0; k < 512; k++) {
					if ((virtualPageDir[k] & X86_64_PDE_PRESENT) == 0)
						continue;

					address = virtualPageDir[k] & X86_64_PDE_ADDRESS_MASK;
					page = vm_lookup_page(address / B_PAGE_SIZE);
					if (page == NULL) {
						panic("page table %u %u %u on invalid page %#"
							B_PRIxPHYSADDR "\n", i, j, k, address);
					}

					DEBUG_PAGE_ACCESS_START(page);
					vm_page_set_state(page, PAGE_STATE_FREE);
				}

				address = virtualPDPT[j] & X86_64_PDPTE_ADDRESS_MASK;
				page = vm_lookup_page(address / B_PAGE_SIZE);
				if (page == NULL) {
					panic("page directory %u %u on invalid page %#"
						B_PRIxPHYSADDR "\n", i, j, address);
				}

				DEBUG_PAGE_ACCESS_START(page);
				vm_page_set_state(page, PAGE_STATE_FREE);
			}

			address = virtualPML4[i] & X86_64_PML4E_ADDRESS_MASK;
			page = vm_lookup_page(address / B_PAGE_SIZE);
			if (page == NULL) {
				panic("PDPT %u on invalid page %#" B_PRIxPHYSADDR "\n", i,
					address);
			}

			DEBUG_PAGE_ACCESS_START(page);
			vm_page_set_state(page, PAGE_STATE_FREE);
		}

		fPageMapper->Delete();
	}

	fPagingStructures->RemoveReference();
}


status_t
X86GPAtoHPATranslationMap::Init()
{
	TRACE("X86GPAtoHPATranslationMap::Init()\n");

	X86VMTranslationMap::Init(false);

	fPagingStructures = new(std::nothrow) X86PagingStructures64Bit;
	if (fPagingStructures == NULL)
		return B_NO_MEMORY;

	// Allocate a physical page mapper.
	status_t error = EPTPagingMethod::PhysicalPageMapper()
		->CreateTranslationMapPhysicalPageMapper(&fPageMapper);
	if (error != B_OK)
		return error;

	// Allocate and clear the PMLTop.
	uint64* virtualPMLTop = (uint64*)memalign(B_PAGE_SIZE, B_PAGE_SIZE);
	if (virtualPMLTop == NULL)
		return B_NO_MEMORY;
	memset(virtualPMLTop, 0, B_PAGE_SIZE);

	// Look up the PMLTop physical address.
	phys_addr_t physicalPMLTop;
	vm_get_page_mapping(VMAddressSpace::KernelID(), (addr_t)virtualPMLTop,
		&physicalPMLTop);

	// Initialize the paging structures.
	fPagingStructures->Init(virtualPMLTop, physicalPMLTop);

	return B_OK;
}


size_t
X86GPAtoHPATranslationMap::MaxPagesNeededToMap(addr_t start, addr_t end) const
{
	// If start == 0, the actual base address is not yet known to the caller and
	// we shall assume the worst case, which is where the start address is the
	// last page covered by a PDPT or PML4.
	if (start == 0) {
		start = (fLA57 ? k64BitPML4TRange : k64BitPDPTRange) - B_PAGE_SIZE;
		end += start;
	}

	size_t requiredPML4s = 0;
	if (fLA57) {
		requiredPML4s = end / k64BitPML4TRange + 1
			- start / k64BitPML4TRange;
	}
	size_t requiredPDPTs = end / k64BitPDPTRange + 1
		- start / k64BitPDPTRange;
	size_t requiredPageDirs = end / k64BitPageDirectoryRange + 1
		- start / k64BitPageDirectoryRange;
	size_t requiredPageTables = end / k64BitPageTableRange + 1
		- start / k64BitPageTableRange;

	return requiredPML4s + requiredPDPTs + requiredPageDirs
		+ requiredPageTables;
}


status_t
X86GPAtoHPATranslationMap::Map(addr_t virtualAddress, phys_addr_t physicalAddress,
	uint32 attributes, uint32 memoryType, vm_page_reservation* reservation)
{
	TRACE("X86GPAtoHPATranslationMap::Map(%#" B_PRIxADDR ", %#" B_PRIxPHYSADDR
		")\n", virtualAddress, physicalAddress);

	ThreadCPUPinner pinner(thread_get_current_thread());

	// Look up the page table for the virtual address, allocating new tables
	// if required. Shouldn't fail.
	uint64* entry = EPTPagingMethod::PageTableEntryForAddress(
		fPagingStructures->VirtualPMLTop(), virtualAddress, fIsKernelMap,
		true, reservation, fPageMapper, fMapCount);
	ASSERT(entry != NULL);

	// The entry should not already exist.
	ASSERT_PRINT((*entry & X86_64_PTE_PRESENT) == 0,
		"virtual address: %#" B_PRIxADDR ", existing pte: %#" B_PRIx64,
		virtualAddress, *entry);

	// Fill in the table entry.
	EPTPagingMethod::PutPageTableEntryInTable(entry, physicalAddress,
		attributes, memoryType, fIsKernelMap);

	// Note: We don't need to invalidate the TLB for this address, as previously
	// the entry was not present and the TLB doesn't cache those entries.

	fMapCount++;

	return 0;
}


status_t
X86GPAtoHPATranslationMap::Unmap(addr_t start, addr_t end)
{
	start = ROUNDDOWN(start, B_PAGE_SIZE);
	if (start >= end)
		return B_OK;

	TRACE("X86GPAtoHPATranslationMap::Unmap(%#" B_PRIxADDR ", %#" B_PRIxADDR
		")\n", start, end);

	ThreadCPUPinner pinner(thread_get_current_thread());

	do {
		uint64* pageTable = EPTPagingMethod::PageTableForAddress(
			fPagingStructures->VirtualPMLTop(), start, fIsKernelMap, false,
			NULL, fPageMapper, fMapCount);
		if (pageTable == NULL) {
			// Move on to the next page table.
			start = ROUNDUP(start + 1, k64BitPageTableRange);
			continue;
		}

		for (uint32 index = start / B_PAGE_SIZE % k64BitTableEntryCount;
				index < k64BitTableEntryCount && start < end;
				index++, start += B_PAGE_SIZE) {
			if ((pageTable[index] & X86_64_PTE_PRESENT) == 0)
				continue;

			TRACE("X86GPAtoHPATranslationMap::Unmap(): removing page %#"
				B_PRIxADDR " (%#" B_PRIxPHYSADDR ")\n", start,
				pageTable[index] & X86_64_PTE_ADDRESS_MASK);

			uint64 oldEntry = EPTPagingMethod::ClearTableEntryFlags(
				&pageTable[index], X86_64_PTE_PRESENT);
			fMapCount--;

			if ((oldEntry & X86_64_PTE_ACCESSED) != 0) {
				// Note, that we only need to invalidate the address, if the
				// accessed flags was set, since only then the entry could have
				// been in any TLB.
				InvalidatePage(start);
			}
		}
	} while (start != 0 && start < end);

	return B_OK;
}


status_t
X86GPAtoHPATranslationMap::DebugMarkRangePresent(addr_t start, addr_t end,
	bool markPresent)
{
	start = ROUNDDOWN(start, B_PAGE_SIZE);
	if (start >= end)
		return B_OK;

	TRACE("X86GPAtoHPATranslationMap::DebugMarkRangePresent(%#" B_PRIxADDR
		", %#" B_PRIxADDR ")\n", start, end);

	ThreadCPUPinner pinner(thread_get_current_thread());

	do {
		uint64* pageTable = EPTPagingMethod::PageTableForAddress(
			fPagingStructures->VirtualPMLTop(), start, fIsKernelMap, false,
			NULL, fPageMapper, fMapCount);
		if (pageTable == NULL) {
			// Move on to the next page table.
			start = ROUNDUP(start + 1, k64BitPageTableRange);
			continue;
		}

		for (uint32 index = start / B_PAGE_SIZE % k64BitTableEntryCount;
				index < k64BitTableEntryCount && start < end;
				index++, start += B_PAGE_SIZE) {
			if ((pageTable[index] & X86_64_PTE_PRESENT) == 0) {
				if (!markPresent)
					continue;

				EPTPagingMethod::SetTableEntryFlags(&pageTable[index],
					X86_64_PTE_PRESENT);
			} else {
				if (markPresent)
					continue;

				uint64 oldEntry = EPTPagingMethod::ClearTableEntryFlags(
					&pageTable[index], X86_64_PTE_PRESENT);

				if ((oldEntry & X86_64_PTE_ACCESSED) != 0) {
					// Note, that we only need to invalidate the address, if the
					// accessed flags was set, since only then the entry could
					// have been in any TLB.
					InvalidatePage(start);
				}
			}
		}
	} while (start != 0 && start < end);

	return B_OK;
}


status_t
X86GPAtoHPATranslationMap::UnmapPage(VMArea* area, addr_t address,
	bool updatePageQueue)
{
	ASSERT(address % B_PAGE_SIZE == 0);

	TRACE("X86GPAtoHPATranslationMap::UnmapPage(%#" B_PRIxADDR ")\n", address);

	ThreadCPUPinner pinner(thread_get_current_thread());

	// Look up the page table for the virtual address.
	uint64* entry = EPTPagingMethod::PageTableEntryForAddress(
		fPagingStructures->VirtualPMLTop(), address, fIsKernelMap,
		false, NULL, fPageMapper, fMapCount);
	if (entry == NULL)
		return B_ENTRY_NOT_FOUND;

	RecursiveLocker locker(fLock);

	uint64 oldEntry = EPTPagingMethod::ClearTableEntry(entry);

	pinner.Unlock();

	if ((oldEntry & X86_64_PTE_PRESENT) == 0)
		return B_ENTRY_NOT_FOUND;

	fMapCount--;

	if ((oldEntry & X86_64_PTE_ACCESSED) != 0) {
		// Note, that we only need to invalidate the address, if the
		// accessed flags was set, since only then the entry could have been
		// in any TLB.
		InvalidatePage(address);

		Flush();

		// NOTE: Between clearing the page table entry and Flush() other
		// processors (actually even this processor with another thread of the
		// same team) could still access the page in question via their cached
		// entry. We can obviously lose a modified flag in this case, with the
		// effect that the page looks unmodified (and might thus be recycled),
		// but is actually modified.
		// In most cases this is harmless, but for vm_remove_all_page_mappings()
		// this is actually a problem.
		// Interestingly FreeBSD seems to ignore this problem as well
		// (cf. pmap_remove_all()), unless I've missed something.
	}

	locker.Detach();
		// PageUnmapped() will unlock for us

	PageUnmapped(area, (oldEntry & X86_64_PTE_ADDRESS_MASK) / B_PAGE_SIZE,
		(oldEntry & X86_64_PTE_ACCESSED) != 0,
		(oldEntry & X86_64_PTE_DIRTY) != 0, updatePageQueue);

	return B_OK;
}


void
X86GPAtoHPATranslationMap::UnmapPages(VMArea* area, addr_t base, size_t size,
	bool updatePageQueue)
{
	if (size == 0)
		return;

	addr_t start = base;
	addr_t end = base + size - 1;

	TRACE("X86GPAtoHPATranslationMap::UnmapPages(%p, %#" B_PRIxADDR ", %#"
		B_PRIxADDR ")\n", area, start, end);

	VMAreaMappings queue;

	RecursiveLocker locker(fLock);
	ThreadCPUPinner pinner(thread_get_current_thread());

	do {
		uint64* pageTable = EPTPagingMethod::PageTableForAddress(
			fPagingStructures->VirtualPMLTop(), start, fIsKernelMap, false,
			NULL, fPageMapper, fMapCount);
		if (pageTable == NULL) {
			// Move on to the next page table.
			start = ROUNDUP(start + 1, k64BitPageTableRange);
			continue;
		}

		for (uint32 index = start / B_PAGE_SIZE % k64BitTableEntryCount;
				index < k64BitTableEntryCount && start < end;
				index++, start += B_PAGE_SIZE) {
			uint64 oldEntry = EPTPagingMethod::ClearTableEntry(
				&pageTable[index]);
			if ((oldEntry & X86_64_PTE_PRESENT) == 0)
				continue;

			fMapCount--;

			if ((oldEntry & X86_64_PTE_ACCESSED) != 0) {
				// Note, that we only need to invalidate the address, if the
				// accessed flags was set, since only then the entry could have
				// been in any TLB.
				InvalidatePage(start);
			}

			if (area->cache_type != CACHE_TYPE_DEVICE) {
				// get the page
				vm_page* page = vm_lookup_page(
					(oldEntry & X86_64_PTE_ADDRESS_MASK) / B_PAGE_SIZE);
				ASSERT(page != NULL);

				DEBUG_PAGE_ACCESS_START(page);

				// transfer the accessed/dirty flags to the page
				if ((oldEntry & X86_64_PTE_ACCESSED) != 0)
					page->accessed = true;
				if ((oldEntry & X86_64_PTE_DIRTY) != 0)
					page->modified = true;

				// remove the mapping object/decrement the wired_count of the
				// page
				if (area->wiring == B_NO_LOCK) {
					vm_page_mapping* mapping = NULL;
					vm_page_mappings::Iterator iterator
						= page->mappings.GetIterator();
					while ((mapping = iterator.Next()) != NULL) {
						if (mapping->area == area)
							break;
					}

					ASSERT(mapping != NULL);

					area->mappings.Remove(mapping);
					page->mappings.Remove(mapping);
					queue.Add(mapping);
				} else
					page->DecrementWiredCount();

				if (!page->IsMapped()) {
					atomic_add(&gMappedPagesCount, -1);

					if (updatePageQueue) {
						if (page->Cache()->temporary)
							vm_page_set_state(page, PAGE_STATE_INACTIVE);
						else if (page->modified)
							vm_page_set_state(page, PAGE_STATE_MODIFIED);
						else
							vm_page_set_state(page, PAGE_STATE_CACHED);
					}
				}

				DEBUG_PAGE_ACCESS_END(page);
			}
		}

		Flush();
			// flush explicitly, since we directly use the lock
	} while (start != 0 && start < end);

	// TODO: As in UnmapPage() we can lose page dirty flags here. ATM it's not
	// really critical here, as in all cases this method is used, the unmapped
	// area range is unmapped for good (resized/cut) and the pages will likely
	// be freed.

	locker.Unlock();

	// free removed mappings
	bool isKernelSpace = area->address_space == VMAddressSpace::Kernel();
	uint32 freeFlags = CACHE_DONT_WAIT_FOR_MEMORY
		| (isKernelSpace ? CACHE_DONT_LOCK_KERNEL_SPACE : 0);
	while (vm_page_mapping* mapping = queue.RemoveHead())
		vm_free_page_mapping(mapping->page->physical_page_number, mapping, freeFlags);
}


void
X86GPAtoHPATranslationMap::UnmapArea(VMArea* area, bool deletingAddressSpace,
	bool ignoreTopCachePageFlags)
{
	TRACE("X86GPAtoHPATranslationMap::UnmapArea(%p)\n", area);

	if (area->cache_type == CACHE_TYPE_DEVICE || area->wiring != B_NO_LOCK) {
		X86GPAtoHPATranslationMap::UnmapPages(area, area->Base(), area->Size(),
			true);
		return;
	}

	bool unmapPages = !deletingAddressSpace || !ignoreTopCachePageFlags;

	RecursiveLocker locker(fLock);
	ThreadCPUPinner pinner(thread_get_current_thread());

	VMAreaMappings mappings;
	mappings.MoveFrom(&area->mappings);

	for (VMAreaMappings::Iterator it = mappings.GetIterator();
			vm_page_mapping* mapping = it.Next();) {
		vm_page* page = mapping->page;
		page->mappings.Remove(mapping);

		VMCache* cache = page->Cache();

		bool pageFullyUnmapped = false;
		if (!page->IsMapped()) {
			atomic_add(&gMappedPagesCount, -1);
			pageFullyUnmapped = true;
		}

		if (unmapPages || cache != area->cache) {
			addr_t address = area->Base()
				+ ((page->cache_offset * B_PAGE_SIZE) - area->cache_offset);

			uint64* entry = EPTPagingMethod::PageTableEntryForAddress(
				fPagingStructures->VirtualPMLTop(), address, fIsKernelMap,
				false, NULL, fPageMapper, fMapCount);
			if (entry == NULL) {
				panic("page %p has mapping for area %p (%#" B_PRIxADDR "), but "
					"has no page table", page, area, address);
				continue;
			}

			uint64 oldEntry = EPTPagingMethod::ClearTableEntry(entry);

			if ((oldEntry & X86_64_PTE_PRESENT) == 0) {
				panic("page %p has mapping for area %p (%#" B_PRIxADDR "), but "
					"has no page table entry", page, area, address);
				continue;
			}

			// transfer the accessed/dirty flags to the page and invalidate
			// the mapping, if necessary
			if ((oldEntry & X86_64_PTE_ACCESSED) != 0) {
				page->accessed = true;

				if (!deletingAddressSpace)
					InvalidatePage(address);
			}

			if ((oldEntry & X86_64_PTE_DIRTY) != 0)
				page->modified = true;

			if (pageFullyUnmapped) {
				DEBUG_PAGE_ACCESS_START(page);

				if (cache->temporary)
					vm_page_set_state(page, PAGE_STATE_INACTIVE);
				else if (page->modified)
					vm_page_set_state(page, PAGE_STATE_MODIFIED);
				else
					vm_page_set_state(page, PAGE_STATE_CACHED);

				DEBUG_PAGE_ACCESS_END(page);
			}
		}

		fMapCount--;
	}

	Flush();
		// flush explicitely, since we directly use the lock

	locker.Unlock();

	bool isKernelSpace = area->address_space == VMAddressSpace::Kernel();
	uint32 freeFlags = CACHE_DONT_WAIT_FOR_MEMORY
		| (isKernelSpace ? CACHE_DONT_LOCK_KERNEL_SPACE : 0);
	while (vm_page_mapping* mapping = mappings.RemoveHead())
		vm_free_page_mapping(mapping->page->physical_page_number, mapping, freeFlags);
}


status_t
X86GPAtoHPATranslationMap::Query(addr_t virtualAddress,
	phys_addr_t* _physicalAddress, uint32* _flags)
{
	*_flags = 0;
	*_physicalAddress = 0;

	ThreadCPUPinner pinner(thread_get_current_thread());

	// This function may be called on the physical map area, so we must handle
	// large pages here. Look up the page directory entry for the virtual
	// address.
	uint64* pde = EPTPagingMethod::PageDirectoryEntryForAddress(
		fPagingStructures->VirtualPMLTop(), virtualAddress, fIsKernelMap,
		false, NULL, fPageMapper, fMapCount);
	if (pde == NULL || (*pde & X86_64_PDE_PRESENT) == 0)
		return B_OK;

	uint64 entry;
	if ((*pde & X86_64_PDE_LARGE_PAGE) != 0) {
		entry = *pde;
		*_physicalAddress = (entry & X86_64_PDE_ADDRESS_MASK)
			+ (virtualAddress % 0x200000);
	} else {
		uint64* virtualPageTable = (uint64*)fPageMapper->GetPageTableAt(
			*pde & X86_64_PDE_ADDRESS_MASK);
		entry = virtualPageTable[VADDR_TO_PTE(virtualAddress)];
		*_physicalAddress = entry & X86_64_PTE_ADDRESS_MASK;
	}

	// Translate the page state flags.
	if ((entry & X86_64_PTE_USER) != 0) {
		*_flags |= ((entry & X86_64_PTE_WRITABLE) != 0 ? B_WRITE_AREA : 0)
			| B_READ_AREA
			| ((entry & X86_64_PTE_NOT_EXECUTABLE) == 0 ? B_EXECUTE_AREA : 0);
	}

	*_flags |= ((entry & X86_64_PTE_WRITABLE) != 0 ? B_KERNEL_WRITE_AREA : 0)
		| B_KERNEL_READ_AREA
		| ((entry & X86_64_PTE_NOT_EXECUTABLE) == 0 ? B_KERNEL_EXECUTE_AREA : 0)
		| ((entry & X86_64_PTE_DIRTY) != 0 ? PAGE_MODIFIED : 0)
		| ((entry & X86_64_PTE_ACCESSED) != 0 ? PAGE_ACCESSED : 0)
		| ((entry & X86_64_PTE_PRESENT) != 0 ? PAGE_PRESENT : 0);

	TRACE("X86GPAtoHPATranslationMap::Query(%#" B_PRIxADDR ") -> %#"
		B_PRIxPHYSADDR " %#" B_PRIx32 " (entry: %#" B_PRIx64 ")\n",
		virtualAddress, *_physicalAddress, *_flags, entry);

	return B_OK;
}


status_t
X86GPAtoHPATranslationMap::QueryInterrupt(addr_t virtualAddress,
	phys_addr_t* _physicalAddress, uint32* _flags)
{
	// With our page mapper, there is no difference in getting a page table
	// when interrupts are enabled or disabled, so just call Query().
	return Query(virtualAddress, _physicalAddress, _flags);
}


status_t
X86GPAtoHPATranslationMap::Protect(addr_t start, addr_t end, uint32 attributes,
	uint32 memoryType)
{
	start = ROUNDDOWN(start, B_PAGE_SIZE);
	if (start >= end)
		return B_OK;

	TRACE("X86GPAtoHPATranslationMap::Protect(%#" B_PRIxADDR ", %#" B_PRIxADDR
		", %#" B_PRIx32 ")\n", start, end, attributes);

	// compute protection flags
	uint64 newProtectionFlags = 0;
	if ((attributes & B_USER_PROTECTION) != 0) {
		newProtectionFlags = X86_64_PTE_USER;
		if ((attributes & B_WRITE_AREA) != 0)
			newProtectionFlags |= X86_64_PTE_WRITABLE;
		if ((attributes & B_EXECUTE_AREA) == 0
			&& x86_check_feature(IA32_FEATURE_AMD_EXT_NX, FEATURE_EXT_AMD)) {
			newProtectionFlags |= X86_64_PTE_NOT_EXECUTABLE;
		}
	} else if ((attributes & B_KERNEL_WRITE_AREA) != 0)
		newProtectionFlags = X86_64_PTE_WRITABLE;

	ThreadCPUPinner pinner(thread_get_current_thread());

	do {
		uint64* pageTable = EPTPagingMethod::PageTableForAddress(
			fPagingStructures->VirtualPMLTop(), start, fIsKernelMap, false,
			NULL, fPageMapper, fMapCount);
		if (pageTable == NULL) {
			// Move on to the next page table.
			start = ROUNDUP(start + 1, k64BitPageTableRange);
			continue;
		}

		for (uint32 index = start / B_PAGE_SIZE % k64BitTableEntryCount;
				index < k64BitTableEntryCount && start < end;
				index++, start += B_PAGE_SIZE) {
			uint64 entry = pageTable[index];
			if ((entry & X86_64_PTE_PRESENT) == 0)
				continue;

			TRACE("X86GPAtoHPATranslationMap::Protect(): protect page %#"
				B_PRIxADDR "\n", start);

			// set the new protection flags -- we want to do that atomically,
			// without changing the accessed or dirty flag
			uint64 oldEntry;
			while (true) {
				oldEntry = EPTPagingMethod::TestAndSetTableEntry(
					&pageTable[index],
					(entry & ~(X86_64_PTE_PROTECTION_MASK
							| X86_64_PTE_MEMORY_TYPE_MASK))
						| newProtectionFlags
						| EPTPagingMethod::MemoryTypeToPageTableEntryFlags(
							memoryType),
					entry);
				if (oldEntry == entry)
					break;
				entry = oldEntry;
			}

			if ((oldEntry & X86_64_PTE_ACCESSED) != 0) {
				// Note, that we only need to invalidate the address, if the
				// accessed flag was set, since only then the entry could have
				// been in any TLB.
				InvalidatePage(start);
			}
		}
	} while (start != 0 && start < end);

	return B_OK;
}


status_t
X86GPAtoHPATranslationMap::ClearFlags(addr_t address, uint32 flags)
{
	TRACE("X86GPAtoHPATranslationMap::ClearFlags(%#" B_PRIxADDR ", %#" B_PRIx32
		")\n", address, flags);

	ThreadCPUPinner pinner(thread_get_current_thread());

	uint64* entry = EPTPagingMethod::PageTableEntryForAddress(
		fPagingStructures->VirtualPMLTop(), address, fIsKernelMap,
		false, NULL, fPageMapper, fMapCount);
	if (entry == NULL)
		return B_OK;

	uint64 flagsToClear = ((flags & PAGE_MODIFIED) ? X86_64_PTE_DIRTY : 0)
		| ((flags & PAGE_ACCESSED) ? X86_64_PTE_ACCESSED : 0);

	uint64 oldEntry = EPTPagingMethod::ClearTableEntryFlags(entry,
		flagsToClear);

	if ((oldEntry & flagsToClear) != 0)
		InvalidatePage(address);

	return B_OK;
}


bool
X86GPAtoHPATranslationMap::ClearAccessedAndModified(VMArea* area, addr_t address,
	bool unmapIfUnaccessed, bool& _modified)
{
	ASSERT(address % B_PAGE_SIZE == 0);

	TRACE("X86GPAtoHPATranslationMap::ClearAccessedAndModified(%#" B_PRIxADDR
		")\n", address);

	RecursiveLocker locker(fLock);
	ThreadCPUPinner pinner(thread_get_current_thread());

	uint64* entry = EPTPagingMethod::PageTableEntryForAddress(
		fPagingStructures->VirtualPMLTop(), address, fIsKernelMap,
		false, NULL, fPageMapper, fMapCount);
	if (entry == NULL)
		return false;

	uint64 oldEntry;

	if (unmapIfUnaccessed) {
		while (true) {
			oldEntry = *entry;
			if ((oldEntry & X86_64_PTE_PRESENT) == 0) {
				// page mapping not valid
				return false;
			}

			if (oldEntry & X86_64_PTE_ACCESSED) {
				// page was accessed -- just clear the flags
				oldEntry = EPTPagingMethod::ClearTableEntryFlags(entry,
					X86_64_PTE_ACCESSED | X86_64_PTE_DIRTY);
				break;
			}

			// page hasn't been accessed -- unmap it
			if (EPTPagingMethod::TestAndSetTableEntry(entry, 0, oldEntry)
					== oldEntry) {
				break;
			}

			// something changed -- check again
		}
	} else {
		oldEntry = EPTPagingMethod::ClearTableEntryFlags(entry,
			X86_64_PTE_ACCESSED | X86_64_PTE_DIRTY);
	}

	pinner.Unlock();

	_modified = (oldEntry & X86_64_PTE_DIRTY) != 0;

	if ((oldEntry & X86_64_PTE_ACCESSED) != 0) {
		// Note, that we only need to invalidate the address, if the
		// accessed flags was set, since only then the entry could have been
		// in any TLB.
		InvalidatePage(address);
		Flush();

		return true;
	}

	if (!unmapIfUnaccessed)
		return false;

	// We have unmapped the address. Do the "high level" stuff.

	fMapCount--;

	locker.Detach();
		// UnaccessedPageUnmapped() will unlock for us

	UnaccessedPageUnmapped(area,
		(oldEntry & X86_64_PTE_ADDRESS_MASK) / B_PAGE_SIZE);

	return false;
}


X86PagingStructures*
X86GPAtoHPATranslationMap::PagingStructures() const
{
	return fPagingStructures;
}

phys_addr_t
X86GPAtoHPATranslationMap::PhysicalPML4() const
{
	// TODO: If Haiku runs with 5-level paging
	// enabled the mapper will set up structures
	// of 5-levels for EPT. We would have to
	// return here a pointer to PML4 NOT PML5
	// if (la57) ...

	return fPagingStructures->PhysicalPMLTop();
}
