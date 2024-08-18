/*
 * Copyright 2014, Paweł Dziepak, pdziepak@quarnos.org.
 * Copyright 2012, Alex Smith, alex@alex-smith.me.uk.
 * Copyright 2010, Ingo Weinhold, ingo_weinhold@gmx.de.
 * Distributed under the terms of the MIT License.
 */
#ifndef KERNEL_ARCH_X86_PAGING_64BIT_EPT_PAGING_METHOD_H
#define KERNEL_ARCH_X86_PAGING_64BIT_EPT_PAGING_METHOD_H


#include <atomic>

#include <KernelExport.h>

#include <lock.h>
#include <vm/vm_types.h>

#include "paging/64bit/paging.h"
#include "paging/X86PagingMethod.h"
#include "paging/X86PagingStructures.h"


class TranslationMapPhysicalPageMapper;
class X86PhysicalPageMapper;
struct vm_page_reservation;


class EPTPagingMethod final : public X86PagingMethod {
public:
								EPTPagingMethod(bool la57);
	virtual						~EPTPagingMethod();

	virtual	status_t			Init(kernel_args* args,
									VMPhysicalPageMapper** _physicalPageMapper);
	virtual	status_t			InitPostArea(kernel_args* args);

	virtual	status_t			CreateTranslationMap(bool kernel,
									VMTranslationMap** _map);

	virtual	status_t			MapEarly(kernel_args* args,
									addr_t virtualAddress,
									phys_addr_t physicalAddress,
									uint8 attributes,
									page_num_t (*get_free_page)(kernel_args*));

	virtual	bool				IsKernelPageAccessible(addr_t virtualAddress,
									uint32 protection);

	inline	X86PhysicalPageMapper* PhysicalPageMapper() const
									{ return fPhysicalPageMapper; }
	inline	TranslationMapPhysicalPageMapper* KernelPhysicalPageMapper() const
									{ return fKernelPhysicalPageMapper; }

	inline	uint64*				KernelVirtualPMLTop() const
									{ return fKernelVirtualPMLTop; }
	inline	phys_addr_t			KernelPhysicalPMLTop() const
									{ return fKernelPhysicalPMLTop; }

	static	EPTPagingMethod* Method();

	static	uint64*				PageDirectoryForAddress(uint64* virtualPML4,
									addr_t virtualAddress, bool isKernel,
									bool allocateTables,
									vm_page_reservation* reservation,
									TranslationMapPhysicalPageMapper*
										pageMapper, int32& mapCount);
	static	uint64*				PageDirectoryEntryForAddress(
									uint64* virtualPML4, addr_t virtualAddress,
									bool isKernel, bool allocateTables,
									vm_page_reservation* reservation,
									TranslationMapPhysicalPageMapper*
										pageMapper, int32& mapCount);
	static	uint64*				PageTableForAddress(uint64* virtualPML4,
									addr_t virtualAddress, bool isKernel,
									bool allocateTables,
									vm_page_reservation* reservation,
									TranslationMapPhysicalPageMapper*
										pageMapper, int32& mapCount);
	static	uint64*				PageTableEntryForAddress(uint64* virtualPML4,
									addr_t virtualAddress, bool isKernel,
									bool allocateTables,
									vm_page_reservation* reservation,
									TranslationMapPhysicalPageMapper*
										pageMapper, int32& mapCount);

	static	void				PutPageTableEntryInTable(
									uint64* entry, phys_addr_t physicalAddress,
									uint32 attributes, uint32 memoryType,
									bool globalPage);
	static	void				SetTableEntry(uint64_t* entry,
									uint64_t newEntry);
	static	uint64_t			SetTableEntryFlags(uint64_t* entryPointer,
									uint64_t flags);
	static	uint64				TestAndSetTableEntry(uint64* entry,
									uint64 newEntry, uint64 oldEntry);
	static	uint64_t			ClearTableEntry(uint64_t* entryPointer);
	static	uint64_t			ClearTableEntryFlags(uint64_t* entryPointer,
									uint64_t flags);

	static	uint64				MemoryTypeToPageTableEntryFlags(
									uint32 memoryType);

private:
	static	void				_EnableExecutionDisable(void* dummy, int cpu);

			phys_addr_t			fKernelPhysicalPMLTop;
			uint64*				fKernelVirtualPMLTop;

			X86PhysicalPageMapper* fPhysicalPageMapper;
			TranslationMapPhysicalPageMapper* fKernelPhysicalPageMapper;

	static	bool				la57;
};


static_assert(sizeof(std::atomic<uint64_t>) == sizeof(uint64_t),
	"Non-trivial representation of atomic uint64_t.");


/*static*/ inline EPTPagingMethod*
EPTPagingMethod::Method()
{
	return static_cast<EPTPagingMethod*>(gX86PagingMethod);
}


/*static*/ inline void
EPTPagingMethod::SetTableEntry(uint64_t* entryPointer, uint64_t newEntry)
{
	auto& entry = *reinterpret_cast<std::atomic<uint64_t>*>(entryPointer);
	entry.store(newEntry, std::memory_order_relaxed);
}


/*static*/ inline uint64_t
EPTPagingMethod::SetTableEntryFlags(uint64_t* entryPointer, uint64_t flags)
{
	auto& entry = *reinterpret_cast<std::atomic<uint64_t>*>(entryPointer);
	return entry.fetch_or(flags);
}


/*static*/ inline uint64
EPTPagingMethod::TestAndSetTableEntry(uint64* entry, uint64 newEntry, uint64 oldEntry)
{
	return atomic_test_and_set64((int64*)entry, newEntry, oldEntry);
}


/*static*/ inline uint64_t
EPTPagingMethod::ClearTableEntry(uint64_t* entryPointer)
{
	auto& entry = *reinterpret_cast<std::atomic<uint64_t>*>(entryPointer);
	return entry.exchange(0);
}


/*static*/ inline uint64_t
EPTPagingMethod::ClearTableEntryFlags(uint64_t* entryPointer,
	uint64_t flags)
{
	auto& entry = *reinterpret_cast<std::atomic<uint64_t>*>(entryPointer);
	return entry.fetch_and(~flags);
}


/*static*/ inline uint64
EPTPagingMethod::MemoryTypeToPageTableEntryFlags(uint32 memoryType)
{
	switch (memoryType) {
		case B_MTR_UC:
			return EPT_PTE_CACHING_DISABLED;

		case B_MTR_WC:
			return EPT_PTE_WRITE_COMBINING;

		case B_MTR_WT:
			return X86_64_PTE_WRITE_THROUGH;

		case B_MTR_WP:
			return EPT_PTE_WRITE_PROTECT;
		case B_MTR_WB:
		default:
			return EPT_PTE_WRITE_BACK;
	}
}


#endif	// KERNEL_ARCH_X86_PAGING_64BIT_EPT_PAGING_METHOD_H
