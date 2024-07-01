/*
 * Copyright 2024 Daniel Martin, dalmemail@gmail.com
 * All rights reserved. Distributed under the terms of the MIT License.
 */


#include <Drivers.h>

extern "C" {
#include "nvmm.h"
#include "nvmm_internal.h"
#include "nvmm_os.h"
#include "x86/nvmm_x86.h"
}

#include <drivers/KernelExport.h>
#include <OS.h>

#include <arch/x86/arch_cpu.h>
#include <kernel/heap.h>
#include <kernel/smp.h>
#include <kernel/thread.h>

#include <sys/ioccom.h>

#include <StackOrHeapArray.h>

#include <vm/VMCache.h>
#include <vm/VMAddressSpace.h>

#include <paging/64bit/X86GPAtoHPATranslationMap.h>

#define __unused __attribute__ ((unused))


extern "C" void x86_get_cpuid(uint32_t eax, cpuid_desc_t *descriptors)
{
	cpuid_info info;
	if (get_cpuid(&info, eax, 0) != B_OK) {
		descriptors->eax = 0;
		descriptors->ebx = 0;
		descriptors->ecx = 0;
		descriptors->edx = 0;
	}
	else {
		descriptors->eax = info.regs.eax;
		descriptors->ebx = info.regs.ebx;
		descriptors->ecx = info.regs.ecx;
		descriptors->edx = info.regs.edx;
	}
}


extern "C" int haiku_get_xsave_mask()
{
	if (x86_check_feature(IA32_FEATURE_EXT_XSAVE, FEATURE_EXT))
		return IA32_XCR0_X87 | IA32_XCR0_SSE;

	return 0;
}


extern "C" int32 haiku_smp_get_current_cpu()
{
	return smp_get_current_cpu();
}


extern "C" int32 haiku_smp_get_num_cpus()
{
	return smp_get_num_cpus();
}


extern "C" thread_id haiku_get_current_thread_id()
{
	return thread_get_current_thread_id();
}


extern "C"
void
os_ipi_unicast(os_cpu_t *cpu, void (*func)(void *, int), void *arg)
{
	call_single_cpu_sync(*cpu, func, arg);
}


extern "C"
int
haiku_thread_bind()
{
	thread_pin_to_current_cpu(thread_get_current_thread());
	// curlwp_bind() returns int
	return 0;
}


extern "C"
void
haiku_thread_unbind()
{
	thread_unpin_from_current_cpu(thread_get_current_thread());
}


extern "C"
status_t
os_mtx_lock(os_mtx_t *lock)
{
	// if interrupts are disabled
	if (os_preempt_disabled())
		while (mutex_trylock(lock) != B_OK);
	else
		return mutex_lock(lock);

	return B_OK;
}

/*---------------------------------------------------------------------------------------*/

// aka os_vmmap_t
extern "C" struct haiku_map {
	VMAddressSpace *address_space;
};


// aka os_vmobj_t
extern "C" struct haiku_vmobj {
	VMCache *cache;
	int32	ref_count;
};


// aka os_vmspace_t
extern "C" struct haiku_vmspace {
	VMAddressSpace *address_space;
	// this holds exactly the same as address_space (duplicated)
	// but os_vmobj_map expects an os_vmmap_t*
	os_vmmap_t *vm_map;
	struct pmap pmap;
};


// aka os_cpuset_t
extern "C" struct haiku_cpuset {
	CPUSet *set;
};

os_vmmap_t *os_kernel_map;
cpu_status *interrupt_status;


extern "C"
void *
os_pagemem_zalloc(size_t size)
{
	void *ptr;
	size_t alloc_size = roundup(size, PAGE_SIZE);
	area_id area = create_area("os_pagemem_zalloc_area", &ptr, B_ANY_KERNEL_ADDRESS,
		alloc_size, B_FULL_LOCK, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA);

	if (area < 0)
		return NULL;

	memset(ptr, 0, alloc_size);

	return ptr;
}


extern "C"
void
os_pagemem_free(void *ptr, size_t size __unused)
{
	delete_area(area_for(ptr));
}


extern "C"
int
os_contigpa_zalloc(paddr_t *pa, vaddr_t *va, size_t npages)
{
	area_id area = create_area("os_contigpa_zalloc_area", (void **)va, B_ANY_KERNEL_ADDRESS,
		npages * PAGE_SIZE, B_CONTIGUOUS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA);

	if (area < 0)
		return area;

	memset((void *)*va, 0, npages * PAGE_SIZE);

	physical_entry entry;
	status_t status = get_memory_map((void *)*va, 1, &entry, 1);
	if (status < B_OK) {
		delete_area(area);
		return status;
	}
	*pa = entry.address;

	return 0;
}


extern "C"
void
os_contigpa_free(paddr_t pa __unused, vaddr_t va, size_t npages __unused)
{
	delete_area(area_for((void *)va));
}


extern "C"
os_vmspace_t *
os_vmspace_create(vaddr_t vmin, vaddr_t vmax)
{
	if (vmax < vmin)
		return NULL;

	os_vmspace_t *ret = (os_vmspace_t *)os_mem_alloc(sizeof(os_vmspace_t));
	if (ret == NULL)
		return NULL;

	ret->vm_map = (os_vmmap_t *)os_mem_alloc(sizeof(os_vmmap_t));
	if (ret->vm_map == NULL) {
		os_mem_free(ret, sizeof(os_vmspace_t));
		return NULL;
	}

	size_t size = vmax - vmin + 1;
	status_t status = VMAddressSpace::Create(0, vmin, size, false, true, &ret->address_space);
	if (status != B_OK) {
		os_mem_free(ret->vm_map, sizeof(os_vmmap_t));
		os_mem_free(ret, sizeof(os_vmspace_t));
		return NULL;
	}
	ret->vm_map->address_space = ret->address_space;

	ret->pmap.pm_invgen = 0;

	return ret;
}


extern "C"
void
os_vmspace_destroy(os_vmspace_t *vm)
{
	vm->address_space->Put();
	os_mem_free(vm->vm_map, sizeof(os_vmmap_t));
	os_mem_free(vm, sizeof(os_vmspace_t));
}


extern "C"
int
os_vmspace_fault(os_vmspace_t *vm, vaddr_t va, vm_prot_t prot)
{
	VMArea *area = vm->address_space->LookupArea(va);
	if (area == NULL)
		return 1;

	// TODO: Probably through page->cache_ref->cache
	//if ((area->protection & prot) != prot)
	//	return 1;

	// TODO: Page could be swapped out to disk?

	return 0;
}


extern "C"
struct pmap*
os_vmspace_pmap(os_vmspace_t *vm)
{
	return &vm->pmap;
}


extern "C"
paddr_t
os_vmspace_pdirpa(os_vmspace_t *vm)
{
	X86GPAtoHPATranslationMap *map;
	map = (X86GPAtoHPATranslationMap *)vm->address_space->TranslationMap();

	return map->PhysicalPML4();
}


extern "C"
os_vmmap_t *
os_vmspace_get_vmmap(os_vmspace_t *vm)
{
	return vm->vm_map;
}


extern "C"
os_vmobj_t *
os_vmobj_create(voff_t size)
{
	os_vmobj_t *ret = (os_vmobj_t *)os_mem_alloc(sizeof(os_vmobj_t));
	if (ret == NULL)
		return NULL;

	int32 numPages = size / PAGE_SIZE;
	if (size % PAGE_SIZE != 0)
		numPages++;

	status_t status = VMCacheFactory::CreateAnonymousCache(ret->cache, false,
				numPages, 0, true, 0);
	if (status != B_OK) {
		os_mem_free(ret, sizeof(os_vmobj_t));
		return NULL;
	}
	ret->ref_count = 0;

	return ret;
}


extern "C"
void
os_vmobj_ref(os_vmobj_t *vmobj)
{
	atomic_add(&vmobj->ref_count, 1);
}


extern "C"
void
os_vmobj_rel(os_vmobj_t *vmobj)
{
	int32 previous = atomic_add(&vmobj->ref_count, -1);
	if (previous == 0) {
		vmobj->cache->Delete();
		os_mem_free(vmobj, sizeof(os_vmobj_t));
	}
}


extern "C"
int
os_vmobj_map(os_vmmap_t *map, vaddr_t *addr, vsize_t size, os_vmobj_t *vmobj,
	voff_t offset, bool wired, bool fixed, bool shared, int prot, int maxprot)
{
	if (!vmobj->cache->Lock())
		return B_ERROR;

	status_t status = map->address_space->WriteLock();
	if (status != B_OK)
		return status;

	uint32 wiring = wired ? B_FULL_LOCK : B_NO_LOCK;
	int mapping = REGION_PRIVATE_MAP;
	uint32 flags = 0;
	bool kernel = false;
	if (map->address_space == VMAddressSpace::Kernel())
		kernel = true;

	virtual_address_restrictions addressRestrictions = {
		.address = (void *)*addr,
		.address_specification = fixed ? B_EXACT_ADDRESS : B_ANY_ADDRESS,
		.alignment = B_PAGE_SIZE,
	};
	VMArea *area;
	status = map_backing_store(map->address_space, vmobj->cache, vmobj->cache->virtual_base,
		"nvmm_vmobj_area", size, wiring, prot, maxprot, mapping, flags,
		&addressRestrictions, kernel, &area, (void **)addr);

	map->address_space->WriteUnlock();
	vmobj->cache->Unlock();

	return status;
}


extern "C"
void
os_vmobj_unmap(os_vmmap_t *map, vaddr_t start, vaddr_t end,
	bool wired __unused)
{
	bool kernel = map->address_space == VMAddressSpace::Kernel();
	map->address_space->WriteLock();
	discard_address_range(map->address_space, start, end - start, kernel);
	map->address_space->WriteUnlock();
}


extern "C"
os_vmmap_t *
os_get_curproc_map()
{
	os_vmmap_t *ret = (os_vmmap_t *)os_mem_alloc(sizeof(os_vmmap_t));
	if (ret == NULL)
		return NULL;

	ret->address_space = VMAddressSpace::GetCurrent();
	return ret;
}


extern "C"
void
os_free_curproc_map(os_vmmap_t *map)
{
	os_mem_free(map, sizeof(os_vmmap_t));
}


extern "C"
status_t
os_cpuset_init(os_cpuset_t *cpuset)
{
	cpuset = (os_cpuset_t *)malloc(sizeof(os_cpuset_t));
	if (cpuset == NULL)
		return B_NO_MEMORY;

	cpuset->set = new CPUSet();

	return B_OK;
}


extern "C"
void
os_cpuset_destroy(os_cpuset_t *cpuset)
{
	delete cpuset->set;
	free(cpuset);
}


extern "C"
bool
os_cpuset_isset(os_cpuset_t *cpuset, int32 cpu)
{
	return cpuset->set->GetBit(cpu);
}


extern "C"
void
os_cpuset_clear(os_cpuset_t *cpuset, int32 cpu)
{
	cpuset->set->ClearBitAtomic(cpu);
}


extern "C"
void
os_cpuset_setrunning(os_cpuset_t *cpuset)
{
	cpuset->set->SetAll();
}


/*---------------------------------------------------------------------------------------*/


int32 api_version = B_CUR_DRIVER_API_VERSION;

static const char *sDevices[] = { "misc/nvmm", NULL };

static status_t nvmm_open_hook(const char *name, uint32 flags, void **cookie);
static status_t nvmm_close_hook(void *cookie);
static status_t nvmm_free_hook(void* cookie);
static status_t nvmm_control_hook(void *cookie, uint32 op, void *data, size_t len);

static device_hooks sHooks = {
	.open = nvmm_open_hook,
	.close = nvmm_close_hook,
	.free = nvmm_free_hook,
	.control = nvmm_control_hook,
};


static status_t
nvmm_open_hook(const char *name, uint32 flags, void **cookie)
{
	if (!(flags & O_CLOEXEC))
		return B_BAD_VALUE;

	//TODO: Root owner not supported yet
	struct nvmm_owner *owner;
	owner = (struct nvmm_owner *)os_mem_alloc(sizeof(*owner));
	if (owner == NULL)
		return B_NO_MEMORY;

	owner->pid = getpid();
	*cookie = owner;

	return B_OK;
}


static status_t
nvmm_close_hook(void *cookie)
{
	if (cookie == NULL)
		return B_NO_INIT;

	struct nvmm_owner *owner = (struct nvmm_owner *)cookie;
	nvmm_kill_machines(owner);
	TRACE_ALWAYS("%d\n", owner->pid);

	return B_OK;
}


static status_t
nvmm_free_hook(void* cookie)
{
	if (cookie == NULL)
		return B_NO_INIT;

	os_mem_free(cookie, sizeof(struct nvmm_owner));

	return B_OK;
}


static status_t
nvmm_control_hook(void *cookie, uint32 op, void *data, size_t len)
{
	len = IOCPARM_LEN(op);
	BStackOrHeapArray<char, 128> kernel_data(len);

	struct nvmm_owner owner = { .pid = getpid(), };
	status_t status = user_memcpy(kernel_data, data, len);
	if (status < 0)
		return status;

	status_t ioctl_status = nvmm_ioctl(&owner, op, kernel_data);

	status = user_memcpy(data, kernel_data, len);
	if (status < 0)
		return status;

	return ioctl_status;
}


status_t
init_hardware(void)
{
	if (nvmm_ident() == NULL) {
		TRACE_ALWAYS("nvmm: cpu not supported\n");
		return B_ERROR;
	}
	return B_OK;
}


const char**
publish_devices(void)
{
	TRACE_ALWAYS("nvmm: publish_devices\n");
	return sDevices;
}


device_hooks*
find_device(const char* name)
{
	TRACE_ALWAYS("nvmm: find_device\n");
	return &sHooks;
}


status_t
init_driver(void)
{
	status_t status;
	int32 n_cpus;
	if (nvmm_init())
		return B_ERROR;

	os_kernel_map = (os_vmmap_t *)malloc(sizeof(os_vmmap_t));
	if (os_kernel_map == NULL) {
		status = B_NO_MEMORY;
		goto err1;
	}

	os_kernel_map->address_space = VMAddressSpace::Kernel();

	n_cpus = smp_get_num_cpus();
	if (n_cpus <= 0) {
		status = B_BAD_VALUE;
		goto err2;
	}

	interrupt_status = (cpu_status *)malloc(n_cpus * sizeof(cpu_status));
	if (interrupt_status == NULL) {
		status = B_NO_MEMORY;
		goto err2;
	}

	TRACE_ALWAYS("nvmm: init_driver OK\n");
	return B_OK;

err2:
	free(os_kernel_map);
err1:
	nvmm_fini();
	return status;
}


void
uninit_driver(void)
{
	TRACE_ALWAYS("nvmm: uninit_driver\n");
	nvmm_fini();
	free(os_kernel_map);
	free(interrupt_status);
}
