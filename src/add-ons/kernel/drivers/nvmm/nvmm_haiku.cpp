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

#include <sys/ioccom.h>

#include <StackOrHeapArray.h>

#include <vm/VMCache.h>
#include <vm/VMAddressSpace.h>

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


/*---------------------------------------------------------------------------------------*/

// aka os_vmspace_t
extern "C" struct haiku_vmspace {
	VMAddressSpace *address_space;
};


// aka os_vmobj_t
extern "C" struct haiku_vmobj {
	VMCache *cache;
	int32	ref_count;
};


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

	status_t status;
	status = VMAddressSpace::Create(0, vmin, vmax - vmin + 1, false, &ret->address_space);
	if (status != B_OK) {
		os_mem_free(ret, sizeof(os_vmspace_t));
		return NULL;
	}

	return ret;
}


extern "C"
void
os_vmspace_destroy(os_vmspace_t *vm)
{
	if (vm) {
		VMAddressSpace::Delete(vm->address_space);
		os_mem_free(vm, sizeof(os_vmspace_t));
	}
}


extern "C"
int
os_vmspace_fault(os_vmspace_t *vm, vaddr_t va, vm_prot_t prot)
{
	VMArea *area = vm->address_space->LookupArea(va);
	if (area == NULL)
		return 1;

	if ((area->protection && prot) != prot)
		return 1;

	// TODO: Page could be swapped out to disk?

	return 0;
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

	status_t status = VMCacheFactory::CreateAnonymousCache(ret->cache, false, numPages, 0, true, 0);
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


/*---------------------------------------------------------------------------------------*/


int32 api_version = B_CUR_DRIVER_API_VERSION;

static const char *sNVMMDevice = "nvmm/nvmm";
static const char *sDevices[] = { NULL, NULL };

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
	sDevices[0] = (const char*)sNVMMDevice;
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
	if (nvmm_init())
		return B_ERROR;

	TRACE_ALWAYS("nvmm: init_driver OK\n");
	return B_OK;
}


void
uninit_driver(void)
{
	TRACE_ALWAYS("nvmm: uninit_driver\n");
	nvmm_fini();
}
