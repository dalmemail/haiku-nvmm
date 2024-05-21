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

#include <OS.h>

#include <arch/x86/arch_cpu.h>
#include <kernel/heap.h>
#include <kernel/smp.h>
#include <kernel/vm/vm.h>

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

extern "C"
int
os_contigpa_zalloc(paddr_t *pa, vaddr_t *va, size_t npages)
{
	area_id area = create_area(NULL, (void **)va, B_ANY_KERNEL_ADDRESS,
		npages * PAGE_SIZE, B_CONTIGUOUS, B_READ_AREA | B_WRITE_AREA);

	if (area < 0)
		return area;

	memset(va, 0, npages * PAGE_SIZE);

	status_t status = vm_get_page_mapping(0, *va, pa);
	if (status < 0) {
		delete_area(area);
		return status;
	}

	return 0;
}

extern "C"
void
os_contigpa_free(paddr_t pa __unused, vaddr_t va, size_t npages __unused)
{
	delete_area(area_for((void *)va));
}


/*---------------------------------------------------------------------------------------*/


int32 api_version = B_CUR_DRIVER_API_VERSION;


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
	return NULL;
}


device_hooks*
find_device(const char* name)
{
	TRACE_ALWAYS("nvmm: find_device\n");
	return NULL;
}


status_t
init_driver(void)
{
	TRACE_ALWAYS("nvmm: init_driver\n");
	return B_OK;
}


void
uninit_driver(void)
{
	TRACE_ALWAYS("nvmm: uninit_driver\n");
}
