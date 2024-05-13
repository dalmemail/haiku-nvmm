/*
 * Copyright 2024 Daniel Martin, dalmemail@gmail.com
 * All rights reserved. Distributed under the terms of the MIT License.
 */


#include <Drivers.h>

#include "nvmm.h"
#include "nvmm_internal.h"
#include "nvmm_os.h"
#include "x86/nvmm_x86.h"

#include <OS.h>


void x86_get_cpuid(uint32_t eax, cpuid_desc_t *descriptors)
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
