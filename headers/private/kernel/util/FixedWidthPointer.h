/*
 * Copyright 2012, Alex Smith, alex@alex-smith.me.uk.
 * Distributed under the terms of the MIT License.
 */
#ifndef KERNEL_UTIL_FIXED_WIDTH_POINTER_H
#define KERNEL_UTIL_FIXED_WIDTH_POINTER_H

typedef union { uint64 padding; void *ptr; } FixedWidthPointer;

#endif	/* KERNEL_UTIL_FIXED_WIDTH_POINTER_H */
