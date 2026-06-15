// Copyright 2025 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// L1->L1 (TCDM->TCDM) iDMA copy across a range of sizes.
//
// Both buffers live in the cluster TCDM, so the iDMA address decoder routes the
// source to an OBI read and the destination to an OBI write -- they share the
// single OBI/TCDM manager port of idma_inst64_top, exercising concurrent OBI
// read+write (the read/write response demux). snrt_fence() orders the buffer
// initialisation (core stores) before the DMA's wide, highest-priority writes.

#include <snrt.h>

#define MAXN 256

int main() {
#ifdef SNRT_SUPPORTS_DMA
    if (!snrt_is_dm_core()) return 0;  // only the DMA core
    uint32_t errors = 0;

    static uint32_t src[MAXN];
    static uint32_t dst[MAXN];

    const uint32_t sizes[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};

    for (uint32_t s = 0; s < sizeof(sizes) / sizeof(sizes[0]); s++) {
        uint32_t n = sizes[s];
        for (uint32_t i = 0; i < n; i++) {
            src[i] = 0xC0DE0000u | i;
            dst[i] = 0xDEAD0000u | i;
        }
        snrt_fence();
        snrt_dma_start_1d(dst, src, n * sizeof(uint32_t), 0);
        snrt_dma_wait_all(0);
        for (uint32_t i = 0; i < n; i++)
            errors += (dst[i] != (0xC0DE0000u | i));
    }

    return errors;
#endif
}
