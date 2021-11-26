/*
 *  jornada720-common.h
 *
 *  Common definitions for jornada720 sounddriver
 *
 *  Copyright (C) 2021 Timo Biesenbach
 *  
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef JORNADA720_COMMON_H
#define JORNADA720_COMMON_H

// ******** Configuration switches ********
// Startup sound, undef below to disable
#undef STARTUP_CHIME

// Disable sleep ability for now
#undef CONFIG_PM_SLEEP

// Fixed samplerate
// #define RATE_FIXED
#undef RATE_FIXED

// ********* Common constants **********
// One DMA transfer is up to 4kb, 0x800 defined in sa1111.h, 1ffc = (1<<13)-1 ~& 0x03 (max as per intel datasheet)
#define MIN_DMA_BLOCK_SIZE (0x0800)
#define MAX_DMA_BLOCK_SIZE (0x1ffc)

// Buffer sizes for dma allocation
#define MIN_BUFFER_SIZE		(16*1024)
#define MAX_BUFFER_SIZE		(64*1024)

#endif