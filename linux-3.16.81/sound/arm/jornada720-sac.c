/*
 *  jornada720-sac.h
 *
 *  Register interface to SA1111 Serial Audio Controller and L3 bus
 *
 *  Copyright (C) 2021 Timo Biesenbach
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/hrtimer.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/types.h>
// Hardware stuff
#include <linux/kernel.h>
#include <linux/kern_levels.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <mach/jornada720.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/hardware/sa1111.h>

#include "jornada720-common.h"
#include "jornada720-sac.h"

#ifdef DEBUG_SAC
#define DEBUG
#else
#undef DEBUG
#endif

// SAC module lock
static DEFINE_SPINLOCK(snd_jornada720_sa1111_sac_lock);

// SA1111 Sound Controller register write interface
void         sa1111_sac_writereg(struct sa1111_dev *devptr, unsigned int val, u32 reg) {
	sa1111_writel(val, devptr->mapbase + reg);
}

// SA1111 Sound Controller register read interface
unsigned int sa1111_sac_readreg(struct sa1111_dev *devptr, u32 reg) {
	return sa1111_readl(devptr->mapbase + reg);
}

// Send bytes via SA1111-L3
void 		   sa1111_l3_send_byte(struct sa1111_dev *devptr, unsigned char addr, unsigned char dat) {
	int i=0;
	unsigned int SASCR;
	unsigned int SACR1;
	
	// Make sure only one thread is in the critical section below.
	spin_lock(&snd_jornada720_sa1111_sac_lock);
	sa1111_sac_writereg(devptr, 0, SA1111_L3_CAR);
	sa1111_sac_writereg(devptr, 0, SA1111_L3_CDR);
	mdelay(1);
	SASCR = SASCR_DTS|SASCR_RDD;
	sa1111_sac_writereg(devptr, SASCR, SA1111_SASCR);
	sa1111_sac_writereg(devptr, addr,  SA1111_L3_CAR);
	sa1111_sac_writereg(devptr, dat,   SA1111_L3_CDR);

	// Wait for L3 to come back in 200ms
	while (((sa1111_sac_readreg(devptr, SA1111_SASR0) & SASR0_L3WD) == 0) && (i < 200)) {
		mdelay(1);
		i++;
	}
	// If still not confirmed, restart L3 and retry the transmission
	if (((sa1111_sac_readreg(devptr, SA1111_SASR0) & SASR0_L3WD) == 0)) {
		DPRINTK(KERN_INFO "sac: Avoided crash in l3_sa1111_send_byte. Trying to reset L3.\n");
		SACR1 = sa1111_sac_readreg(devptr, SA1111_SACR1);
		SACR1 &= ~SACR1_L3EN;
		sa1111_sac_writereg(devptr, SACR1, SA1111_SACR1);
		mdelay(100);
		SACR1 = sa1111_sac_readreg(devptr, SA1111_SACR1);
		SACR1 |= SACR1_L3EN;
		sa1111_sac_writereg(devptr, SACR1, SA1111_SACR1);
		mdelay(100);

		// Retry transmission
		sa1111_sac_writereg(devptr, 0, SA1111_L3_CAR);
		sa1111_sac_writereg(devptr, 0, SA1111_L3_CDR);
		mdelay(1);
		SASCR = SASCR_DTS|SASCR_RDD;
		sa1111_sac_writereg(devptr, SASCR, SA1111_SASCR);
		sa1111_sac_writereg(devptr, addr,  SA1111_L3_CAR);
		sa1111_sac_writereg(devptr, dat,   SA1111_L3_CDR);

		// Wait for L3 to come back in 200ms
		while (((sa1111_sac_readreg(devptr, SA1111_SASR0) & SASR0_L3WD) == 0) && (i < 200)) {
			mdelay(1);
			i++;
		}
	}
	
	SASCR = SASCR_DTS|SASCR_RDD;
	sa1111_sac_writereg(devptr, SASCR, SA1111_SASCR);
	
	// Give up the lock
	spin_unlock(&snd_jornada720_sa1111_sac_lock);

	// Wait 20msec before next transfer (uda1344 L3 is limited to 64f/s)
	mdelay(20);
}

// Will initialize the SA1111 and its L3 hardware
void sa1111_audio_init(struct sa1111_dev *devptr) {
	// For register bitbanging
	unsigned int val; 

	// Get access to the "parent" sa1111 chip 
	struct sa1111 *sachip = get_sa1111_base_drv(devptr);

	DPRINTK(KERN_INFO "sac: SA1111 init...");
	DPRINTK(KERN_INFO "sac: SA1111 device id: %d\n", devptr->devid);
	DPRINTK(KERN_INFO "sac: SA1111 chip base: 0x%lxh\n", sachip->base);
	DPRINTK(KERN_INFO "sac: SA1111 SAC  base: 0x%lxh\n", devptr->mapbase);

	// Make sure only one thread is in the critical section below.
	spin_lock(&snd_jornada720_sa1111_sac_lock);
	
	PPSR &= ~(PPC_LDD3 | PPC_LDD4); // 5/6 are not leds
	PPDR |= PPC_LDD3 | PPC_LDD4;
	PPSR |= PPC_LDD4; /* enable speaker */
	PPSR |= PPC_LDD3; /* enable microphone */
	DPRINTK(KERN_INFO "sac: SA1111 speaker/mic pre-amps enabled\n");
	
	// deselect AC Link
	sa1111_select_audio_mode(devptr, SA1111_AUDIO_I2S);
	DPRINTK(KERN_INFO "sac: SA1111 I2S protocol enabled\n");

	/* Enable the I2S clock and L3 bus clock. This is a function in another SA1111 block
	 * which is why we need the sachip stuff (should probably be a function in sa1111.c/h)
	 */
	val = sa1111_readl(sachip->base + SA1111_SKPCR);
	val|= (SKPCR_I2SCLKEN | SKPCR_L3CLKEN);
	sa1111_writel(val, sachip->base + SA1111_SKPCR);
	DPRINTK(KERN_INFO "sac: SA1111 I2S and L3 clocks enabled\n");

	/* Activate and reset the Serial Audio Controller */
	val = sa1111_sac_readreg(devptr, SA1111_SACR0);
	val |= (SACR0_ENB | SACR0_RST);
	sa1111_sac_writereg(devptr, val, SA1111_SACR0);

	mdelay(5);

	val = sa1111_sac_readreg(devptr, SA1111_SACR0);
	val &= ~SACR0_RST;
	sa1111_sac_writereg(devptr, val, SA1111_SACR0);
	DPRINTK(KERN_INFO "sac: SA1111 SAC reset and enabled\n");

	sa1111_sac_writereg(devptr, SACR1_L3EN, SA1111_SACR1);
	DPRINTK(KERN_INFO "sac: SA1111 L3 interface enabled\n");

	// Set samplerate
	sa1111_set_audio_rate(devptr, 22050);
	int rate = sa1111_get_audio_rate(devptr);
	
	spin_unlock(&snd_jornada720_sa1111_sac_lock);

	DPRINTK(KERN_INFO "sac:SA1111 audio samplerate: %d\n", rate);
	DPRINTK(KERN_INFO "sac: done init.\n");
}