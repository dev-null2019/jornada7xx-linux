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
#ifndef JORNADA720_SAC_H
#define JORNADA720_SAC_H

#include <asm/hardware/sa1111.h>

/* SA1111 SAC IRQs (Note: These are relative to the sa1111 irqbase!) */
#define AUDXMTDMADONEA		(32)
#define AUDRCVDMADONEA		(33)
#define AUDXMTDMADONEB		(34)
#define AUDRCVDMADONEB		(35)

/*	Get the parent device driver structure from a child function device
 *  Copied here from sa1111.c since unfortunately not exported by sa1111.h  */
struct sa1111 {
	struct device	*dev;
	struct clk	*clk;
	unsigned long	phys;
	int		irq;
	int		irq_base;	/* base for cascaded on-chip IRQs */
	spinlock_t	lock;
	void __iomem	*base;
	struct sa1111_platform_data *pdata;
#ifdef CONFIG_PM
	void		*saved_state;
#endif
};

/* Access the parent (sa1111) chip driver data */
#define get_sa1111_base_drv(sadev) \
	((struct sa1111 *)dev_get_drvdata(((struct sa1111_dev *)sadev)->dev.parent))

/* Access the parent (sa1111) chip driver data */
#define get_sa1111_irq(irq, sadev) \
	(get_sa1111_base_drv(sadev)->irq_base + irq)

#define TO_SA1111_IRQ(irq, sadev) \
	(get_sa1111_base_drv(sadev)->irq_base + irq)

#define FROM_SA1111_IRQ(irq, sadev) \
	(irq - get_sa1111_base_drv(sadev)->irq_base)

/* SA1111 SAC Audio initilization */
extern void         sa1111_audio_init(struct sa1111_dev *devptr);

/* SA1111 SAC Register write */
extern void         sa1111_sac_writereg(struct sa1111_dev *devptr, unsigned int val, u32 reg);

/* SA1111 SAC Register read */
extern unsigned int sa1111_sac_readreg(struct sa1111_dev *devptr, u32 reg);

/* SA1111 SAC send byte via L3 bus */
extern void 		sa1111_l3_send_byte(struct sa1111_dev *devptr, unsigned char addr, unsigned char dat);

// From top ifndef
#endif