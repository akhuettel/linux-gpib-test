/***************************************************************************
                              nec7210/interrupt.c
                             -------------------

    begin                : Dec 2001
    copyright            : (C) 2001, 2002 by Frank Mori Hess
    email                : fmhess@users.sourceforge.net
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "tnt4882.h"
#include <asm/bitops.h>
#include <asm/dma.h>

/*
 * GPIB interrupt service routines
 */

void tnt4882_interrupt(int irq, void *arg, struct pt_regs *registerp)
{
	gpib_device_t *device = arg;
	tnt4882_private_t *priv = device->private_data;
printk("BSR 0x%x CSR 0x%x\n", readb(priv->nec7210_priv.iobase + 0x1f),
	readb(priv->nec7210_priv.iobase + 0x17));

	nec7210_interrupt(device, &priv->nec7210_priv);

}

