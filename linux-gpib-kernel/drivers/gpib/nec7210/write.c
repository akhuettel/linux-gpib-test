/***************************************************************************
                              nec7210/write.c
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
#include "board.h"
#include <linux/string.h>
#include <asm/dma.h>

static int pio_write_wait(gpib_board_t *board, nec7210_private_t *priv,
	short wake_on_lacs, short wake_on_atn, short wake_on_bus_error)
{
	// wait until byte is ready to be sent
	if(wait_event_interruptible(board->wait,
		(test_bit(TACS_NUM, &board->status) && test_bit(WRITE_READY_BN, &priv->state)) ||
		test_bit(DEV_CLEAR_BN, &priv->state) ||
		(wake_on_bus_error && test_bit(BUS_ERROR_BN, &priv->state)) ||
		(wake_on_lacs && test_bit(LACS_NUM, &board->status)) ||
		(wake_on_atn && test_bit(ATN_NUM, &board->status)) ||
#if (GPIB_CONFIG_DEVICE==1)
   		test_bit(ADSC_BN, &priv->state) ||
		test_bit(APT_NUM, &board->status) ||
#endif
		test_bit(TIMO_NUM, &board->status)))
	{
		GPIB_DPRINTK( "gpib write interrupted\n" );
		return -ERESTARTSYS;
	}
	if(test_bit(TIMO_NUM, &board->status))
	{
		GPIB_DPRINTK("nec7210: write timed out\n");
		return -ETIMEDOUT;
	}
	if(test_bit(DEV_CLEAR_BN, &priv->state))
	{
		GPIB_DPRINTK("nec7210: write interrupted by clear\n");
		return -EINTR;
	}
#if (GPIB_CONFIG_DEVICE==1)
	if(test_bit(ADSC_BN, &priv->state))
	{
		GPIB_DPRINTK("nec7210: address change detected (untalk)\n");
		return -EINTR;
	}
	if(wake_on_lacs && test_bit(LACS_NUM, &board->status))
	{
		GPIB_DPRINTK("nec7210: addressed as listener\n");
		return -EINTR;
	}
	if(wake_on_atn && test_bit(ATN_NUM, &board->status))
	{
		GPIB_DPRINTK("nec7210: ATN asserted\n");
		return -EINTR;
	}
	if(test_bit(APT_NUM, &board->status))
	{
		GPIB_DPRINTK("nec7210: secondary detected\n");
		return -EINTR;
	}
#endif
	if(wake_on_bus_error && test_and_clear_bit(BUS_ERROR_BN, &priv->state))
	{
		GPIB_DPRINTK("nec7210: bus error on write\n");
		return -EIO;
	}
	return 0;
}

static int pio_write(gpib_board_t *board, nec7210_private_t *priv,
	uint8_t *buffer, size_t length, size_t *bytes_written, int send_eoi)
{
	size_t last_count = 0;
	ssize_t retval = 0;
	unsigned long flags;
	const int max_bus_errors = (length > 1000) ? length : 1000;
	int bus_error_count = 0;
	*bytes_written = 0;

	smp_mb__before_atomic();
	clear_bit(BUS_ERROR_BN, &priv->state);
	smp_mb__after_atomic();

	while(*bytes_written < length)
	{
		if(need_resched())
			schedule();

		retval = pio_write_wait(board, priv, 0, 0, priv->type == NEC7210);
		if(retval == -EIO)
		{
			/* resend last byte on bus error */
			*bytes_written = last_count;
			GPIB_DPRINTK("resending %c\n", buffer[*bytes_written]);
			/* we can get unrecoverable bus errors,
			* so give up after a while */
			bus_error_count++;
			if(bus_error_count > max_bus_errors) return retval;
			else continue;
		}else if( retval < 0 ) goto done;

		if (send_eoi && (*bytes_written == (length - 1)))
			write_byte(priv, AUX_SEOI, AUXMR);

		spin_lock_irqsave(&board->spinlock, flags);
		clear_bit(BUS_ERROR_BN, &priv->state);
		clear_bit(WRITE_READY_BN, &priv->state);
		last_count = *bytes_written;
		write_byte(priv, buffer[(*bytes_written)++], CDOR);
		spin_unlock_irqrestore(&board->spinlock, flags);
	}
	retval = pio_write_wait(board, priv, 1, 1, priv->type == NEC7210);

done:
	if (retval < 0) {
		if ((retval == -EIO) || test_and_clear_bit(BUS_ERROR_BN, &priv->state))
			*bytes_written = last_count;
	}
	return retval;
}
#if 0
static ssize_t __dma_write(gpib_board_t *board, nec7210_private_t *priv, dma_addr_t address, size_t length)
{
	unsigned long flags, dma_irq_flags;
	int residue = 0;
	int retval = 0;

	spin_lock_irqsave(&board->spinlock, flags);

	/* program dma controller */
	dma_irq_flags = claim_dma_lock();
	disable_dma(priv->dma_channel);
	clear_dma_ff(priv->dma_channel);
	set_dma_count(priv->dma_channel, length);
	set_dma_addr(priv->dma_channel, address);
	set_dma_mode(priv->dma_channel, DMA_MODE_WRITE );
	enable_dma(priv->dma_channel);
	release_dma_lock(dma_irq_flags);

	// enable board's dma for output
	nec7210_set_reg_bits( priv, IMR2, HR_DMAO, HR_DMAO );

	smp_mb__before_atomic();
	clear_bit(WRITE_READY_BN, &priv->state);
	set_bit(DMA_WRITE_IN_PROGRESS_BN, &priv->state);
	smp_mb__after_atomic();

	spin_unlock_irqrestore(&board->spinlock, flags);

	// suspend until message is sent
	if(wait_event_interruptible(board->wait, test_bit(DMA_WRITE_IN_PROGRESS_BN, &priv->state) == 0 ||
		test_bit( BUS_ERROR_BN, &priv->state ) || test_bit( DEV_CLEAR_BN, &priv->state ) ||
		test_bit(TIMO_NUM, &board->status)))
	{
		GPIB_DPRINTK( "gpib write interrupted!\n" );
		retval = -ERESTARTSYS;
	}
	if(test_bit(TIMO_NUM, &board->status))
		retval = -ETIMEDOUT;
	if( test_and_clear_bit( DEV_CLEAR_BN, &priv->state ) )
		retval = -EINTR;
	if( test_and_clear_bit( BUS_ERROR_BN, &priv->state ) )
		retval = -EIO;

	// disable board's dma
	nec7210_set_reg_bits( priv, IMR2, HR_DMAO, 0 );

	dma_irq_flags = claim_dma_lock();
	clear_dma_ff(priv->dma_channel);
	disable_dma(priv->dma_channel);
	residue = get_dma_residue(priv->dma_channel);
	release_dma_lock( dma_irq_flags );

	if(residue)
		retval = -EPIPE;

	return retval ? retval : length;
}

static ssize_t dma_write(gpib_board_t *board, nec7210_private_t *priv, uint8_t *buffer, size_t length)
{
	size_t remain = length;
	size_t transfer_size;
	ssize_t retval = 0;

	while(remain > 0)
	{
		transfer_size = (priv->dma_buffer_length < remain) ? priv->dma_buffer_length : remain;
		memcpy(priv->dma_buffer, buffer, transfer_size);
		retval = __dma_write(board, priv, priv->dma_buffer_addr, transfer_size);
		if(retval < 0) break;
		remain -= retval;
		buffer += retval;
	}

	if(retval < 0) return retval;

	return length - remain;
}
#endif
int nec7210_write(gpib_board_t *board, nec7210_private_t *priv, uint8_t *buffer, size_t length,
	int send_eoi, size_t *bytes_written)
{
	int retval = 0;


	unsigned int adr1_bits = 0;
	/* we need to temporarily disable the minor address in order
	 to see the untalk  start HPDRIVE extension*/
	adr1_bits = read_byte(priv, ADR1);
	if ((adr1_bits & (HR_DT | HR_DL | ADDRESS_MASK)) == (HR_DL | 0x1f))
		write_byte(priv, HR_ARS | HR_DT | HR_DL, ADR);
	smp_mb__before_atomic();
	clear_bit(LACS_NUM, &board->status);
	clear_bit(ATN_NUM, &board->status);
	clear_bit(ADSC_BN, &priv->state);
        /*  end HPDRIVE extension */
	clear_bit( DEV_CLEAR_BN, &priv->state ); //XXX
	smp_mb__after_atomic();

	*bytes_written = 0;

	if (length == 0) return 0;
	retval = pio_write(board, priv, buffer, length, bytes_written, send_eoi);
	/* restore minor addressing HPDRIVE extension */
	if ((adr1_bits & (HR_DT | HR_DL | ADDRESS_MASK)) == (HR_DL | 0x1f))
		write_byte(priv, HR_ARS | HR_DL | 0x1f, ADR);

	return retval;
}

EXPORT_SYMBOL(nec7210_write);











