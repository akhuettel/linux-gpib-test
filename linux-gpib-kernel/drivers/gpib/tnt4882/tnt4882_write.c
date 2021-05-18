/***************************************************************************
                          tnt4882_write.c  -  description
                             -------------------

    copyright            : (C) 2003 by Frank Mori Hess
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
#include <linux/delay.h>

static int fifo_space_available( tnt4882_private_t *tnt_priv )
{
	int status2;
	int retval;

	status2 = tnt_readb( tnt_priv, STS2 );
	retval = ( status2 & AFFN ) && ( status2 & BFFN );

	return retval;
}

static int fifo_xfer_done( tnt4882_private_t *tnt_priv )
{
	int status1;
	int retval;

	status1 = tnt_readb( tnt_priv, STS1 );
	retval = status1 & ( S_DONE | S_HALT );

	return retval;
}

static unsigned tnt_transfer_count(tnt4882_private_t *tnt_priv)
{
	unsigned count = 0;
	count |= tnt_readb(tnt_priv, CNT0) & 0xff;
	count |= (tnt_readb(tnt_priv, CNT1) << 8) & 0xff00;
	count |= (tnt_readb(tnt_priv, CNT2) << 16) & 0xff0000;
	count |= (tnt_readb(tnt_priv, CNT3) << 24) & 0xff000000;
	// return two's complement
	return -count;
};

static int write_wait( gpib_board_t *board, tnt4882_private_t *tnt_priv,
		int wait_for_done, int send_commands )
{
	nec7210_private_t *nec_priv = &tnt_priv->nec7210_priv;

	if( wait_event_interruptible( board->wait,
		( !wait_for_done && fifo_space_available( tnt_priv ) ) ||
		fifo_xfer_done( tnt_priv ) ||
		test_bit( BUS_ERROR_BN, &nec_priv->state ) ||
		test_bit( DEV_CLEAR_BN, &nec_priv->state ) ||
#if (GPIB_CONFIG_DEVICE==1)
		(!send_commands &&
 			(test_bit( ADSC_BN, &nec_priv->state ) ||
			 test_bit( LACS_NUM, &board->status )  ||
			 test_bit( APT_NUM, &board->status ))) ||
#endif
		test_bit( TIMO_NUM, &board->status ) ) )
	{
		GPIB_DPRINTK( "gpib write interrupted\n" );
		return -ERESTARTSYS;
	}
	if( test_bit( TIMO_NUM, &board->status ) )
	{
		printk("tnt4882: write timed out\n");
		return -ETIMEDOUT;
	}
	if( test_and_clear_bit( BUS_ERROR_BN, &nec_priv->state ) )
	{
		printk("tnt4882: write bus error\n");
		return (send_commands) ? -ENOTCONN : -ECOMM;
	}
	if( test_bit( DEV_CLEAR_BN, &nec_priv->state ) )
	{
		printk("tnt4882: device clear interrupted write\n" );
		return -EINTR;
	}
#if (GPIB_CONFIG_DEVICE==1)
	if (!send_commands) {
		if( test_bit( ADSC_BN, &nec_priv->state ) ){
			if ( !test_bit( TACS_NUM, &board->status ) )	{
				GPIB_DPRINTK("tnt4882: write interrupted (untalk)\n");
				return -EINTR;
			}
		}
		if( test_bit( LACS_NUM, &board->status ) )
		{
			GPIB_DPRINTK("tnt4882: write interrupted (device addressed as listener)\n");
			return -EINTR;
		}
		if( test_bit( APT_NUM, &board->status ) )
		{
			GPIB_DPRINTK("tnt4882: write interrupted (secondary received)\n");
			return -EINTR;
		}
	}
#endif
	return 0;
}

static int generic_write( gpib_board_t *board, uint8_t *buffer, size_t length,
	int send_eoi, int send_commands, size_t *bytes_written)
{
	size_t count = 0;
	ssize_t retval = 0;
	tnt4882_private_t *tnt_priv = board->private_data;
	nec7210_private_t *nec_priv = &tnt_priv->nec7210_priv;
	unsigned int bits, imr0_bits, imr1_bits, imr2_bits;
	int32_t hw_count;
	unsigned long flags;
 	unsigned int adr1_bits = 0;
#if (GPIB_CONFIG_DEVICE==1)

 	if (!send_commands) {
 		// we now should be already addressed as talker - make sure we are in TACS before trying to send data
		if ((read_byte(nec_priv, ADSR) & HR_TA) == 0) {
			// don't continue in case of change in addressing
			if (read_byte(nec_priv, ISR2) == HR_ADSC) {
				printk("tnt4882: address change detected (aborting transfer)\n");
				return -EINTR;
			}
			else {
				printk("tnt4882: write without being addressed as talker\n");
				return -EINTR;
			}
		}
 	}
#endif
 	read_byte(nec_priv, ISR2);	// clear status 2 for HPDRIVE

 	*bytes_written = 0;
	// FIXME: really, DEV_CLEAR_BN should happen elsewhere to prevent race
	smp_mb__before_atomic();
	clear_bit(DEV_CLEAR_BN, &nec_priv->state);
	smp_mb__after_atomic();

	imr1_bits = nec_priv->reg_bits[ IMR1 ];
	imr2_bits = nec_priv->reg_bits[ IMR2 ];

 	if (send_commands)
 		nec7210_set_reg_bits( nec_priv, IMR1, 0xff, HR_ERRIE | HR_DECIE );
 	else
 		nec7210_set_reg_bits( nec_priv, IMR1, 0xff, HR_ERRIE | HR_DECIE | HR_APTIE);
	/* HPDRIVE extension */
	/* if extended dual addressing with minor address 31 is being used (HP Identify hack),
	 * we need to temporarily disable minor talk address in order to see the untalk on
	 * the major address during writes */
	if (!send_commands) {
		adr1_bits = read_byte(nec_priv, ADR1);
		if ((adr1_bits & (HR_DT | HR_DL | ADDRESS_MASK)) == (HR_DL | 0x1f)) {
			write_byte( nec_priv, HR_ARS | HR_DT | HR_DL, ADR);
		}
	}
	if(( nec_priv->type != TNT4882 ) && ( nec_priv->type != TNT5004 ))
		nec7210_set_reg_bits( nec_priv, IMR2, 0xff, HR_DMAO | HR_ACIE );
	else
		nec7210_set_reg_bits( nec_priv, IMR2, 0xff, HR_ACIE);
	imr0_bits = tnt_priv->imr0_bits;
	if (send_commands)
		tnt_priv->imr0_bits &= ~TNT_ATNI_BIT;
	else
		tnt_priv->imr0_bits |= TNT_ATNI_BIT;
	tnt_writeb(tnt_priv, tnt_priv->imr0_bits, IMR0);

	tnt_writeb( tnt_priv, RESET_FIFO, CMDR );
	udelay(1);

	bits = TNT_TLCHE | TNT_B_16BIT;
	if( send_eoi )
	{
		bits |= TNT_CCEN;
		if((nec_priv->type != TNT4882 ) && (nec_priv->type != TNT5004 ))
			tnt_writeb( tnt_priv, AUX_SEOI, CCR );
	}
	if( send_commands )
		bits |= TNT_COMMAND;
	tnt_writeb( tnt_priv, bits, CFG );

	// load 2's complement of count into hardware counters
	hw_count = -length;
	tnt_writeb( tnt_priv, hw_count & 0xff, CNT0 );
	tnt_writeb( tnt_priv, ( hw_count >> 8 ) & 0xff, CNT1 );
	tnt_writeb( tnt_priv, ( hw_count >> 16 ) & 0xff, CNT2 );
	tnt_writeb( tnt_priv, ( hw_count >> 24 ) & 0xff, CNT3 );

	nec7210_set_handshake_mode(board, nec_priv, HR_HLDA);		// HPDRIVE hold-off on all
	write_byte(nec_priv, AUX_HLDI, AUXMR);				// HPDRIVE hold-off immediate
	tnt_writeb( tnt_priv, GO, CMDR );
	udelay(1);

	spin_lock_irqsave( &board->spinlock, flags );
	tnt_priv->imr3_bits |= HR_DONE;
	tnt_writeb( tnt_priv, tnt_priv->imr3_bits, IMR3 );
	spin_unlock_irqrestore( &board->spinlock, flags );
	if (!send_commands) {
		clear_bit( ADSC_BN, &nec_priv->state);	// HPDRIVE reset ADSC
	}

	while( count < length  )
	{
		// wait until byte is ready to be sent
		retval = write_wait( board, tnt_priv, 0, send_commands );
		if( retval < 0 ) break;
		if( fifo_xfer_done( tnt_priv ) ) break;

		spin_lock_irqsave( &board->spinlock, flags );
		while( fifo_space_available( tnt_priv ) && count < length )
		{
			uint16_t word;

			word = buffer[ count++ ] & 0xff;
			if( count < length )
				word |= ( buffer[ count++ ] << 8 ) & 0xff00;
			tnt_priv->io_writew( word, nec_priv->iobase + FIFOB );
		}
		tnt_priv->imr3_bits |= HR_NFF;
		tnt_writeb( tnt_priv, tnt_priv->imr3_bits, IMR3 );
		spin_unlock_irqrestore( &board->spinlock, flags );

		if(need_resched())
			schedule();
	}
	// wait last byte has been sent
        /* Start HPDRIVE extension */
	if (send_commands) {
		if(retval == 0)
			retval = write_wait(board, tnt_priv, 1, send_commands);
	}
	else {
		if ((adr1_bits & (HR_DT | HR_DL | ADDRESS_MASK)) == (HR_DL | 0x1f))
			/* re-enable secondary address for extended dual addressing with
			 * minor address 31 (HP Identify hack) */
			write_byte( nec_priv, HR_ARS | HR_DL | 0x1f, ADR);
		if ((retval == 0) && test_bit( TACS_NUM, &board->status ))
			retval = write_wait(board, tnt_priv, 1, send_commands);
	}
	/* End HPDRIVE extension */
	tnt_writeb( tnt_priv, STOP, CMDR );
	udelay(1);

	nec7210_set_reg_bits( nec_priv, IMR1, 0xff, imr1_bits );
	nec7210_set_reg_bits( nec_priv, IMR2, 0xff, imr2_bits );
	tnt_priv->imr0_bits = imr0_bits;
	tnt_writeb(tnt_priv, tnt_priv->imr0_bits, IMR0);
	/* force handling of any interrupts that happened
	 * while they were masked (this appears to be needed)*/
	tnt4882_internal_interrupt(board);
	*bytes_written = length - tnt_transfer_count(tnt_priv);
	return retval;
}

int tnt4882_accel_write(gpib_board_t *board, uint8_t *buffer, size_t length, int send_eoi, size_t *bytes_written)
{
	return generic_write( board, buffer, length, send_eoi, 0, bytes_written);
}

int tnt4882_command(gpib_board_t *board, uint8_t *buffer, size_t length, size_t *bytes_written)
{
	return generic_write( board, buffer, length, 0, 1, bytes_written);
}
