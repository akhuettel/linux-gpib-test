/***************************************************************************
                              nec7210/util.c
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
#include <linux/delay.h>

int admr_bits = HR_TRM0 | HR_TRM1;

void nec7210_enable_eos(uint8_t eos_byte, int compare_8_bits)
{
	DBGin("bdSetEOS");
	GPIBout(EOSR, eos_byte);
	auxa_bits |= HR_REOS;
	if(compare_8_bits)
		auxa_bits |= HR_BIN;
	else
		auxa_bits &= ~HR_BIN;
	GPIBout(AUXMR, auxa_bits);
	DBGout();
}

void nec7210_disable_eos(void)
{
	auxa_bits &= ~HR_REOS;
	GPIBout(AUXMR, auxa_bits);
}

int nec7210_parallel_poll(uint8_t *result)
{
	int ret;

	// enable command out interrupts
	imr2_bits |= HR_COIE;
	GPIBout(IMR2, imr2_bits);

	// execute parallel poll
	GPIBout(AUXMR, AUX_EPP);

	// wait for result
	ret = wait_event_interruptible(nec7210_command_wait, test_bit(0, &command_out_ready));

	// disable command out interrupts
	imr2_bits &= ~HR_COIE;
	GPIBout(IMR2, imr2_bits);

	if(ret)
	{
		printk("gpib: parallel poll interrupted\n");
		return -1;
	}

	*result = GPIBin(CPTR);

	return 0;
}

int nec7210_serial_poll_response(uint8_t status)
{
	GPIBout(SPMR, 0);		/* clear current serial poll status */
	GPIBout(SPMR, status);		/* set new status to v */

	return 0;
}

void nec7210_primary_address(unsigned int address)
{
	// put primary address in address0
	GPIBout(ADR, (address & ADDRESS_MASK));
}

void nec7210_secondary_address(unsigned int address, int enable)
{
	if(enable)
	{
		// put secondary address in address1
		GPIBout(ADR, HR_ARS | (address & ADDRESS_MASK));
		// go to address mode 2
		admr_bits &= ~HR_ADM0;
		admr_bits |= HR_ADM1;
		GPIBout(ADMR, admr_bits);
	}else
	{
		// disable address1 register
		GPIBout(ADR, HR_ARS | HR_DT | HR_DL);
		// go to address mode 1
		admr_bits |= HR_ADM0;
		admr_bits &= ~HR_ADM1;
		GPIBout(ADMR, admr_bits);
	}
}

unsigned int nec7210_update_status(void)
{
	int address_status_bits = GPIBin(ADSR);

	/* everything but ATN is updated by
	* interrupt handler */
	if(address_status_bits & HR_NATN)
		clear_bit(ATN_NUM, &board.status);
	else
		set_bit(ATN_NUM, &board.status);

	return board.status;
}


