/***************************************************************************
                          lib/ibSad.c
                             -------------------

    copyright            : (C) 2001,2002,2003 by Frank Mori Hess
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

#include "ib_internal.h"

int internal_ibsad( ibConf_t *conf, int address )
{
	ibBoard_t *board;
	sad_ioctl_t sad_cmd;
	int sad = address - sad_offset;
	int retval;

	board = interfaceBoard( conf );

	if( sad > 30 )
	{
		setIberr( EARG );
		return -1;
	}

	sad_cmd.handle = conf->handle;
	sad_cmd.sad = sad;
	retval = ioctl( board->fileno, IBSAD, &sad_cmd );
	if( retval < 0 )
	{
		fprintf( stderr, "libgpib: failed to change gpib secondary address\n" );
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

	conf->settings.sad = sad;

	return 0;
}

int ibsad( int ud, int v )
{
	ibConf_t *conf;
	int retval;

	conf = enter_library( ud );
	if( conf == NULL )
		return exit_library( ud, 1 );

	retval = internal_ibsad( conf, v );
	if( retval < 0 )
	{
		return exit_library( ud, 1 );
	}

	return exit_library( ud, 0 );
}

void ReleaseDacHoldoff( int boardID, int do_accept )
{
	ibConf_t *conf;
	ibBoard_t *board;
	int retval;

	conf = enter_library( boardID );
	if( conf == NULL )
	{
		exit_library( boardID, 1 );
		return;
	}
	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		exit_library( boardID, 1 );
		return;
	}
	board = interfaceBoard( conf );

	retval = release_dac_holdoff( board, do_accept );
	if( retval < 0 )
	{
		if( errno == ETIMEDOUT )
			conf->timed_out = 1;
		exit_library( boardID, 1 );
		return;
	}

	exit_library( boardID, 0 );
}

void SetAddressMode( int boardID, int address_mode, int sad )
{
	ibConf_t *conf;
	ibBoard_t *board;
	int retval;

	if( address_mode > 3 )
	{
		setIberr( EARG );
		return ;
	}

	conf = enter_library( boardID );
	if( conf == NULL )
	{
		exit_library( boardID, 1 );
		return;
	}
	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		exit_library( boardID, 1 );
		return;
	}
	board = interfaceBoard( conf );

	retval = set_address_mode( board, address_mode, sad );
	if( retval < 0 )
	{
		if( errno == ETIMEDOUT )
			conf->timed_out = 1;
		exit_library( boardID, 1 );
		return;
	}

	exit_library( boardID, 0 );
}

void GetAddressState( int boardID, unsigned int *secondary, int *is_minor )
{
	ibConf_t *conf;
	ibBoard_t *board;
	int retval;

	conf = enter_library( boardID );
	if( conf == NULL )
	{
		exit_library( boardID, 1 );
		return;
	}
	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		exit_library( boardID, 1 );
		return;
	}
	board = interfaceBoard( conf );

	retval = get_address_state( board, secondary, is_minor );
	if( retval < 0 )
	{
		if( errno == ETIMEDOUT )
			conf->timed_out = 1;
		exit_library( boardID, 1 );
		return;
	}

	exit_library( boardID, 0 );
}

int release_dac_holdoff( ibBoard_t *board, int do_accept )
{
	int retval;

	retval = ioctl( board->fileno, IBRELEASE_DAC_HOLDOFF, &do_accept );
	if( retval < 0 )
	{
		fprintf( stderr, "libgpib: IBRELEASE_DAC_HOLDOFF ioctl failed\n" );
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}
	return 0;
}

int set_address_mode( ibBoard_t *board, int address_mode, int sad )
{
	int retval;
	set_address_mode_ioctl_t cmd;

	cmd.address_mode = address_mode;
	cmd.sad = sad;
	retval = ioctl( board->fileno, IBSET_ADDRESS_MODE, &cmd );
	if( retval < 0 )
	{
		fprintf( stderr, "libgpib: IBSET_ADDRESS_MODE ioctl failed\n" );
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}
	return 0;
}

int get_address_state( ibBoard_t *board, unsigned int *secondary, int *is_minor )
{
	get_address_state_ioctl_t cmd;
	int retval;

	retval = ioctl( board->fileno, IBGET_ADDRESS_STATE, &cmd );
	if( retval < 0 )
	{
		fprintf( stderr, "libgpib: IBGET_ADDRESS_STATE ioctl failed (retval=%d)\n", retval);
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

	*secondary = cmd.secondary;
	*is_minor = cmd.is_minor;

	return 0;
}

