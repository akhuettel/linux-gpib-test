
#include "ib_internal.h"
#include <ibP.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "parse.h"

ibConf_t *ibConfigs[NUM_CONFIGS];

int ibParseConfigFile(char *filename)
{
	extern FILE *gpib_yyin;
	FILE *infile;
	int stat = 0;

	if ((infile = fopen(filename, "r")) == NULL)
	{
		fprintf(stderr, "failed to open configuration file\n");
		iberr = ECFG;
		return -1;
	}

	/*ibPutMsg("Parse Config %s",filename); */
	gpib_yyin = infile;
	gpib_yyrestart(gpib_yyin); /* Hmm ? */

	// initialize line counter to 1 before calling yyparse
	yylloc.first_line = 1;
	ibBoardDefaultValues();
	// initialize variables used in yyparse() - see ibConfYacc.y
	findIndex = 0;
	if(gpib_yyparse() < 0)
	{
		fprintf(stderr, "failed to parse configuration file\n");
		stat = -1 ;
	}
	fclose(infile);

	return stat;
}

/**********************************************************************/

int ibGetDescriptor(ibConf_t p)
{
	ibConf_t *conf;
	int ib_ndev;

	/* check validity of values */
	if( p.pad >= IB_MAXDEV )
	{
		iberr = ETAB;
		return -1;
	}
	// search for an unused descriptor
	for(ib_ndev = 0; ib_ndev < NUM_CONFIGS; ib_ndev++)
	{
		if(ibConfigs[ib_ndev] == NULL)
		{
			ibConfigs[ib_ndev] = malloc(sizeof(ibConf_t));
			break;
		}
	}
	if( ib_ndev == NUM_CONFIGS)
	{
		iberr = ETAB;
		return -1;
	}
	conf = ibConfigs[ib_ndev];
	/* put entry to the table */
	strncpy(conf->name, p.name, sizeof(conf->name) );
	conf->board = p.board;
	conf->pad = p.pad;
	conf->sad = p.sad;
	conf->flags = p.flags;
	conf->eos = p.eos;
	conf->eos_flags = p.eos_flags;
	conf->usec_timeout = p.usec_timeout;
	conf->send_eoi = p.send_eoi;
	conf->is_interface = p.is_interface;
	conf->is_open = p.is_open;
	conf->has_lock = p.has_lock;

	strncpy(conf->init_string, p.init_string, sizeof(conf->init_string));
	return ib_ndev;
}

/**********************************************************************/
int ibFindDevIndex(char *name)
{
	int i;

	for(i = 0; i < FIND_CONFIGS_LENGTH; i++)
	{
		if(!strcmp(ibFindConfigs[i].name, name)) return i;
	}

	return -1;
}

int ibCheckDescriptor(int ud)
{
	if(ud < 0 || ud > NUM_CONFIGS || ibConfigs[ud] == NULL)
		return -1;
	return 0;
}

void init_ibconf( ibConf_t *conf )
{
	conf->name[0] = 0;
	conf->pad = -1;
	conf->sad = -1;
	conf->init_string[0] = 0;     
	conf->board = -1;           
	conf->eos = 0;             
	conf->eos_flags = 0;
	conf->flags = 0;                
	conf->usec_timeout = 3000000;
	conf->send_eoi = 1;
	conf->is_interface = 0;
	conf->is_open = 0;
	conf->has_lock = 0;
}

int ib_lock_mutex( ibBoard_t *board )
{
	int lock = 1;
	return ioctl( board->fileno, IBMUTEX, &lock );
}

int ib_unlock_mutex( ibBoard_t *board )
{
	int unlock = 0;
	return ioctl( board->fileno, IBMUTEX, &unlock );
}

int open_gpib_device( ibBoard_t *board, ibConf_t *conf )
{
	open_close_dev_ioctl_t open_cmd;
	int retval;

	if( conf->is_open ) return 0;

	if( conf->is_interface == 0 )
	{
		open_cmd.pad = conf->pad;
		open_cmd.sad = conf->sad;
		retval = ioctl( board->fileno, IBOPENDEV, &open_cmd );
		if( retval < 0 ) return retval;
	}

	conf->is_open = 1;

	return 0;
}

int close_gpib_device( ibBoard_t *board, ibConf_t *conf )
{
	open_close_dev_ioctl_t close_cmd;
	int retval;

	if( conf->is_open == 0 ) return 0;

	if( conf->is_interface == 0 )
	{
		close_cmd.pad = conf->pad;
		close_cmd.sad = conf->sad;
		retval = ioctl( board->fileno, IBCLOSEDEV, &close_cmd );
		if( retval < 0 ) return retval;
	}

	conf->is_open = 0;

	return 0;
}

int gpibi_change_address( ibBoard_t *board, ibConf_t *conf, unsigned int pad, int sad )
{
	int retval;

	if ( conf->is_interface )
	{
		if( pad != conf->pad )
		{
			retval = ioctl( board->fileno, IBPAD, &pad );
			if( retval < 0 ) return retval;
		}

		if( sad != conf->sad )
		{
			retval = ioctl( board->fileno, IBSAD, &sad );
			if( retval < 0 ) return retval;
		}
	}

	retval = close_gpib_device( board, conf );
	if( retval < 0 ) return retval;

	conf->pad = pad;
	conf->sad = sad;

	retval = open_gpib_device( board, conf );
	if( retval < 0 ) return retval;

	return 0;
}

int lock_board_mutex( ibBoard_t *board )
{
	static const int lock = 1;
	int retval;

	retval = ioctl( board->fileno, IBMUTEX, &lock );
	if( retval < 0 )
		fprintf( stderr, "libgpib: error locking board mutex!\n");
	return retval;
}

int unlock_board_mutex( ibBoard_t *board )
{
	static const int unlock = 0;
	int retval;

	retval = ioctl( board->fileno, IBMUTEX, &unlock );
	if( retval < 0 )
		fprintf( stderr, "libgpib: error unlocking board mutex!\n");
	return retval;
}

ibConf_t * enter_library( int ud, int lock_library )
{
	ibConf_t *conf = ibConfigs[ ud ];
	ibBoard_t *board;
	int retval;

	if( ibCheckDescriptor( ud ) < 0 ) return NULL;

	board = &ibBoard[ conf->board ];

	if( lock_library )
	{
		retval = lock_board_mutex( board );
		if( retval < 0 )
		{
			return NULL;
		}
		conf->has_lock = 1;
	}

	return conf;
}

//XXX
int ibstatus( ibConf_t *conf, int error )
{
	int status = 0;

	status |= CMPL;
	if( error ) status |= ERR;

	ibsta = status;

	return status;
}

int exit_library( int ud, int error )
{
	ibConf_t *conf = ibConfigs[ ud ];
	ibBoard_t *board;
	int retval;

	if( ibCheckDescriptor( ud ) < 0 ) return ERR;

	board = &ibBoard[ conf->board ];

	if( conf->has_lock )
	{
		conf->has_lock = 0;
		retval = unlock_board_mutex( board );
		if( retval < 0 )
		{
			error++;
		}
	}
	
	return ibstatus( conf, error );
}
