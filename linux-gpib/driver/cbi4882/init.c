

#include <board.h>


ibbase = IBBASE;	/* base addr of GPIB interface registers  */
ibirq  = IBIRQ;	/* interrupt request line for GPIB (1-7)  */
ibdma  = IBDMA;     /* DMA channel                            */

uint8       board_type = CBI_ISA_GPIB;
uint8       CurHSMode = 0;      /* hs mode register value */
uint8       CurIRQreg = 0;      /* hs IRQ register value */

/*
 * BDONL
 * Initialize the interface hardware.
 */
IBLCL int bdonl(int v)
{
	uint8_t		s;

	DBGin("bdonl");

	/* CBI 4882 reset */
	GPIBout(HS_INT_LEVEL, HS_RESET7210 );
	GPIBout(HS_INT_LEVEL, 0 );

	GPIBout(HS_MODE, HS_TX_ENABLE | HS_RX_ENABLE ); /* reset state machine */
	GPIBout(HS_MODE, 0); /* disable system control */
	CurHSMode |= HS_SYS_CONTROL;
	GPIBout(HS_MODE, CurHSMode ); /* enable syscntrl */

	/* now its time to check the board type */

	GPIBout( SPMR, 0xaa );
             /* check if serial poll registers are readable & writeable */
        if( GPIBin( SPSR ) != 0xaa ) {
	  printk("GPIB Board is not a CBI488.2! \n");
	  return(0);
	}

        GPIBout( AUXMR, AUX_PAGE + 1 ); /* select paged in register */
        if( GPIBin( SPSR ) != 0xaa ) {  /* does SPMR still return 0xaa ? */
	  DBGprint(DBG_BRANCH, ("    CBI488.2 old type ") );
	}

        if( GPIBin(HS_MODE) == 0xff ) {
	  DBGprint(DBG_BRANCH, (", LC variant\n") );
          board_type = CBI_ISA_GPIB_LC;
	}
        

        GPIBout(AUXMR, AUX_PON);
	GPIBout(AUXMR, AUX_CR);                     /* 7210 chip reset */
	GPIBout(AUXMR, AUX_CIFC);


	s = GPIBin(CPTR);                           /* clear registers by reading */

	s = GPIBin(ISR1);
	s = GPIBin(ISR2);
	GPIBout(IMR1, 0);                           /* disable all interrupts */
	GPIBout(IMR2, 0);

	GPIBout(SPMR, 0);

	GPIBout(ADR,(PAD & LOMASK));                /* set GPIB address; MTA=PAD|100, MLA=PAD|040 */
#if (SAD)
	GPIBout(ADR, HR_ARS | (SAD & LOMASK));      /* enable secondary addressing */
	GPIBout(ADMR, HR_TRM1 | HR_TRM0 | HR_ADM1);
#else
	GPIBout(ADR, HR_ARS | HR_DT | HR_DL);       /* disable secondary addressing */
	GPIBout(ADMR, HR_TRM1 | HR_TRM0 | HR_ADM0);
#endif

	GPIBout(EOSR, 0);
	GPIBout(AUXMR, ICR | 20);                    /* set internal counter register N= 8 */
	GPIBout(AUXMR, PPR | HR_PPU);               /* parallel poll unconfigure */
	GPIBout(AUXMR, auxrabits);


#if !defined( CBI_PCI )
	GPIBout(AUXMR, AUXRB | 0);                  /* set INT pin to active high */
#else
        GPIBout(AUXMR, AUXRB | HR_INV );           /* On PCI boards set INT pin to active low */
#endif


#if 0
	GPIBout(AUXMR, AUXRB | HR_TRI);
#endif
	GPIBout(AUXMR, AUXRE | 0);
	GPIBout(AUXMR, AUX_LOSPEED ); 
                        /* disable hispeed mode (long T1delay)*/


#if 0
	GPIBout(timer, 0xC4);                       /* 0xC4 = 7.5 usec (60 * 0.125) */
#endif
        if(v) GPIBout(AUXMR, AUX_PON);

	


	DBGout();
	return(1);
}


/*is called by ibsic */

IBLCL void fix4882Bug(void )
{
#if !defined(CBI_PCI)
  int i;
  extern int myPAD;
  char cmdString[8];

   if(board_type == CBI_ISA_GPIB_LC || board_type == CBI_ISA_GPIB ){
   DBGin("fix4882Bug");

        i=0;
#if 0
  	cmdString[i]   = UNL;
        cmdString[i++] = UNT;
#endif
  	cmdString[i] = myPAD | TAD;
        cmdString[i++] = myPAD | LAD;
	if( ibcmd(cmdString, i) & ERR ) {
	  printk("problem fixing bug");
	}
	
	GPIBout(CDOR, 0x20 ); /*send a byte */
	i= GPIBin(DIR);  /*read back*/
	i= GPIBin(ISR1); /*throw away any errors */

        i=0;
  	cmdString[i]   = UNL;
        cmdString[i++] = UNT;
	if( ibcmd(cmdString, i) & ERR ) {
	  printk("problem fixing bug");
	}

	printk("setting IRQ to %d \n",ibirq);
	setup4882int(ibirq);

	DBGout();
   }
#endif
}

IBLCL void setup4882int( int level ) 
{

  DBGin("setup4882Int");

#if !defined(CBI_PCI)
  switch( level ) {
  case 2:
    CurIRQreg |= 1;
    break;
  case 3:
    CurIRQreg |= 2;
    break;
  case 4:
    CurIRQreg |= 3;
    break;
  case 5:
    CurIRQreg |= 4;
    break;
  case 7:
    CurIRQreg |= 5;
    break;
  case 10:
    CurIRQreg |= 6;
    break;
  case 11:
    CurIRQreg |= 7;
    break;
  default:
    printk("IRQ level %d not supported with this board\n",level);
    break;    
  }
  GPIBout(HS_INT_LEVEL, CurIRQreg );
  GPIBout(HS_MODE,      CurHSMode );
#endif
  DBGout();
}

