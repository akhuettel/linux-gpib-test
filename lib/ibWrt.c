
#include "ib_internal.h"
#include <ibP.h>

int ibwrt(int ud, void *rd, unsigned long cnt)
{
  iblcleos(ud);
  ibtmo(ud, CONF(ud,tmo));

  return  ibBoardFunc(  CONF(ud,board),
		        (CONF(ud,flags) & CN_ISCNTL ? IBWRT : DVWRT ),
                        CONF(ud,padsad), 
                        rd, cnt);
}




