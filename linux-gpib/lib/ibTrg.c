
#include <ib.h>
#include <ibP.h>

int ibtrg(int ud)
{
return  ibBoardFunc(CONF(ud,board),DVTRG , CONF(ud,padsad));
}

