/***** Public Functions ******/
extern  int dvrsp(gpib_board_t *board, int padsad, uint8_t *result);
extern  int ibAPWait(gpib_board_t *board, int pad);
extern  int ibAPrsp(gpib_board_t *board, int padsad, char *spb);
extern  void ibAPE(gpib_board_t *board, int pad, int v);
extern  int ibcac(gpib_board_t *board, int sync);
extern  ssize_t ibcmd(gpib_board_t *board, uint8_t *buf, size_t length);
extern  int ibgts(gpib_board_t *board);
extern  int ibonl(gpib_board_t *board, int v);
extern  int iblines(gpib_board_t *board, int *buf);
extern  ssize_t ibrd(gpib_board_t *board, uint8_t *buf, size_t length, int *end_flag);
extern  int ibrpp(gpib_board_t *board, uint8_t *buf);
extern  int ibrsv(gpib_board_t *board, uint8_t poll_status);
extern  int ibsic(gpib_board_t *board);
extern  int ibsre(gpib_board_t *board, int enable);
extern  int ibpad(gpib_board_t *board, int v);
extern  int ibsad(gpib_board_t *board, int v);
extern  int ibtmo(gpib_board_t *board, unsigned int v);
extern  int ibeot(gpib_board_t *board, int send_eoi);
extern  int ibeos(gpib_board_t *board, int v);
extern  int ibwait(gpib_board_t *board, unsigned int mask);
extern  ssize_t ibwrt(gpib_board_t *board, uint8_t *buf, size_t cnt, int more);
extern unsigned int ibstatus(gpib_board_t *board);

