/***************************************************************************
                               sys/osfuncs.c
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

#include <ibsys.h>

#include <linux/fcntl.h>

#define GIVE_UP(a) {up(&device->mutex); return a;}

int ib_opened=0;
int ib_exclusive=0;

IBLCL int ibopen(struct inode *inode, struct file *filep)
{
	if( ib_exclusive )
	{
		return (-EBUSY);
	}


	if ( filep->f_flags & O_EXCL )
	{
		if (ib_opened)
		{
			return (-EBUSY);
		}
		ib_exclusive=1;
	}

// this is a temporary hack to allocate the gpib0 device
if(device_array[0] == NULL)
{
	device_array[0] = kmalloc(sizeof(gpib_device_t), GFP_KERNEL);

#ifdef NIPCIIa
	device_array[0]->interface = &pc2a_interface;
#warning using pc2a driver
#endif

#ifdef CBI_PCI
	device_array[0]->interface = &cb_pci_interface;
#warning using cb_pci driver
#endif

#ifdef CBI_PCMCIA
	device_array[0]->interface = &cb_pcmcia_interface;
#warning using cb_pcmcia driver
#endif

#if !defined(NIPCIIa) && !defined(CBI_4882)
	device_array[0]->interface = &pc2_interface;
#warning using pc2 driver
#endif

	init_waitqueue_head(&device_array[0]->wait);
}

	ib_opened++;

	return 0;
}


IBLCL int ibclose(struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	gpib_device_t *device = device_array[minor - 1];

	if ((pgmstat & PS_ONLINE) && ib_opened == 1 )
		ibonl(device, 0);
	ib_opened--;

	if( ib_exclusive )
		ib_exclusive = 0;

	// temporary hack
	if(ib_opened == 0)
	{
		kfree(device_array[0]);
		device_array[0] = NULL;
	}

	return 0;
}

IBLCL int ibioctl(struct inode *inode, struct file *filep, unsigned int cmd,
 unsigned long arg)
{
	int	retval = 0; 		/* assume everything OK for now */
	ibarg_t m_ibarg,*ibargp;
	int	bufsize;
	int	remain;
	char 	*buf;
	char 	*userbuf;
	char 	c;
	ssize_t ret;
	int end_flag = 0;
	unsigned int minor = MINOR(inode->i_rdev);
	gpib_device_t *device;

	if(minor > MAX_NUM_GPIB_DEVICES)
	{
		printk("gpib: invalid minor number of device file\n");
		return -ENODEV;
	}
	device = device_array[minor - 1];
	if(device == NULL)
	{
		printk("gpib: no device configured at minor number %i\n", minor);
		return -ENODEV;
	}

printk("ioclt %i\n", cmd);

	ibargp = (ibarg_t *) &m_ibarg;

	/* Check the arg buffer is readable & writable by the current process */
	retval = verify_area(VERIFY_WRITE, (void *)arg, sizeof(ibarg_t));
	if (retval)
	{
		return (retval);
	}

	retval = verify_area(VERIFY_READ, (void *)arg, sizeof(ibarg_t));
	if (retval)
	{
		return (retval);
	}

	copy_from_user( ibargp , (ibarg_t *) arg , sizeof(ibarg_t));

	ibargp->ib_iberr = EDVR;

// XXX
#if 0
	if( cmd == IBAPWAIT )
	{
		/* special case for IBAPWAIT : does his own locking */
		ibAPWait(ibargp->ib_arg);
		ibargp->ib_ibsta = ibsta;
		ibargp->ib_iberr = iberr;
		ibargp->ib_ibcnt = ibcnt;
		copy_to_user((ibarg_t *) arg, (ibarg_t *) ibargp , sizeof(ibarg_t));

		return retval;
	}
#endif

	/* lock other processes from performing commands */
	retval = down_interruptible(&device->mutex);
	if(retval)
	{
		printk("gpib: ioctl interrupted while waiting on lock\n");
		return -ERESTARTSYS;
	}

//XXX a lot of the content of this switch should be split out into seperate functions
	switch (cmd)
	{
		case IBRD:	// XXX read should not be an ioctl
			/* Check write access to buffer */
			retval = verify_area(VERIFY_WRITE, ibargp->ib_buf, ibargp->ib_cnt);
			if (retval)
				GIVE_UP (retval);

			/* Get a DMA buffer */
			bufsize = ibargp->ib_cnt;
			if ((buf = osGetDMABuffer( &bufsize )) == NULL)
			{
				GIVE_UP( -ENOMEM ) ;
			}
			/* Read DMA buffer loads till we fill the user supplied buffer */
			userbuf = ibargp->ib_buf;
			remain = ibargp->ib_cnt;
			do
			{
				ret = ibrd(device, buf, (bufsize < remain) ? bufsize : remain, &end_flag);
				if(ret < 0)
				{
					retval = -EIO;
					break;
				}
				copy_to_user( userbuf, buf, ret );
				remain -= ret;
				userbuf += ret;
			}while (remain > 0 && end_flag == 0);
			ibargp->ib_ibcnt = ibargp->ib_cnt - remain;
			/* Free the DMA buffer */
			osFreeDMABuffer( buf );
			break;
		case IBWRT:	// XXX write should not be an ioclt

			/* Check read access to buffer */
			retval = verify_area(VERIFY_READ, ibargp->ib_buf, ibargp->ib_cnt);
			if (retval)
				GIVE_UP(retval);
			/* Get a DMA buffer */
			bufsize = ibargp->ib_cnt;
			if ((buf = osGetDMABuffer( &bufsize )) == NULL)
			{
				GIVE_UP(-ENOMEM);
			}
			/* Write DMA buffer loads till we empty the user supplied buffer */
			userbuf = ibargp->ib_buf;
			remain = ibargp->ib_cnt;
			do
			{
				copy_from_user( buf, userbuf, (bufsize < remain) ? bufsize : remain );
				ret = ibwrt(device, buf, (bufsize < remain) ? bufsize : remain, (bufsize < remain)
 );
				if(ret < 0)
				{
					retval = -EIO;
					break;
				}
				remain -= ret;
				userbuf += ret;
			}while (remain > 0);
			ibargp->ib_ibcnt = ibargp->ib_cnt - remain;
			/* Free the DMA buffer */
			osFreeDMABuffer( buf );
			break;
		case IBCMD:
			/* Check read access to buffer */
			retval = verify_area(VERIFY_READ, ibargp->ib_buf, ibargp->ib_cnt);
			if (retval)
				GIVE_UP(retval);

			/* Get a DMA buffer */
			bufsize = ibargp->ib_cnt;
			if ((buf = osGetDMABuffer( &bufsize )) == NULL) {
				GIVE_UP(-ENOMEM);
			}

			/* Write DMA buffer loads till we empty the user supplied buffer */
			userbuf = ibargp->ib_buf;
			remain = ibargp->ib_cnt;
			while (remain > 0 && !(ibstatus(device) & (TIMO)))
			{
				copy_from_user( buf, userbuf, (bufsize < remain) ? bufsize : remain );
				ret = ibcmd(device, buf, (bufsize < remain) ? bufsize : remain );
				if(ret < 0)
				{
					retval = -EIO;
					break;
				}
				remain -= ret;
				userbuf += ret;
			}
			ibargp->ib_ibcnt = ibargp->ib_cnt - remain;

			/* Free the DMA buffer */
			osFreeDMABuffer( buf );

			break;

		case IBWAIT:
			DBGprint(DBG_DATA,("**arg=%x",ibargp->ib_arg));
			retval = ibwait(device, ibargp->ib_arg);
			break;
		case IBRPP:
			/* Check write access to Poll byte */
			retval = verify_area(VERIFY_WRITE, ibargp->ib_buf, 1);
			if (retval)
				GIVE_UP(retval);

			retval = ibrpp(device, &c);
			put_user( c, ibargp->ib_buf );
			break;
		case IBONL:
			retval = ibonl(device, ibargp->ib_arg);
			break;
		case IBAPE:
			ibAPE(device, ibargp->ib_arg,ibargp->ib_cnt);
			break;
		case IBSIC:
			retval = ibsic(device);
			break;
		case IBSRE:
			retval = ibsre(device, ibargp->ib_arg);
			break;
		case IBGTS:
			retval = ibgts(device);
			break;
		case IBCAC:
			retval = ibcac(device, ibargp->ib_arg);
			break;
		case IBSDBG:
			break;
		case IBLINES:
			retval = iblines(device, &ibargp->ib_ret);
			break;
		case IBPAD:
			retval = ibpad(device, ibargp->ib_arg);
			break;
		case IBSAD:
			retval = ibsad(device, ibargp->ib_arg);
			break;
		case IBTMO:
			retval = ibtmo(device, ibargp->ib_arg);
			break;
		case IBEOT:
			retval = ibeot(device, ibargp->ib_arg);
			break;
		case IBEOS:
			retval = ibeos(device, ibargp->ib_arg);
			break;
		case IBRSV:
			retval = ibrsv(device, ibargp->ib_arg);
			break;
		case DVTRG:
			retval = dvtrg(device, ibargp->ib_arg);
			break;
		case DVCLR:
			retval = dvclr(device, ibargp->ib_arg);
			break;
		case DVRSP:
			/* Check write access to Poll byte */
			retval = verify_area(VERIFY_WRITE, ibargp->ib_buf, 1);
			if (retval)
			{
				GIVE_UP(retval);
			}
			retval = dvrsp(device, ibargp->ib_arg, &c);

			put_user( c, ibargp->ib_buf );

			break;
		case IBAPRSP:
			retval = verify_area(VERIFY_WRITE, ibargp->ib_buf, 1);
			if (retval)
			{
				GIVE_UP(retval);
			}
			retval = ibAPrsp(device, ibargp->ib_arg, &c);
			put_user( c, ibargp->ib_buf );
			break;
		case DVRD:	// XXX unnecessary, should be in user space lib
			/* Check write access to buffer */
			retval = verify_area(VERIFY_WRITE, ibargp->ib_buf, ibargp->ib_cnt);
			if (retval)
				GIVE_UP(retval);

			/* Get a DMA buffer */
			bufsize = ibargp->ib_cnt;
			if ((buf = osGetDMABuffer( &bufsize )) == NULL)
			{
				GIVE_UP(-ENOMEM);
			}
			if(receive_setup(device, ibargp->ib_arg))
			{
				retval = -EIO;
				break;
			}
			/* Read DMA buffer loads till we fill the user supplied buffer */
			userbuf = ibargp->ib_buf;
			remain = ibargp->ib_cnt;
			do
			{
				ret = ibrd(device, buf, (bufsize < remain) ? bufsize : remain, &end_flag);
				if(ret < 0)
				{
					retval = -EIO;
					break;
				}
				copy_to_user( userbuf, buf, ret );
				remain -= ret;
				userbuf += ret;
			}while (remain > 0  && end_flag == 0);	//!(ibstatus() & TIMO));
			ibargp->ib_ibcnt = ibargp->ib_cnt - remain;
			/* Free the DMA buffer */
			osFreeDMABuffer( buf );
			break;
		case DVWRT:	// XXX unnecessary, should be in user space lib

			/* Check read access to buffer */
			retval = verify_area(VERIFY_READ, ibargp->ib_buf, ibargp->ib_cnt);
			if (retval)
				GIVE_UP(retval);

			/* Get a DMA buffer */
			bufsize = ibargp->ib_cnt;
			if ((buf = osGetDMABuffer( &bufsize )) == NULL) {
				GIVE_UP(-ENOMEM);
			}

			/* Write DMA buffer loads till we empty the user supplied buffer */
			userbuf = ibargp->ib_buf;
			remain = ibargp->ib_cnt;
			while (remain > 0  && !(ibstatus(device) & (TIMO)))
			{
				copy_from_user( buf, userbuf, (bufsize < remain) ? bufsize : remain );
				ret = dvwrt(device, ibargp->ib_arg, buf, (bufsize < remain) ? bufsize : remain );
				if(ret < 0)
				{
					retval = -EIO;
					break;
				}
				remain -= ret;
				userbuf += ret;
			}
			ibargp->ib_ibcnt = ibargp->ib_cnt - remain;

			/* Free the DMA buffer */
			osFreeDMABuffer( buf );

			break;

		/* special configuration options */
		case CFCBASE:
			osChngBase(device, ibargp->ib_arg);
			break;
		case CFCIRQ:
			osChngIRQ(device, ibargp->ib_arg);
			break;
		case CFCDMA:
			osChngDMA(device, ibargp->ib_arg);
			break;
		case CFCDMABUFFER:
			if (ibargp->ib_arg > MAX_DMA_SIZE)
			{
				GIVE_UP(-EINVAL);
			}
			if ( ibargp->ib_arg > gpib_dma_size )
			{
				gpib_dma_size = ibargp->ib_arg;
				osMemInit();
				printk("-- DMA Buffer now %d Bytes\n",gpib_dma_size);
			}else
			{
				printk("-- DMA Buffer Not Changed \n");
			}
			break;

		default:
			retval = -ENOTTY;
			break;
	}

	// return status bits
	ibargp->ib_ibsta = ibstatus(device);
	if(retval)
		ibargp->ib_ibsta |= ERR;
	else
		ibargp->ib_ibsta &= ~ERR;
	if(end_flag)
		ibargp->ib_ibsta |= END;
	else
		ibargp->ib_ibsta &= ~END;
	// XXX io is always complete since we don't support asynchronous transfers yet
	ibargp->ib_ibsta |= CMPL;

	copy_to_user((ibarg_t *) arg, (ibarg_t *) ibargp , sizeof(ibarg_t));

	GIVE_UP(retval);
}






































