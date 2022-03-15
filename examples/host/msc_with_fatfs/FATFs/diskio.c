/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/
#include "ffconf.h"
#include "diskio.h"
#include "hardware/structs/usb.h"
#include "hardware/address_mapped.h"

extern uint64_t last_interrupt_requested_at;

static DSTATUS disk_state[CFG_TUH_DEVICE_MAX];
	
void diskio_init(void)
{
	printf("I");
	memset(disk_state, STA_NOINIT, CFG_TUH_DEVICE_MAX);
}

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */

DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{
	printf("Dinit");
	disk_state[drv] &= (~STA_NOINIT); // clear NOINIT bit
	return disk_state[drv];
}

void disk_deinitialize ( BYTE drv )
{
	disk_state[drv] |= STA_NOINIT; // set NOINIT bit
}

/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{
	printf("S[%x]", disk_state[drv]);
	return disk_state[drv];
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

volatile bool read_flag;
volatile bool write_flag;

bool tuh_msc_read10_cb(uint8_t dev_addr, const msc_cbw_t *cbw, const msc_csw_t *csw) {
	printf("#");
	read_flag = true;
}
bool tuh_msc_write10_cb(uint8_t dev_addr, const msc_cbw_t *cbw, const msc_csw_t *csw) {
 	printf("#");
	 write_flag = true;
}

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{
	printf("R[%x]", sector);
	//printf(" - - - - - - - - - - - - - - - - - sector %x\n", sector);
	uint8_t usb_addr = drv+1;
	read_flag = false;
	if ( !tuh_msc_read10(usb_addr, 0, buff, sector, count, tuh_msc_read10_cb)) {
		printf(":(");
		return RES_ERROR;
	}
	printf("-");
	uint64_t time_limit = time_us_64() + TIMEOUT_WAIT_US;
	while (!read_flag) { 
		tuh_task(); 
		if (time_us_64() > time_limit) {
			printf("!"); //__breakpoint();
			return RES_ERROR;
		}
	}
	printf("r\n");
	return RES_OK; // wait_for_io_complete(usb_addr);
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{
	uint8_t usb_addr = drv+1;
	write_flag = false;
	printf("W[%x]", sector);
	if ( !tuh_msc_write10(usb_addr, 0, buff, sector, count, tuh_msc_write10_cb) ) {
		printf(":(");
		return RES_ERROR;
	}		
	printf("-");
	uint64_t time_limit = time_us_64() + TIMEOUT_WAIT_US;
	while (!write_flag) { 
		if (time_us_64() > time_limit) {
			printf("!"); //__breakpoint();
			return RES_ERROR;
		}
		tuh_task(); 
	}
	printf("w\n");
	return RES_OK;
}
#endif /* _READONLY */

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	printf("C[%x]", ctrl);
	if (ctrl == CTRL_SYNC)
		return RES_OK;
	else
		return RES_PARERR;
}

DWORD get_fattime (void)
{
	printf("M");
	return ((DWORD)(20 + 1) << 25) |
	              ((DWORD)1 << 21) |
	              ((DWORD)1 << 16) |
	              ((DWORD)1 << 11) |
	              ((DWORD)1 << 5)  |
	       (((DWORD)1 >> 1) << 0);
}
