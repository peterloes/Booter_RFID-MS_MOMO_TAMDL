/***************************************************************************//**
 * @file
 * @brief	Flash
 * @author	Ralf Gerhauser
 * @version	2018-02-19
 *
 * This module contains all routines to program a firmware image into FLASH.
 *
 ****************************************************************************//*
Revision History:
2018-02-19,rage	Use INT_En/Disable() instead of __en/disable_irq().
2016-02-24,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "em_device.h"
#include "em_assert.h"
#include "em_int.h"
#include "Flash.h"
#include "Logging.h"
#include "ff.h"		// FS_FAT12/16/32
#include "diskio.h"	// DSTATUS
#include "microsd.h"

/*================================ Local Data ================================*/

    /* Buffer to hold one FLASH sector to program */
static uint8_t	l_SectorBuf[FLASH_PAGE_SIZE];

    /* File handle for log file */
static FIL	l_fh;


/***************************************************************************//**
 *
 * @brief	Find Application Project Info
 *
 * This routine searches the project information of the application stored
 * in FLASH.
 *
 * @return
 *	Address of the application's @ref PRJ_INFO, or NULL if not found.
 *
 ******************************************************************************/
PRJ_INFO *FindPrjInfo(void)
{
char *pFlash;

    /*
     * IAR libraries do not provide function memmem() like GNU does, so we
     * have to implement our own version here.
     */
    for (pFlash = (char *)FIRMWARE_ADDRESS;
	 pFlash < (char *)FIRMWARE_ADDRESS + FLASH_SIZE - 16;  pFlash++)
    {
	if (strcmp (pFlash, "$PRJ_INFO$") == 0)
	    return (PRJ_INFO *)pFlash;
    }

    return NULL;
}


/***************************************************************************//**
 *
 * @brief	Update Application Firmware in FLASH
 *
 * This routine reads the file specified by @p filename and stores its firmware
 * image into FLASH.
 *
 * @param[in] filename
 *	Name of the firmware update file to load.
 *
 * @return
 * 	Error code: 0 if no error occurred, 4 in case of a file read error,
 * 	5 for a FLASH erase error, and 6 for a FLASH write error.
 *
 ******************************************************************************/
int	 FirmwareUpdate(char *filename)
{
FRESULT		 res;		// FatFs function common result code
msc_Return_TypeDef ret;		// FLASH library return code
int		 status = 0;	// return status is OK for default
uint32_t	 fileSize;	// number of bytes to read
uint32_t	 offs;		// byte offset within file and FLASH
uint16_t	 bytesRead;	// number of bytes really read from the file

    /* Check filename */
    EFM_ASSERT(filename != NULL);

    /* Open update file */
    res = f_open (&l_fh, filename,  FA_READ);
    if (res != FR_OK)
    {
	LogError ("FirmwareUpdate: Open File - Error Code %d", res);
	l_fh.fs = NULL;		// invalidate file handle
	return 4;
    }

    /* Enable the flash controller for writing */
    MSC_Init();

    /*
     * Read update file - the FLASH sector size determines the portion that
     * is read from file, and erased and programmed in FLASH.
     */
    fileSize = f_size(&l_fh);

    for (offs = 0;  offs < fileSize;  offs += FLASH_PAGE_SIZE)
    {
	/* Read one sector from the update file */
	res = f_read (&l_fh, l_SectorBuf, FLASH_PAGE_SIZE, &bytesRead);
	if (res != FR_OK)
	{
	    LogError ("FirmwareUpdate: Read File - Error Code %d", res);
	    l_fh.fs = NULL;		// invalidate file handle
	    status = 4;
	    break;
	}

	/* Erase the respective sector in FLASH */
	INT_Disable();			// disable interrupts
	ret = MSC_ErasePage ((uint32_t *)(FIRMWARE_ADDRESS + offs));
	INT_Enable();			// enable interrupts

	if (ret != mscReturnOk)
	{
	    LogError ("FirmwareUpdate: Erase Sector @ 0x%X - Error Code %d",
		      FIRMWARE_ADDRESS + offs, ret);
	    l_fh.fs = NULL;		// invalidate file handle
	    status = 5;
	    break;
	}

	POWER_LED = 1;			// LED ON

	/* Write data to FLASH */
	INT_Disable();			// disable interrupts
	ret = MSC_WriteWord ((uint32_t *)(FIRMWARE_ADDRESS + offs),
			     l_SectorBuf, (bytesRead + 3) & ~3);
	INT_Enable();			// enable interrupts

	if (ret != mscReturnOk)
	{
	    LogError ("FirmwareUpdate: Write Sector @ 0x%X - Error Code %d",
		      FIRMWARE_ADDRESS + offs, ret);
	    l_fh.fs = NULL;		// invalidate file handle
	    status = 6;
	    break;
	}

	POWER_LED = 0;			// LED OFF
    }


    /* Disable flash write access again */
    MSC_Deinit();

    /* Close file */
    f_close (&l_fh);

    return status;	// return error code (0 = OK)
}
