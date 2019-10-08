/***************************************************************************//**
 * @file
 * @brief	Logging
 * @author	Ralf Gerhauser
 * @version	2018-02-19
 *
 * This module provides a logging facility to send messages to the LEUART and
 * store them into a file on the SD-Card.
 *
 ****************************************************************************//*
Revision History:
2018-02-19,rage	Use INT_En/Disable() instead of __en/disable_irq().
2016,02,17,rage	Taken from SNB_Heaven, removed time stamp generation.
2015-07-09,rage	IAR Compiler: Use sprintf() instead siprintf().
2015-04-02,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "em_device.h"
#include "em_assert.h"
#include "em_int.h"
#include "Logging.h"
#include "ff.h"		// FS_FAT12/16/32
#include "diskio.h"	// DSTATUS
#include "microsd.h"

/*=============================== Definitions ================================*/

#if LOG_MONITOR_FUNCTION == NONE
    #undef LOG_MONITOR_FUNCTION
#endif

/*================================ Local Data ================================*/

/*
 * The Log Buffer and its indices.  To ensure efficient data handling, all log
 * messages are directly stored as a consecutive stream of characters into the
 * log buffer, terminated by 0.  If the remaining amount of bytes to the end of
 * the buffer is less than LOG_ENTRY_MAX_SIZE, the storage wraps around, and
 * <idxLogPut> is set to 0, i.e. the beginning of the buffer.  This is marked
 * by an extra 0 byte, directly after the terminating 0 of the previous string.
 */
static char	l_LogBuf[LOG_BUF_SIZE];
static int	idxLogPut, idxLogGet;

    /* Counter for lost log entries */
static uint32_t	l_LostEntryCnt;

    /* Counter how many error messages may still be generated */
static int	l_ErrMsgCnt;

    /* Flag to trigger a flush of the log buffer, see LOG_SAMPLE_TIMEOUT. */
static volatile bool l_flgLogFlushTrigger;

    /* Counter to specify how often the Log Flush LED will flash */
static volatile uint8_t l_LogFlushLED_FlashCnt;

    /* File handle for log file */
static FIL	l_fh;

/*=========================== Forward Declarations ===========================*/

static void	logMsg(const char *prefix, const char *frmt, va_list args);


/***************************************************************************//**
 *
 * @brief	Initialize the Logging Facility
 *
 * This routine must be called once to initialize the logging facility.
 * It sets up the log buffer and allocates timers.  Disk access is not done
 * here because it is not available at this early point, see function
 * @ref LogFlushCheck() for this.
 *
 ******************************************************************************/
void	 LogInit (void)
{
    /* initialize indices */
    idxLogGet = idxLogPut = 0;
}


/***************************************************************************//**
 *
 * @brief	Open Log File
 *
 * This routine (re-)opens the log file for writing.
 *
 * @param[in] filepattern
 *	Filename to compare all file entries in the root directory of the disk
 *	with.  The filename must follow the DOS 8.3 notation, i.e. 8 characters
 *	for the basename and 3 characters extension, separated by a dot.
 *	An asterisk (*) at the end of the basename or/and extension is treated
 *	as wildcard, the further characters will not be compared.
 *
 * @param[in] filename
 *	Fall-back filename to use if no appropriate file pattern could be
 *	found on the disk.
 *
 * @return
 *	Result: true if file exists or could be opened, false if not.
 *
 ******************************************************************************/
bool	 LogFileOpen (char *filepattern, char *filename)
{
FRESULT	 res;		// FatFs function common result code
char	*pStr;		// string pointer


    /* Parameter Check */
    if (filepattern != NULL)
    {
	/* Find filename with specified pattern on the SD-Card */
	pStr = FindFile ("/", filepattern);
	if (pStr != NULL)
	    filename = pStr;	// found pattern on disk
    }

    /* Check if file on the disk, or a fall-back name exists */
    if (filename == NULL)
	return false;		// file not found and no fall-back specified

    /* Discard old file handle, open new file */
    res = f_open (&l_fh, filename,  FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
    if (res == FR_OK)
    {
	res = f_lseek (&l_fh, f_size(&l_fh));
    }

    if (res != FR_OK)
    {
	LogError ("LogFileOpen: Error Code %d", res);
	l_fh.fs = NULL;		// invalidate file handle
    }
    else
    {
	l_ErrMsgCnt = 2;
    }

    /* Power off the SD-Card Interface */
    MICROSD_PowerOff();

    return true;
}


/***************************************************************************//**
 *
 * @brief	Log a Message
 *
 * This routine writes a log message into the buffer.  It may be called from
 * interrupt context.
 *
 * The format of a log message is:
 * 20151231-235900 \<message\>
 *
 ******************************************************************************/
void	 Log (const char *frmt, ...)
{
va_list	 args;


    /* disable interrupts to prevent interferring of other logs */
    INT_Disable();

    /* build variable argument list and call logMsg() */
    va_start(args, frmt);
    logMsg (NULL, frmt, args);
    va_end(args);

    /* enable interrupts again */
    INT_Enable();
}


/***************************************************************************//**
 *
 * @brief	Log an Error Message
 *
 * This routine writes an error log message into the buffer.  It may be called
 * from interrupt context.
 *
 * The format of an error log message is:
 * 20151231-235900 ERROR \<message\>
 *
 ******************************************************************************/
void	 LogError (const char *frmt, ...)
{
va_list	 args;


    /* disable interrupts to prevent interfering of other logs */
    INT_Disable();

    /* build variable argument list and call logMsg() */
    va_start(args, frmt);
    logMsg ("ERROR ", frmt, args);
    va_end(args);

    /* enable interrupts again */
    INT_Enable();
}


/***************************************************************************//**
 *
 * @brief	Flush Log Buffer
 *
 * This routine flushes the log buffer, i.e. its contents is written to disk.
 *
 ******************************************************************************/
void	 LogFlush (void)
{
FRESULT	 res;		// FatFs function common result code
int	 cnt;
UINT	 bytesWr;


    /* See if Log File is open */
    if (IsFileHandleValid(&l_fh) == false)
	return;			// no file open or invalid file handle

    /* Switch the SD-Card Interface on */
    MICROSD_PowerOn();

    /* Re-Initialize disk (mount is still the same!) */
    if (disk_initialize(0) != 0)
    {
	if (--l_ErrMsgCnt >= 0)
	    LogError ("LogFlush: Init Failed");
    }
    else
    {
	/* Write all log messages to disk */
	while (idxLogGet != idxLogPut)
	{
	    cnt = l_LogBuf[idxLogGet];	// get string length
	    if (cnt == 0)
	    {
		idxLogGet = 0;		// length of 0 indicates wrap-around
		cnt = l_LogBuf[idxLogGet];
	    }

	    /* write string to file without the terminating 0 (EOS) */
	    res = f_write (&l_fh, l_LogBuf + idxLogGet + 1, cnt, &bytesWr);
	    if (res != FR_OK)
	    {
		if (--l_ErrMsgCnt >= 0)
		    LogError ("LogFlush: Error Code %d", res);
		break;
	    }

	    if (bytesWr < cnt)
	    {
		if (--l_ErrMsgCnt >= 0)
		    LogError ("LogFlush: Disk Full");
		res = FR_DISK_ERR;
		break;
	    }

	    /* update index, consider <len> byte and EOS */
	    idxLogGet += (cnt + 2);
	}

	/* Synchronize file system */
	if (res == FR_OK)
	    f_sync (&l_fh);
    }

    /* Switch the SD-Card Interface off */
    MICROSD_PowerOff();
}


/***************************************************************************//**
 *
 * @brief	Check if Log Buffer should be Flushed
 *
 * This routine is periodically called from the main loop to check if the log
 * buffer should be flushed, i.e. l_flgLogFlushTrigger is set, or more than
 * @ref LOG_SAMPLE_MAX_SIZE bytes have been stored in the log buffer.
 *
 ******************************************************************************/
void	 LogFlushCheck (void)
{
int	 cnt;			// allocated space in the log buffer

    cnt = idxLogPut - idxLogGet;	// calculate allocated space
    if (cnt < 0)
	cnt += LOG_BUF_SIZE;		// wrap around

    if (cnt > LOG_SAMPLE_MAX_SIZE  ||  l_flgLogFlushTrigger)
    {
	l_flgLogFlushTrigger = false;

	LogFlush();
    }
}


/***************************************************************************//**
 *
 * @brief	Log Message
 *
 * This routine writes the current time stamp, an optional prefix, and the
 * specified log message into the buffer.
 *
 * The format of a log message is:
 * \<prefix\> \<message\>
 *
 ******************************************************************************/
static void	logMsg(const char *prefix, const char *frmt, va_list args)
{
char	 tmpBuffer[LOG_ENTRY_MAX_SIZE];	// use this if the log buffer is full
char	*pBuf;				// pointer to the buffer to use
int	 cnt, num;			// message length, available space


    /* Check if there is enough space in the log buffer */
    num = LOG_BUF_SIZE - idxLogPut;	// distance to end of buffer
    if (num > LOG_ENTRY_MAX_SIZE)
	num = 0;			// enough space, no additional memory

    cnt = idxLogPut + num - idxLogGet;	// calculate allocated space
    if (cnt < 0)
	cnt += LOG_BUF_SIZE;		// wrap around

    cnt = LOG_BUF_SIZE - cnt - 1;	// calculate free space

    if (cnt < LOG_ENTRY_MAX_SIZE)
    {
	/* Not enough space in buffer - skip entry and count as "lost" */
	l_LostEntryCnt++;

#ifdef LOG_MONITOR_FUNCTION
	sprintf (tmpBuffer, "ERROR: Log Buffer Out of Memory"
			    " - lost %ld Messages\n", l_LostEntryCnt);
	LOG_MONITOR_FUNCTION (tmpBuffer);
#endif
	pBuf = tmpBuffer;		// use temporary buffer
    }
    else
    {
	/* There is enough memory in log buffer */
	if (num > 0)
	{
	    l_LogBuf[idxLogPut] = 0;	// mark wrap-around
	    idxLogPut = 0;		// adjust start of new log message
	}

	pBuf = l_LogBuf + idxLogPut;	// use standard log buffer
    }

    /* Reserve one byte for string length information */
    cnt = 1;

    /* Store optional prefix */
    if (prefix != NULL)
    {
	if (*prefix != EOS)
	{
	    strcpy (pBuf + cnt, prefix);
	    cnt += strlen(prefix);
	}
    }

    /* Build and store the log message */
    cnt += vsprintf(pBuf + cnt, frmt, args);

    /* add <CR><LF> */
    strcpy (pBuf + cnt, "\r\n");
    cnt += 3;			// <CR> <LF> EOS

    /* Immediately send the complete log message to the monitor output */
#ifdef LOG_MONITOR_FUNCTION
    LOG_MONITOR_FUNCTION (pBuf + 1);
#endif

    /* Check length */
    EFM_ASSERT(cnt <= LOG_ENTRY_MAX_SIZE);

    /* If used, update standard log buffer */
    if (pBuf != tmpBuffer)
    {
	/* Store string length */
	l_LogBuf[idxLogPut] = cnt - 2;	// no <len> byte, no EOS

	/* Update index */
	idxLogPut += cnt;
    }
}
