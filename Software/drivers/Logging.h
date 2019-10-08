/***************************************************************************//**
 * @file
 * @brief	Header file of module Logging.c
 * @author	Ralf Gerhauser
 * @version	2015-04-02
 ****************************************************************************//*
Revision History:
2016,02,17,rage	Taken from SNB_Heaven, adapted for EFM32_Boot.
*/

#ifndef __INC_Logging_h
#define __INC_Logging_h

/*=============================== Header Files ===============================*/

#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

    /*!@brief Enable logging code in this project. */
#define LOGGING

    /*!@brief Size of the log buffer in bytes. */
#ifndef LOG_BUF_SIZE
    #define LOG_BUF_SIZE	2048
#endif

    /*!@brief Maximum size (bytes) of logs in the buffer before it is flushed. */
#ifndef LOG_SAMPLE_MAX_SIZE
    #define LOG_SAMPLE_MAX_SIZE	1024
#endif

    /*!@brief Timeout in seconds to wait for another sample until flushing
     * the log buffer.
     */
#ifndef LOG_SAMPLE_TIMEOUT
    #define LOG_SAMPLE_TIMEOUT	5
#endif

    /*!@brief   Maximum size of one log entry in bytes.
     * @details This value specifies the maximum length of one log message.
     * It is important for the management of the log buffer @ref l_LogBuf in
     * that way as the storage wraps around, if the amount of bytes to the end
     * of the buffer is less than this value.
     */
#ifndef LOG_ENTRY_MAX_SIZE
    #define LOG_ENTRY_MAX_SIZE	100
#endif

    /*!@brief Use this define to specify a function to be called for monitoring
     * the log activity.  A typical candidate is a put-string routine which
     * outputs the log messages to a UART interface.  Example:
     * <b>\#define LOG_MONITOR_FUNCTION drvLEUART_puts</b>
     */
#ifndef LOG_MONITOR_FUNCTION
    #define LOG_MONITOR_FUNCTION	NONE
#endif

/*================================ Global Data ===============================*/

    /* Filename of the current Log File on the SD-Card */
extern char	g_LogFilename[14];

/*================================ Prototypes ================================*/

void	 LogInit (void);		// Initialize the logging facility
bool	 LogFileOpen (char *filepattern, char *filename); // Open Log File
void	 Log (const char *frmt, ...);		// Log a message
void	 LogError (const char *frmt, ...);	// Log an error
void	 LogFlush (void);		// Flush the log buffer
void	 LogFlushCheck (void);		// Check if to flush the log buffer


#endif /* __INC_Logging_h */
