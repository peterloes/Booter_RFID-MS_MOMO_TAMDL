/***************************************************************************//**
 * @file
 * @brief	Project configuration file
 * @author	Ralf Gerhauser
 * @version	2018-02-19
 *
 * This file allows to set miscellaneous configuration parameters.  It must be
 * included by all modules.
 *
 ****************************************************************************//*
Revision History:
2018-02-19,rage	Added prototype for DelayTick().
2016-02-10,rage	Derived from SNB_Heaven.
*/

#ifndef __INC_config_h
#define __INC_config_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"

/*=============================== Definitions ================================*/

/*
 * Basic defines - should all be moved to Generic.h
 */
    /* terminators for lists and strings */

#define	EOL		NULL		/* EndOfList		*/
#define EOS		'\0'		/* EndOfString		*/
#define	NONE		(-1)

    /* macro to calculate the number of elements of an array */
#define ELEM_CNT(array)  (sizeof (array) / sizeof ((array)[0]))

/*
 * LED Definitions
 */
    /*!@brief GPIO Port of the (red) Power-LED. */
#define POWER_LED_PORT		gpioPortA
    /*!@brief GPIO Pin of the (red) Power-LED. */
#define POWER_LED_PIN		2
    /*! @brief Macro to set or clear the Power-LED */
#define POWER_LED   IO_Bit(GPIO->P[POWER_LED_PORT].DOUT, POWER_LED_PIN)

/*!
 * @brief MPU Clock Configuration.
 *
 * Set to 0 to use the internal RC oscillator, if 1 the external 32MHz XTAL
 * is used.
 */
#define USE_EXT_32MHZ_CLOCK	0

/*
 * Configuration for module AlarmClock
 */
    /*!@brief RTC frequency in [Hz]. */
#define RTC_COUNTS_PER_SEC	32768


/*
 * Configuration for module "Logging.c"
 */
    /*!@brief Size of the log buffer in bytes. */
#define LOG_BUF_SIZE	4096

    /*!@brief Use this define to specify a function to be called for monitoring
     * the log activity.  Here, monitoring is done via the LEUART interface.
     */
#define LOG_MONITOR_FUNCTION	drvLEUART_puts

/* forward declaration */
void    drvLEUART_puts(const char *str);


/*!@name DMA Channel Assignment
 *
 * The following definitions assign the 8 DMA channels to the respective
 * devices or drivers.  These defines are used as index within the global
 * DMA_DESCRIPTOR_TypeDef structure @ref g_DMA_ControlBlock.
 */
//@{
#define DMA_CHAN_LEUART_RX	0	//! LEUART Rx uses DMA channel 0
#define DMA_CHAN_LEUART_TX	1	//! LEUART Tx uses DMA channel 1
//@}


/*================================== Macros ==================================*/

#if defined (__GNUC__)	// GNU Library offers a smaller version of printf()
#define sprintf		siprintf
#define vsprintf	vsiprintf
#endif

#ifdef DEBUG
    /*
     * Debugging output via ITM or LEUART
     */
    #if DEBUG_VIA_ITM
	#define DBG_PUTC(ch)	ITM_SendChar(ch)
	#define DBG_PUTS(str)	ITM_SendStr(str)
	uint32_t ITM_SendChar (uint32_t ch);
	void ITM_SendStr(const char *pStr);
    #else
	#define DBG_PUTC(ch)	drvLEUART_putc(ch)
	#define DBG_PUTS(str)	drvLEUART_puts(str)
	void	drvLEUART_putc(char ch);
	void	drvLEUART_puts(const char *str);
    #endif
    void dbgInit(void);
#else
    #define DBG_PUTC(ch)
    #define DBG_PUTS(str)
#endif

    /*! Macro to address a single bit in the I/O range (peripheral range) in
     *  an atomic manner.
     * @param address   I/O register address.
     * @param bitNum    Bit number within this register.
     */
#define IO_BIT_ADDR(address, bitNum)					\
	((__IO uint32_t *) (BITBAND_PER_BASE				\
			+ (((uint32_t)(address)) - PER_MEM_BASE) * 32	\
			+ (bitNum) * 4))

    /*! Shortcut to directly access an I/O-bit. */
#define IO_Bit(regName, bitNum)	*IO_BIT_ADDR(&regName, bitNum)

    /*! Macro to address a single bit in an SRAM variable in an atomic manner.
     * @param address   Address of the variable in SRAM.
     * @param bitNum    Bit number within this variable.
     */
#define SRAM_BIT_ADDR(address, bitNum)					\
	((__IO uint32_t *) (BITBAND_RAM_BASE				\
			+ (((uint32_t)(address)) - RAM_MEM_BASE) * 32	\
			+ (bitNum) * 4))

    /*! Shortcut to directly access a bit in a variable. */
#define Bit(varName, bitNum)	*SRAM_BIT_ADDR(&varName, bitNum)

/*=========================== Typedefs and Structs ===========================*/

/*!@brief Structure to hold Project Information */
typedef struct
{
    char const  ID[12];
    char const  Date[16];
    char const  Time[10];
    char const  Version[16];
} PRJ_INFO;

/*======================== External Data and Routines ========================*/

extern volatile bool	 g_flgIRQ;	// Flag: Interrupt occurred

void	msDelay (uint32_t ms);		// Delay routine
void	DelayTick (void);		// Delay one tick


#endif /* __INC_config_h */
