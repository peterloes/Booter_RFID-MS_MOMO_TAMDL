/***************************************************************************//**
 * @file
 * @brief	EFM32 Boot
 * @author	Ralf Gerhauser
 * @version	2018-03-28
 *
 * This is a boot software for the EFM32 architecture.  Its main purpose is
 * to verify and start the application software.
 *
 * Parts of the code are based on the example code of AN0019 "EEPROM Emulation"
 * from Energy Micro AS.
 *
 ***************************************************************************//**
 *
 * Parts are Copyright 2013 Energy Micro AS, http://www.energymicro.com
 *
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ****************************************************************************//*
Revision History:
2018-03-28,rage	Changed probing for platform "TAMDL".
2018-02-19,rage	Added support for platform "TAMDL".
2016-04-14,rage	StartFirmware: Corrected check for initial SP and PC.
2016-02-27,rage	Renamed board types to "APRDL" and "MAPRDL".
2016-02-10,rage	Initial version.
*/

/*!
 * @mainpage
 * <b>Description</b><br>
 * <b>EFM32_Boot</b> is a boot software for the EFM32 architecture.  Its main
 * purpose is to verify and start the application software.  Before doing this,
 * it checks whether there exists an update file on the SD-Card.  If so, the
 * application software in FLASH will be updated.
 *
 * <b>Memory Map</b><br>
 * FLASH and SRAM memory ranges:
 * <center><table>
 * <tr><th>Address</th> <th>Description</th></tr>
 * <tr><td>0x00000000</td> <td>Start of FLASH, here resides the Booter (32KB)</td></tr>
 * <tr><td>0x00008000</td> <td>Start of Application Software (up to 96KB)</td></tr>
 * <tr><td>0x0001FFFF</td> <td>End of 128KB FLASH</td></tr>
 * <tr><td>0x10000000</td> <td>Start of SRAM when accessed as Code</td></tr>
 * <tr><td>0x10003FFF</td> <td>End of 16KB SRAM when accessed as Code</td></tr>
 * <tr><td>0x20000000</td> <td>Start of SRAM when accessed as Data</td></tr>
 * <tr><td>0x20003FFF</td> <td>End of 16KB SRAM when accessed as Data</td></tr>
 * </table></center>
 *
 * <b>Supported Platforms</b><br>
 * - APRDL, the name of the update file is <b>APRDL.UPD</b>
 * - TAMDL, the name of the update file is <b>TAMDL.UPD</b>
 * - MAPRDL, the name of the update file is <b>MAPRDL.UPD</b>
 * - Individual platforms, based on MAPRDL.  The name must be specified to
 *   the Makefile as variable <i>PLATFORM</i>, e.g. "make PLATFORM=MOMO" will
 *   generate a booter that expects an update file <b>MOMO.UPD</b>.
 *
 * If no PLATFORM variable has been specified, the booter determines the
 * platform by configuring PA3 as input and reading its default state.  For
 * APRDL this is connected to the KEY_S1 signal, which has a pull-up resistor,
 * therefore its default state is 1.  For MAPRDL and TAMDL this pin is used for
 * CAM1_ENABLE, which has a pull-down resistor and is 0.<br>
 * To distinguish between MAPRDL and TAMDL, pin PC15 will be probed.  On
 * platform MAPRDL, this is signal PC15_LA_MOSFET which is connected to
 * pull-down resistor R66.  TAMDL has a pull-up resistor connected to PC15.
 *
 * The following table lists the relevant pins, their functionality, and
 * default state:
 * <center><table>
 * <tr><th>Pin</th><th>APRDL</th><th>MAPRDL</th><th>TAMDL</th></tr>
 * <tr><td>PA3</td><td>KEY_S1 (1)</td><td>CAM1_ENABLE (0)</td>
 *	<td>CAM1_ENABLE (0)</td></tr>
 * <tr><td>PC15</td><td>LA_MOSFET (0)</td><td>LA_MOSFET (0)</td>
 *	<td>Pull-Up (1)</td></tr>
 * </table></center>
 *
 * <b>Microcontroller</b><br>
 * The heart of the board is an EFM32G230 microcontroller.  It provides two
 * different clock domains: All low-energy peripheral is clocked via a
 * 32.768kHz external XTAL.  The MCU and other high performance peripheral
 * uses a high-frequency clock.  The board can be configured to use the
 * internal RC-oscillator, or an external 32MHz XTAL for that purpose,
 * see define @ref USE_EXT_32MHZ_CLOCK.
 *
 * <b>LEDs</b><br>
 * There are two LEDs, but only the red Power-On LED is used by the booter to
 * indicate certain states or error codes.
 * - Under normal conditions, the LED just flashes once for a short time before
 *   the application software is started.
 * - If a matching firmware update file has been found on the SD-Card, this
 *   will be programmed in FLASH.  When the update is done, the LED will
 *   permanently flicker to indicate that firmware has been updated and it is
 *   now time to remove the SD-Card containing the update file.
 * - If an error condition happens, the LED is used to inform the operator
 *   about the error code.  It starts with a pause of 800ms, followed by a
 *   number of LED flashes.  The number represents the error code:
 *   <center><table>
 *   <tr><th>Code</th> <th>Error Description</th></tr>
 *   <tr><td>  1 </td> <td>FLASH is not programmed (no application)</td></tr>
 *   <tr><td>  2 </td> <td>Corrupted application image in FLASH</td></tr>
 *   <tr><td>  3 </td> <td>No update file found for this board</td></tr>
 *   <tr><td>  4 </td> <td>Update failed: File read error</td></tr>
 *   <tr><td>  5 </td> <td>Update failed: FLASH erase error</td></tr>
 *   <tr><td>  6 </td> <td>Update failed: FLASH write error</td></tr>
 *   </table></center>
 *
 * <b>SD-Card</b><br>
 * An SD-Card is used to provide firmware update images and to store logging
 * information in file "UPDATE.TXT".  Only formatted cards can be used,
 * supported file systems are FAT12, FAT16, and FAT32.  The filenames must
 * follow the DOS schema 8+3, i.e. maximum 8 characters for the basename,
 * and 3 for the extension.
 * If the SD-Card contains a file <b><i>platform</i>.UPD</b>, where
 * <b><i>platform</i></b> specifies the board the firmware update image
 * is meant to be, the booter will update the application software in
 * FLASH with the image from this file.
 *
 * <b>Low-Energy UART</b><br>
 * The Low-Power UART (LEUART) provides a connection to a host computer (PC).
 * It can be used as monitoring and debugging interface.  All log messages,
 * written to the SD-Card, are sent through this interface also.  This behaviour
 * can be changed by defining @ref LOG_MONITOR_FUNCTION to @ref NONE.
 * The character format for this UART is 9600 baud, 8 data bits, no parity.
 *
 * <b>Booter Firmware</b><br>
 * The firmware consists of an initialization part and a main loop, also called
 * service execution loop.  The initialization part sets up all modules, enables
 * devices and interrupts.  The service execution loop handles all tasks that
 * must not be executed in interrupt context.
 *
 * After power-up or reset the following actions are performed:
 * -# Basic initialization of MCU and clocks
 * -# Power-On LED is switched on to show the firmware is alive
 * -# Real-Time-Counter is set up as time base
 * -# Low-Energy UART and DMA is configured
 * -# SD-Card interface is initialized
 * -# LED is switched off
 * -# If no SD-Card is present, immediately start the application
 * -# If an SD-Card is present, but it does not contain any firmware update
 *    files, immediately start the application
 * -# If an SD-Card is present and it contains firmware update files, but
 *    not for this board, show error code 3 via LED
 * -# If an SD-Card is present and contains a firmware update file for this
 *    board, read it and program the image into the FLASH
 * -# If there are any errors, show the error number via LED, see above.
 * -# Finally wait until the SD-Card containing the firmware update file has
 *    been removed.
 */

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_rtc.h"
#include "em_emu.h"
#include "em_dma.h"
#include "config.h"	// include project configuration parameters
#include "LEUART.h"
#include "Logging.h"
#include "Flash.h"	// firmware update routines to erase and program FLASH

#include "ff.h"		// FS_FAT12/16/32
#include "diskio.h"	// DSTATUS
#include "microsd.h"	// g_MicroSD_PwrPort, g_MicroSD_PwrPin, DiskInit(),...

/*=============================== Definitions ================================*/

/*!@brief Calculate maximum value to prevent overflow of a 32bit register. */
#define MAX_VALUE_FOR_32BIT	(0xFFFFFFFFUL / RTC_COUNTS_PER_SEC)

/*================================ Global Data ===============================*/

extern PRJ_INFO const  prj;		// Project Information


/*! @brief Global DMA Control Block.
 *
 * It contains the configuration for all 8 DMA channels which may be used by
 * various peripheral devices, e.g. ADC, DAC, USART, LEUART, I2C, and others.
 * The entries of this array will be set by the initialization routines of the
 * driver, which was assigned to the respective channel.  Unused entries remain
 * zero.  There is a total of 16 entries in the array.  The first 8 are used
 * for the primary DMA structures, the second 8 for alternate DMA structures
 * as used for DMA scatter-gather mode, where one buffer is still available,
 * while the other can be re-configured.  This application uses only the first
 * 8 entries.
 *
 * @see  DMA Channel Assignment
 *
 * @note This array must be aligned to 256!
 */
#if defined (__ICCARM__)
    #pragma data_alignment=256
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2];
#elif defined (__CC_ARM)
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#elif defined (__GNUC__)
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#else
    #error Undefined toolkit, need to define alignment
#endif


/*! @brief Global DMA Callback Structure.
 *
 * This array contains the addresses of the DMA callback functions, which are
 * executed for a dedicated DMA channel at the end of a DMA transfer.
 * The entries of this array will be set by the initialization routines of the
 * driver, which was assigned to the respective channel.  Unused entries remain
 * zero.
 */
DMA_CB_TypeDef g_DMA_Callback[DMA_CHAN_COUNT];

/*! @brief Flag to indicate that an Interrupt occurred in the meantime.
 *
 * This flag must be set <b>true</b> by any interrupt service routine that
 * requires actions in the service execution loop of main().  This prevents
 * the system from entering sleep mode, so the action can be taken before.
 *
 * @note This flag is not used in this project since the booter never enters
 * 	 any energy saving mode!
 */
volatile bool		g_flgIRQ;

/*=========================== Forward Declarations ===========================*/

static void cmuSetup(void);
static int  StartFirmware(void);


/******************************************************************************
 * @brief  Main function
 *****************************************************************************/
int main( void )
{
char	*pPlatform;
char	 FirmwareFilename[40];
int	 n=0, errorCode;

    /* Initialize chip - handle erratas */
    CHIP_Init();

    /* Set up clocks */
    cmuSetup();

    /* Enable clock to GPIO */
    CMU_ClockEnable (cmuClock_GPIO, true);

    /* Configure PA2 to drive the red Power-On LED (LED1) - show we are alive */
    GPIO_PinModeSet (POWER_LED_PORT, POWER_LED_PIN, gpioModePushPull, 1);

    /* Configure PA3 and PC15 as input to determine platform (board type) */
    GPIO_PinModeSet (gpioPortA, 3, gpioModeInput, 0);
    GPIO_PinModeSet (gpioPortC,15, gpioModeInput, 0);

    /* Configure the RTC */
    RTC_Init_TypeDef rtcInit;
    rtcInit.debugRun = false;
    rtcInit.comp0Top = false;
    rtcInit.enable = false;

    /* Initialize the RTC */
    RTC_Init (&rtcInit);

    /*
     * We just use one interrupt:
     *   Overflow - not used
     *   COMP0    - for the 1/2 second base clock
     *   COMP1    - not used
     */
    RTC_IntEnable (RTC_IEN_COMP0);

    /* Enable RTC */
    RTC_Enable (true);

    /* Enable RTC interrupts */
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_EnableIRQ(RTC_IRQn);

    /* Init Low Energy UART with 9600bd (this is the maximum) */
    drvLEUART_Init (9600);

#ifdef DEBUG
    dbgInit();
#endif

    /* Output version string to LEUART */
#ifdef PLATFORM
    drvLEUART_puts("\n***** " PLATFORM "_Boot V");
#else
    drvLEUART_puts("\n***** EFM32_Boot V");
#endif
    drvLEUART_puts(prj.Version);
    drvLEUART_puts(" *****\n\n");

    /* Initialize Logging (do this early) */
    LogInit();

    /* Log Firmware Revision */
    Log ("======================================");
    Log ("EFM32_Boot V%s (%s %s)", prj.Version, prj.Date, prj.Time);

    /*
     * Determine Platform (board)
     *
     * This may be specified via Makefile, e.g. make PLATFORM="TAMDL", otherwise
     * the booter tries to determine it automatically via pin level:
     * Port A3 is 0 for MAPRDL (SERENITY) and TAMDL while it is 1 for APRDL
     * (SNB_Heaven).
     */
#ifdef PLATFORM
    pPlatform = PLATFORM;
#else
    if (GPIO->P[gpioPortA].DIN & (1 << 3))
    {
	pPlatform = "APRDL";
    }
    else
    {
	pPlatform = "MAPRDL";		// assume MAPRDL (SERENITY)

	/*
	 * To distinguish between MAPRDL and TAMDL, pin PC15 will be probed.
	 * On platform MAPRDL, this is signal PC15_LA_MOSFET which is connected
	 * to pull-down resistor R66.  TAMDL has a pull-up resistor connected
	 * to PC15.
	 */
	if (GPIO->P[gpioPortC].DIN & (1 << 15))
	{
	    /* PC15 is 1 - must be TAMDL, overwrite settings */
	    pPlatform = "TAMDL";	// is TAMDL
	    g_MicroSD_PwrPort = TAMDL_MICROSD_PWR_GPIO_PORT;
	    g_MicroSD_PwrPin  = TAMDL_MICROSD_PWR_PIN;
	}
    }
#endif
    Log ("Platform: %s", pPlatform);
    sprintf (FirmwareFilename, "%s.UPD", pPlatform);

    /* Log information about the MCU Type and S/N */
    uint32_t uniquHi = DEVINFO->UNIQUEH;
    Log ("MCU: %s HW-ID: 0x%08lX%08lX",
	 PART_NUMBER, uniquHi, DEVINFO->UNIQUEL);

    /* Initialize SD-Card Interface */
    DiskInit();

    /* Switch Power-LED OFF */
    POWER_LED = 0;

    /* Check if an SD-Card is present */
    if (DiskCheck())
    {
	/* Yes - see if any "*.UPD" files exist */
	if (FindFile ("/", "*.UPD") != NULL)
	{
	    /*
	     * Found update image(s), so the booter will NOT start the
	     * application in FLASH until this SD-Card has been removed.
	     * Next find out, if there exists an firmware update file for
	     * this board.
	     */
	    if (FindFile ("/", FirmwareFilename) != NULL)
	    {
	    PRJ_INFO	*pPrjInfo;
	    char	 oldVersion[16] = "<unknown>";
	    char	 newVersion[16] = "<unknown>";

		/* Yes, then proceed firmware update */
		Log ("Found update file for this board, reprogramming FLASH");

		/* Find version of the current (old) application image */
		pPrjInfo = FindPrjInfo();
		if (pPrjInfo != NULL)
		    strcpy (oldVersion, pPrjInfo->Version);

		/* Perform firmware update */
		errorCode = FirmwareUpdate (FirmwareFilename);

		/* Find version of the new application image */
		pPrjInfo = FindPrjInfo();
		if (pPrjInfo != NULL)
		    strcpy (newVersion, pPrjInfo->Version);

		switch (errorCode)
		{
		    case 0: Log ("Update from version %s to %s finished"
				 " - no errors", oldVersion, newVersion);
			    break;

		    case 4: Log ("Update aborted - File READ ERROR");
			    break;

		    case 5: Log ("Update aborted - FLASH ERASE ERROR");
			    break;

		    case 6: Log ("Update aborted - FLASH WRITE ERROR");
			    break;

		    default:
			    Log ("Update aborted - Unknown Error %d",
				 errorCode);
			    break;
		}
	    }
	    else
	    {
		/* No, there isn't any update file for this board */
		Log ("There exist no update file for this board!");

		errorCode = 3;	// Error Code 3: No Update Image for this board
	    }

	    /* Write all logging to file "UPDATE.TXT" */
	    Log ("Waiting until the SD-Card has been removed");

	    /* Open or create Log File */
	    if (LogFileOpen("UPDATE.TXT", "UPDATE.TXT"))
	    {
		/* Flush current log buffer */
		LogFlush();
	    }

	    /*
	     * Wait until the SD-Card has been removed.  Show error code
	     * while waiting.
	     */
	    do
	    {
		if (errorCode == 0)
		{
		    /* No error, flicker LED */
		    POWER_LED = (++n & 0x1) ? 1 : 0;
		    msDelay(50);		// pause
		}
		else
		{
		    /*
		     * Show error code via LED pattern:
		     * repeat: Pause - <errorCode> x LED pulses
		     */
		    POWER_LED = 0;
		    msDelay(800);		// pause

		    for (n = 0;  n < errorCode;  n++)
		    {
			POWER_LED = 1;		// LED ON
			msDelay(200);
			POWER_LED = 0;		// LED OFF
			msDelay(200);
		    }
		}

		/* Check for SD-Card (again) */
		(void)DiskCheck();

	    } while (! IsDiskRemoved());

	} // end: found any firmware files
	else
	{
	    drvLEUART_puts("No update files on SD-Card\n");
	}
    } // end: SF-Card present

    drvLEUART_puts("Starting Application...\n\n");

    /* Switch LED off and wait until UART DMA transfer is done */
    POWER_LED = 0;
    while (! IsDmaTransferDone())
	;

    /* Bring the LEUART and its DMA into a quiescent state */
    drvLEUART_Deinit();

    /*
     * Verify if FLASH firmware image exists and start it.
     */
    errorCode = StartFirmware(); // never returns - except if there is no image!

    while (true)
    {
	msDelay(800);		// pause

	for (n = 0;  n < errorCode;  n++)
	{
	    POWER_LED = 1;	// LED ON
	    msDelay(200);
	    POWER_LED = 0;	// LED OFF
	    msDelay(200);
	}

	msDelay(1000);		// pause

	/*
	 * To be able to detect another update image, we simply
	 * perform a RESET.
	 */
	NVIC_SystemReset();
    }
}


/******************************************************************************
 * @brief   Configure Clocks
 *
 * This local routine is called once from main() to configure all required
 * clocks of the EFM32 device.
 *
 *****************************************************************************/
static void cmuSetup(void)
{
    /* Start LFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

#if USE_EXT_32MHZ_CLOCK
    /* Start HFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    /* Select HFXO as clock source for HFCLK */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    /* Disable HFRCO */
    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
#endif

    /* Route the LFXO clock to the RTC and set the prescaler */
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);	// RTC, LETIMER
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);	// LEUART0/1
    CMU_ClockEnable(cmuClock_RTC, true);

    /* Prescaler of 1 = 30 us of resolution and overflow each 8 min */
    CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

    /* Enable clock to low energy modules */
    CMU_ClockEnable(cmuClock_CORELE, true);

    /* Enable clock for HF peripherals (ADC, DAC, I2C, TIMER, and USART) */
    CMU_ClockEnable(cmuClock_HFPER, true);

    /* Enable clock to GPIO */
    CMU_ClockEnable(cmuClock_GPIO, true);
}


/***************************************************************************//**
 *
 * @brief	Start Firmware
 *
 * This routine verifies if there is a firmware image programmed in FLASH
 * memory and starts it.
 *
 * @return
 * This routine usually does not return because it passes control to the
 * application.  However, if there was no valid firmware image found in the
 * FLASH, it returns error code 2.  If the FLASH is not programmed at all,
 * error code 1 is returned.
 *
 ******************************************************************************/
static int StartFirmware(void)
{
uint32_t *pInitialSP = (uint32_t *)(FIRMWARE_ADDRESS + 0);
uint32_t *pInitialPC = (uint32_t *)(FIRMWARE_ADDRESS + 4);
typedef void (*FCT_PTR) (void);
FCT_PTR	 pStartAppl;
uint32_t  initSP, initPC;


    /* Load initial SP, PC from application's Vector Table */
    initSP = *pInitialSP;
    initPC = *pInitialPC;

    /* Verify if values are valid */
    if (initSP == 0xFFFFFFFF  ||  initPC == 0xFFFFFFFF)
	return 1;

    if (initSP < SRAM_BASE         ||  initSP > SRAM_BASE  + SRAM_SIZE
    ||  initPC < FIRMWARE_ADDRESS  ||  initPC > FLASH_BASE + FLASH_SIZE)
	return 2;

    /* Disable interrupts */
    __disable_irq();

    /* Start Application */
    SCB->VTOR = FIRMWARE_ADDRESS;
    __set_MSP(initSP);
    pStartAppl = (FCT_PTR)initPC;

    pStartAppl();

    return 9;		// shall never happen
}


/***************************************************************************//**
 *
 * @brief	Handler for RTC Interrupt
 *
 * This is the handler for the Real Time Counter (RTC) interrupt. It checks if
 * the <b>COMP0</b> which is used for the 1/2s base clock triggered.
 *
 ******************************************************************************/
void	RTC_IRQHandler (void)
{
uint32_t	status;			// interrupt status flags

    /* get interrupt status and mask out disabled IRQs */
    status  = RTC->IF;
    status &= RTC->IEN;

    /* Check for COMP0 interrupt which occurs every second */
    if (status & RTC_IF_COMP0)
    {
	/* Generate next COMP0 interrupt after another second */
	RTC_CompareSet (0, (RTC->COMP0 + RTC_COUNTS_PER_SEC/2) & 0xFFFFFF);
	RTC->IFC = RTC_IFC_COMP0;
    }
}


/***************************************************************************//**
 *
 * @brief	Delay for milliseconds
 *
 * This is a delay routine, it returns to the caller after the specified amount
 * of milliseconds has elapsed.
 *
 * @note
 * This routine permanently reads the Real Time Counter until the calculated
 * value has been reached.  No interrupts are used, i.e. the CPU is kept busy
 * all the time.  Usually delay routines are only used for hardware-related
 * timing constraints and must not be called from interrupt routines.
 *
 * @param[in] ms
 *	Duration in milliseconds to wait before returning to the caller.
 *
 ******************************************************************************/
void	msDelay (uint32_t ms)
{
uint32_t baseCnt, duration;

    /* Parameter check */
    EFM_ASSERT (0 < ms  &&  ms <= MAX_VALUE_FOR_32BIT);

    /* Get current time counter value */
    baseCnt = RTC->CNT;

    /* Convert the [ms] value in number of ticks */
    duration = ((ms * RTC_COUNTS_PER_SEC) / 1000) & 0x00FFFFFF;

    /* See how much ticks have been elapsed */
    while (((RTC->CNT - baseCnt) & 0x00FFFFFF) < duration)
	;
}


/***************************************************************************//**
 *
 * @brief	Delay for one tick
 *
 * This is a delay routine for very short durations, it returns to the caller
 * after one tick, i.e. with a 32kHz XTAL about 30 microseconds (may be up to
 * 59 microseconds).
 *
 ******************************************************************************/
void	DelayTick (void)
{
uint32_t currCnt;

    /* Get current time counter value */
    currCnt = RTC->CNT;

    /* First synchronize with next counter change */
    while (RTC->CNT == currCnt)
	;

    /* Wait another tick change */
    currCnt = RTC->CNT;
    while (RTC->CNT == currCnt)
	;
}

