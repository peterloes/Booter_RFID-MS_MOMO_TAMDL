/***************************************************************************//**
 * @file
 * @brief	Header file of module Flash.c
 * @author	Ralf Gerhauser
 * @version	2018-02-19
 ****************************************************************************//*
Revision History:
2018-02-19,rage	Removed filename defines.
2016-02-27,rage	Renamed board types to "APRDL" and "MAPRDL".
2016-02-24,rage	Initial version.
*/

#ifndef __INC_Flash_h
#define __INC_Flash_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "config.h"		// include project configuration parameters
#include "em_msc.h"

/*=============================== Definitions ================================*/

/*!@brief Base address of the application firmware image. */
#define FIRMWARE_ADDRESS	0x00008000

/*================================ Prototypes ================================*/

PRJ_INFO *FindPrjInfo(void);
int	  FirmwareUpdate(char *filename);


#endif /* __INC_Flash_h */
