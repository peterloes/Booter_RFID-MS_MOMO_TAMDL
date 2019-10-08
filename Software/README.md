﻿# Booter for Scale
with MOMO


EFM32_Boot is a boot software for the EFM32 architecture. Its main
purpose is to verify and start the application software. Before doing this,
it checks whether there exists an update file on the SD-Card. If so, the
application software in FLASH will be updated.

- APRDL, the name of the update file is APRDL.UPD
- TAMDL, the name of the update file is TAMDL.UPD
- MAPRDL, the name of the update file is MAPRDL.UPD
- Individual platforms, based on MAPRDL. 
  The name must be specified to the Makefile as variable PLATFORM,
  e.g. "make PLATFORM=MOMO" will generate a booter that expects
  an update file MOMO.UPD.
 
If no PLATFORM variable has been specified, the booter determines the
platform by configuring PA3 as input and reading its default state.  For
APRDL this is connected to the KEY_S1 signal, which has a pull-up resistor,
therefore its default state is 1.  For MAPRDL and TAMDL this pin is used for
CAM1_ENABLE, which has a pull-down resistor and is 0.
To distinguish between MAPRDL and TAMDL, pin PC15 will be probed.
On platform MAPRDL, this is signal PC15_LA_MOSFET which is connected to
pull-down resistor R66.  TAMDL has a pull-up resistor connected to PC15.

![My image](https://github.com/peterloes/Booter_Scales/blob/master/1_Starting_Application.jpg)


The Booter features EFM32 ...the world´s most energy friendly microcontrollers

ARM Cortex-M3 EFM32G230F128
