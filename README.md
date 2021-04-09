# Power Monitor
test Differential input. the absolute encoder AC5115 shows the angle and voltage on the display. 
Board TM4C123GXL.
Display BOOSTTXL-K350QVG-S1.

Build Tiva C library for TM4C1294NCPDT
export COMPILER=gcc CC=arm-none-eabi-gcc CFLAGS="-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -std=gnu99 -Os -ffunction-sections -fdata-sections"
Build Tiva C library for TM4C123GXL
export COMPILER=gcc CC=arm-none-eabi-gcc CFLAGS="-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -MD -Wall -pedantic"

# References:
https://github.com/MajenkoLibraries/Average
