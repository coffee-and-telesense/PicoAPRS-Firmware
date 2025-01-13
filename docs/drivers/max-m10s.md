# Guide for using the max-m10s driver and api

## Getting started
The first thing to do is to make sure you have the stm32 CubeMX tool setup with the proper pin configurations. This library is currently only implemented using the I2C bus. Configure your pins as follows:
1. Select PB7 and PB6 as the SDA and SCL respectively. Note that these pins correspond to A5 (SCL), and A4 (SDA) of the nucleo-l432kc. TODO: Add directions for portability to other nucleo boards as well. Refer to datasheet for other boards.
2. Got to connectivity, then click I2C1. 
3. Under parameter settings set I2C speed mode to fast mode (400 KHz).
4. Under GPIO settings set SCL and SDA ot Pull-up type.

