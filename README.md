# STM32F103_GT911_Conf_Reader

Simple program to read the configuration of a GT911 touch panel.

Output is via USB. Under linux it will identify it self as /dev/ttyUSB*

The connections for the panel are on pin
PB10 SCL
PB11 SDA

It waits untill a key is pressed and will then read the configuration from register 0x8047 until 0x8100
