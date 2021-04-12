# STM32F103_GT911_Conf_Reader

Simple program to read the configuration of a GT911 touch panel.

Output is via USB. Under linux it will identify it self as /dev/ttyUSB*

The connections for the panel are on pins:
PB10 --> SCL
PB11 --> SDA
Don't forget to add the 2K2 pullup resisters to the 3.3V on each line

It waits untill a key is pressed and will then read the configuration from register 0x8047 until 0x8100

Programming STM32F103C8T6 bluepill with ST-Link v2

openocd -f STM32F103C8T6.cfg -c init -c targets -c halt -c "flash write_image erase f103_i2c_tp_conf_reader.elf" -c "verify_image f103_i2c_tp_conf_reader.elf"

.elf file is found in dist/debug/GNU_ARM-Linux
