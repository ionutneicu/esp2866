#!/bin/sh


sudo python esptool.py --port /dev/ttyUSB0  erase_flash
sudo python esptool.py --port /dev/ttyUSB0  write_flash 0x3fc000 esp_init_data_default.bin
sudo python esptool.py --port /dev/ttyUSB0  write_flash 0x3fc000 ../../bins/esp_init_data_default.bin
sudo python esptool.py --port /dev/ttyUSB0  write_flash 0x3fe000 ../../bins/blank.bin
sudo python esptool.py --port /dev/ttyUSB0  write_flash 0x00000 ../../esp8266_bin/eagle.flash.bin
sudo python esptool.py --port /dev/ttyUSB0  write_flash 0x20000 ../../esp8266_bin/eagle.irom0text.bin
