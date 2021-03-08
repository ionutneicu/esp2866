#!/bin/sh


DEVICE=/dev/my_uart

#sudo python ${ESPTOOL_PATH}/esptool.py --port ${DEVICE}  erase_flash
#sudo python ${ESPTOOL_PATH}/esptool.py --port ${DEVICE}  write_flash 0x3fc000 esp_init_data_default.bin
#sudo python ${ESPTOOL_PATH}/esptool.py --port ${DEVICE}  write_flash 0x3fc000 ../../bins/esp_init_data_default.bin
#sudo python ${ESPTOOL_PATH}/esptool.py --port ${DEVICE}  write_flash 0x3fe000 ../../bins/blank.bin
sudo python ${ESPTOOL_PATH}/esptool.py --port ${DEVICE}  write_flash 0x00000  ../../esp8266_bin/eagle.flash.bin 0x20000  ../../esp8266_bin/eagle.irom0text.bin
