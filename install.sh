#!/bin/sh


#clone RTOS SDK and set it to 2.0.0

git clone -b v2.0.0 https://github.com/espressif/ESP8266_RTOS_SDK.git



#add esptool
git clone https://github.com/espressif/esptool.git


#add dependencies as descrined by https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/linux-setup.html
sudo apt-get install gcc git wget make libncurses-dev flex bison gperf python python-serial


#install the compiler in the current directoru
wget -qO - https://dl.espressif.com/dl/xtensa-lx106-elf-linux64-1.22.0-100-ge567ec7-5.2.0.tar.gz | tar -xvz


#prepare env.sh so it can be executed from any place

echo "export PATH=\"\${PATH}:$( realpath ./xtensa-lx106-elf )/bin\"" >> env.sh
echo "export SDK_PATH=\"$( realpath ESP8266_RTOS_SDK)\"" >> env.sh
echo "export BIN_PATH=\"$( realpath esp8266_bin )\"" >> env.sh
echo "export ESPTOOL_PATH=\"$( realpath esptool)\"" >> env.sh

#add missing libhal

wget -P ./ESP8266_RTOS_SDK/lib https://github.com/esp8266/esp8266-wiki/raw/master/libs/libhal.a


#optionally exec env.sh
. ./env.sh












