#!/bin/sh


#clone RTOS SDK and set it to 2.0.0

git clone -b v2.0.0 git@github.com:espressif/ESP8266_RTOS_SDK.git



#add esptool
git clone git@github.com:espressif/esptool.git


#add dependencies as descrined by https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/linux-setup.html
sudo apt-get install gcc git wget make libncurses-dev flex bison gperf python python-serial


#install the compiler in the current directoru
wget -qO - https://dl.espressif.com/dl/xtensa-lx106-elf-linux64-1.22.0-100-ge567ec7-5.2.0.tar.gz | tar -xvz


#exec env.sh

. ./env.sh












