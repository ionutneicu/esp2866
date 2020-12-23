# esp8266


### Install environment:

Run 

    $ ./install.sh and enter superuser password when prompted.


### Setup compile environment - everytime you want to compile:


    $ source ./env.sh
    

### Run ADC hello example:

Go to apps/adc_hello.

Run first

    $ source ../../env.sh

Compile using gen_misc.sh:

    $ ./gen_misc.sh

Press first y, then 0, then Enter ( defaults ) to the end.

### Burn:

Attach device on USB cable, then run

    $ dmesg | grep tty

In necessary, modify scripts/burn_flash.sh and scripts/burn_flash_all.sh with the value of what dmesg returned.

Run ( once ) from adc_hello folder

    $ ../../scripts/burn_flash_all.sh

### Modify code, recompile and burn again ( as many times you want ):

Modify code, then run 

    $ make

then

    $ ../../scripts/burn_flash.sh
    
