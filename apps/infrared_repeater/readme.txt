This is a simple infrared repeater. See the board.jpg for possible setup.
It records raw pulse fom an IR sensor connected to the GPIO 5 (D1 on NodeMCU board)
and repeats the signals to an IR LED connected to the GPIO 12(D6 on NodeMCU board). 
As the IR remote pulses are modulated at 38kHz or so, the IR sensor internally demodulates them so
when playing back to the IR led, they are modulated back at 38Hz.
It was tested with TSOP31238 and 38138 IR sensors, see the attached picture 'board.jpg'.


To setup the overall environment, including the compiler, use the root readme.txt

To setup the build environment after the overall env is installed, use

	source ../../env.sh

then compile using

	make

To burn onto ESP8266, run

	../../scripts/burn_flash.sh

To run minicom:

	sudo minicom --device=[/dev/ttyUSBx] -b115200

Replace /dev/ttyUSBx with the port the ESP8266 is attached to, can be found using 'dmesg' command.

Point the IR led towards the home appliance you want to control.
Point the remote to the IR sensor.
Press a key on the IR remote.
The device will repat the command within a second.
