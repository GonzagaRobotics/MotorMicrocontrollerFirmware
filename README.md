# Motor Microcontroller Firmware
## Serial Communications Branch

## Introduction
Provides a control interface for moving the drive motors on the robot, enabling it to drive around.

## Uploading

Install PlatformIO. You can try getting it from the Ubuntu repo's, but I've found the Python environmnents works better if you follow the recommended way of installing it, the install script. You can find a guide [here](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html).

Then, ensuring nothing is currently reading from the serial output of the esp32, run `pio run --target upload` in the root directory of this repository.

## Interfacing

Connect your laptop/desktop to the esp32. You can either use the Arduino IDE or the command line. 

For the Arduino IDE, ensure the board is recognized, then open up Serial Monitor (often bound to Control-Shift-m).

For command line, determine where the esp32 is setup. For UNIX systems (e.g. MacOS, Linux), it's in `/dev/`, likely `/dev/ttyUSB0`. To send commands, run `echo TEXT TO SEND HERE > /dev/ttyUSB0`, replacing `/dev/ttyUSB0` with your device. You can read serial sent back for debugging with `cat /dev/ttyUSB0`.

Commands are of the form TODO ADD THE FORM HERE
