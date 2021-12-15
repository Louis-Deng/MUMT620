# MUMT620

Playable Digital Effect (Arduino/C++/Serial Messaging/Synthesis Toolkit) by Louis Deng, for MUMT620 Final Project

REQUIRES STK TO COMPILE

https://github.com/thestk/stk

Software codes only...

circuit diagram to be added...

for macOS, you need to BUILD using this command line in the directory holding mumt620.cpp

g++ -g ~/mumt620.cpp -o ~/mumt620.o -Istk/include/ -Lstk/src/ -D__MACOSX_CORE__ -lstk -lpthread -framework CoreAudio -framework CoreMIDI -framework CoreFoundation

supposed your directory contains: 

mumt620.cpp

stk (folder)

Otherwise, run the executable: 

./mumt620.o arg1

'arg1' being the serial port address, you can check this using

ls /dev/tty.usb.*

and use whatever * is for the first argument when you run mumt620.o


