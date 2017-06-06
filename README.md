# avr-fan-control

This is a repository for a little PC fan controller from an Arduino UNO. Currently it is only tested on an Atmega328p from an Arduino UNO board but I would like to expand it if other need it.
If you are stuck in a pinch and your PC fans won't run from your motherboard(the inspiration for this) feel free to use this code.

# All You Need

The Arduino cannot power the fans. This has to be done externally. If you are running this from a PC then you can pull a nice 12 V line from your PSU.
If not then you will need to get some.
The C code for the arduino is intended to be compiled with the WinAVR compiler here https://sourceforge.net/projects/winavr/files/
Install that and run the bat file rm_make_load to load to the board. You may need to change the active COM port in the script. 

The pde is a Processing 3 file for a GUI. Processing 3 can be downloaded here https://processing.org/download/
The two systems are meant to run together to provide simple fan control from your computer. If you open the pde in processing you can export the code as a standalone application. 

