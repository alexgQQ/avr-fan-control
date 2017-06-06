rm -f *.o main.elf main.hex main.map
make all
avrdude -F -V -c Arduino -p atmega328p -P COM3 -u -U flash:w:main.hex
pause