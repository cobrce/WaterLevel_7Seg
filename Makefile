# --
# Copyright (c) 2016, Lukasz Marcin Podkalicki <lpodkalicki@gmail.com>
# --

MCU=atmega16
FUSE_L=0xE4
FUSE_H=0xD7
F_CPU=8000000
CC=avr-g++
LD=avr-ld
OBJCOPY=avr-objcopy
SIZE=avr-size
AVRDUDE=avrdude
CFLAGS=-std=c++0x -Wall -g -Os  -gdwarf-2 -mmcu=${MCU} -DF_CPU=${F_CPU} -I. -Ivl53l0x-non-arduino/util
TARGET=main

SRCS = main.cpp  $(wildcard vl53l0x-non-arduino/*.c) $(wildcard vl53l0x-non-arduino/util/*.c)



all:
	${CC} ${CFLAGS} -o ${TARGET}.o ${SRCS}
	${LD} -o ${TARGET}.elf ${TARGET}.o
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.o ${TARGET}.hex
	${OBJCOPY} -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex ${TARGET}.elf ${TARGET}.eep
	${SIZE} -C --mcu=${MCU} ${TARGET}.elf

flash: all
	${AVRDUDE} -p ${MCU} -c stk500v1 -P COM5 -b 115200 -V -U flash:w:${TARGET}.hex:i -U eeprom:w:${TARGET}.eep:a -F

fuse:
	$(AVRDUDE) -p ${MCU} -c stk500v1 -P COM5 -b 115200 -U hfuse:w:${FUSE_H}:m -U lfuse:w:${FUSE_L}:m

clean:
	rm -f *.c~ *.o *.elf *.hex
