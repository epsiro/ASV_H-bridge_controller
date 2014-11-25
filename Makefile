DEVICE = atxmega32a4
AVRDUDE = avrdude -p x32a4 -c jtag2pdi -P usb
#F_CPU=32000000UL
F_CPU=8000000UL

CC = avr-gcc
CFLAGS = -g -W -Wall -O2 -std=gnu99 -mmcu=$(DEVICE) -DF_CPU=$(F_CPU) -lm -pedantic

OBJECTS = main.o

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

all:	main.hex

#fuse:
#	$(AVRDUDE) -U hfuse:w:0xd8:m -U lfuse:w:0xef:m

clean:
	rm -f *.hex *.lst *.o *.bin

$(OBJECTS): | depend

main.bin: $(OBJECTS)
	$(CC) $(CFLAGS) -o main.bin $(OBJECTS)

main.hex: main.bin
	avr-objcopy -j .text -j .data -O ihex main.bin main.hex
	avr-size --totals *.o
	#avr-size -C --mcu=$(DEVICE) *.o

read:
	$(AVRDUDE) -U eeprom:r:eeprom.dat:r
	hd eeprom.dat

depend:
	@$(CC) -MM $(ALL_CFLAGS) *.c | sed 's/$$/ Makefile/'
