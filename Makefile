MCU = atxmega32a4u
PROG_MCU = x32a4u
CC = avr-gcc
CFLAGS = -mmcu=$(MCU) -Wall -g -std=gnu99 -O2 -I .
PROGRAMMER = atmelice_pdi

OUTPUT = main
HEX_OUTPUT = $(OUTPUT).hex
SOURCES = ./main.c ./serial.c ./serial.h
OBJS = $(SOURCES:%.c=%.o)

$(HEX_OUTPUT): $(OUTPUT)
	avr-objcopy -O ihex $< $@

$(OUTPUT): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS)

%.o: %.c sw.h
	$(CC) $(CFLAGS) -c $<

clean:
	rm $(OUTPUT) $(HEX_OUTPUT) $(OBJS)

erase:
	avrdude -p $(PROG_MCU) -c $(PROGRAMMER) -P usb -e

program: $(HEX_OUTPUT)
	avrdude -p $(PROG_MCU) -c $(PROGRAMMER) -P usb -U $(HEX_OUTPUT)

debug: program
	avarice --pdi --dragon 127.0.0.1:4242

.PHONY: erase clean program
