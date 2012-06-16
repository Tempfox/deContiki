OUTFILE=deRFnode

ifndef TARGET
  TARGET:=deRFmega128_deRFnode_2XXX2
endif
  
ifeq ($(TARGET), deRFmega128_deRFnode_2XXX2)
   MCU=atmega128rfa1
   ${info TARGET=deRFmega128_deRFnode_2XXX2, MCU=atmega128rfa1}
else ifeq ($(TARGET), deRFmega128_deRFtoRCB)
   MCU=atmega128rfa1
   ${info TARGET=deRFmega128_deRFtoRCB, MCU=atmega128rfa1}
else
   ${error ERROR: unknown TARGET}
endif

JTAGICEMKII="C:/Program Files (x86)/Atmel/AVR Tools/JTAGICEmkII/jtagiceii.exe"

all: 
	${MAKE} -f Makefile.$(TARGET) TARGET=$(TARGET) NOAVRSIZE=1 $(OUTFILE).elf
	avr-objcopy -O ihex -R .eeprom $(OUTFILE).elf $(OUTFILE).hex
	avr-size -C --mcu=$(MCU) $(OUTFILE).elf

#dumpDefs:
#	$(CC) $(CFLAGS) -dM -E - < RESTserver.c 1>defines.txt

clean:
	${MAKE} -f Makefile.$(TARGET) TARGET=$(TARGET) clean
	rm -f symbols.c symbols.h *.elf *.hex
	rm -rf obj_$(TARGET)
	
flash:
	$(JTAGICEMKII) -s -c usb -d $(MCU)
#	$(JTAGICEMKII) -R -z -e -c usb -d $(MCU) -pf -vf -if $(OUTFILE).hex
	$(JTAGICEMKII) -R -z -e -c usb -d $(MCU) -pa -va -ia $(OUTFILE).elf	