.PHONY: all clean
ENABLE_USB=1
CFLAGS = -g -O3 --std=gnu99 --pedantic
OBJECTS=main.o byte_utils.o ihex.o stm8.o

ifeq ($(ENABLE_USB),1)
OBJECTS+=usb.o stlink.o stlinkv2.o 
CFLAGS+=-DENABLE_USB `pkg-config --cflags libusb-1.0` 
LIBS+= `pkg-config --libs libusb-1.0`
endif

PLATFORM=$(shell uname -s)
ifeq ($(PLATFORM),Darwin)
  MacOSSDK=`xcrun --show-sdk-path`
  CFLAGS += -I$(MacOSSDK)/usr/include/ -I$(MacOSSDK)/usr/include/sys -I$(MacOSSDK)/usr/include/machine
endif
BIN = stm8flash

all: $(OBJECTS)
	$(CC) $(OBJECTS) $(LIBS) -o $(BIN)

clean:
	-rm -f $(OBJECTS) $(BIN)

install:
	cp $(BIN) $(DESTDIR)/usr/bin/
