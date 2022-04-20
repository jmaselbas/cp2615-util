# SPDX-License-Identifier: BSD-2-Clause
SRC = cp2615-util.c
BIN = cp2615-util

# Flags
CFLAGS = -Wall -O2 `pkg-config --cflags libusb-1.0`
LDFLAGS = `pkg-config --libs libusb-1.0`

$(BIN): arg.h
all: $(BIN)


install_rules: 90-cp2615.rules
	cp 90-cp2615.rules /etc/udev/rules.d/

clean:
	rm $(BIN)
