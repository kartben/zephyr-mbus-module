#
# Copyright (c) 2017 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0
#

PREFIX   ?= .
LIB_DIR  ?= $(PREFIX)/

CPPFLAGS ?= -Iinclude -MD
OBJS      = src/mbus.o src/mbus-protocol.o src/mbus-protocol-aux.o src/mbus-serial.o
#src/mbus-tcp.o
#HEADERS   = include/mbus.h include/mbus-protocol.h
#include/mbus-protocol-aux.h
#include/mbus-serial.h include/mbus-tcp.h
AR       ?= ar
ARFLAGS   = -crus
LIBNM     = libmbus.a
LIB       = $(LIBNM)($(OBJS))

all: $(LIB)
	cp $(LIBNM) $(LIB_DIR)/

clean:
	rm -rf $(OBJS) $(LIB)
