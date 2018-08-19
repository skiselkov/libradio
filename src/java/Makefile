# CDDL HEADER START
#
# This file and its contents are supplied under the terms of the
# Common Development and Distribution License ("CDDL"), version 1.0.
# You may only use this file in accordance with the terms of version
# 1.0 of the CDDL.
#
# A full copy of the text of the CDDL should have accompanied this
# source.  A copy of the CDDL is also available via the Internet at
# http://www.illumos.org/license/CDDL.
#
# CDDL HEADER END
#
# Copyright 2018 Saso Kiselkov. All rights reserved.
#

CLASSES = \
    RadioModel.class

OBJS = \
    RadioModel.o \
    ../itm_c.o \
    ../itm.o

DEFINES = -DIBM=0 -DLIN=1 -DAPL=0

CFLAGS = \
    $(DEFINES) -O3 -g3 \
    -I.. \
    -I$(ACFUTILS)/src \
    -I$(ACFUTILS)/glew \
    -I$(JAVA_INCLUDE_DIR) -I$(JAVA_INCLUDE_DIR)/linux \
    -fPIC \
    $(shell $(ACFUTILS)/pkg-config-deps linux-64 --cflags) \
    -W -Wall -Wextra -Werror -Wno-unused-local-typedefs -Wunused-result

CXXFLAGS = $(DEFINES) -fPIC -O3 -g3

LIBS = -L$(ACFUTILS)/qmake/lin64 -lacfutils -lm -lpthread -static-libstdc++ \
    -static-libgcc \
    $(shell $(ACFUTILS)/pkg-config-deps linux-64 --libs)

all : $(CLASSES) lib/libRadioModel.so
zip : RadioModel.zip

RadioModel.zip : $(CLASSES) lib/libRadioModel.so data
	rm -f RadioModel.zip
	zip -r RadioModel.zip $(CLASSES) $(patsubst %.class,%.java,$(CLASSES)) \
	    lib/libRadioModel.so data

lib/libRadioModel.so : $(OBJS)
	mkdir -p lib
	$(CC) $(CFLAGS) $(LDFLAGS) -shared -o $@ \
	    -Wl,-soname,RadioModel $^ $(LIBS)

%.class : %.java
	javac $^
	javah $(basename $^)

clean :
	rm -f $(OBJS) $(CLASSES) lib/libRadioModel.so
