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
# Copyright 2023 Saso Kiselkov. All rights reserved.
#

CLASSES = \
    com/vspro/util/RadioModel.class

OBJS = \
    RadioModel.o \
    ../itm_c.o \
    ../itm.o

LDFLAGS = -Wl,--exclude-libs,ALL -fvisibility=hidden

ifneq ($(win32),yes)

# Linux

CC=gcc
CXX=g++

DEFINES = -DIBM=0 -DLIN=1 -DAPL=0 -DRM_EXPORT -D_LACF_WITHOUT_XPLM -D_GNU_SOURCE

CFLAGS = -std=c99 \
    $(DEFINES) -O3 -g3 \
    -I.. \
    -I$(ACFUTILS)/src \
    -I$(ACFUTILS)/glew \
    -I$(JAVA_INCLUDE_DIR) -I$(JAVA_INCLUDE_DIR)/linux \
    -fPIC \
    $(shell $(ACFUTILS)/pkg-config-deps linux-64 --cflags) \
    -W -Wall -Wextra -Werror -Wno-unused-local-typedefs -Wunused-result \
    -fvisibility=hidden -DLIBEXPORT=''

CXXFLAGS = $(DEFINES) -fPIC -O3 -g3 -fvisibility=hidden -DLIBEXPORT=''

LIBS = -L$(ACFUTILS)/qmake/lin64 -lacfutils \
    $(shell $(ACFUTILS)/pkg-config-deps linux-64 --libs) \
    -lm -lpthread -static-libstdc++ -static-libgcc

RADIOLIB=lib/libRadioModel.so

else

# Windows

CC=x86_64-w64-mingw32-gcc
CXX=x86_64-w64-mingw32-g++

DEFINES = -DIBM=1 -DLIN=0 -DAPL=0 -DRM_EXPORT='__declspec(dllexport)'

CFLAGS = \
    $(DEFINES) -O3 -g3 \
    -I.. \
    -I$(ACFUTILS)/src \
    -I$(ACFUTILS)/glew \
    -I$(JAVA_INCLUDE_DIR) -I$(JAVA_INCLUDE_DIR)/linux \
    $(shell $(ACFUTILS)/pkg-config-deps win-64 --cflags) \
    -W -Wall -Wextra -Werror -Wno-unused-local-typedefs -Wunused-result \
    -fvisibility=hidden -DLIBEXPORT='__declspec(dllexport)'

CXXFLAGS = $(DEFINES) -O3 -g3 -fvisibility=hidden \
    -DLIBEXPORT='__declspec(dllexport)'

LIBS = -L$(ACFUTILS)/qmake/win64 -lacfutils \
    $(shell $(ACFUTILS)/pkg-config-deps win-64 --libs) \
    $(shell x86_64-w64-mingw32-g++ -print-file-name=libstdc++.a) \
    -lm -static-libstdc++ -static-libgcc

RADIOLIB=lib/RadioModel.dll

endif

all : $(CLASSES) $(RADIOLIB)
zip : RadioModel.zip

RadioModel.zip : $(CLASSES) $(RADIOLIB) data
	rm -rf RadioModel.zip doc
	( mkdir doc && cd doc && \
	    javadoc $(addprefix ../,$(patsubst %.class,%.java,$(CLASSES))) )
	zip -r RadioModel.zip $(CLASSES) $(patsubst %.class,%.java,$(CLASSES)) \
	    $(wildcard lib/*RadioModel.*) data doc

lib/libRadioModel.so : $(OBJS)
	mkdir -p lib
	$(CXX) $(CFLAGS) $(LDFLAGS) -shared -o $@ \
	    -Wl,-soname,RadioModel $^ $(LIBS)

lib/RadioModel.dll : $(OBJS)
	mkdir -p lib
	$(CC) $(CFLAGS) $(LDFLAGS) -fno-stack-protector \
	    -shared -o $@ -Wl,-soname,RadioModel $^ $(LIBS)

%.class : %.java
	javac $^
	javah $(subst /,.,$(basename $^))
	sed -i 's/JNIEXPORT/LIBEXPORT JNIEXPORT/g' \
	    $(subst /,_,$(patsubst %.java,%.h,$^))

clean :
	rm -f $(OBJS) $(CLASSES) $(wildcard lib/*RadioModel.*) RadioModel.zip
