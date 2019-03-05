#
# CDDL HEADER START
#
# The contents of this file are subject to the terms of the
# Common Development and Distribution License, Version 1.0 only
# (the "License").  You may not use this file except in compliance
# with the License.
#
# You can obtain a copy of the license in the file COPYING
# or http://www.opensource.org/licenses/CDDL-1.0.
# See the License for the specific language governing permissions
# and limitations under the License.
#
# When distributing Covered Code, include this CDDL HEADER in each
# file and include the License file COPYING.
# If applicable, add the following below this CDDL HEADER, with the
# fields enclosed by brackets "[]" replaced with your own identifying
# information: Portions Copyright [yyyy] [name of copyright owner]
#
# CDDL HEADER END
#
# Copyright 2019 Saso Kiselkov. All rights reserved.
#

CC = x86_64-w64-mingw32-gcc
CXX = $(CC)
LD = $(CC)

OBJS=../distort.o ../kiss_fft.o
CFLAGS += -fvisibility=hidden -DIBM=1
LDFLAGS += -fvisibility=hidden -DIBM=1

all : distort.dll

clean :
	rm -f distort.dll $(OBJS)

distort.dll : $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -shared -o $@ $^ $(LIBS)
