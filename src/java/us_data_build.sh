#!/bin/bash
#
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
# Copyright 2024 Saso Kiselkov. All rights reserved.
#

set -e

source build.common.inc

TILES=(
    +20-090
    +20-100
    +20-110
    +30-080
    +30-090
    +30-100
    +30-110
    +30-120
    +30-130
    +40-070
    +40-080
    +40-090
    +40-100
    +40-110
    +40-120
    +40-130
)

for TILE in ${TILES[*]}; do
	gen_tile_par "$TILE" "$XPDIR" "$OUTDIR"
done

while [ $(jobs | wc -l) -gt 0 ]; do
	wait -n
done
