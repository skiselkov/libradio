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
    +00-080
    +00-070
    +00-060
    +00-050

    -10-080
    -10-070
    -10-060
    -10-050
    -10-040

    -20-080
    -20-070
    -20-060
    -20-050
    -20-040

    -30-080
    -30-070
    -30-060
    -30-050

    -40-080
    -40-070
    -40-060

    -50-080
    -50-070
    -50-050

    -60-080
    -60-070
    -60-060
    -60-040
)

for TILE in ${TILES[*]}; do
	gen_tile_par "$TILE" "$XPDIR" "$OUTDIR"
done

while [ $(jobs | wc -l) -gt 0 ]; do
	wait -n
done
