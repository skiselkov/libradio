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
# Copyright 2019 Saso Kiselkov. All rights reserved.
#

set -e

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

DEMDIR="$1"
if [[ "$DEMDIR" = "" ]]; then
	echo "Need path to dem_dsf_extract repo as first arg" >&2
	exit 1
fi
shift

XPDIR="$1"
if [[ "$XPDIR" = "" ]]; then
	echo "Need XPDIR as second arg" >&2
	exit 1
fi
shift

OUTDIR="$1"
if [[ "$OUTDIR" = "" ]]; then
	echo "Need OUTDIR as third arg" >&2
	exit 1
fi
shift

STITCH="$DEMDIR/dem_stitch.sh"
if ! [ -x "$STITCH" ]; then
	echo "$STITCH not found or not executable" >&2
	exit 1
fi

RESOLUTION="$1"
if [[ "$RESOLUTION" = "" ]]; then
	RESOLUTION=1024
fi

mkdir -p "$OUTDIR"
cd "$OUTDIR"

for TILE in ${TILES[*]}; do
	TILEPATH="$XPDIR/Global Scenery/X-Plane 11 Global Scenery/Earth nav data/$TILE"
	echo "$TILEPATH"
	if ! [ -d "$TILEPATH" ]; then
		TILEPATH="$XPDIR/Global Scenery/X-Plane 11 Demo Areas/Earth nav data/$TILE"
		if ! [ -d "$TILEPATH" ]; then
			echo "Tile $TILE not found in XP install" >&2
			exit 1
		fi
	fi
	"$STITCH" -sr "$RESOLUTION" "$TILEPATH"
	WATERTILE="$XPDIR/Resources/map data/water/$TILE"
	if [ -d "$WATERTILE" ]; then
		cp -r "$WATERTILE" .
	fi
done
