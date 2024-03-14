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

REALPATH="realpath"

while getopts "hr:t:" opt; do
	case "$opt" in
	    h)
		echo "$0 [-r RESOLUTION] [-t <maxthreads>] <path_to_dsf_dem_extract_repo>" \
		    "<xpdir> <outdir>"
		exit
		;;
	    r)
		RESOLUTION="$OPTARG"
		;;
	    t)
		THRLIM="$OPTARG"
		;;
	    ?)
		echo "Invalid option: -${OPTARG}. Try \"$0 -h\" for help" >&2
		exit 1
		;;
	esac
done

for (( I=0; I + 1 < "$OPTIND"; I++ )); do
	shift
done

DEMDIR="$1"
if [ -z "$DEMDIR" ]; then
	echo "Need path to dsf_dem_extract repo as first arg" >&2
	exit 1
fi
shift
DEMDIR="$("$REALPATH" "$DEMDIR")"

XPDIR="$1"
if [ -z "$XPDIR" ]; then
	echo "Need XPDIR as second arg" >&2
	exit 1
fi
shift
XPDIR="$("$REALPATH" "$XPDIR")"

OUTDIR="$1"
if [ -z "$OUTDIR" ]; then
	echo "Need OUTDIR as third arg" >&2
	exit 1
fi
shift
OUTDIR="$("$REALPATH" "$OUTDIR")"

STITCH="$DEMDIR/dem_stitch.sh"
if ! [ -x "$STITCH" ]; then
	echo "$STITCH not found or not executable" >&2
	exit 1
fi
STITCH="$("$REALPATH" "$STITCH")"

if [ -z "$RESOLUTION" ]; then
	RESOLUTION=1024
fi

mkdir -p "$OUTDIR"
cd "$OUTDIR"

if [[ "$(uname)" = "Darwin" ]]; then
	NCPUS=$(( $(sysctl -n hw.ncpu) + 1 ))
else
	NCPUS=$(( $(grep 'processor[[:space:]]\+:' /proc/cpuinfo  | wc -l) + \
	    1 ))
fi

if [ -n "$THRLIM" ]; then
	if [ "$NCPUS" -gt "$THRLIM" ]; then
		NCPUS="$THRLIM"
	fi
fi

function gen_tile() {
	TILE="$1"
	TILEPATH="$XPDIR/Global Scenery/X-Plane 12 Global Scenery/Earth nav data/$TILE"
	if ! [ -d "$TILEPATH" ]; then
		TILEPATH="$XPDIR/Global Scenery/X-Plane 12 Demo Areas/Earth nav data/$TILE"
	fi
	if ! [ -d "$TILEPATH" ]; then
		TILEPATH="$XPDIR/Global Scenery/X-Plane 11 Global Scenery/Earth nav data/$TILE"
	fi
	if ! [ -d "$TILEPATH" ]; then
		TILEPATH="$XPDIR/Global Scenery/X-Plane 11 Demo Areas/Earth nav data/$TILE"
	fi
	if ! [ -d "$TILEPATH" ]; then
		echo "Tile $TILE not found in XP install" >&2
		exit 1
	fi
	echo "$TILEPATH"
	"$STITCH" -sr "$RESOLUTION" "$TILEPATH"
	WATERTILE="$XPDIR/Resources/map data/water/$TILE"
	if [ -d "$WATERTILE" ]; then
		cp -r "$WATERTILE" .
	fi
}

function gen_tile_par()
{
	while (( $(jobs | wc -l) > $NCPUS )); do
		wait -n
	done
	TILE="$1"
	XPDIR="$2"
	OUTDIR="$3"
	gen_tile "$TILE" "$XPDIR" "$OUTDIR" &
}
