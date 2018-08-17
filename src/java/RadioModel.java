/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
*/
/*
 * Copyright 2018 Saso Kiselkov. All rights reserved.
 */

public class RadioModel
{
	static { System.loadLibrary("RadioModel"); }

	public static native void init(String tile_path);
	public static native void fini();
	public static native int countBytes();

	public static native double pointToPoint(double freq_mhz,
	    double xmit_gain, double recv_min_gain,
	    double acf_lat, double acf_lon, double acf_elev,
	    double twr_lat, double twr_lon, double twr_elev);

	public static native void paintMap(double freq_mhz, double xmit_gain,
	    double recv_min_gain, double acf_elev, double twr_lat,
	    double twr_lon, double twr_elev, double max_terr_elev,
	    int pixel_size, String out_file);
}
