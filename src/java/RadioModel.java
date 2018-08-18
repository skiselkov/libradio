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

	/*
	 * Initializes the radio model and loads the terrain data. This must
	 * be called before sending any queries to the radio model. Once
	 * initialized, the library can be called from any number of parallel
	 * threads. No locking is required, as once the initialized, the
	 * radio model is immutable (until it is de-initialized using
	 * RadioModel.fini()).
	 *
	 * @param data_dir Path to a directory containing Earth orbit texture
	 *	normal data extracted from X-Plane. This must be assembled as
	 *	follows:
	 *	1) at the top level of the data directory are individual
	 *	   normal map files, name for example: "+30-110-nrm.png".
	 *	   The normal maps can use any internal resolution, not just
	 *	   the stock 1024x1024 shipped in X-Plane.
	 *	2) For each normal map, there can be an optional subdirectory
	 *	   containing water mask shape files. The subdirectory needs
	 *	   to have the same name as the normal map (e.g. "+30-110").
	 *	3) Inside of each water mask directory are 1-degree-sized
	 *	   water mask shape files (also extracted from X-Plane).
	 *	   One tile will typically consist of two files named e.g.
	 *	   "+34-102.shx" and "+34-102.shp".
	 * @param spacing When constructing a terrain relief between two
	 *	radio endpoints, libradio will space the points `spacing'
	 *	meters apart. The denser the points, the more accurate the
	 *	reception model will be, but the longer it takes to calculate.
	 *	If you specify `0' for this parameter, the default is 500.
	 * @param max_pts Maximum number of relief points when the distance
	 *	between transmitter and receiver is very large. The library
	 *	will simply increase spacing between points to keep the
	 *	number of relief points at this number and limit excessive
	 *	amounts of CPU used. If you specify `0' for this parameter,
	 *	the default is 500.
	 */
	public static native void init(String data_dir, int spacing,
	    int max_pts);

	/*
	 * When you are done with the radio model and wish to unload it
	 * from memory, you must call this method. Please note that the
	 * RadioModel is a C library, so the Java garbage collector doesn't
	 * automatically manage the radio model for you. You need to
	 * explicitly dispose of it when you don't need it anymore.
	 * It is safe to call this method multiple times, the radio model
	 * will automatically be deinitialized only once.
	 */
	public static native void fini();

	/*
	 * Memory usage estimator for diagnostic purposes. Returns the
	 * approximate amount of memory (in bytes) used to store the
	 * height maps and water masks.
	 */
	public static native int countBytes();

	/*
	 * Main `worker' method of the model. You call this function when
	 * you want to compute the radio reception level between two points
	 * in space.
	 *
	 * @param freq_mhz The frequency (in MHz!) of the radio signal
	 *	being propagated.
	 * @param horiz_pol Flag indicating whether the signal is
	 *	horizontally or vertically polarized.
	 */
	public static native double pointToPoint(double freq_mhz,
	    boolean horiz_pol, double xmit_gain, double recv_min_gain,
	    double acf_lat, double acf_lon, double acf_elev,
	    double twr_lat, double twr_lon, double twr_elev,
	    boolean itm_relaxed);

	public static native void paintMapMulti(double freq_mhz,
	    boolean horiz_pol, double xmit_gain, double recv_min_gain,
	    double acf_elev, double twr_lats[], double twr_lons[],
	    double twr_elevs[], double deg_range, double ctr_lat,
	    double ctr_lon, int pixel_size, String out_file);

	public static void paintMap(double freq_mhz, boolean horiz_pol,
	    double xmit_gain, double recv_min_gain, double acf_elev,
	    double twr_lat, double twr_lon, double twr_elev,
	    double deg_range, int pixel_size, String out_file)
	{
		double[] lats = { twr_lat };
		double[] lons = { twr_lon };
		double[] elevs = { twr_elev };

		paintMapMulti(freq_mhz, horiz_pol, xmit_gain, recv_min_gain,
		    acf_elev, lats, lons, elevs, deg_range, twr_lat,
		    twr_lon, pixel_size, out_file);
	}

	public static native double elevProbe(double lat, double lon);
}
