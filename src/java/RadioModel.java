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
	 * @param max_dist To optimize and speed up lookups for stations that
	 *	are too far apart to ever see each other, this parameter
	 *	the station distance (in meters) at which point the radio
	 *	model simply returns minimum receiver gain. If you set this
	 *	parameter to `0', the default value is 600 km. The minimum
	 *	possible value for this parameter is 2km and the maximum
	 *	is 1000 km.
	 */
	public static native void init(String data_dir, int spacing,
	    int max_pts, int max_dist);

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
	 * in space. All positions are specified as geodesic coordinates
	 * (degrees latitude x longitude x meters absolute elevation on the
	 * WGS-84 ellipsoid). Please note that there are no strict
	 * transmitter/receiver roles between the two stations (the laws of
	 * physics are the same no matter which direction you are traveling!).
	 *
	 * @param freq_mhz The frequency (in MHz!) of the radio signal
	 *	being propagated.
	 * @param horiz_pol Flag indicating whether the signal is
	 *	horizontally or vertically polarized. Most signals used
	 *	in aviation (with the exception of DME) use horizontal
	 *	polarization.
	 * @param xmit_gain A base transmitter gain value indicating
	 *	approximate signal strength in dB. Typical values would
	 *	be +40 for a low power transmitter, +50 for medium
	 *	power, +60 for a powerful transmitter and +80 for a very
	 *	powerful transmitter.
	 * @param recv_min_gain A minimum dB gain value before the model
	 *	considers the signal too low power to receive at all.
	 *	These range typically from -70 to -100. These values are
	 *	very transmission method dependent and so need to be
	 *	determined experimentally.
	 * @param sta1_lat Station 1 position latitude.
	 * @param sta1_lon Station 1 position longitude.
	 * @param sta1_elev Station 1 absolute elevation.
	 * @param sta2_lat Station 2 position latitude.
	 * @param sta2_lon Station 2 position longitude.
	 * @param sta2_elev Station 2 absolute elevation.
	 *
	 * @return Signal level (in dB) at the receiver side (whichever
	 *	of the two stations that may be). This will typically be
	 *	a negative value ranging from approx 0 dB to recv_min_gain
	 *	(e.g. -100). If the station is determined to be too far to
	 *	ever be able to be received, this method simply returns
	 *	recv_min_gain as a more efficient shortcut.
	 */
	public static native double pointToPoint(double freq_mhz,
	    boolean horiz_pol, double xmit_gain, double recv_min_gain,
	    double sta1_lat, double sta1_lon, double sta1_elev,
	    double sta2_lat, double sta2_lon, double sta2_elev);

	/*
	 * Paints a receive signal level intensity map for a receiver moving
	 * among potentially multiple transmitters. This is useful for
	 * estimating dead spots in an area of interest. Most of the arguments
	 * are similar to the pointToPoint method. The notable difference is
	 * that here the point-to-point model is run for every pixel of the
	 * rendered image against each of the sets of fixed stations.
	 *
	 * @param freq_mhz The frequency (in MHz!) of the radio signal
	 *	being propagated.
	 * @param horiz_pol Flag indicating whether the signal is
	 *	horizontally or vertically polarized.
	 * @param xmit_gain A base transmitter gain value indicating
	 *	approximate signal strength in dB.
	 * @param recv_min_gain A minimum dB gain value before the model
	 *	considers the signal too low power to receive at all.
	 * @param sta1_elev Absolute elevation of the "moving" station in
	 *	meters. The method automatically changes the station's
	 *	latitude x longitude to pass every part of the rendered image.
	 * @param sta2_lats An array of latitudes for the "fixed" stations.
	 * @param sta2_lons An array of longitudes for the "fixed" stations.
	 * @param sta2_elevs An array of elevations for the "fixed" stations.
	 * @param deg_range Size of the rendered image in degrees of
	 *	geographic coordinates. The map is square-shaped and uses
	 *	the Mercator projection.
	 * @param ctr_lat Latitude of the centerpoint of the rendered map.
	 * @param ctr_lon Longitude of the centerpoint of the rendered map.
	 * @param pixel_size Pixel resolution of the rendered map. Please
	 *	note that settings this very can significantly slow down
	 *	map painting, so it's not recommended to use map sizes much
	 *	above 1024 or 2048.
	 * @param out_file Output PNG image file name.
	 */
	public static native void paintMapMulti(double freq_mhz,
	    boolean horiz_pol, double xmit_gain, double recv_min_gain,
	    double sta1_elev, double sta2_lats[], double sta2_lons[],
	    double sta2_elevs[], double deg_range, double ctr_lat,
	    double ctr_lon, int pixel_size, String out_file);

	/*
	 * Shorthand version of paintMapMulti where only a single fixed
	 * station is being considered. The map is centered on station 2.
	 */
	public static void paintMap(double freq_mhz, boolean horiz_pol,
	    double xmit_gain, double recv_min_gain, double sta1_elev,
	    double sta2_lat, double sta2_lon, double sta2_elev,
	    double deg_range, int pixel_size, String out_file)
	{
		double[] lats = { sta2_lat };
		double[] lons = { sta2_lon };
		double[] elevs = { sta2_elev };

		paintMapMulti(freq_mhz, horiz_pol, xmit_gain, recv_min_gain,
		    sta1_elev, lats, lons, elevs, deg_range, sta2_lat,
		    sta2_lon, pixel_size, out_file);
	}

	/*
	 * Elevation model probing function. Useful for comparing terrain
	 * model accuracy with expected station position data.
	 *
	 * @param lat Latitude of the probe point.
	 * @param lon Longitude of the probe point.
	 *
	 * @return Local elevation at the probe point in meters AMSL.
	 */
	public static native double elevProbe(double lat, double lon);

	static { System.loadLibrary("RadioModel"); }
}
