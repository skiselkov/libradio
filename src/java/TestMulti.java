import java.util.Scanner;
import java.util.regex.*;
import java.util.*;

import com.vspro.util.RadioModel;

public class TestMulti {
    static double ft2m(double ft)
    {
	return (ft / 3.281);
    }

    static double m2ft(double ft)
    {
	return (ft * 3.281);
    }
    static int count_freqs(ArrayList<Double> freqs, double freq,
	ArrayList<String> names)
    {
	int count = 0;
	for (int i = 0; i < freqs.size(); i++) {
		double f = freqs.get(i).doubleValue();
		if (Math.abs(f - freq) < 0.01) {
			System.out.println("Selected " + names.get(i));
			count++;
		}
	}
	return (count);
    }

    public static void main(String[] args)
    {
	Scanner input = new Scanner(System.in);
	double recv_min_gain = -100;
	String[] type_regex = {
	    ".*_CD",
	    ".*_GC",
	    ".*_LC",
	    "Z[A-Z][A-Z]_.*"
	};
	double[] deg_ranges = {
	    0.5,
	    0.5,
	    3,
	    7
	};
	double[] test_hgts = {
	    10,
	    10,
	    3000,
	    10000
	};
	int[] spacing = {
	    200,
	    200,
	    0,
	    0
	};
	ArrayList<String> names = new ArrayList<String>();
	ArrayList<Double> freqs = new ArrayList<Double>();
	ArrayList<Double> lats = new ArrayList<Double>();
	ArrayList<Double> lons = new ArrayList<Double>();
	ArrayList<Double> elevs = new ArrayList<Double>();
	boolean agl = false;
	double test_freq = 124.62;
	double test_elev = ft2m(1000);
	double xmit_power = 60;
	int test_type = 3;
	int resolution = 1024;
	Pattern pat;
	String out_dir = ".";
	String data_dir = "data";
	double rng_boost = 5;

	for (int i = 0, n = args.length; i < n; i++) {
		if (args[i].equals("-freq") && i + 1 < n) {
			i++;
			test_freq = Double.parseDouble(args[i]);
		} else if (args[i].equals("-data")) {
			i++;
			data_dir = args[i];
		} else if (args[i].equals("-agl")) {
			agl = true;
		} else if (args[i].equals("-type") && i + 1 < n) {
			i++;
			test_type = Integer.parseInt(args[i]);
		} else if (args[i].equals("-resolution") && i + 1 < n) {
			i++;
			resolution = Integer.parseInt(args[i]);
		} else if (args[i].equals("-elev") && i + 1 < n) {
			i++;
			test_elev = Double.parseDouble(args[i]);
		} else if (args[i].equals("-power") && i + 1 < n) {
			i++;
			xmit_power = Double.parseDouble(args[i]);
		} else if (args[i].equals("-outdir") && i + 1 < n) {
			i++;
			out_dir = args[i];
		} else if (args[i].equals("-rngboost") && i + 1 < n) {
			i++;
			rng_boost = Double.parseDouble(args[i]);
		} else {
			System.out.println("Unknown argument " + args[i]);
			return;
		}
	}

	pat = Pattern.compile(type_regex[test_type]);

	RadioModel.init(data_dir, spacing[test_type], 0, 0);
	System.out.println("Memory consumed: " +
	    (RadioModel.countBytes() >> 10) + " kB");

	while (input.hasNext()) {
		String line = input.nextLine();
		String[] comps;
		String name;

		if (line.length() == 0 || line.charAt(0) == '#')
			continue;
		comps = line.split(":");
		if (comps.length < 5)
			continue;
		name = comps[0];

		if (pat.matcher(name).matches()) {
			double freq = Double.parseDouble(comps[2]) / 100.0;
			double lat = Double.parseDouble(comps[3]);
			double lon = Double.parseDouble(comps[4]);
			double elev = ft2m(Double.parseDouble(comps[5]));

			/*
			 * Skip invalid entries.
			 */
			if (lon < -180 || lon > 180 || lat < -90 || lat > 90 ||
			    elev < -450 || elev > 9000 || freq > 150)
				continue;

			names.add(name);
			freqs.add(freq);
			lats.add(lat);
			lons.add(lon);
			elevs.add(elev + 50);
			if (test_type != 3) {
				long start, end;
				double d_t;

				start = System.currentTimeMillis();
				RadioModel.paintMap(freq, true, xmit_power,
				    recv_min_gain,
				    elev + ft2m(test_hgts[test_type]), false,
				    lat, lon, elev + 50, deg_ranges[test_type],
				    resolution, "out/" + name + ".png");
				end = System.currentTimeMillis();
				d_t = (end - start) / 1000.0;
				System.out.format(
				    "took: %.1f secs, %d samples/sec\n", d_t,
				    (int)((resolution * resolution) / d_t));
			}
		}
	}

	if (test_type == 3) {
		int num_navaids = count_freqs(freqs, test_freq, names);
		double[] lats_arr = new double[num_navaids];
		double[] lons_arr = new double[num_navaids];
		double[] elevs_arr = new double[num_navaids];
		double min_lat = 90, max_lat = -90, min_lon = 180,
		    max_lon = -180;
		double ctr_lat, ctr_lon, deg_range;
		String agl_flag = (agl ? "_agl" : "_msl");
		String filename;
		long start, end;
		double d_t;

		for (int i = 0, j = 0; i < freqs.size(); i++) {
			double f = freqs.get(i).doubleValue();
			if (Math.abs(f - test_freq) < 0.01) {
				lats_arr[j] = lats.get(i).doubleValue();
				lons_arr[j] = lons.get(i).doubleValue();
				elevs_arr[j] = elevs.get(i).doubleValue();

				min_lat = Math.min(min_lat, lats_arr[j]);
				max_lat = Math.max(max_lat, lats_arr[j]);
				min_lon = Math.min(min_lon, lons_arr[j]);
				max_lon = Math.max(max_lon, lons_arr[j]);
				j++;
			}
		}
		ctr_lat = (max_lat + min_lat) / 2;
		ctr_lon = (max_lon + min_lon) / 2;
		deg_range = Math.max(max_lat - min_lat, max_lon - min_lon);
		if (deg_range <= rng_boost)
			deg_range = rng_boost;
		else
			deg_range += rng_boost;

		System.out.println("ctr: " + ctr_lat + "  lon: " + ctr_lon +
		    "  range: " + deg_range + "  num: " + num_navaids);

		filename = String.format(out_dir + "/test_%.02f_%.0f%s.png",
		    test_freq, test_elev, agl_flag);

		start = System.currentTimeMillis();
		RadioModel.paintMapMulti(130, true, xmit_power,
		    recv_min_gain, test_elev, true,
		    lats_arr, lons_arr, elevs_arr, deg_range,
		    ctr_lat, ctr_lon, resolution, filename);
		end = System.currentTimeMillis();
		d_t = (end - start) / 1000.0;
		System.out.format("took: %.1f secs, %d samples/sec\n", d_t,
		    (int)((resolution * resolution * num_navaids) / d_t));
	}

	RadioModel.fini();
    }
}
