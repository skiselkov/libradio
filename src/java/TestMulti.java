import java.util.Scanner;
import java.util.regex.*;
import java.util.*;

public class TestMulti {
    static double ft2m(double ft)
    {
	return (ft / 3.281);
    }

    static double m2ft(double ft)
    {
	return (ft * 3.281);
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
	double[] xmit_power = {
	    35,
	    35,
	    46,
	    70
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
	    5000
	};
	int[] spacing = {
	    200,
	    200,
	    0,
	    0
	};
	int test_type = 1;
	Pattern pat = Pattern.compile(type_regex[test_type]);
	int resolution = 1024;
	ArrayList<Double> lats = new ArrayList<Double>();
	ArrayList<Double> lons = new ArrayList<Double>();
	ArrayList<Double> elevs = new ArrayList<Double>();

	RadioModel.init("data", spacing[test_type], 0, 0);

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

			System.out.println(name + ":  lat: " + lat + " lon: " +
			    lon + " elev: " + (int)m2ft(RadioModel.elevProbe(
			    lat, lon)) + " ft");
			lats.add(lat);
			lons.add(lon);
			elevs.add(elev + 50);
			if (test_type != 3) {
				long start, end;
				double d_t;

				start = System.currentTimeMillis();
				RadioModel.paintMap(freq, true,
				    xmit_power[test_type], recv_min_gain,
				    elev + ft2m(test_hgts[test_type]),
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
		double[] lats_arr = new double[lats.size()];
		double[] lons_arr = new double[lons.size()];
		double[] elevs_arr = new double[elevs.size()];
		double min_lat = 90, max_lat = -90, min_lon = 180,
		    max_lon = -180;
		double ctr_lat, ctr_lon, deg_range;

		for (int i = 0; i < lats_arr.length; i++) {
			lats_arr[i] = lats.get(i).doubleValue();
			lons_arr[i] = lons.get(i).doubleValue();
			elevs_arr[i] = elevs.get(i).doubleValue();

			min_lat = Math.min(min_lat, lats_arr[i]);
			max_lat = Math.max(max_lat, lats_arr[i]);
			min_lon = Math.min(min_lon, lons_arr[i]);
			max_lon = Math.max(max_lon, lons_arr[i]);
		}
		ctr_lat = (max_lat + min_lat) / 2;
		ctr_lon = (max_lon + min_lon) / 2;
		deg_range = Math.max(max_lat - min_lat, max_lon - min_lon);
		if (deg_range <= 10)
			deg_range = 10;
		else
			deg_range += 10;

		System.out.println("ctr: " + ctr_lat + "  lon: " + ctr_lon +
		    "  range: " + deg_range);

		RadioModel.paintMapMulti(130.0, true, xmit_power[test_type],
		    recv_min_gain, ft2m(5000), lats_arr, lons_arr, elevs_arr,
		    deg_range, ctr_lat, ctr_lon, resolution, "test.png");
	}

	RadioModel.fini();
    }
}
