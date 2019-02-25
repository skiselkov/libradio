import java.io.File;
import java.lang.Runtime;
import java.lang.Thread;
import java.util.ArrayList;
import com.vspro.util.RadioModel;

class Worker extends Thread {
    String filename;
    double xmit_power;
    double recv_min_gain;
    double acf_elev;
    boolean acf_agl;
    double twr_lat;
    double twr_lon;
    double twr_elev;
    int resolution;

    Worker(String in_filename, double in_xmit_power, double in_recv_min_gain,
	double in_acf_elev, boolean in_acf_agl,
	double in_twr_lat, double in_twr_lon, double in_twr_elev,
	int in_resolution)
    {
	filename = in_filename;
	xmit_power = in_xmit_power;
	recv_min_gain = in_recv_min_gain;
	acf_elev = in_acf_elev;
	acf_agl = in_acf_agl;
	twr_lat = in_twr_lat;
	twr_lon = in_twr_lon;
	twr_elev = in_twr_elev;
	resolution = in_resolution;
    }
    public void run()
    {
	long start, end;
	double d_t;

	System.out.println("Generating " + filename + " ... ");

	start = System.currentTimeMillis();
	RadioModel.paintMap(135, true, xmit_power, recv_min_gain, acf_elev,
	    acf_agl, twr_lat, twr_lon, twr_elev + 40, 1,
	    resolution, filename);
	end = System.currentTimeMillis();
	d_t = (end - start) / 1000.0;

	System.out.format("%s done, took %.3fs @ %d samples / sec\n",
	    filename, d_t, (long)((resolution * resolution) / d_t));
    }
}

public class Test {
    static double ft2m(double ft)
    {
	return (ft / 3.281);
    }

    static int count_alive(ArrayList<Worker> workers)
    {
	int num_alive = 0;

	for (int i = 0; i < workers.size(); i++) {
		if (workers.get(i).isAlive())
			num_alive++;
	}

	return (num_alive);
    }

    public static void main(String[] args)
    {
	/* elevations in feet */
	double[][] locations = {
	    {33.9425, -118.408, 127},	/* LAX */
	    {34.6293, -118.084, 2542},	/* PMD */
	    {36.7765, -119.719, 335},	/* FAT */
	    {33.8297, -116.5067, 476},	/* PSP */
	    {37.6188, -122.3756, 13},	/* SFO */
	    {34.228, -118.0552, 6300},	/* MT WILSON */
	    {33.6756, -117.8681, 56}	/* SNA tower */
	};
	String filenames[] = {
	    "01_LAX",
	    "02_PMD",
	    "03_FAT",
	    "04_PSP",
	    "05_SFO",
	    "06_MtWilson",
	    "07_SNA"
	};
	double[] acf_hgt = {0, 100, 200, 500, 1000};
	double[] xmit_power = {40, 50, 60, 70, 80};
	double recv_min_gain = -100;
	int resolution = 1024;
	ArrayList<Worker> workers = new ArrayList<Worker>();
	int num_cpus = Runtime.getRuntime().availableProcessors();

	RadioModel.init("data", 0, 0, 0);
	System.out.println("All height maps loaded, memory required: " +
	    (RadioModel.countBytes() >> 20) + " MB");
	try {
		(new File("out")).mkdir();
	} catch (Exception e) {/*ignored*/}

	for (int i = 0; i < locations.length; i++) {
		for (int j = 0; j < acf_hgt.length; j++) {
			for (int k = 0; k < xmit_power.length; k++) {
				double twr_lat = locations[i][0];
				double twr_lon = locations[i][1];
				double twr_elev = locations[i][2];
				String filename = "out/" + filenames[i] + "_" +
				    (int)acf_hgt[j] + "ft_" +
				    (int)xmit_power[k] + "dB.png";
				Worker w;

				w = new Worker(filename, xmit_power[k],
				    recv_min_gain, ft2m(twr_elev + acf_hgt[j]),
				    false, twr_lat, twr_lon,
				    ft2m(twr_elev + 150), resolution);
				w.start();
				workers.add(w);

				while (count_alive(workers) >= num_cpus) {
					try {
						Thread.sleep(100);
					} catch (Exception e) {/* ignored */}
				}
			}
		}
	}
	for (int i = 0; i < workers.size(); i++) {
		try {
			workers.get(i).join();
		} catch (Exception e) {/* ignored */}
	}
	RadioModel.fini();
    }
}
