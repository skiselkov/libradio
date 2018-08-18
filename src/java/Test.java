public class Test {
    static double ft2m(double ft)
    {
	return (ft / 3.281);
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
	};
	String filenames[] = {
	    "01_LAX",
	    "02_PMD",
	    "03_FAT",
	    "04_PSP",
	    "05_SFO",
	    "06_MtWilson"
	};
	double[] acf_hgt = {0, 5000, 10000, 20000, 30000};
	double[] xmit_power = {40, 50, 80};
	double recv_min_gain = -100;
	int resolution = 1024;

	RadioModel.init("data", 0, 0);
	System.out.println("Memory used: " +
	    (RadioModel.countBytes() >> 20) + " MB");

	for (int i = 0; i < locations.length; i++) {
		for (int j = 0; j < acf_hgt.length; j++) {
			for (int k = 0; k < xmit_power.length; k++) {
				double twr_lat = locations[i][0];
				double twr_lon = locations[i][1];
				double twr_elev = locations[i][2];
				long start, end;
				double d_t;

				start = System.currentTimeMillis();
				RadioModel.paintMap(135, true, xmit_power[k],
				    recv_min_gain, ft2m(twr_elev + acf_hgt[j]),
				    twr_lat, twr_lon, ft2m(twr_elev) + 40,
				    xmit_power[k] / 10, resolution, "out2/" +
				    filenames[i] + "_" + (int)acf_hgt[j] +
				    "ft_" + (int)xmit_power[k] + "dB.png");
				end = System.currentTimeMillis();
				d_t = (end - start) / 1000.0;

				System.out.format("took %.3fs @ %d " +
				    "samples / sec\n", d_t,
				    (long)((resolution * resolution) / d_t));
			}
		}
	}
	RadioModel.fini();
    }
}
