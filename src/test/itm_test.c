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

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "itm_c.h"

#define	DIST_NM	80
#define	NUM_PTS	((long)(2 * DIST_NM * 1.852))
#define	DIST	(DIST_NM * 1852.0)

int
main(void)
{
	double elev[NUM_PTS];
	int propmode = ITM_PROPMODE_UNKNOWN;
	double dbloss = 0, deltaH = 0;
	int result;

	memset(elev, 0, sizeof (elev));

	for (int alt = 1000; alt <= 60000; alt += 1000) {
		double altm = alt / 3.281;
		result = itm_point_to_pointMDH(elev, NUM_PTS, DIST, 100, altm,
		    ITM_DIELEC_GND_AVG, ITM_CONDUCT_GND_AVG, ITM_NS_AVG, 114.0,
		    ITM_ENV_CONTINENTAL_TEMPERATE, ITM_POL_VERT, ITM_ACCUR_MAX,
		    ITM_ACCUR_MAX, ITM_ACCUR_MAX, &dbloss, &propmode, &deltaH);
		printf("[%d]  res: %d  dbloss: %.1f  mode: %d  deltaH: %f\n",
		    alt, result, dbloss - 90, propmode, deltaH);
	}

	return (0);
}
