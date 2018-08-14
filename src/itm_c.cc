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

#include <string.h>
#include <stdlib.h>

#include "itm.h"
#include "itm_c.h"

/*
 * Runs the ITM in a point-to-point model. The meanings of the parameters
 * are as follows:
 *
 * @param elev An array of terrain elevation points located between the
 *	receiver and transmitter. Point heights are in meters. The first
 *	point is assumed to be at the transmitter and the last point at
 *	the receiver. Any extra points are spaced equally between them.
 * @param n_elev_pts Number of points in `elev'.
 * @param distance Distance (in meters) between the receiver and transmitter.
 * @param tht_m Height of transmitter above ground (in meters).
 * @param rht_m Height of receiver above ground (in meters).
 * @param eps_dielect Surface dielectric constant (relative permittivity).
 *	See ITM_DIELEC_* constants in itm_c.h.
 * @param sgm_conductivity Ground conductivity. See ITM_CONDUCT_* constants.
 * @param eno_ns_surfref Surface reference Ns number. See ITM_NS_* constants.
 * @param frq_mhz Center frequency in MHz.
 * @param radio_climate A constant describing the overall radio environment.
 * @param pol Signal polarization. See itm_pol_t.
 * @param time_accur Time accuracy confidence factor (0.01 - 0.99).
 * @param loc_accur Location accuracy confidence factor (0.01 - 0.99).
 * @param conf_accur General (?) confidence factor (0.01 - 0.99).
 * @param dbloss_p Output signal level drop (in dB). Will be filled when
 *	pointer is not NULL.
 * @param propmode_p Output propagation mode (see ITM_PROPMODE_* constants).
 *	Will be filled when pointer is not NULL.
 * @param deltaH_p Unknown. Will be filled when pointer is not NULL.
 *
 * @return Status of computational result. See ITM_RESULT_* constants.
 */
int
itm_point_to_pointMDH(double *elev, unsigned n_elev_pts, double distance,
    double tht_m, double rht_m, double eps_dielect, double sgm_conductivity,
    double eno_ns_surfref, double frq_mhz, itm_env_t radio_climate,
    itm_pol_t pol, double time_accur, double loc_accur, double conf_accur,
    double *dbloss_p, int *propmode_p, double *deltaH_p)
{
	double dbloss, deltaH;
	int propmode, errnum;
	double *e;

	e = (double *)calloc(n_elev_pts + 2, sizeof (*e));
	e[0] = n_elev_pts - 1;
	e[1] = distance / (n_elev_pts - 1);
	memcpy(&e[2], elev, n_elev_pts * sizeof (*e));

	point_to_pointMDH(e, tht_m, rht_m, eps_dielect, sgm_conductivity,
	    eno_ns_surfref, frq_mhz, radio_climate, pol, time_accur, loc_accur,
	    conf_accur, dbloss, propmode, deltaH, errnum);

	if (dbloss_p != NULL)
		*dbloss_p = dbloss;
	if (propmode_p != NULL)
		*propmode_p = propmode;
	if (deltaH_p != NULL)
		*deltaH_p = deltaH;

	free(e);

	return (errnum);
}

const char *
itm_propmode2str(int propmode)
{
	switch (propmode) {
	case ITM_PROPMODE_LOS:
		return ("line of sight");
	case ITM_PROPMODE_SH_DIFF:
		return ("single horizon, diffraction");
	case ITM_PROPMODE_SH_TROPO:
		return ("single horizon, troposcatter");
	case ITM_PROPMODE_DH_DIFF:
		return ("double horizon, diffraction");
	case ITM_PROPMODE_DH_TROPO:
		return ("double horizon, troposcatter");
	default:
		return ("unknown");
	}
}
