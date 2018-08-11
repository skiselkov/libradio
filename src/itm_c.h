/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensource.org/licenses/CDDL-1.0.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file COPYING.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2018 Saso Kiselkov. All rights reserved.
 */

#ifndef	_LIBRADIO_ITM_C_H_
#define	_LIBRADIO_ITM_C_H_

#ifdef	__cplusplus
extern "C" {
#endif

/*
 * Surface relative permittivity (dimensionless).
 */
#define	ITM_DIELEC_GND_AVG	15.0
#define	ITM_DIELEC_GND_POOR	4.0
#define	ITM_DIELEC_GND_GOOD	25.0
#define	ITM_DIELEC_WATER_FRESH	81.0
#define	ITM_DIELEC_WATER_SALT	81.0

/*
 * Surface conductivity (Siemens per meter).
 */
#define	ITM_CONDUCT_GND_AVG	0.005
#define	ITM_CONDUCT_GND_POOR	0.001
#define	ITM_CONDUCT_GND_GOOD	0.02
#define	ITM_CONDUCT_WATER_FRESH	0.01
#define	ITM_CONDUCT_WATER_SALT	5.0

/*
 * Surface reference (Ns, dimensionless).
 */
#define	ITM_NS_EQUATORIAL		360.0
#define	ITM_NS_CONTINENTAL_SUBTROPICAL	320.0
#define	ITM_NS_MARITIME_SUBTROPICAL	370.0
#define	ITM_NS_DESERT			280.0
#define	ITM_NS_CONTINENTAL_TEMPERATE	301.0
#define	ITM_NS_MARITIME_TEMPERATE_LAND	320.0
#define	ITM_NS_MARITIME_TEMPERATE_SEA	350.0
#define	ITM_NS_AVG			ITM_NS_CONTINENTAL_TEMPERATE

/*
 * Radio propagation environments (aka "radio climates").
 */
typedef enum {
	ITM_ENV_EQUATORIAL =			1,
	ITM_ENV_CONTINENTAL_SUBTROPICAL =	2,
	ITM_ENV_MARITIME_TROPICAL =		3,
	ITM_ENV_DESERT =			4,
	ITM_ENV_CONTINENTAL_TEMPERATE =		5,
	ITM_ENV_MARITIME_TEMPERATE_LAND =	6,
	ITM_ENV_MARITIME_TEMPERATE_SEA =	7
} itm_env_t;

/*
 * Radio wave polarization.
 */
typedef enum {
	ITM_POL_HORIZ =	0,
	ITM_POL_VERT =	1
} itm_pol_t;

/*
 * Propagation mode (enum).
 */
#define	ITM_PROPMODE_UNKNOWN	-1
#define	ITM_PROPMODE_LOS	0	/* Line of sight */
#define	ITM_PROPMODE_SH_DIFF	5	/* Single horizon, diffraction */
#define	ITM_PROPMODE_SH_TROPO	6	/* Single horizon, troposcatter */
#define	ITM_PROPMODE_DH_DIFF	9	/* Double horizon, diffraction */
#define	ITM_PROPMODE_DH_TROPO	10	/* Double horizon, troposcatter */

/*
 * Return result value. Any return value other than ITM_RESULT_SUCCESS
 * indicates an error.
 */
#define	ITM_RESULT_SUCCESS		0
#define	ITM_RESULT_ERANGE_SINGLE	1
#define	ITM_RESULT_DFLT_SUBST		2
#define	ITM_RESULT_ERANGE_MULTI		3

#define	ITM_ACCUR_MAX			0.99
#define	ITM_ACCUR_MIN			0.01

int itm_point_to_pointMDH(double *elev, unsigned n_elev_pts, double distance,
    double tht_m, double rht_m, double eps_dielect, double sgm_conductivity,
    double eno_ns_surfref, double frq_mhz, itm_env_t radio_climate,
    itm_pol_t pol, double time_accur, double loc_accur, double conf_accur,
    double *dbloss_p, int *propmode_p, double *deltaH_p);

const char *itm_propmode2str(int propmode);

#ifdef	__cplusplus
}
#endif

#endif	/* _LIBRADIO_ITM_C_H_ */
