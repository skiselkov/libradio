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

#ifndef	_LIBRADIO_NAVRAD_H_
#define	_LIBRADIO_NAVRAD_H_

#include <opengpws/xplane_api.h>

#include "navaiddb.h"

#include "itm_c.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	NUM_NAV_RADIOS		2
#define	MAX_NUM_DMES		8
#define	NAVRAD_AUDIO_SRATE	48000

typedef enum {
	NAVRAD_TYPE_VLOC,
	NAVRAD_TYPE_ADF,
	NAVRAD_TYPE_DME
} navrad_type_t;

typedef enum {
	ADF_MODE_ADF,
	ADF_MODE_ANT,
	ADF_MODE_ADF_BFO,
	ADF_MODE_ANT_BFO
} adf_mode_t;

typedef void (*libradio_profile_debug_cb_t)(const egpws_terr_probe_t *probe,
    double p1_hgt, double p2_hgt, void *userinfo);
/*
 * You can use the libradio_compute_signal_prop function to perform your own
 * custom radio propagation computations. This performs all the terrain
 * profile lookup and ITM computations for you. Use this function sparingly,
 * as it involves an OpenGPWS terrain probe with up 600 points, which is
 * not exactly cheap (doing a few of these checks per frame is ok, but don't
 * invoke it hundreds of times per frame - cache the results for a few secs).
 *
 * @param p1 Station 1 position, elevation in meters.
 * @param p2 Station 2 position, elevation in meters.
 * @param p1_min_hgt Minimum height above ground allowable for station 1.
 *	If the terrain lookup ends up with p1 being closer to the ground
 *	than p1_min_tgt, its elevation is adjusted up to be at least
 *	p1_min_hgt meters above ground.
 * @param p2_min_hgt Same as p1_min_hgt, but for point p2.
 * @param freq Frequency of the signal in Hz.
 * @param pol Polarization of the signal.
 * @param dbloss_out Optional output parameter (set to NULL if not needed).
 *	This provides the signal loss in dB between the two stations.
 *	N.B. this will be a positive number, but it is actually the LOSS of
 *	signal level. To obtain the absolute signal level, you should do:
 *		signal_at_receiver = signal_at_transmitter - dbloss;
 * @param propmode_out Optional output parameter (set to NULL if not needed).
 *	This provides the principal propagation mode for the signal between
 *	the stations. This will be one of the ITM_PROPMODE_ macros defined
 *	in itm_c.h. You can use itm_propmode2str to convert this into a short
 *	readable description of the propagation mode.
 * @param profile_debug_cb An optional output callback (set to NULL if not
 *	needed). If provided, this will be called with the terrain profile
 *	that was constructed out of the OpenGPWS and the actual heights of
 *	points p1 and p2 above their local ground.
 * @param userinfo Optional userinfo pointer that will be passed to
 *	profile_debug_cb.
 */
void libradio_compute_signal_prop(geo_pos3_t p1, geo_pos3_t p2,
    double p1_min_hgt, double p2_min_hgt, uint64_t freq, itm_pol_t pol,
    double *dbloss_out, int *propmode_out,
    libradio_profile_debug_cb_t profile_debug_cb, void *userinfo);

bool_t navrad_init(navaiddb_t *db);
bool_t navrad_init2(navaiddb_t *db, unsigned num_dmes);
void navrad_fini(void);

void navrad_set_freq(navrad_type_t type, unsigned nr, uint64_t freq);
void navrad_set_failed(navrad_type_t type, unsigned nr, bool_t flag);

uint64_t navrad_get_freq(navrad_type_t type, unsigned nr);
double navrad_get_signal_quality(navrad_type_t type, unsigned nr);
double navrad_get_bearing(navrad_type_t type, unsigned nr);
double navrad_get_dme(navrad_type_t type, unsigned nr);
bool_t navrad_get_ID(navrad_type_t type, unsigned nr, char id[8]);
bool_t navrad_have_bearing(navrad_type_t type, unsigned nr);
/*
 * These are specific to VLOC-type radios, so no need to pass radio type.
 */
void navrad_set_obs(unsigned nr, double obs);
double navrad_get_radial(unsigned nr);
double navrad_get_hdef(unsigned nr, bool_t pilot, bool_t *tofrom);
double navrad_get_vdef(unsigned nr);
double navrad_get_loc_ddm(unsigned nr);
double navrad_get_gp_ddm(unsigned nr);
double navrad_get_fcrs(unsigned nr);
double navrad_get_gs(unsigned nr);
bool_t navrad_is_loc(unsigned nr);

/*
 * These are specific to ADF-type radios, so no need to pass radio type.
 */
void navrad_set_adf_mode(unsigned nr, adf_mode_t mode);
adf_mode_t navrad_get_adf_mode(unsigned nr);

void navrad_set_brg_override(navrad_type_t type, unsigned nr, bool_t flag);
bool_t navrad_get_brg_override(navrad_type_t type, unsigned nr);

#define	NAVRAD_MAX_STREAMS	4
int16_t *navrad_get_audio_buf(navrad_type_t type, unsigned nr, double volume,
    bool_t squelch, bool_t agc, size_t *num_samples);
int16_t *navrad_get_audio_buf2(navrad_type_t type, unsigned nr, double volume,
    bool_t squelch, bool_t agc, unsigned stream_id, size_t *num_samples);
void navrad_free_audio_buf(int16_t *buf);
void navrad_done_audio(unsigned nr);
void navrad_sync_streams(navrad_type_t type, unsigned nr);

#ifdef	__cplusplus
}
#endif

#endif	/* _LIBRADIO_NAVRAD_H_ */
