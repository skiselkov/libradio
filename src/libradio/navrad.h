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

#include <libradio/navaiddb.h>

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

int16_t *navrad_get_audio_buf(navrad_type_t type, unsigned nr, double volume,
    bool_t squelch, bool_t agc, size_t *num_samples);
void navrad_free_audio_buf(int16_t *buf);
void navrad_done_audio(unsigned nr);

#ifdef	__cplusplus
}
#endif

#endif	/* _LIBRADIO_NAVRAD_H_ */
