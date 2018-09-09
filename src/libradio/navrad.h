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
#define	NAVRAD_AUDIO_SRATE	48000

bool_t navrad_init(navaiddb_t *db);
void navrad_fini(void);

uint64_t navrad_get_freq(unsigned nr);
double navrad_get_bearing(unsigned nr);
double navrad_get_radial(unsigned nr);
double navrad_get_dme(unsigned nr);
double navrad_get_hdef(unsigned nr, bool_t pilot, bool_t *tofrom);
double navrad_get_vdef(unsigned nr);
bool_t navrad_is_loc(unsigned nr);
bool_t navrad_get_ID(unsigned nr, char id[8]);

int16_t *navrad_get_audio_buf(unsigned nr, double volume, bool_t is_dme,
    bool_t squelch, size_t *num_samples);
void navrad_free_audio_buf(int16_t *buf);
void navrad_done_audio(unsigned nr);

#ifdef	__cplusplus
}
#endif

#endif	/* _LIBRADIO_NAVRAD_H_ */
