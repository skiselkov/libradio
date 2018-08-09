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

#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <acfutils/crc64.h>
#include <acfutils/dr.h>
#include <acfutils/math.h>
#include <acfutils/perf.h>
#include <acfutils/worker.h>
#include <acfutils/time.h>

#include "opengpws/xplane_api.h"

#include "distort.h"
#include "libradio/navrad.h"
#include "itm_c.h"

#define	WORKER_INTVAL		500000
#define	DEF_UPD_RATE		0.5
#define	MIN_DELTA_T		0.01
#define	NAVAID_SRCH_RANGE	NM2MET(300)
#define	ANT_BASE_GAIN		92.0	/* dB */
#define	INTERFERENCE_LIMIT	16.0	/* dB */
#define	NOISE_FLOOR_AUDIO	-60.0	/* dB */
#define	NOISE_FLOOR_SIGNAL	-65.0	/* dB */
#define	NOISE_FLOOR_TEST	-80.0	/* dB */

#define	AUDIO_BUF_LEN		11	/* seconds */
#define	AUDIO_BUF_NUM_SAMPLES	4800
#define	AUDIO_BUF_NUM_CHUNKS	110

static bool_t inited = B_FALSE;

typedef struct {
	navaid_t	*navaid;
	/*
	 * `signal_db' is the dB level of the signal used by code that
	 * generates signal processing outputs.
	 * `signal_db_tgt' is the dB level assessed by the radio path
	 * computation code. Because that code only runs at infrequent
	 * intervals, if we used its output directly, we could get
	 * stepwise behavior (e.g. stepping volume levels in audio output).
	 * To avoid this, we smoothly transfer from signal_db_tgt to
	 * signal_db using FILTER_IN.
	 */
	double		signal_db;
	double		signal_db_tgt;
	bool_t		outdated;

	/*
	 * Control chunks for the navaid audio generator. The value of
	 * the chunk is simply a boolean '0' for 'silence' or '1' for
	 * a 1 kHz tone. This is generated from audio_buf_chunks_encode.
	 */
	uint8_t		audio_chunks[AUDIO_BUF_NUM_CHUNKS];
	unsigned	cur_audio_chunk;

	avl_node_t	node;
} radio_navaid_t;

typedef struct {
	unsigned	nr;

	mutex_t		lock;
	uint64_t	freq;
	double		hdef;
	double		hdef_tgt;
	double		vdef;
	double		vdef_tgt;
	distort_t	*distort;

	avl_tree_t	vors;
	avl_tree_t	dmes;

	struct {
		dr_t	ovrd;
		dr_t	freq;
		dr_t	dir_degt;

		dr_t	hdef_pilot;
		dr_t	vdef_pilot;
		dr_t	fromto_pilot;

		dr_t	hdef_copilot;
		dr_t	vdef_copilot;
		dr_t	fromto_copilot;

		dr_t	dme_nm;
	} drs;
} radio_t;

static struct {
	navaiddb_t		*db;

	mutex_t			lock;
	geo_pos3_t		pos;
	double			magvar;
	double			last_t;

	radio_t			radios[NUM_NAV_RADIOS];
	worker_t		worker;

	const egpws_intf_t	*opengpws;
} navrad;

static struct {
	dr_t		lat;
	dr_t		lon;
	dr_t		elev;
	dr_t		sim_time;
	dr_t		magvar;
} drs;

static const char *morse_table[] = {
    "00000",	/* 0 */
    "10000",	/* 1 */
    "11000",	/* 2 */
    "11100",	/* 3 */
    "11110",	/* 4 */
    "11111",	/* 5 */
    "01111",	/* 6 */
    "00111",	/* 7 */
    "00011",	/* 8 */
    "00001",	/* 9 */
    "10",	/* A */
    "0111",	/* B */
    "0101",	/* C */
    "011",	/* D */
    "1",	/* E */
    "1101",	/* F */
    "001",	/* G */
    "1111",	/* H */
    "11",	/* I */
    "1000",	/* J */
    "010",	/* K */
    "1011",	/* L */
    "00",	/* M */
    "01",	/* N */
    "000",	/* O */
    "1001",	/* P */
    "0010",	/* Q */
    "101",	/* R */
    "111",	/* S */
    "0",	/* T */
    "110",	/* U */
    "1110",	/* V */
    "100",	/* W */
    "0110",	/* X */
    "0100",	/* Y */
    "0011"	/* Z */
};

#define	ONE_KHZ_NUM_SAMPLES	(NAVRAD_AUDIO_SRATE / 1000)
static const int16_t one_khz_tone[ONE_KHZ_NUM_SAMPLES] = {
    0,
    4276,
    8480,
    12539,
    16383,
    19947,
    23169,
    25995,
    28377,
    30272,
    31650,
    32486,
    32766,
    32486,
    31650,
    30272,
    28377,
    25995,
    23169,
    19947,
    16383,
    12539,
    8480,
    4276,
    0,
    -4276,
    -8480,
    -12539,
    -16383,
    -19947,
    -23169,
    -25995,
    -28377,
    -30272,
    -31650,
    -32486,
    -32766,
    -32486,
    -31650,
    -30272,
    -28377,
    -25995,
    -23169,
    -19947,
    -16383,
    -12539,
    -8480,
    -4276
};

static void
audio_buf_chunks_encode(radio_navaid_t *rnav)
{
	navaid_t *nav = rnav->navaid;

	memset(rnav->audio_chunks, 0, sizeof (rnav->audio_chunks));

	for (int i = 0, j = 0, n = MIN(strlen(nav->id), 5); i < n; i++) {
		char c = nav->id[i];
		const char *codestr;

		if (c >= '0' && c <= '9')
			codestr = morse_table[c - '0'];
		else if (c >= 'A' && c <= 'Z')
			codestr = morse_table[c - 'A' + 10];
		else
			continue;

		for (int k = 0, nk = strlen(codestr); k < nk; k++) {
			if (codestr[k] == '0') {
				/* dash: 300 ms */
				rnav->audio_chunks[j++] = 1;
				rnav->audio_chunks[j++] = 1;
				rnav->audio_chunks[j++] = 1;
			} else {
				/* dot: 100 ms */
				rnav->audio_chunks[j++] = 1;
			}
			j++;
		}
		/* add a single chunk space between letters */
		j++;
		ASSERT3U(j, <, AUDIO_BUF_NUM_CHUNKS);
	}
}

static void
signal_levels_update(avl_tree_t *tree, double d_t)
{
	for (radio_navaid_t *rnav = avl_first(tree); rnav != NULL;
	    rnav = AVL_NEXT(tree, rnav)) {
		FILTER_IN(rnav->signal_db, rnav->signal_db_tgt, d_t,
		    USEC2SEC(WORKER_INTVAL));
	}
}

static void
radio_floop_cb(radio_t *radio, double d_t)
{
	mutex_enter(&radio->lock);
	radio->freq = dr_getf(&radio->drs.freq) * 10000;
	FILTER_IN(radio->hdef, radio->hdef_tgt, d_t, DEF_UPD_RATE);
	FILTER_IN(radio->vdef, radio->vdef_tgt, d_t, DEF_UPD_RATE);

	signal_levels_update(&radio->vors, d_t);
	signal_levels_update(&radio->dmes, d_t);

	mutex_exit(&radio->lock);
}

static void
flush_navaid_tree(avl_tree_t *tree)
{
	void *cookie = NULL;
	radio_navaid_t *rnav;

	while ((rnav = avl_destroy_nodes(tree, &cookie)) != NULL)
		free(rnav);
}

static void
radio_refresh_navaid_list_type(radio_t *radio, avl_tree_t *tree,
    geo_pos2_t pos, uint64_t freq, navaid_type_t type)
{
	navaid_list_t *list;

	list = navaiddb_query(navrad.db, GEO3_TO_GEO2(pos), NAVAID_SRCH_RANGE,
	    NULL, &freq, &type);

	mutex_enter(&radio->lock);

	/* mark all navaids as outdated */
	for (radio_navaid_t *rnav = avl_first(tree); rnav != NULL;
	    rnav = AVL_NEXT(tree, rnav)) {
		rnav->outdated = B_TRUE;
	}
	/* process the list, adding new navaids & marking old ones */
	for (size_t i = 0; i < list->num_navaids; i++) {
		radio_navaid_t *rnav;
		radio_navaid_t srch = { .navaid = list->navaids[i] };
		avl_index_t where;

		rnav = avl_find(tree, &srch, &where);
		if (rnav == NULL) {
			rnav = calloc(1, sizeof (*rnav));
			rnav->navaid = list->navaids[i];
			audio_buf_chunks_encode(rnav);
			rnav->cur_audio_chunk =
			    crc64_rand() % AUDIO_BUF_NUM_CHUNKS;
			avl_insert(tree, rnav, where);
		} else {
			rnav->outdated = B_FALSE;
		}
	}
	/* remove any navaids we haven't seen in the new list */
	for (radio_navaid_t *rnav = avl_first(tree),
	    *rnav_next = NULL; rnav != NULL; rnav = rnav_next) {
		rnav_next = AVL_NEXT(tree, rnav);
		if (rnav->outdated) {
			avl_remove(tree, rnav);
			free(rnav);
		}
	}

	mutex_exit(&radio->lock);

	navaiddb_list_free(list);
}

static void
radio_refresh_navaid_list(radio_t *radio, geo_pos2_t pos, uint64_t freq)
{
	if (is_valid_vor_freq(freq / 1000000.0)) {
		radio_refresh_navaid_list_type(radio, &radio->vors, pos, freq,
		    NAVAID_VOR);
		radio_refresh_navaid_list_type(radio, &radio->dmes, pos, freq,
		    NAVAID_DME);
	} else {
		mutex_enter(&radio->lock);
		flush_navaid_tree(&radio->vors);
		flush_navaid_tree(&radio->dmes);
		mutex_exit(&radio->lock);
	}
}

static void
radio_navaid_recompute_signal(radio_navaid_t *rnav, uint64_t freq,
    geo_pos3_t pos, fpp_t *fpp)
{
	navaid_t *nav = rnav->navaid;
	vect2_t v = geo2fpp(GEO3_TO_GEO2(nav->pos), fpp);
	enum {
	    MAX_PTS = 600,
	    SPACING = 250,		/* meters */
	    MIN_DIST = 1000,		/* meters */
	    MAX_DIST = 1000000,		/* meters */
	    WATER_OCEAN_MIN = 40000,	/* meters */
	    WATER_OCEAN_MAX = 100000	/* meters */
	};
	double dist = clamp(vect2_abs(v), MIN_DIST, MAX_DIST);
	egpws_terr_probe_t probe;
	double dbloss, water_part, water_length, dielec, conduct, water_conduct;
	int propmode;
	double water_fract = 0;
	double elev_test[2] = {0, 0};

	ASSERT(!IS_NULL_VECT(v));

	/*
	 * Some navaids will be below our radio horizon. Just perform a quick
	 * "best case" test for them and discard them if there's no chance
	 * that they'll be visible.
	 */
	itm_point_to_pointMDH(elev_test, 2, dist, nav->pos.elev + 10, pos.elev,
	    ITM_DIELEC_GND_AVG, ITM_CONDUCT_GND_AVG, ITM_NS_AVG,
	    freq / 1000000.0, ITM_ENV_CONTINENTAL_TEMPERATE, ITM_POL_VERT,
	    ITM_ACCUR_MAX, ITM_ACCUR_MAX, ITM_ACCUR_MAX, &dbloss, NULL, NULL);
	if (ANT_BASE_GAIN - dbloss < NOISE_FLOOR_TEST) {
		rnav->signal_db_tgt = ANT_BASE_GAIN - dbloss;
		return;
	}

	memset(&probe, 0, sizeof (probe));
	probe.num_pts = clampi(dist / SPACING, 2, MAX_PTS);
	water_part = 1.0 / probe.num_pts;
	probe.in_pts = calloc(probe.num_pts, sizeof (*probe.in_pts));
	probe.out_elev = calloc(probe.num_pts, sizeof (*probe.out_elev));
	probe.out_water = calloc(probe.num_pts, sizeof (*probe.out_water));

	for (unsigned i = 0; i < probe.num_pts; i++) {
		vect2_t p = vect2_scmul(v, i / (double)(probe.num_pts - 1));
		probe.in_pts[i] = fpp2geo(p, fpp);
	}
	navrad.opengpws->terr_probe(&probe);
	for (unsigned i = 0; i < probe.num_pts; i++)
		water_fract += probe.out_water[i] * water_part;
	water_fract = clamp(water_fract, 0, 1);

	water_length = dist * water_fract;
	water_conduct = wavg(ITM_CONDUCT_WATER_FRESH, ITM_CONDUCT_WATER_SALT,
	    iter_fract(water_length, WATER_OCEAN_MIN, WATER_OCEAN_MAX, B_TRUE));

	dielec = wavg(ITM_DIELEC_GND_AVG, ITM_DIELEC_WATER_FRESH, water_fract);
	conduct = wavg(ITM_CONDUCT_GND_AVG, water_conduct, water_fract);

	itm_point_to_pointMDH(probe.out_elev, probe.num_pts, dist,
	    nav->pos.elev + 10, pos.elev, dielec, conduct, ITM_NS_AVG,
	    (freq / 1000000.0), ITM_ENV_CONTINENTAL_TEMPERATE,
	    ITM_POL_VERT, ITM_ACCUR_MAX, ITM_ACCUR_MAX, ITM_ACCUR_MAX, &dbloss,
	    &propmode, NULL);

	rnav->signal_db_tgt = ANT_BASE_GAIN - dbloss;
}

/*
 * This is just a rough guesstimate of the xmit frequency portion of a DME
 * based on VOR frequency. It's not meant to be an accurate frequency table,
 * just something we can use to estimate signal propagation (so we only need
 * to get into the "ballpark").
 */
static inline uint64_t
vor2dme(uint64_t freq)
{
	double fract = (freq - 108000000.0) / (118000000.0 - 108000000.0);
	return (1041000000 + 109000000 * fract);
}

static void
radio_worker(radio_t *radio, geo_pos3_t pos, fpp_t *fpp)
{
	uint64_t freq;
	uint64_t dme_freq;

	mutex_enter(&radio->lock);
	freq = radio->freq;
	mutex_exit(&radio->lock);

	radio_refresh_navaid_list(radio, GEO3_TO_GEO2(pos), freq);

	dme_freq = vor2dme(freq);
	/*
	 * Don't have to grab the lock here, since we're the only ones that
	 * can modify the trees and we're not going to be doing so here.
	 */
	for (radio_navaid_t *rnav = avl_first(&radio->vors); rnav != NULL;
	    rnav = AVL_NEXT(&radio->vors, rnav)) {
		radio_navaid_recompute_signal(rnav, freq, pos, fpp);
	}
	for (radio_navaid_t *rnav = avl_first(&radio->dmes); rnav != NULL;
	    rnav = AVL_NEXT(&radio->dmes, rnav)) {
		radio_navaid_recompute_signal(rnav, dme_freq, pos, fpp);
	}
}

static float
floop_cb(float elapsed1, float elapsed2, int counter, void *refcon)
{
	double now = dr_getf(&drs.sim_time);
	double d_t = ABS(now - navrad.last_t);

	UNUSED(elapsed1);
	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);

	if (d_t < MIN_DELTA_T)
		goto out;

	mutex_enter(&navrad.lock);
	navrad.pos = GEO_POS3(dr_getf(&drs.lat), dr_getf(&drs.lon),
	    dr_getf(&drs.elev));
	navrad.magvar = dr_getf(&drs.magvar);
	mutex_exit(&navrad.lock);

	for (int i = 0; i < NUM_NAV_RADIOS; i++)
		radio_floop_cb(&navrad.radios[i], d_t);

out:
	navrad.last_t = now;

	return (-1);
}

static bool_t
worker_cb(void *userinfo)
{
	geo_pos3_t pos;
	fpp_t fpp;

	UNUSED(userinfo);

	if (!navrad.opengpws->is_inited())
		return (B_TRUE);

	mutex_enter(&navrad.lock);
	pos = navrad.pos;
	mutex_exit(&navrad.lock);

	fpp = gnomo_fpp_init(GEO3_TO_GEO2(pos), 0, &wgs84, B_TRUE);

	for (int i = 0; i < NUM_NAV_RADIOS; i++)
		radio_worker(&navrad.radios[i], pos, &fpp);

	return (B_TRUE);
}

static int
navrad_navaid_compar(const void *a, const void *b)
{
	const radio_navaid_t *rna = a, *rnb = b;

	if (rna->navaid < rnb->navaid)
		return (-1);
	if (rna->navaid > rnb->navaid)
		return (1);
	return (0);
}

static void
radio_init(radio_t *radio, int nr)
{
	radio->nr = nr;
	mutex_init(&radio->lock);
	avl_create(&radio->vors, navrad_navaid_compar,
	    sizeof (radio_navaid_t), offsetof(radio_navaid_t, node));
	avl_create(&radio->dmes, navrad_navaid_compar,
	    sizeof (radio_navaid_t), offsetof(radio_navaid_t, node));
	radio->distort = distort_init(NAVRAD_AUDIO_SRATE);

	fdr_find(&radio->drs.ovrd,
	    "sim/operation/override/override_nav%d_needles", nr);
	fdr_find(&radio->drs.freq,
	    "sim/cockpit2/radios/actuators/nav%d_frequency_hz", nr);
	fdr_find(&radio->drs.dir_degt, "sim/cockpit/radios/nav%d_dir_degt", nr);
	fdr_find(&radio->drs.hdef_pilot,
	    "sim/cockpit/radios/nav%d_hdef_dot", nr);
	fdr_find(&radio->drs.vdef_pilot,
	    "sim/cockpit/radios/nav%d_vdef_dot", nr);
	fdr_find(&radio->drs.hdef_copilot,
	    "sim/cockpit/radios/nav%d_hdef_dot2", nr);
	fdr_find(&radio->drs.vdef_copilot,
	    "sim/cockpit/radios/nav%d_vdef_dot2", nr);
	fdr_find(&radio->drs.dme_nm,
	    "sim/cockpit/radios/nav%d_dme_dist_m", nr);
	fdr_find(&radio->drs.fromto_pilot,
	    "sim/cockpit/radios/nav%d_fromto", nr);
	fdr_find(&radio->drs.fromto_copilot,
	    "sim/cockpit/radios/nav%d_fromto2", nr);
}

static void
radio_fini(radio_t *radio)
{
	void *cookie;
	radio_navaid_t *nav;

	cookie = NULL;
	while ((nav = avl_destroy_nodes(&radio->vors, &cookie)) != NULL)
		free(nav);
	avl_destroy(&radio->vors);

	cookie = NULL;
	while ((nav = avl_destroy_nodes(&radio->dmes, &cookie)) != NULL)
		free(nav);
	avl_destroy(&radio->dmes);

	if (radio->distort != NULL)
		distort_fini(radio->distort);

	mutex_destroy(&radio->lock);
}

static navaid_t *
radio_get_strongest_navaid(radio_t *radio, avl_tree_t *tree, double *signal_db)
{
	radio_navaid_t *strongest = NULL, *second = NULL;
	navaid_t *winner = NULL;

	mutex_enter(&radio->lock);

	for (radio_navaid_t *rnav = avl_first(tree); rnav != NULL;
	    rnav = AVL_NEXT(tree, rnav)) {
		if (rnav->signal_db < NOISE_FLOOR_SIGNAL)
			continue;
		if (strongest == NULL)
			strongest = rnav;
		if (rnav->signal_db > strongest->signal_db) {
			second = strongest;
			strongest = rnav;
		} else if (second != NULL &&
		    rnav->signal_db > second->signal_db) {
			second = rnav;
		}
	}
	if (strongest != NULL)
		ASSERT3P(strongest, !=, second);

	if (second != NULL &&
	    strongest->signal_db - second->signal_db < INTERFERENCE_LIMIT)
		strongest = NULL;

	if (strongest != NULL) {
		winner = strongest->navaid;
		if (signal_db != NULL)
			*signal_db = strongest->signal_db;
	}

	mutex_exit(&radio->lock);

	return (winner);
}

static double
signal_error(navaid_t *nav, double brg, double signal_db, double dist)
{
	double pos_seed = crc64(&nav->pos, sizeof (nav->pos)) /
	    (double)UINT64_MAX;
	double time_seed = navrad.last_t / 1800.0;
	double signal_seed = clamp(signal_db / NOISE_FLOOR_SIGNAL, 0, 1);

	signal_seed = POW4(signal_seed) * POW4(signal_seed);

	return (signal_seed * sin(pos_seed + time_seed + brg / 20 +
	    dist / 10000));
}

static double
brg2navaid(navaid_t *nav, double *dist)
{
	geo_pos2_t pos;
	fpp_t fpp;
	vect2_t v;

	mutex_enter(&navrad.lock);
	pos = GEO3_TO_GEO2(navrad.pos);
	mutex_exit(&navrad.lock);

	fpp = gnomo_fpp_init(pos, 0, &wgs84, B_FALSE);
	v = geo2fpp(GEO3_TO_GEO2(nav->pos), &fpp);

	if (dist != NULL)
		*dist = vect2_abs(v);

	return (dir2hdg(v));
}

static double
radio_get_bearing(radio_t *radio, bool_t magnetic)
{
	double brg, error, signal_db, rng;
	navaid_t *nav = radio_get_strongest_navaid(radio, &radio->vors,
	    &signal_db);
	enum { MAX_ERROR = 10 };

	if (nav == NULL)
		return (NAN);

	brg = brg2navaid(nav, &rng);
	error = MAX_ERROR * signal_error(nav, brg, signal_db, rng);
	printf("brg error: %.2f\n", error);

	return (brg + error + (magnetic ? navrad.magvar : 0));
}

static double
radio_get_radial(radio_t *radio)
{
	double radial, error, signal_db, rng;
	navaid_t *nav = radio_get_strongest_navaid(radio, &radio->vors,
	    &signal_db);
	enum { MAX_ERROR = 10 };

	if (nav == NULL)
		return (NAN);

	radial = normalize_hdg(brg2navaid(nav, &rng) + 180);
	error = MAX_ERROR * signal_error(nav, radial, signal_db, rng);
	printf("brg error: %.2f\n", error);

	return (radial + error - nav->vor.magvar);
}

static double
radio_get_dme(radio_t *radio)
{
	double dist, brg, rng, error, signal_db;
	navaid_t *nav = radio_get_strongest_navaid(radio, &radio->dmes,
	    &signal_db);
	enum { MAX_ERROR = (unsigned)NM2MET(3) };
	vect3_t pos_3d;
	geo_pos3_t pos;

	if (nav == NULL)
		return (NAN);

	mutex_enter(&navrad.lock);
	pos = navrad.pos;
	mutex_exit(&navrad.lock);

	pos_3d = geo2ecef(pos, &wgs84);
	dist = vect3_abs(vect3_sub(pos_3d, nav->ecef));

	brg = brg2navaid(nav, &rng);
	error = MAX_ERROR * signal_error(nav, brg, signal_db, rng);
	printf("dme error: %.2f\n", error);

	return (dist + error + nav->dme.bias);
}

bool_t
navrad_init(navaiddb_t *db)
{
	XPLMPluginID opengpws;

	ASSERT(!inited);
	inited = B_TRUE;

	memset(&navrad, 0, sizeof (navrad));
	navrad.db = db;
	mutex_init(&navrad.lock);

	fdr_find(&drs.lat, "sim/flightmodel/position/latitude");
	fdr_find(&drs.lon, "sim/flightmodel/position/longitude");
	fdr_find(&drs.elev, "sim/flightmodel/position/elevation");
	fdr_find(&drs.sim_time, "sim/time/total_running_time_sec");
	fdr_find(&drs.magvar, "sim/flightmodel/position/magnetic_variation");

	for (int i = 0; i < NUM_NAV_RADIOS; i++)
		radio_init(&navrad.radios[i], i + 1);

	XPLMRegisterFlightLoopCallback(floop_cb, -1, NULL);

	opengpws = XPLMFindPluginBySignature(OPENGPWS_PLUGIN_SIG);
	if (opengpws == XPLM_NO_PLUGIN_ID) {
		logMsg("Cannot contact OpenGPWS for the terrain data. "
		    "Is OpenGPWS installed?");
		goto errout;
	}
	XPLMSendMessageToPlugin(opengpws, EGPWS_GET_INTF, &navrad.opengpws);
	VERIFY(navrad.opengpws != NULL);

	worker_init(&navrad.worker, worker_cb, WORKER_INTVAL, NULL,
	    "navrad-worker");

	return (B_TRUE);
errout:
	navrad_fini();
	return (B_FALSE);
}

void
navrad_fini(void)
{
	if (!inited)
		return;
	inited = B_FALSE;

	worker_fini(&navrad.worker);

	for (int i = 0; i < NUM_NAV_RADIOS; i++)
		radio_fini(&navrad.radios[i]);

	mutex_destroy(&navrad.lock);
	XPLMUnregisterFlightLoopCallback(floop_cb, NULL);
}

double
navrad_get_bearing(unsigned nr, bool_t magnetic)
{
	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	return (radio_get_bearing(&navrad.radios[nr], magnetic));
}

double
navrad_get_radial(unsigned nr)
{
	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	return (radio_get_radial(&navrad.radios[nr]));
}

double
navrad_get_dme(unsigned nr)
{
	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	return (radio_get_dme(&navrad.radios[nr]));
}

int16_t *
navrad_get_audio_buf(unsigned nr, double volume, size_t *num_samples)
{
	int16_t *buf = calloc(AUDIO_BUF_NUM_SAMPLES, sizeof (*buf));
	double max_db = NOISE_FLOOR_AUDIO;
	double snr;
	radio_t *radio;
	double noise_level;

	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	radio = &navrad.radios[nr];

	mutex_enter(&radio->lock);

	for (radio_navaid_t *rnav = avl_first(&radio->vors); rnav != NULL;
	    rnav = AVL_NEXT(&radio->vors, rnav)) {
		max_db = MAX(max_db, rnav->signal_db);
	}
	snr = max_db - NOISE_FLOOR_SIGNAL;
	noise_level = 1 / MAX(max_db - NOISE_FLOOR_AUDIO, 1);

	for (radio_navaid_t *rnav = avl_first(&radio->vors); rnav != NULL;
	    rnav = AVL_NEXT(&radio->vors, rnav)) {

		rnav->cur_audio_chunk++;
		if (rnav->cur_audio_chunk >= AUDIO_BUF_NUM_CHUNKS)
			rnav->cur_audio_chunk = 0;

		if (rnav->signal_db <= NOISE_FLOOR_SIGNAL)
			continue;

		if (rnav->audio_chunks[rnav->cur_audio_chunk] != 0) {
			double level = (rnav->signal_db - NOISE_FLOOR_SIGNAL) /
			    snr;
			for (size_t i = 0; i < AUDIO_BUF_NUM_SAMPLES;
			    i += ONE_KHZ_NUM_SAMPLES) {
				for (size_t j = 0; j < ONE_KHZ_NUM_SAMPLES; j++)
					buf[i + j] += one_khz_tone[j] * level *
					    0.5;
			}
		}
	}

	distort(radio->distort, buf, AUDIO_BUF_NUM_SAMPLES, volume,
	    noise_level);

	mutex_exit(&radio->lock);

	*num_samples = AUDIO_BUF_NUM_SAMPLES;
	return (buf);
}

void
navrad_free_audio_buf(int16_t *buf)
{
	free(buf);
}

void
navrad_done_audio(unsigned nr)
{
	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	distort_clear_buffers(navrad.radios[nr].distort);
}
