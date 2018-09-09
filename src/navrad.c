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

#include <time.h>

#include <XPLMDisplay.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <acfutils/crc64.h>
#include <acfutils/dr.h>
#include <acfutils/math.h>
#include <acfutils/mt_cairo_render.h>
#include <acfutils/perf.h>
#include <acfutils/worker.h>
#include <acfutils/time.h>

#include "opengpws/xplane_api.h"

#include "distort.h"
#include "libradio/navrad.h"
#include "itm_c.h"

#define	WORKER_INTVAL		500000
#define	DEF_UPD_RATE		1
#define	MIN_DELTA_T		0.01
#define	NAVAID_SRCH_RANGE	NM2MET(300)
#define	ANT_BASE_GAIN		92.0	/* dB */
#define	INTERFERENCE_LIMIT	16.0	/* dB */
#define	NOISE_LEVEL_AUDIO	-55.0	/* dB */
#define	NOISE_FLOOR_AUDIO	-80.0	/* dB */
#define	NOISE_FLOOR_NAV_ID	-73.0	/* dB */
#define	NOISE_FLOOR_SIGNAL	-70.0	/* dB */
#define	NOISE_FLOOR_TEST	-85.0	/* dB */
#define	NOISE_FLOOR_TOO_FAR	-100.0	/* dB */
#define	HDEF_MAX		2.5	/* dots */
#define	VDEF_MAX		2.5	/* dots */
#define	HDEF_VOR_DEG_PER_DOT	2.0	/* degrees per dot */
#define	HDEF_LOC_DEG_PER_DOT	1.0	/* degrees per dot */
#define	VDEF_GS_DEG_PER_DOT	3.5714	/* degrees per dot */
#define	HSENS_VOR		12.0	/* dot deflection per deg correction */
#define	HSENS_LOC		12.0	/* dot deflection per deg correction */
#define	HDEF_FEEDBACK		15	/* dots per second */
#define	MAX_INTCPT_ANGLE	30	/* degrees */
#define	HDEF_RATE_UPD_RATE	0.4	/* FILTER_IN rate arg */
#define	VDEF_RATE_UPD_RATE	0.4	/* FILTER_IN rate arg */
#define	AP_STEER_UPD_RATE	0.25	/* FILTER_IN rate arg */
#define	BRG_UPD_RATE		2.5	/* FILTER_IN rate arg */
#define	DME_UPD_RATE		1	/* FILTER_IN rate arg */
#define	AP_GS_CAPTURE_VDEF	0.2	/* dots */
#define	DME_CHG_DELAY		1	/* seconds */

#define	AUDIO_BUF_NUM_CHUNKS	110
#define	VOR_BUF_NUM_SAMPLES	4800
#define	DME_BUF_NUM_SAMPLES	4788

#define	NAVRAD_LOCK_DELAY	3	/* seconds */
#define	NAVRAD_PARKED_BRG	90	/* bearing pointer parked pos */

#define	MAX_DR_VALS		8

static bool_t inited = B_FALSE;

typedef struct radio_s radio_t;

/*
 * Sources for these enum values:
 *	1) http://www.xsquawkbox.net/xpsdk/mediawiki/Sim/cockpit/autopilot/\
 *	   autopilot_state
 *	2) http://developer.x-plane.com/?article=\
 *	   track-to-intercept-making-sense-of-lnav-and-locapp-modes-in-11-10
 */
typedef enum {
	/* Present in XP 11.00 */
	AP_ATHR =		0x00000,
	AP_HDG_SEL =		0x00002,
	AP_WING_LVL =		0x00004,
	AP_IAS_HOLD =		0x00008,
	AP_VS =			0x00010,
	AP_ALT_HOLD_ARM =	0x00020,
	AP_FLCH =		0x00040,
	AP_HNAV_ARM =		0x00100,
	AP_HNAV =		0x00200,
	AP_GS_ARM =		0x00400,
	AP_GS =			0x00800,
	AP_FMS_ARM =		0x01000,
	AP_FMS =		0x02000,
	AP_ALT_HOLD =		0x04000,
	AP_TOGA_HORIZ =		0x08000,
	AP_TOGA_VERT =		0x10000,
	AP_VNAV_ARM =		0x20000,
	AP_VNAV =		0x40000,
	/* Introduced since XP 11.10 */
	AP_GPSS =		0x80000
} ap_state_t;

typedef struct {
	radio_t		*radio;
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
	double		signal_db_omni;
	double		signal_db_tgt;
	bool_t		outdated;
	int		propmode;

	/* Only valid for VORs! */
	double		gnd_dist;
	double		slant_angle;
	double		radial_degt;

	/*
	 * Control chunks for the navaid audio generator. The value of
	 * the chunk is simply a boolean '0' for 'silence' or '1' for
	 * a 1 kHz tone. This is generated from audio_buf_chunks_encode.
	 */
	uint8_t		audio_chunks[AUDIO_BUF_NUM_CHUNKS];
	unsigned	cur_audio_chunk;

	avl_node_t	node;
} radio_navaid_t;

struct radio_s {
	unsigned	nr;

	dr_t		fail_dr[2];
	bool_t		failed;

	mutex_t		lock;
	uint64_t	freq;
	double		freq_chg_t;
	double		hdef_pilot;
	bool_t		tofrom_pilot;
	double		hdef_copilot;
	bool_t		tofrom_copilot;
	double		hdef_lock_t;
	double		gs;
	distort_t	*distort_vloc;
	distort_t	*distort_dme;
	double		brg;
	double		brg_lock_t;
	double		dme;
	double		dme_lock_t;

	double		vdef;
	double		vdef_prev;
	double		vdef_rate;
	double		vdef_lock_t;

	avl_tree_t	vlocs;
	avl_tree_t	gses;
	avl_tree_t	dmes;

	struct {
		char		id[8];
		dr_t		id_dr;
		navaid_type_t	type;
		dr_t		type_dr;
		float		signal_db;
		dr_t		signal_db_dr;
		int		propmode;
		dr_t		propmode_dr;
	} dr_vals[MAX_DR_VALS];

	struct {
		dr_t	ovrd_nav_needles;
		dr_t	freq;
		dr_t	dir_degt;
		dr_t	slope_degt;

		dr_t	crs_degm_pilot;
		dr_t	fromto_pilot;
		dr_t	hdef_pilot;
		dr_t	vdef_pilot;

		dr_t	crs_degm_copilot;
		dr_t	fromto_copilot;
		dr_t	hdef_copilot;
		dr_t	vdef_copilot;

		dr_t	dme_nm;
	} drs;
};

static struct {
	navaiddb_t		*db;

	mutex_t			lock;
	geo_pos3_t		pos;
	double			magvar;
	double			cur_t;
	double			last_t;

	struct {
		double		hdef_prev;
		double		hdef_rate;
		double		steer_tgt;
		bool_t		ovrd_act;
	} ap;

	radio_t			radios[NUM_NAV_RADIOS];
	worker_t		worker;

	const egpws_intf_t	*opengpws;
} navrad;

static struct {
	mutex_t			lock;

	unsigned		nr;
	dr_t			nr_dr;
	char			id[8];
	dr_t			id_dr;
	navaid_type_t		type;
	dr_t			type_dr;

	mutex_t			render_lock;
	double			*elev;
	size_t			num_pts;
	double			acf_alt;
	double			nav_alt;
	double			dist;
	uint64_t		freq;

	XPLMWindowID		win;
	mt_cairo_render_t	*mtcr;
} profile_debug;

static struct {
	dr_t		lat;
	dr_t		lon;
	dr_t		elev;
	dr_t		sim_time;
	dr_t		magvar;

	dr_t		ap_steer_deg_mag;
	dr_t		ovrd_nav_heading;
	dr_t		ovrd_dme;
	dr_t		ovrd_ap;
	dr_t		dme_nm;
	dr_t		hsi_sel;
	dr_t		ap_state;
	dr_t		hdg;
	dr_t		hpath;
	dr_t		alpha;
	dr_t		ap_bc;
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

/*
 * 1 KHz harmonic tone at 48 kHz sampling rate (48 samples).
 */
#define	VOR_TONE_NUM_SAMPLES	(NAVRAD_AUDIO_SRATE / 1000)
CTASSERT(NAVRAD_AUDIO_SRATE % 1000 == 0);
static const int16_t vor_tone[VOR_TONE_NUM_SAMPLES] = {
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

/*
 * Unlike a VOR, a DME generates a square pattern at 1350 Hz. This doesn't
 * divide our 48 kHz sampling rate evenly, so we instead fudge it and use
 * a 1333 Hz tone. Close enough to not be perceptibly different.
 */
#define	DME_TONE_NUM_SAMPLES	36
static const int16_t dme_tone[DME_TONE_NUM_SAMPLES] = {
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767,
    -32767
};

static double brg2navaid(navaid_t *nav, double *dist);
static double radio_get_hdef(radio_t *radio, bool_t pilot, bool_t *tofrom);
static void radio_hdef_update(radio_t *radio, bool_t pilot, double d_t);
static void radio_vdef_update(radio_t *radio, double d_t);
static double radio_get_bearing(radio_t *radio);
static double radio_get_dme(radio_t *radio);
static void radio_brg_update(radio_t *radio, double d_t);
static void radio_dme_update(radio_t *radio, double d_t);

/*
 * Computes the actual signal level at the receiver, applying various
 * propagation modeling modifiers depending on the type of navaid and
 * our relative position to it. Depending on navaid type, this function
 * modifies the signals as follows:
 *
 * 1) For VORs, when in line-of-sight propagation mode, we simulate the
 *	cone of confusion above the VOR station.
 * 2) For VORs, we suppress the transmission level by up to 20 dB,
 *	depending on the declared service volume for the navaid. This
 *	avoids too much interference from VORs intended for short-range
 *	navigation up at the high flight levels.
 * 3) For DMEs, applies a similar service-volume modification curve to
 *	the signal level. If the DME has a non-NAN `brg' passed as an
 *	argument, it is assumed to be a bearing-biased DME and we will
 *	apply the LOC signal curve.
 * 4) For LOCs, applies signal diminishing based on service volume AND
 *	diminishes the signal even more with increasing lateral deviation
 *	from the LOC centerline. If `has_bc' is B_TRUE, this maintains a
 *	somewhat narrower signal beam at the backcourse to the navaid.
 *	If `has_bc' is B_FALSE, no back-beam projection is simulated.
 * 5) For GSes does the same as for LOCs, except that no back-beam
 *	projection is simulated here.
 */
static void
comp_signal_db(radio_navaid_t *rnav, fpp_t *fpp, bool_t has_bc, double brg)
{
	const vect2_t vor_dist_curve[] = {
	    VECT2(NM2MET(0), -20),
	    VECT2(NM2MET(20), -20),
	    VECT2(NM2MET(100), 0),
	    VECT2(NM2MET(120), 0),
	    NULL_VECT2
	};
	const vect2_t dme_dist_curve[] = {
	    VECT2(NM2MET(0), 0),
	    VECT2(NM2MET(20), 0),
	    VECT2(NM2MET(100), 20),
	    VECT2(NM2MET(120), 20),
	    NULL_VECT2
	};
	const vect2_t ils_dme_dist_curve[] = {
	    VECT2(NM2MET(0), -9),
	    VECT2(NM2MET(20), -9),
	    VECT2(NM2MET(100), 11),
	    VECT2(NM2MET(120), 11),
	    NULL_VECT2
	};
	const vect2_t loc_dist_curve[] = {
	    VECT2(NM2MET(0), -30),
	    VECT2(NM2MET(10), -30),
	    VECT2(NM2MET(40), -20),
	    VECT2(NM2MET(50), -20),
	    NULL_VECT2
	};
	const vect2_t gs_dist_curve[] = {
	    VECT2(NM2MET(0), -25),
	    VECT2(NM2MET(10), -25),
	    VECT2(NM2MET(40), -15),
	    VECT2(NM2MET(50), -15),
	    NULL_VECT2
	};
	const vect2_t loc_rbrg_curve[] = {
	    VECT2(0, 0),
	    VECT2(30, -5),
	    VECT2(60, -10),
	    VECT2(90, -20),
	    VECT2(120, -20),
	    VECT2(160, -10),
	    VECT2(180, -3),
	    NULL_VECT2
	};
	const vect2_t loc_rbrg_nobc_curve[] = {
	    VECT2(0, 0),
	    VECT2(30, -5),
	    VECT2(60, -15),
	    VECT2(90, -30),
	    NULL_VECT2
	};
	const vect2_t gs_rbrg_curve[] = {
	    VECT2(0, 0),
	    VECT2(20, -5),
	    VECT2(60, -10),
	    VECT2(90, -40),
	    NULL_VECT2
	};
	navaid_t *nav = rnav->navaid;

	ASSERT(nav != NULL);

	switch (nav->type) {
	case NAVAID_VOR: {
		double angle_error = 0;

		if (rnav->propmode == ITM_PROPMODE_LOS) {
			vect2_t pos_2d = geo2fpp(GEO3_TO_GEO2(nav->pos), fpp);
			const vect2_t angle_corr_curve[] = {
			    VECT2(-5, -50),
			    VECT2(-2.5, -20),
			    VECT2(0, -10),
			    VECT2(10, -3),
			    VECT2(20, 0),
			    VECT2(30, 0),
			    VECT2(40, -3),
			    VECT2(50, -10),
			    VECT2(60, -20),
			    VECT2(90, -60),
			    NULL_VECT2
			};

			rnav->radial_degt = dir2hdg(pos_2d);
			rnav->gnd_dist = MAX(vect2_abs(pos_2d), 1);
			rnav->slant_angle = RAD2DEG(atan((navrad.pos.elev -
			    nav->pos.elev) / rnav->gnd_dist));
			angle_error = fx_lin_multi(rnav->slant_angle,
			    angle_corr_curve, B_TRUE);
		}

		rnav->signal_db = rnav->signal_db_omni +
		    fx_lin_multi(nav->range, vor_dist_curve, B_TRUE) +
		    angle_error;
		break;
	}
	case NAVAID_DME:
		if (is_valid_loc_freq(nav->freq / 1000000.0)) {
			rnav->signal_db = rnav->signal_db_omni +
			    fx_lin_multi(nav->range, ils_dme_dist_curve,
			    B_TRUE);
			if (!isnan(brg)) {
				/*
				 * On directional & paired DMEs, apply
				 * appropriate bearing bias to the signal
				 * level (especially important for opposing
				 * runways using the same ILS frequency!)
				 */
				double brg_fm_nav = brg2navaid(nav, NULL);
				double rbrg = fabs(rel_hdg(brg, brg_fm_nav));
				if (has_bc) {
					rnav->signal_db += fx_lin_multi(rbrg,
					    loc_rbrg_curve, B_TRUE);
				} else {
					rnav->signal_db += fx_lin_multi(rbrg,
					    loc_rbrg_nobc_curve, B_TRUE);
				}
			}
		} else {
			rnav->signal_db = rnav->signal_db_omni +
			    fx_lin_multi(nav->range, dme_dist_curve, B_TRUE);
		}
		break;
	case NAVAID_LOC:
	case NAVAID_GS: {
		double crs = (nav->type == NAVAID_LOC ? nav->loc.brg :
		    nav->gs.brg);
		double brg_fm_nav = brg2navaid(nav, NULL);
		double rbrg = fabs(rel_hdg(crs, brg_fm_nav));
		double signal_db = rnav->signal_db_omni;

		if (nav->type == NAVAID_LOC) {
			if (has_bc) {
				signal_db += fx_lin_multi(rbrg,
				    loc_rbrg_curve, B_TRUE);
			} else {
				signal_db += fx_lin_multi(rbrg,
				    loc_rbrg_nobc_curve, B_TRUE);
			}
			signal_db += fx_lin_multi(nav->range, loc_dist_curve,
			    B_TRUE);
		} else {
			signal_db += fx_lin_multi(rbrg, gs_rbrg_curve, B_TRUE);
			signal_db += fx_lin_multi(nav->range, gs_dist_curve,
			    B_TRUE);
		}

		rnav->signal_db = signal_db;
		break;
	}
	default:
		rnav->signal_db = rnav->signal_db_omni;
		break;
	}
}

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
		/* add two chunks between letters */
		j += 2;
		ASSERT3U(j, <, AUDIO_BUF_NUM_CHUNKS);
	}
}

/*
 * Locates a navaid which might be conflicting with `rnav' in tree `tree'.
 * This is used to locate conflicting opposite-facing LOC transmitters and
 * disable back-beam simulation. IRL these two transmitters would never be
 * run at the same time, but we don't know which one is currently in use,
 * so we instead modify the transmission diagram to kill the back-beam.
 */
static navaid_t *
find_conflicting_navaid(avl_tree_t *tree, radio_navaid_t *rnav)
{
	navaid_t *nav = rnav->navaid;

	if (nav->type != NAVAID_LOC && nav->type != NAVAID_DME)
		return (NULL);

	for (radio_navaid_t *oth_rnav = avl_first(tree); oth_rnav != NULL;
	    oth_rnav = AVL_NEXT(tree, oth_rnav)) {
		navaid_t *oth_nav = oth_rnav->navaid;

		if (oth_rnav == rnav)
			continue;
		if (strcmp(oth_nav->icao, nav->icao) == 0)
			return (oth_nav);
	}

	return (NULL);
}

static double
find_paired_loc_brg(avl_tree_t *tree, radio_navaid_t *rnav)
{
	navaid_t *nav = rnav->navaid;

	ASSERT3U(nav->type, ==, NAVAID_DME);

	for (radio_navaid_t *oth_rnav = avl_first(tree); oth_rnav != NULL;
	    oth_rnav = AVL_NEXT(tree, oth_rnav)) {
		navaid_t *oth_nav = oth_rnav->navaid;

		if (oth_nav->type == NAVAID_LOC &&
		    strcmp(nav->id, oth_nav->id) == 0 &&
		    strcmp(nav->icao, oth_nav->icao) == 0) {
			return (oth_nav->loc.brg);
		}
	}

	return (NAN);
}

static void
signal_levels_update(avl_tree_t *tree, double d_t, fpp_t *fpp,
    avl_tree_t *vlocs_tree)
{
	for (radio_navaid_t *rnav = avl_first(tree); rnav != NULL;
	    rnav = AVL_NEXT(tree, rnav)) {
		bool_t has_bc = (find_conflicting_navaid(tree, rnav) == NULL);
		double brg = NAN;

		if (vlocs_tree != NULL)
			brg = find_paired_loc_brg(vlocs_tree, rnav);
		FILTER_IN(rnav->signal_db_omni, rnav->signal_db_tgt, d_t,
		    USEC2SEC(WORKER_INTVAL));
		comp_signal_db(rnav, fpp, has_bc, brg);
	}
}

static void
ap_drs_config(double d_t)
{
	int ap_state = dr_geti(&drs.ap_state);
	int hsi_sel = dr_geti(&drs.hsi_sel);
	radio_t *radio;
	double beta, hdef, corr_def, intcpt, corr;
	bool_t is_loc;

	switch (hsi_sel) {
	case 0:
		radio = &navrad.radios[0];
		break;
	case 1:
		radio = &navrad.radios[1];
		break;
	default:
		dr_seti(&drs.ovrd_nav_heading, 0);
		return;
	}
	is_loc = is_valid_loc_freq(radio->freq / 1000000.0);

	if ((ap_state & AP_HNAV_ARM) && !(ap_state & AP_HNAV))
		dr_seti(&drs.ap_state, AP_HNAV_ARM | AP_HNAV);
	dr_seti(&drs.ovrd_nav_heading, 1);

	hdef = dr_getf(&radio->drs.hdef_pilot);
	FILTER_IN(navrad.ap.hdef_rate, (hdef - navrad.ap.hdef_prev) / d_t, d_t,
	    HDEF_RATE_UPD_RATE);
	beta = rel_hdg(normalize_hdg(dr_getf(&drs.hpath)),
	    normalize_hdg(dr_getf(&drs.hdg)));
	if (is_loc) {
		const vect2_t sens[] = {
		    VECT2(0, 24),
		    VECT2(0.5, 18),
		    VECT2(2.5, HSENS_LOC),
		    NULL_VECT2
		};
		corr_def = hdef * fx_lin_multi(ABS(hdef), sens, B_TRUE);
	} else {
		corr_def = hdef * HSENS_VOR;
	}
	if (!isnan(radio->dme) && ABS(hdef) < HDEF_MAX) {
		const vect2_t loc_pts[] = {
		    VECT2(NM2MET(0), 0.75),
		    VECT2(NM2MET(1), 0.75),
		    VECT2(NM2MET(15), 2),
		    VECT2(NM2MET(20), 2),
		    NULL_VECT2
		};
		const vect2_t vor_pts[] = {
		    VECT2(NM2MET(0), 0.5),
		    VECT2(NM2MET(5), 0.5),
		    VECT2(NM2MET(20), 2),
		    VECT2(NM2MET(40), 2),
		    NULL_VECT2
		};
		double mult;
		if (is_loc)
			mult = fx_lin_multi(radio->dme, loc_pts, B_TRUE);
		else
			mult = fx_lin_multi(radio->dme, vor_pts, B_TRUE);
		corr_def *= mult;
	}
	corr = clamp(corr_def + navrad.ap.hdef_rate * HDEF_FEEDBACK,
	    -MAX_INTCPT_ANGLE, MAX_INTCPT_ANGLE);
	if (dr_geti(&drs.ap_bc) != 0)
		corr = -corr;
	intcpt = normalize_hdg(dr_getf(&radio->drs.crs_degm_pilot) + corr +
	    beta);
	FILTER_IN(navrad.ap.steer_tgt, intcpt, d_t, AP_STEER_UPD_RATE);
	dr_setf(&drs.ap_steer_deg_mag, navrad.ap.steer_tgt);

	navrad.ap.hdef_prev = hdef;

	if ((ap_state & AP_GS_ARM) && (ap_state & AP_HNAV) &&
	    ABS(radio->vdef) < AP_GS_CAPTURE_VDEF) {
		dr_seti(&drs.ovrd_ap, 1);
		dr_seti(&drs.ap_state, AP_GS_ARM | AP_GS);
		navrad.ap.ovrd_act = B_TRUE;
	} else if (!(ap_state & AP_GS) && navrad.ap.ovrd_act) {
		dr_seti(&drs.ovrd_ap, 0);
		navrad.ap.ovrd_act = B_FALSE;
	}
	if ((isnan(radio->vdef) || isnan(radio->gs)) && (ap_state & AP_GS)) {
		dr_seti(&drs.ap_state, AP_GS);
		dr_seti(&drs.ovrd_ap, 0);
		navrad.ap.ovrd_act = B_FALSE;
	}
}

static void
ap_radio_drs_config(radio_t *radio, double d_t)
{
	double hdef;
	bool_t tofrom;

	dr_seti(&radio->drs.ovrd_nav_needles, 1);
	hdef = radio_get_hdef(radio, B_TRUE, &tofrom);
	if (!isnan(hdef)) {
		dr_setf(&radio->drs.hdef_pilot, hdef);
		dr_seti(&radio->drs.fromto_pilot, 1 + tofrom);
	} else {
		dr_seti(&radio->drs.fromto_pilot, 0);
	}

	hdef = radio_get_hdef(radio, B_FALSE, &tofrom);
	if (!isnan(hdef)) {
		dr_setf(&radio->drs.hdef_copilot, hdef);
		dr_seti(&radio->drs.fromto_copilot, 1 + tofrom);
	} else {
		dr_seti(&radio->drs.fromto_copilot, 0);
	}

	if (!isnan(radio->brg)) {
		dr_setf(&radio->drs.dir_degt, normalize_hdg(radio->brg));
	} else {
		double brg = dr_getf(&radio->drs.dir_degt);
		if (brg > 180)
			brg -= 360;
		FILTER_IN(brg, NAVRAD_PARKED_BRG, d_t, BRG_UPD_RATE);
		dr_setf(&radio->drs.dir_degt, brg);
	}

	if (!isnan(radio->dme))
		dr_setf(&radio->drs.dme_nm, MET2NM(radio->dme));
	else
		dr_setf(&radio->drs.dme_nm, 0);

	if (!isnan(radio->gs) && !isnan(radio->vdef)) {
		enum { MAX_CORR = 4 };
		double tgt = radio->gs + 5 * radio->vdef +
		    10 * radio->vdef_rate;
		tgt = clamp(tgt, radio->gs - MAX_CORR, radio->gs + MAX_CORR);
		dr_setf(&radio->drs.slope_degt, tgt);
	} else {
		dr_setf(&radio->drs.slope_degt, 0);
	}
	
	dr_setf(&radio->drs.vdef_pilot, 0);
	dr_setf(&radio->drs.vdef_copilot, 0);
}

static void
radio_floop_cb(radio_t *radio, double d_t)
{
	uint64_t new_freq = dr_getf(&radio->drs.freq) * 10000;
	fpp_t fpp = gnomo_fpp_init(GEO3_TO_GEO2(navrad.pos), 0, &wgs84,
	    B_FALSE);

	mutex_enter(&radio->lock);

	radio->failed = (dr_geti(&radio->fail_dr[0]) == 6 ||
	    dr_geti(&radio->fail_dr[1]) == 6);
	if (radio->failed)
		new_freq = 0;

	if (radio->freq != new_freq) {
		radio->freq = new_freq;
		radio->freq_chg_t = navrad.cur_t;
	}

	signal_levels_update(&radio->vlocs, d_t, &fpp, NULL);
	signal_levels_update(&radio->gses, d_t, &fpp, NULL);
	signal_levels_update(&radio->dmes, d_t, &fpp, &radio->vlocs);

	ap_drs_config(d_t);
	for (int i = 0; i < NUM_NAV_RADIOS; i++) {
		radio_hdef_update(&navrad.radios[i], B_TRUE, d_t);
		radio_hdef_update(&navrad.radios[i], B_FALSE, d_t);
		radio_vdef_update(&navrad.radios[i], d_t);
		radio_brg_update(&navrad.radios[i], d_t);
		radio_dme_update(&navrad.radios[i], d_t);
		ap_radio_drs_config(&navrad.radios[i], d_t);
	}

	dr_seti(&drs.ovrd_dme, 1);
	if (!isnan(navrad.radios[0].dme))
		dr_setf(&drs.dme_nm, MET2NM(navrad.radios[0].dme));
	else
		dr_setf(&drs.dme_nm, 0);

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
			rnav->radio = radio;
			rnav->navaid = list->navaids[i];
			audio_buf_chunks_encode(rnav);
			rnav->cur_audio_chunk =
			    crc64_rand() % AUDIO_BUF_NUM_CHUNKS;
			rnav->signal_db = NOISE_FLOOR_TOO_FAR;
			rnav->signal_db_omni = NOISE_FLOOR_TOO_FAR;
			rnav->signal_db_tgt = NOISE_FLOOR_TOO_FAR;
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
		radio_refresh_navaid_list_type(radio, &radio->vlocs,
		    pos, freq, NAVAID_VOR);
		flush_navaid_tree(&radio->gses);
		radio_refresh_navaid_list_type(radio, &radio->dmes,
		    pos, freq, NAVAID_DME);
	} else if (is_valid_loc_freq(freq / 1000000.0)) {
		radio_refresh_navaid_list_type(radio, &radio->vlocs,
		    pos, freq, NAVAID_LOC);
		radio_refresh_navaid_list_type(radio, &radio->gses,
		    pos, freq, NAVAID_GS);
		radio_refresh_navaid_list_type(radio, &radio->dmes,
		    pos, freq, NAVAID_DME);
	} else {
		mutex_enter(&radio->lock);
		flush_navaid_tree(&radio->vlocs);
		flush_navaid_tree(&radio->gses);
		flush_navaid_tree(&radio->dmes);
		mutex_exit(&radio->lock);
	}
}

static bool_t
profile_debug_check(radio_navaid_t *rnav)
{
	return (rnav->radio->nr == profile_debug.nr &&
	    rnav->navaid->type == profile_debug.type &&
	    strcmp(rnav->navaid->id, profile_debug.id) == 0);
}

static void
profile_debug_draw_cb(cairo_t *cr, unsigned w, unsigned h, void *userinfo)
{
	double *elev;
	size_t num_pts;
	double acf_alt, nav_alt, dist, extra_alt;
	double max_elev = 10;
	enum { CIRCLE_R = 10, FRESNEL_STEPS = 50 };
	double scale_x, scale_y;
	double dash[2] = {10, 10};
	uint64_t freq;

	UNUSED(userinfo);

	cairo_identity_matrix(cr);

	mutex_enter(&profile_debug.render_lock);

	if (profile_debug.elev == NULL) {
		mutex_exit(&profile_debug.render_lock);
		cairo_set_operator(cr, CAIRO_OPERATOR_CLEAR);
		cairo_paint(cr);
		cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
		cairo_set_source_rgb(cr, 1, 0, 0);
		cairo_move_to(cr, 0, 0);
		cairo_line_to(cr, w, h);
		cairo_move_to(cr, 0, h);
		cairo_line_to(cr, w, 0);
		cairo_stroke(cr);
		return;
	}

	elev = malloc(profile_debug.num_pts * sizeof (*elev));
	memcpy(elev, profile_debug.elev, profile_debug.num_pts *
	    sizeof (*elev));
	num_pts = profile_debug.num_pts;
	acf_alt = profile_debug.acf_alt;
	nav_alt = profile_debug.nav_alt;
	dist = profile_debug.dist;
	freq = profile_debug.freq;
	ASSERT(dist != 0);

	mutex_exit(&profile_debug.render_lock);

	extra_alt = EARTH_MSL * (1 - cos(M_PI * (dist / 40000000)));

	for (size_t i = 0; i < num_pts; i++) {
		double e = elev[i] + extra_alt *
		    sin((i / (double)num_pts) * M_PI);
		max_elev = MAX(max_elev, e);
	}
	max_elev = MAX(max_elev, acf_alt + 10);
	max_elev = MAX(max_elev, nav_alt + 10);
	scale_x = w / (double)(num_pts - 1);
	scale_y = h / max_elev;

	cairo_set_source_rgb(cr, 0.75, 0.75, 1);
	cairo_paint(cr);

	cairo_translate(cr, 0, h);
	cairo_scale(cr, scale_x, -scale_y);

	cairo_set_source_rgb(cr, 0.67, 0.4, 0);
	cairo_move_to(cr, 0, 0);
	for (size_t i = 0; i < num_pts; i++) {
		double e = elev[i] + extra_alt *
		    sin((i / (double)num_pts) * M_PI);
		cairo_line_to(cr, i, e);
	}
	cairo_line_to(cr, num_pts, 0);
	cairo_fill(cr);

	cairo_identity_matrix(cr);

	cairo_translate(cr, 0, h);
	cairo_scale(cr, 1, -1);

	cairo_set_source_rgb(cr, 0.45, 0.25, 0);
	cairo_move_to(cr, 0, 0);
	for (size_t i = 1; i <= 25; i++) {
		cairo_line_to(cr, w * (i / 25.0), scale_y * extra_alt *
		    sin((i / 25.0) * M_PI));
	}
	cairo_fill(cr);

	/* draw the 1st Fresnel zone as a dashed curve */
	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_set_dash(cr, dash, 2, 0);

	cairo_move_to(cr, 0, scale_y * acf_alt);
	for (int i = 1; i <= FRESNEL_STEPS; i++) {
		double f = i / (double)FRESNEL_STEPS;
		double z = sqrt((3e8 / freq) * (f * dist) * ((1 - f) * dist) /
		    dist);
		cairo_line_to(cr, w * f,
		    scale_y * (wavg(acf_alt, nav_alt, f) + z));
	}
	for (int i = FRESNEL_STEPS - 1; i >= 0; i--) {
		double f = i / (double)FRESNEL_STEPS;
		double z = sqrt((3e8 / freq) * (f * dist) * ((1 - f) * dist) /
		    dist);
		cairo_line_to(cr, w * f,
		    scale_y * (wavg(acf_alt, nav_alt, f) - z));
	}
	cairo_stroke(cr);
	cairo_set_dash(cr, NULL, 0, 0);

	cairo_set_source_rgb(cr, 1, 0, 0);
	cairo_move_to(cr, 0, scale_y * acf_alt);
	cairo_line_to(cr, w - 1, scale_y * nav_alt);
	cairo_stroke(cr);

	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_arc(cr, 0, scale_y * acf_alt, CIRCLE_R, 0, DEG2RAD(360));
	cairo_fill(cr);

	cairo_arc(cr, w - 1, scale_y * nav_alt, CIRCLE_R, 0, DEG2RAD(360));
	cairo_fill(cr);

	free(elev);
}

static double
navaid_min_hgt(double dist)
{
	return (MAX(10, MET2NM(dist) / 2));
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
	int propmode, error;
	double water_fract = 0;
	double elev_test[2] = {0, 0};
	itm_pol_t pol;
	double acf_hgt, nav_hgt;

	ASSERT(!IS_NULL_VECT(v));

	if (nav->type == NAVAID_VOR || nav->type == NAVAID_LOC ||
	    nav->type == NAVAID_GS) {
		pol = ITM_POL_HORIZ;
	} else {
		pol = ITM_POL_VERT;
	}

	/*
	 * Some navaids will be below our radio horizon. Just perform a quick
	 * "best case" test for them and discard them if there's no chance
	 * that they'll be visible.
	 */
	error = itm_point_to_pointMDH(elev_test, 2, dist, MAX(pos.elev, 3),
	    MAX(nav->pos.elev, 10), ITM_DIELEC_GND_AVG, ITM_CONDUCT_GND_AVG,
	    ITM_NS_AVG, freq / 1000000.0, ITM_ENV_CONTINENTAL_TEMPERATE, pol,
	    ITM_ACCUR_MAX, ITM_ACCUR_MAX, ITM_ACCUR_MAX, &dbloss, NULL, NULL);
	if (error > ITM_RESULT_ERANGE_MULTI) {
		if (rnav->signal_db_tgt == 0)
			rnav->signal_db_tgt = NOISE_FLOOR_TOO_FAR;
		if (profile_debug_check(rnav)) {
			printf("Navaid %s profile threw error: %d\n", nav->id,
			    error);
			if (profile_debug.elev != NULL) {
				free(profile_debug.elev);
				profile_debug.elev = NULL;
			}
		}
		return;
	}
	if (ANT_BASE_GAIN - dbloss < NOISE_FLOOR_TEST) {
		rnav->signal_db_tgt = NOISE_FLOOR_TOO_FAR;
		if (profile_debug_check(rnav)) {
			printf("Navaid %s definitely out of range (flat "
			    "terrain estimate: %.1f dB)\n", nav->id,
			    ANT_BASE_GAIN - dbloss);
			if (profile_debug.elev != NULL) {
				free(profile_debug.elev);
				profile_debug.elev = NULL;
			}
		}
		return;
	}

	memset(&probe, 0, sizeof (probe));
	probe.num_pts = clampi(dist / SPACING, 2, MAX_PTS);
	water_part = 1.0 / probe.num_pts;
	probe.in_pts = calloc(probe.num_pts, sizeof (*probe.in_pts));
	probe.out_elev = calloc(probe.num_pts, sizeof (*probe.out_elev));
	probe.out_water = calloc(probe.num_pts, sizeof (*probe.out_water));
	probe.filter_lin = B_TRUE;

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

	/*
	 * Some navaid DB entries are incorrect and list the navaid as
	 * "below ground" (or elevation zero if unknown). Correct those
	 * and clamp the height of the navaid to be at a minimum on the
	 * ground (+10 meters for height).
	 */
	acf_hgt = MAX(pos.elev - probe.out_elev[0], 3);
	nav_hgt = MAX(nav->pos.elev - probe.out_elev[probe.num_pts - 1],
	    navaid_min_hgt(dist));

	if (profile_debug_check(rnav)) {
		mutex_enter(&profile_debug.render_lock);
		if (profile_debug.elev != NULL)
			free(profile_debug.elev);
		profile_debug.elev = malloc(probe.num_pts *
		    sizeof (*profile_debug.elev));
		memcpy(profile_debug.elev, probe.out_elev,
		    probe.num_pts * sizeof (*profile_debug.elev));
		profile_debug.num_pts = probe.num_pts;
		profile_debug.acf_alt = probe.out_elev[0] + acf_hgt;
		profile_debug.nav_alt = probe.out_elev[probe.num_pts - 1] +
		    nav_hgt;
		profile_debug.dist = dist;
		profile_debug.freq = navaid_act_freq(nav->type, nav->freq);
		mutex_exit(&profile_debug.render_lock);
	}

	error = itm_point_to_pointMDH(probe.out_elev, probe.num_pts, dist,
	    acf_hgt, nav_hgt, dielec, conduct, ITM_NS_AVG, (freq / 1000000.0),
	    ITM_ENV_CONTINENTAL_TEMPERATE, pol, ITM_ACCUR_MAX, ITM_ACCUR_MAX,
	    ITM_ACCUR_MAX, &dbloss, &propmode, NULL);

	free(probe.in_pts);
	free(probe.out_elev);
	free(probe.out_water);

	if (error > ITM_RESULT_ERANGE_MULTI)
		return;

	rnav->signal_db_tgt = ANT_BASE_GAIN - dbloss;
	rnav->propmode = propmode;
}

static void
radio_dr_slot_populate(radio_t *radio, radio_navaid_t *rnav, unsigned nr)
{
	if (nr >= MAX_DR_VALS)
		return;

	mutex_enter(&radio->lock);
	strlcpy(radio->dr_vals[nr].id, rnav->navaid->id,
	    sizeof (radio->dr_vals[nr].id));
	radio->dr_vals[nr].type = rnav->navaid->type;
	radio->dr_vals[nr].signal_db = rnav->signal_db;
	radio->dr_vals[nr].propmode = rnav->propmode;
	mutex_exit(&radio->lock);
}

static void
radio_worker(radio_t *radio, geo_pos3_t pos, fpp_t *fpp)
{
	uint64_t freq;
	unsigned dr_slot = 0;

	mutex_enter(&radio->lock);
	freq = radio->freq;
	mutex_exit(&radio->lock);

	radio_refresh_navaid_list(radio, GEO3_TO_GEO2(pos), freq);

	/*
	 * Don't have to grab the lock here, since we're the only ones that
	 * can modify the trees and we're not going to be doing so here.
	 */
	for (radio_navaid_t *rnav = avl_first(&radio->vlocs); rnav != NULL;
	    rnav = AVL_NEXT(&radio->vlocs, rnav)) {
		radio_navaid_recompute_signal(rnav,
		    navaid_act_freq(rnav->navaid->type, freq), pos, fpp);
		radio_dr_slot_populate(radio, rnav, dr_slot++);
	}
	for (radio_navaid_t *rnav = avl_first(&radio->gses); rnav != NULL;
	    rnav = AVL_NEXT(&radio->gses, rnav)) {
		radio_navaid_recompute_signal(rnav,
		    navaid_act_freq(rnav->navaid->type, freq), pos, fpp);
		radio_dr_slot_populate(radio, rnav, dr_slot++);
	}
	for (radio_navaid_t *rnav = avl_first(&radio->dmes); rnav != NULL;
	    rnav = AVL_NEXT(&radio->dmes, rnav)) {
		radio_navaid_recompute_signal(rnav,
		    navaid_act_freq(rnav->navaid->type, freq), pos, fpp);
		radio_dr_slot_populate(radio, rnav, dr_slot++);
	}

	mutex_enter(&radio->lock);
	for (; dr_slot < MAX_DR_VALS; dr_slot++) {
		radio->dr_vals[dr_slot].id[0] = '\0';
		radio->dr_vals[dr_slot].type = 0;
		radio->dr_vals[dr_slot].signal_db = 0;
		radio->dr_vals[dr_slot].propmode = 0;
	}
	mutex_exit(&radio->lock);
}

static void
draw_debug_win_cb(XPLMWindowID win_id, void *refcon)
{
	int left, top, right, bottom, w, h;
	UNUSED(refcon);

	ASSERT(profile_debug.mtcr != NULL);

	XPLMGetWindowGeometry(win_id, &left, &top, &right, &bottom);
	w = right - left;
	h = top - bottom;
	mt_cairo_render_draw(profile_debug.mtcr, VECT2(left, bottom),
	    VECT2(w, h));
}

static void
profile_debug_floop(void)
{
	bool_t act;

	if (profile_debug.win != NULL &&
	    !XPLMGetWindowIsVisible(profile_debug.win))
		profile_debug.id[0] = '\0';

	act = (strlen(profile_debug.id) != 0);

	if (act && profile_debug.win == NULL) {
		XPLMCreateWindow_t cr = {
		    .structSize = sizeof (cr),
		    .visible = 1,
		    .left = 10, .bottom = 10,
		    .drawWindowFunc = draw_debug_win_cb,
		    .layer = xplm_WindowLayerFloatingWindows,
		    .decorateAsFloatingWindow = 1
		};
		int w, h;

		XPLMGetScreenSize(&w, &h);
		cr.right = w - 10;
		cr.top = h / 3;
		profile_debug.win = XPLMCreateWindowEx(&cr);
		VERIFY(profile_debug.win != NULL);

		profile_debug.mtcr = mt_cairo_render_init(w, h / 3, 1, NULL,
		    profile_debug_draw_cb, NULL, NULL);
	} else if (!act && profile_debug.win != NULL) {
		XPLMDestroyWindow(profile_debug.win);
		profile_debug.win = NULL;
		mt_cairo_render_fini(profile_debug.mtcr);
		profile_debug.mtcr = NULL;
	}
}

static float
floop_cb(float elapsed1, float elapsed2, int counter, void *refcon)
{
	double d_t;

	navrad.cur_t = dr_getf(&drs.sim_time);
	d_t = ABS(navrad.cur_t - navrad.last_t);

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

	profile_debug_floop();

out:
	navrad.last_t = navrad.cur_t;

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
	avl_create(&radio->vlocs, navrad_navaid_compar,
	    sizeof (radio_navaid_t), offsetof(radio_navaid_t, node));
	avl_create(&radio->gses, navrad_navaid_compar,
	    sizeof (radio_navaid_t), offsetof(radio_navaid_t, node));
	avl_create(&radio->dmes, navrad_navaid_compar,
	    sizeof (radio_navaid_t), offsetof(radio_navaid_t, node));
	radio->distort_vloc = distort_init(NAVRAD_AUDIO_SRATE);
	radio->distort_dme = distort_init(NAVRAD_AUDIO_SRATE);
	fdr_find(&radio->fail_dr[0], "sim/operation/failures/rel_navcom%d", nr);
	fdr_find(&radio->fail_dr[1], "sim/operation/failures/rel_nav%d", nr);

	for (int i = 0; i < MAX_DR_VALS; i++) {
		dr_create_b(&radio->dr_vals[i].id_dr, radio->dr_vals[i].id,
		    sizeof (radio->dr_vals[i].id), B_FALSE,
		    "libradio/radio%d/navaid%d/id", nr, i);
		dr_create_i(&radio->dr_vals[i].type_dr,
		    (int *)&radio->dr_vals[i].type, B_FALSE,
		    "libradio/radio%d/navaid%d/type", nr, i);
		dr_create_f(&radio->dr_vals[i].signal_db_dr,
		    &radio->dr_vals[i].signal_db, B_FALSE,
		    "libradio/radio%d/navaid%d/signal_db", nr, i);
		dr_create_i(&radio->dr_vals[i].propmode_dr,
		    &radio->dr_vals[i].propmode, B_FALSE,
		    "libradio/radio%d/navaid%d/propmode", nr, i);
	}

	fdr_find(&radio->drs.ovrd_nav_needles,
	    "sim/operation/override/override_nav%d_needles", nr);
	fdr_find(&radio->drs.freq,
	    "sim/cockpit2/radios/actuators/nav%d_frequency_hz", nr);
	fdr_find(&radio->drs.dir_degt, "sim/cockpit/radios/nav%d_dir_degt", nr);
	fdr_find(&radio->drs.slope_degt,
	    "sim/cockpit/radios/nav%d_slope_degt", nr);

	fdr_find(&radio->drs.crs_degm_pilot,
	    "sim/cockpit2/radios/actuators/nav%d_obs_deg_mag_pilot", nr);
	fdr_find(&radio->drs.fromto_pilot,
	    "sim/cockpit/radios/nav%d_fromto", nr);
	fdr_find(&radio->drs.hdef_pilot,
	    "sim/cockpit/radios/nav%d_hdef_dot", nr);
	fdr_find(&radio->drs.vdef_pilot,
	    "sim/cockpit/radios/nav%d_vdef_dot", nr);

	fdr_find(&radio->drs.crs_degm_copilot,
	    "sim/cockpit2/radios/actuators/nav%d_obs_deg_mag_copilot", nr);
	fdr_find(&radio->drs.fromto_copilot,
	    "sim/cockpit/radios/nav%d_fromto2", nr);
	fdr_find(&radio->drs.hdef_copilot,
	    "sim/cockpit/radios/nav%d_hdef_dot2", nr);
	fdr_find(&radio->drs.vdef_copilot,
	    "sim/cockpit/radios/nav%d_vdef_dot2", nr);
	fdr_find(&radio->drs.dme_nm,
	    "sim/cockpit/radios/nav%d_dme_dist_m", nr);
}

static void
radio_fini(radio_t *radio)
{
	void *cookie;
	radio_navaid_t *nav;

	cookie = NULL;
	while ((nav = avl_destroy_nodes(&radio->vlocs, &cookie)) != NULL)
		free(nav);
	avl_destroy(&radio->vlocs);

	cookie = NULL;
	while ((nav = avl_destroy_nodes(&radio->gses, &cookie)) != NULL)
		free(nav);
	avl_destroy(&radio->gses);

	cookie = NULL;
	while ((nav = avl_destroy_nodes(&radio->dmes, &cookie)) != NULL)
		free(nav);
	avl_destroy(&radio->dmes);

	if (radio->distort_vloc != NULL)
		distort_fini(radio->distort_vloc);
	if (radio->distort_dme != NULL)
		distort_fini(radio->distort_dme);

	for (int i = 0; i < MAX_DR_VALS; i++) {
		dr_delete(&radio->dr_vals[i].id_dr);
		dr_delete(&radio->dr_vals[i].type_dr);
		dr_delete(&radio->dr_vals[i].signal_db_dr);
		dr_delete(&radio->dr_vals[i].propmode_dr);
	}

	dr_seti(&radio->drs.ovrd_nav_needles, 0);

	mutex_destroy(&radio->lock);
}

static radio_navaid_t *
radio_get_strongest_navaid(radio_t *radio, avl_tree_t *tree,
    double recv_floor)
{
	radio_navaid_t *strongest = NULL, *second = NULL;
	radio_navaid_t *winner = NULL;

	mutex_enter(&radio->lock);

	for (radio_navaid_t *rnav = avl_first(tree); rnav != NULL;
	    rnav = AVL_NEXT(tree, rnav)) {
		double signal_db = rnav->signal_db;

		if (signal_db < recv_floor)
			continue;
		if (strongest == NULL) {
			strongest = rnav;
		} else if (signal_db > strongest->signal_db) {
			second = strongest;
			strongest = rnav;
		} else if (second == NULL || (second != NULL &&
		    rnav->signal_db > second->signal_db)) {
			second = rnav;
		}
	}
	if (strongest != NULL)
		ASSERT3P(strongest, !=, second);

	if (second != NULL &&
	    strongest->signal_db - second->signal_db < INTERFERENCE_LIMIT) {
		winner = NULL;
	} else if (strongest != NULL) {
		winner = strongest;
	}

	mutex_exit(&radio->lock);

	return (winner);
}

static double
signal_error(navaid_t *nav, double signal_db)
{
	double pos_seed = crc64(&nav->pos, sizeof (nav->pos)) /
	    (double)UINT64_MAX;
	double time_seed = navrad.last_t / 3600.0;
	double signal_seed = clamp(signal_db / NOISE_FLOOR_SIGNAL, 0, 1);

	signal_seed = POW4(signal_seed) * POW4(signal_seed);

	return (signal_seed * sin(pos_seed + time_seed));
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
vor_cone_error(radio_navaid_t *rnav)
{
	double f = clamp((rnav->slant_angle - 60) / 30, 0, 1);
	double fact = POW3(f);
	navaid_t *nav = rnav->navaid;
	double freq = nav->freq / 1000000.0;
	enum { MAX_ERROR = 20 };
	/*
	 * We use the current radial to see the randomizer, but amplify the
	 * radial value to amplify any small variations. Also use the VOR
	 * frequency to provide a more unique randomized response for every
	 * VOR.
	 */
	return (MAX_ERROR * fact * sin(rnav->radial_degt * 2.1 + freq) *
	    sin(rnav->radial_degt * 4.35 + freq));
}

static double
radio_get_bearing(radio_t *radio)
{
	double brg, error;
	radio_navaid_t *rnav = radio_get_strongest_navaid(radio, &radio->vlocs,
	    NOISE_FLOOR_AUDIO);
	navaid_t *nav = (rnav != NULL ? rnav->navaid : NULL);
	enum { MAX_ERROR = 3 };

	if (nav == NULL || nav->type == NAVAID_LOC ||
	    ABS(navrad.cur_t - radio->freq_chg_t) < DME_CHG_DELAY)
		return (NAN);

	brg = brg2navaid(nav, NULL);
	error = MAX_ERROR * signal_error(nav, rnav->signal_db) +
	    vor_cone_error(rnav);

	return (normalize_hdg(brg + error));
}

static double
radio_get_radial(radio_t *radio)
{
	double radial, error;
	radio_navaid_t *rnav = radio_get_strongest_navaid(radio, &radio->vlocs,
	    NOISE_FLOOR_AUDIO);
	navaid_t *nav = (rnav != NULL ? rnav->navaid : NULL);
	enum { MAX_ERROR = 3 };

	if (nav == NULL || nav->type == NAVAID_LOC ||
	    ABS(navrad.cur_t - radio->freq_chg_t) < DME_CHG_DELAY)
		return (NAN);

	radial = normalize_hdg(brg2navaid(nav, NULL) + 180);
	error = MAX_ERROR * signal_error(nav, rnav->signal_db) +
	    vor_cone_error(rnav);

	return (normalize_hdg(radial + error - nav->vor.magvar));
}

static double
radio_get_dme(radio_t *radio)
{
	double dist, error;
	radio_navaid_t *rnav = radio_get_strongest_navaid(radio, &radio->dmes,
	    NOISE_FLOOR_AUDIO);
	navaid_t *nav = (rnav != NULL ? rnav->navaid : NULL);
	vect3_t pos_3d;
	geo_pos3_t pos;
	const double MAX_ERROR = 0.025;

	if (nav == NULL ||
	    ABS(navrad.cur_t - radio->freq_chg_t) < DME_CHG_DELAY)
		return (NAN);

	mutex_enter(&navrad.lock);
	pos = navrad.pos;
	mutex_exit(&navrad.lock);

	pos_3d = geo2ecef_mtr(pos, &wgs84);
	dist = vect3_abs(vect3_sub(pos_3d, nav->ecef));

	error = dist * MAX_ERROR * signal_error(nav, rnav->signal_db);

	return (dist + error + nav->dme.bias);
}

static double
radio_comp_hdef_loc(radio_t *radio)
{
	radio_navaid_t *rnav = radio_get_strongest_navaid(radio, &radio->vlocs,
	    NOISE_FLOOR_AUDIO);
	navaid_t *nav = (rnav != NULL ? rnav->navaid : NULL);
	const double MAX_ERROR = 1.5;
	double brg, error, hdef;

	if (nav == NULL ||
	    ABS(navrad.cur_t - radio->freq_chg_t) < DME_CHG_DELAY) {
		return (NAN);
	}
	brg = brg2navaid(nav, NULL);
	error = MAX_ERROR * signal_error(nav, rnav->signal_db);
	brg = normalize_hdg(brg + error);

	hdef = rel_hdg(nav->loc.brg, brg);
	/* simulate reverse sensing for the back-course */
	if (hdef > 90)
		hdef = 180 - hdef;
	else if (hdef < -90)
		hdef = -180 - hdef;
	hdef /= HDEF_LOC_DEG_PER_DOT;

	return (hdef);
}

static double
radio_comp_hdef_vor(radio_t *radio, bool_t pilot, bool_t *tofrom_p)
{
	double radial = radio_get_radial(radio);
	double crs, hdef;

	if (pilot)
		crs = dr_getf(&radio->drs.crs_degm_pilot);
	else
		crs = dr_getf(&radio->drs.crs_degm_copilot);

	if (isnan(radial) || isnan(crs))
		return (NAN);
	radial = normalize_hdg(radial);
	crs = normalize_hdg(crs);

	if (ABS(rel_hdg(crs, radial)) < 90) {
		hdef = rel_hdg(radial, crs);
		*tofrom_p = B_TRUE;
	} else {
		hdef = rel_hdg(normalize_hdg(crs + 180), radial);
		*tofrom_p = B_FALSE;
	}
	hdef /= HDEF_VOR_DEG_PER_DOT;

	return (hdef);
}

static void
radio_hdef_update(radio_t *radio, bool_t pilot, double d_t)
{
	double hdef;
	bool_t tofrom = B_FALSE;

	if (is_valid_loc_freq(radio->freq / 1000000.0)) {
		hdef = radio_comp_hdef_loc(radio);
		tofrom = B_FALSE;
	} else {
		hdef = radio_comp_hdef_vor(radio, pilot, &tofrom);
	}

	if (!isnan(hdef)) {
		if (ABS(navrad.cur_t - radio->hdef_lock_t) < NAVRAD_LOCK_DELAY)
			return;

		if (pilot) {
			FILTER_IN_NAN(radio->hdef_pilot, hdef, d_t,
			    DEF_UPD_RATE);
			radio->tofrom_pilot = tofrom;
		} else {
			FILTER_IN_NAN(radio->hdef_copilot, hdef, d_t,
			    DEF_UPD_RATE);
			radio->tofrom_copilot = tofrom;
		}
	} else {
		if (pilot)
			radio->hdef_pilot = NAN;
		else
			radio->hdef_copilot = NAN;
		radio->hdef_lock_t = NAN;
	}
}

static void
radio_vdef_update(radio_t *radio, double d_t)
{
	radio_navaid_t *rnav = radio_get_strongest_navaid(radio, &radio->gses,
	    NOISE_FLOOR_AUDIO);
	navaid_t *nav = (rnav != NULL ? rnav->navaid : NULL);
	double signal_db = (rnav != NULL ? rnav->signal_db : 0);
	double brg, dist, long_dist, d_elev, angle, vdef, error, angle_eff;
	const double MAX_ERROR = 0.5, OFFPATH_MAX_ERROR = 4.0;
	const double rand_coeffs[] = { M_PI, 2.12, 12.28, 35.12, 75.21 };
	double offpath;
	const vect2_t signal_angle_curve[] = {
	    VECT2(0, 0),
	    VECT2(5, -2),
	    VECT2(20, -5),
	    VECT2(90, -20),
	    NULL_VECT2
	};

	if (nav == NULL) {
		radio->vdef = NAN;
		radio->gs = NAN;
		radio->vdef_lock_t = navrad.cur_t;
		return;
	}

	if (ABS(navrad.cur_t - radio->vdef_lock_t) < NAVRAD_LOCK_DELAY)
		return;

	brg = brg2navaid(nav, &dist);
	offpath = fabs(rel_hdg(brg, nav->gs.brg));
	long_dist = dist * cos(DEG2RAD(offpath));
	d_elev = navrad.pos.elev - nav->pos.elev;
	if (ABS(long_dist) > 0.1)
		angle = RAD2DEG(atan(d_elev / long_dist));
	else
		angle = 90;

	signal_db += fx_lin_multi(ABS(angle), signal_angle_curve, B_TRUE);
	error = MAX_ERROR * signal_error(nav, signal_db + 5);
	error += OFFPATH_MAX_ERROR *
	    sin(nav->gs.brg + offpath / rand_coeffs[0]) *
	    sin(nav->gs.brg + offpath / rand_coeffs[1]) *
	    sin(nav->gs.brg + offpath / rand_coeffs[2]) *
	    sin(offpath / rand_coeffs[3]) * sin(offpath / rand_coeffs[4]);

	angle_eff = (((int)(angle * 1000)) % ((int)(nav->gs.gs * 2 * 1000))) /
	    1000.0;
	vdef = ((angle_eff + error) - nav->gs.gs) * VDEF_GS_DEG_PER_DOT;

	FILTER_IN_NAN(radio->vdef, vdef, d_t, DEF_UPD_RATE);
	radio->vdef = clamp(radio->vdef, -VDEF_MAX, VDEF_MAX);
	radio->gs = nav->gs.gs;

	FILTER_IN(radio->vdef_rate, (radio->vdef - radio->vdef_prev) / d_t,
	    d_t, VDEF_RATE_UPD_RATE);
	radio->vdef_prev = radio->vdef;
}

static void
radio_brg_update(radio_t *radio, double d_t)
{
	double brg = radio_get_bearing(radio);

	if (!isnan(brg)) {
		if (ABS(navrad.cur_t - radio->brg_lock_t) < NAVRAD_LOCK_DELAY)
			return;
		if (isnan(radio->brg))
			radio->brg = NAVRAD_PARKED_BRG;
		brg = normalize_hdg(brg - dr_getf(&drs.hdg));
		if (brg > 180)
			brg -= 360;
		FILTER_IN(radio->brg, brg, d_t, BRG_UPD_RATE);
	} else {
		radio->brg = NAN;
		radio->brg_lock_t = navrad.cur_t;
	}
}

static void
radio_dme_update(radio_t *radio, double d_t)
{
	double dme = radio_get_dme(radio);

	if (!isnan(dme)) {
		if (ABS(navrad.cur_t - radio->dme_lock_t) < NAVRAD_LOCK_DELAY)
			return;
		FILTER_IN_NAN(radio->dme, dme, d_t, DME_UPD_RATE);
	} else {
		radio->dme = NAN;
		radio->dme_lock_t = navrad.cur_t;
	}
}

static double
radio_get_hdef(radio_t *radio, bool_t pilot, bool_t *tofrom)
{
	ASSERT(tofrom != NULL);
	if (pilot && !isnan(radio->hdef_pilot)) {
		*tofrom = radio->tofrom_pilot;
		return (clamp(radio->hdef_pilot, -HDEF_MAX, HDEF_MAX));
	}
	if (!pilot && !isnan(radio->hdef_copilot)) {
		*tofrom = radio->tofrom_copilot;
		return (clamp(radio->hdef_copilot, -HDEF_MAX, HDEF_MAX));
	}
	return (NAN);
}

bool_t
navrad_init(navaiddb_t *db)
{
	XPLMPluginID opengpws;

	ASSERT(!inited);
	inited = B_TRUE;

	memset(&navrad, 0, sizeof (navrad));
	memset(&profile_debug, 0, sizeof (profile_debug));

	navrad.db = db;
	mutex_init(&navrad.lock);

	fdr_find(&drs.lat, "sim/flightmodel/position/latitude");
	fdr_find(&drs.lon, "sim/flightmodel/position/longitude");
	fdr_find(&drs.elev, "sim/flightmodel/position/elevation");
	fdr_find(&drs.sim_time, "sim/time/total_running_time_sec");
	fdr_find(&drs.magvar, "sim/flightmodel/position/magnetic_variation");

	fdr_find(&drs.ap_steer_deg_mag,
	    "sim/cockpit/autopilot/nav_steer_deg_mag");
	fdr_find(&drs.ovrd_nav_heading,
	    "sim/operation/override/override_nav_heading");
	fdr_find(&drs.ovrd_dme, "sim/operation/override/override_dme");
	fdr_find(&drs.dme_nm, "sim/cockpit/radios/standalone_dme_dist_m");
	fdr_find(&drs.hsi_sel,
	    "sim/cockpit2/radios/actuators/HSI_source_select_pilot");
	fdr_find(&drs.ap_state, "sim/cockpit/autopilot/autopilot_state");
	fdr_find(&drs.ap_bc, "sim/cockpit2/autopilot/backcourse_on");
	fdr_find(&drs.hdg, "sim/flightmodel/position/psi");
	fdr_find(&drs.hpath, "sim/flightmodel/position/hpath");

	fdr_find(&drs.ovrd_ap, "sim/operation/override/override_autopilot");
	fdr_find(&drs.alpha, "sim/flightmodel/position/alpha");

	for (int i = 0; i < NUM_NAV_RADIOS; i++)
		radio_init(&navrad.radios[i], i + 1);

	XPLMRegisterFlightLoopCallback(floop_cb, -1, NULL);

	mutex_init(&profile_debug.lock);
	mutex_init(&profile_debug.render_lock);
	dr_create_i(&profile_debug.nr_dr, (int *)&profile_debug.nr, B_TRUE,
	    "libradio/debug/radio");
	dr_create_b(&profile_debug.id_dr, profile_debug.id,
	    sizeof (profile_debug.id) - 1, B_TRUE, "libradio/debug/navaid");
	dr_create_i(&profile_debug.type_dr, (int *)&profile_debug.type,
	    B_TRUE, "libradio/debug/type");

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

	/*
	 * Must go ahead profile_debug destruction to guarantee that
	 * the navrad worker won't try to use destroyed locks.
	 */
	worker_fini(&navrad.worker);

	if (profile_debug.win != NULL)
		XPLMDestroyWindow(profile_debug.win);
	if (profile_debug.mtcr != NULL)
		mt_cairo_render_fini(profile_debug.mtcr);
	free(profile_debug.elev);
	dr_delete(&profile_debug.nr_dr);
	dr_delete(&profile_debug.id_dr);
	dr_delete(&profile_debug.type_dr);
	mutex_destroy(&profile_debug.render_lock);
	mutex_destroy(&profile_debug.lock);

	for (int i = 0; i < NUM_NAV_RADIOS; i++)
		radio_fini(&navrad.radios[i]);

	dr_seti(&drs.ovrd_nav_heading, 0);
	dr_seti(&drs.ovrd_dme, 0);
	dr_seti(&drs.ovrd_ap, 0);

	mutex_destroy(&navrad.lock);
	XPLMUnregisterFlightLoopCallback(floop_cb, NULL);
}

uint64_t
navrad_get_freq(unsigned nr)
{
	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	if (navrad.radios[nr].failed)
		return (0);
	return (navrad.radios[nr].freq);
}

double
navrad_get_bearing(unsigned nr)
{
	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	if (navrad.radios[nr].failed)
		return (NAN);
	return (normalize_hdg(navrad.radios[nr].brg));
}

double
navrad_get_radial(unsigned nr)
{
	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	if (navrad.radios[nr].failed)
		return (NAN);
	return (radio_get_radial(&navrad.radios[nr]));
}

double
navrad_get_dme(unsigned nr)
{
	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	if (navrad.radios[nr].failed)
		return (NAN);
	return (navrad.radios[nr].dme);
}

double
navrad_get_hdef(unsigned nr, bool_t pilot, bool_t *tofrom)
{
	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	if (navrad.radios[nr].failed)
		return (NAN);
	return (radio_get_hdef(&navrad.radios[nr], pilot, tofrom));
}

double
navrad_get_vdef(unsigned nr)
{
	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	if (navrad.radios[nr].failed)
		return (NAN);
	return (navrad.radios[nr].vdef);
}

bool_t 
navrad_is_loc(unsigned nr)
{
	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	return (is_valid_loc_freq(navrad.radios[nr].freq / 1000000.0));
}

bool_t
navrad_get_ID(unsigned nr, char id[8])
{
	radio_t *radio;
	navaid_t *nav;
	radio_navaid_t *rnav;

	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	radio = &navrad.radios[nr];
	if (radio->failed)
		return (B_FALSE);

	mutex_enter(&radio->lock);
	rnav = radio_get_strongest_navaid(radio, &radio->vlocs,
	    NOISE_FLOOR_TEST);
	if (rnav == NULL) {
		mutex_exit(&radio->lock);
		return (B_FALSE);
	}
	nav = rnav->navaid;
	mutex_exit(&radio->lock);

	strlcpy(id, nav->id, 8);
	return (B_TRUE);
}

static int16_t *
get_audio_buf_type(radio_t *radio, avl_tree_t *tree, double volume,
    const int16_t *tone, size_t step, size_t num_samples, bool_t squelch,
    distort_t *distort_ctx)
{
	int16_t *buf = calloc(num_samples, sizeof (*buf));
	double max_db = NOISE_LEVEL_AUDIO;
	double tone_db = NOISE_FLOOR_NAV_ID;
	double span, noise_level;

	mutex_enter(&radio->lock);

	for (radio_navaid_t *rnav = avl_first(tree); rnav != NULL;
	    rnav = AVL_NEXT(tree, rnav)) {
		if (rnav->signal_db <= NOISE_FLOOR_AUDIO)
			continue;
		/*
		 * We use the navaid into the signal estimation only when
		 * there is a tone on the frequency.
		 */
		if (rnav->audio_chunks[rnav->cur_audio_chunk] != 0) {
			max_db = MAX(max_db, rnav->signal_db);
			tone_db = MAX(tone_db, rnav->signal_db);
		}
	}

	if (squelch && tone_db <= NOISE_FLOOR_NAV_ID) {
		mutex_exit(&radio->lock);
		return (buf);
	}

	span = max_db - NOISE_FLOOR_AUDIO;
	noise_level = (NOISE_LEVEL_AUDIO - NOISE_FLOOR_AUDIO) / span;

	for (radio_navaid_t *rnav = avl_first(tree); rnav != NULL;
	    rnav = AVL_NEXT(tree, rnav)) {
		if (rnav->signal_db <= NOISE_FLOOR_AUDIO)
			continue;

		if (rnav->audio_chunks[rnav->cur_audio_chunk] != 0) {
			double level =
			    (rnav->signal_db - NOISE_FLOOR_AUDIO) / span;
			for (size_t i = 0; i < num_samples; i += step) {
				for (size_t j = 0; j < step; j++)
					buf[i + j] += tone[j] * POW2(level);
			}
		}

		rnav->cur_audio_chunk++;
		if (rnav->cur_audio_chunk >= AUDIO_BUF_NUM_CHUNKS)
			rnav->cur_audio_chunk = 0;
	}

	distort(distort_ctx, buf, num_samples, POW2(volume),
	    POW2(noise_level * volume));

	mutex_exit(&radio->lock);

	return (buf);
}

int16_t *
navrad_get_audio_buf(unsigned nr, double volume, bool_t is_dme,
    bool_t squelch, size_t *num_samples)
{
	radio_t *radio;
	size_t samples = (!is_dme ? VOR_BUF_NUM_SAMPLES : DME_BUF_NUM_SAMPLES);
	size_t step = (!is_dme ? VOR_TONE_NUM_SAMPLES : DME_TONE_NUM_SAMPLES);
	avl_tree_t *tree;
	int16_t *buf;
	const int16_t *tone = (!is_dme ? dme_tone : vor_tone);
	distort_t *distort;

	ASSERT3U(nr, <, NUM_NAV_RADIOS);
	radio = &navrad.radios[nr];
	if (radio->failed) {
		*num_samples = 0;
		return (NULL);
	}
	tree = (!is_dme ? &radio->vlocs : &radio->dmes);
	distort = (!is_dme ? radio->distort_vloc : radio->distort_dme);

	buf = get_audio_buf_type(radio, tree, volume, tone, step, samples,
	    squelch, distort);

	*num_samples = samples;
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
	distort_clear_buffers(navrad.radios[nr].distort_vloc);
	distort_clear_buffers(navrad.radios[nr].distort_dme);
}
