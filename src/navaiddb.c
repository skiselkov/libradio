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

#include <errno.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <XPLMGraphics.h>
#include <XPLMScenery.h>

#include <acfutils/log.h>
#include <acfutils/helpers.h>
#include <acfutils/htbl.h>
#include <acfutils/perf.h>
#include <acfutils/safe_alloc.h>
#include <acfutils/thread.h>

#include "libradio/navaiddb.h"

#define	EARTH_NAV_MIN_VERSION	1100
#define	EARTH_NAV_MAX_VERSION	1100

#define	HZ2MHZ(freq)	((freq / 1000) / 1000.0)
#define	HZ2KHZ(freq)	(freq / 1000)

#define	NAVAIDDB_HTBL_SHIFT	17

struct navaiddb_s {
	list_t		navaids;
	airportdb_t	*adb;
	avl_tree_t	lat;
	avl_tree_t	lon;
	avl_tree_t	by_id;
	htbl_t		by_arpt;
};

static inline int
common_latlon_compar(const void *a, const void *b, double pa, double pb)
{
	const navaid_t *na = a, *nb = b;
	int strres;

	if (pa < pb)
		return (-1);
	if (pa > pb)
		return (1);
	if (na->type < nb->type)
		return (-1);
	if (na->type > nb->type)
		return (1);
	strres = strcmp(na->id, nb->id);
	if (strres < 0)
		return (-1);
	if (strres > 0)
		return (1);
	if (na->freq < nb->freq)
		return (-1);
	if (na->freq > nb->freq)
		return (1);

	return (0);
}

static int
id_compar(const void *a, const void *b)
{
	const navaid_t *na = a, *nb = b;
	int res;

	if (na->type < nb->type)
		return (-1);
	if (na->type > nb->type)
		return (1);
	res = strcmp(na->region, nb->region);
	if (res < 0)
		return (-1);
	if (res > 0)
		return (1);
	res = strcmp(na->icao, nb->icao);
	if (res < 0)
		return (-1);
	if (res > 0)
		return (1);
	res = strcmp(na->id, nb->id);
	if (res < 0)
		return (-1);
	if (res > 0)
		return (1);

	return (0);
}

static int
lat_compar(const void *a, const void *b)
{
	return (common_latlon_compar(a, b,
	    ((const navaid_t *)a)->pos.lat, ((const navaid_t *)b)->pos.lat));
}

static int
lon_compar(const void *a, const void *b)
{
	return (common_latlon_compar(a, b,
	    ((const navaid_t *)a)->pos.lon, ((const navaid_t *)b)->pos.lon));
}

static void
navaids_flush(navaiddb_t *db)
{
	void *cookie;
	navaid_t *navaid;

	cookie = NULL;
	while (avl_destroy_nodes(&db->lat, &cookie) != NULL)
		;
	cookie = NULL;
	while (avl_destroy_nodes(&db->lon, &cookie) != NULL)
		;
	cookie = NULL;
	while (avl_destroy_nodes(&db->by_id, &cookie) != NULL)
		;
	htbl_empty(&db->by_arpt, NULL, NULL);
	while ((navaid = list_remove_head(&db->navaids)) != NULL)
		free(navaid);
}

static void
my_strncat(char *buf, const char *append, size_t cap)
{
	size_t l = strlen(buf);

	if (l + 1 < cap)
		snprintf(&buf[l], cap - l, "%s", append);
}

static void
strcat_list(char *dest, size_t cap, char **src, size_t n)
{
	ASSERT(cap > 1);

	if (n > 0)
		my_strncat(dest, src[0], cap);
	for (size_t i = 1; i < n; i++) {
		my_strncat(dest, " ", cap);
		my_strncat(dest, src[i], cap);
	}
}

static navaid_t *
parse_navaid_common(char **comps, size_t n_comps, navaid_type_t type,
    size_t min_n_comps)
{
	navaid_t *nav;

	if (n_comps < min_n_comps)
		return (NULL);

	nav = safe_calloc(1, sizeof (*nav));
	nav->type = type;
	nav->pos.lat = atof(comps[1]);
	nav->pos.lon = atof(comps[2]);
	nav->pos.elev = FEET2MET(atoi(comps[3]));
	nav->xp_elev = NAN;
	if (type == NAVAID_NDB) {
		nav->freq = atoll(comps[4]) * 1000;
	} else if (type == NAVAID_VOR || type == NAVAID_LOC ||
	    type == NAVAID_GS || type == NAVAID_DME) {
		nav->freq = atoll(comps[4]) * 10000;
	} else {
		nav->freq = atoll(comps[4]);
	}
	nav->range = NM2MET(atof(comps[5]));
	if (type != NAVAID_FPAP && type != NAVAID_LTP && type != NAVAID_GLS)
		strlcpy(nav->id, comps[7], sizeof (nav->id));
	strlcpy(nav->icao, comps[8], sizeof (nav->icao));
	strlcpy(nav->region, comps[9], sizeof (nav->region));
	if (type == NAVAID_LOC || type == NAVAID_GS || type == NAVAID_DME) {
		strcat_list(nav->name, sizeof (nav->name), &comps[11],
		    n_comps - 11);
	} else if (type == NAVAID_NDB || type == NAVAID_VOR) {
		strcat_list(nav->name, sizeof (nav->name), &comps[10],
		    n_comps - 10);
	}

	if (!is_valid_lat(nav->pos.lat) || !is_valid_lon(nav->pos.lon) ||
	    !is_valid_elev(nav->pos.elev)) {
		free(nav);
		return (NULL);
	}

	nav->ecef = geo2ecef_mtr(nav->pos, &wgs84);

	return (nav);
}

static bool_t
parse_ndb(char **comps, size_t n_comps, navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_NDB, 11);

	if (nav == NULL || !is_valid_ndb_freq(HZ2KHZ(nav->freq))) {
		free(nav);
		return (B_FALSE);
	}

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_vor(char **comps, size_t n_comps, navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_VOR, 11);

	if (strcmp(comps[n_comps - 1], "TACAN") == 0) {
		/* Skip TACAN stations, as they don't use regular VOR freq */
		free(nav);
		return (B_TRUE);
	}

	if (nav == NULL || !is_valid_vor_freq(HZ2MHZ(nav->freq))) {
		free(nav);
		return (B_FALSE);
	}
	nav->vor.magvar = atof(comps[6]);

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_loc(char **comps, size_t n_comps, navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_LOC, 12);

	if (nav != NULL) {
		double brg_raw = atof(comps[6]);
		/*
		 * X-Plane 11.50 nav data format embeds the magnetic front
		 * course into the bearing field.
		 */
		nav->loc.brg = fmod(brg_raw, 360);
		nav->loc.fcrs = (brg_raw >= 360 ? round(brg_raw / 360) : NAN);
	}
	if (nav == NULL || !is_valid_loc_freq(HZ2MHZ(nav->freq)) ||
	    !is_valid_hdg(nav->loc.brg)) {
		free(nav);
		return (B_FALSE);
	}
	/*
	 * Best effort guess for now. Localizers with an associated runway
	 * will recalculate the actual reference datum distance in
	 * loc_align_with_rwy. 2450m reference datum yields a course sector
	 * angle of 2.5 degrees, which is about okay'ish for most purposes.
	 */
	nav->loc.ref_datum_dist = 2450;
	strlcpy(nav->loc.rwy_id, comps[10], sizeof (nav->loc.rwy_id));

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_gs(char **comps, size_t n_comps, navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_GS, 12);

	if (nav != NULL && strlen(comps[6]) > 3) {
		char tmp[4] = {comps[6][0], comps[6][1], comps[6][2], '\0'};

		nav->gs.brg = atof(&comps[6][3]);
		nav->gs.gs = atoi(tmp) / 100.0;
	}
	if (nav == NULL || !is_valid_loc_freq(HZ2MHZ(nav->freq)) ||
	    !is_valid_hdg(nav->gs.brg) || nav->gs.gs <= 0 || nav->gs.gs > 8) {
		free(nav);
		return (B_FALSE);
	}
	strlcpy(nav->gs.rwy_id, comps[10], sizeof (nav->gs.rwy_id));

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_mrk(char **comps, size_t n_comps, mrk_type_t type,
    navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_MRK, 12);

	if (nav != NULL) {
		nav->mrk.brg = atof(comps[6]);
		nav->mrk.type = type;
	}
	if (nav == NULL ||!is_valid_hdg(nav->mrk.brg)) {
		free(nav);
		return (B_FALSE);
	}
	strlcpy(nav->mrk.rwy_id, comps[10], sizeof (nav->mrk.rwy_id));

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_dme(char **comps, size_t n_comps, navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_DME, 12);

	if (nav != NULL)
		nav->dme.bias = -NM2MET(atof(comps[6]));
	if (nav == NULL ||
	    (!is_valid_vor_freq(HZ2MHZ(nav->freq)) &&
	    !is_valid_loc_freq(HZ2MHZ(nav->freq)))) {
		free(nav);
		return (B_FALSE);
	}
	strlcpy(nav->dme.arpt_id, comps[10], sizeof (nav->dme.arpt_id));

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_fpap(char **comps, size_t n_comps, navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_FPAP, 12);

	if (nav != NULL) {
		if (strcmp(comps[11], "LP") == 0) {
			nav->fpap.perf = FPAP_PERF_LP;
		} else if (strcmp(comps[11], "LPV") == 0) {
			nav->fpap.perf = FPAP_PERF_LPV;
		} else if (strcmp(comps[11], "APV-II") == 0) {
			nav->fpap.perf = FPAP_PERF_APV_II;
		} else if (strcmp(comps[11], "GLS") == 0) {
			nav->fpap.perf = FPAP_PERF_GLS;
		} else {
			free(nav);
			return (B_FALSE);
		}
		nav->fpap.crs = atof(comps[6]);
	}
	if (nav == NULL || !is_valid_hdg(nav->fpap.crs)) {
		free(nav);
		return (B_FALSE);
	}
	snprintf(nav->id, sizeof (nav->id), "%s/%s", comps[8], comps[7]);
	strlcpy(nav->fpap.proc_id, comps[7], sizeof (nav->fpap.proc_id));
	strlcpy(nav->fpap.rwy_id, comps[10], sizeof (nav->fpap.rwy_id));

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_ltp_gls(char **comps, size_t n_comps, navaid_t **nav_pp)
{
	bool_t is_ltp = (atoi(comps[0]) == 16);
	navaid_t *nav = parse_navaid_common(comps, n_comps,
	    is_ltp ? NAVAID_LTP : NAVAID_GLS, 12);

	if (nav != NULL && strlen(comps[6]) > 3) {
		char tmp[4];
		char *crs_str;

		if (atof(comps[6]) < 1000) {
			strlcpy(tmp, "300", sizeof (tmp));
			crs_str = comps[6];
		} else {
			strlcpy(tmp, comps[6], sizeof (tmp));
			crs_str = &comps[6][3];
		}

		if (is_ltp) {
			if (strcmp(comps[11], "WAAS") == 0) {
				nav->ltp.prov = LTP_PROV_WAAS;
			} else if (strcmp(comps[11], "EGNOS") == 0) {
				nav->ltp.prov = LTP_PROV_EGNOS;
			} else if (strcmp(comps[11], "MSAS") == 0) {
				nav->ltp.prov = LTP_PROV_MSAS;
			} else if (strcmp(comps[11], "GP") == 0) {
				nav->ltp.prov = LTP_PROV_GP;
			} else {
				free(nav);
				return (B_FALSE);
			}
			nav->ltp.tch = FEET2MET(atof(comps[5]));
			nav->ltp.crs = atof(crs_str);
			nav->ltp.gs = atoi(tmp) / 100.0;
		} else {
			nav->gls.crs = atof(crs_str);
			nav->gls.gs = atoi(tmp) / 100.0;
		}
	}
	if (nav == NULL ||
	    (is_ltp && (!is_valid_hdg(nav->ltp.crs) || nav->ltp.gs <= 0 ||
	    nav->ltp.gs > 8)) ||
	    (!is_ltp && (!is_valid_hdg(nav->gls.crs) || nav->gls.gs <= 0 ||
	    nav->gls.gs > 8))) {
		free(nav);
		return (B_FALSE);
	}
	snprintf(nav->id, sizeof (nav->id), "%s/%s", comps[8], comps[7]);
	if (is_ltp) {
		strlcpy(nav->ltp.proc_id, comps[7], sizeof (nav->ltp.proc_id));
		strlcpy(nav->ltp.rwy_id, comps[10], sizeof (nav->ltp.rwy_id));
	} else {
		strlcpy(nav->gls.proc_id, comps[7], sizeof (nav->gls.proc_id));
		strlcpy(nav->gls.rwy_id, comps[10], sizeof (nav->gls.rwy_id));
	}

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_line(char *line, navaid_t **nav_pp)
{
	char **comps;
	size_t n_comps;
	bool_t result = B_TRUE;
	int code;

	*nav_pp = NULL;

	strip_space(line);
	comps = strsplit(line, " ", B_TRUE, &n_comps);
	if (n_comps == 0)
		return (B_TRUE);

	code = atoi(comps[0]);
	switch (code) {
	case 2:
		result = parse_ndb(comps, n_comps, nav_pp);
		break;
	case 3:
		result = parse_vor(comps, n_comps, nav_pp);
		break;
	case 4:
	case 5:
		result = parse_loc(comps, n_comps, nav_pp);
		break;
	case 6:
		result = parse_gs(comps, n_comps, nav_pp);
		break;
	case 7:
	case 8:
	case 9:
		result = parse_mrk(comps, n_comps, code - 7, nav_pp);
		break;
	case 12:
	case 13:
		result = parse_dme(comps, n_comps, nav_pp);
		break;
	case 14:
		result = parse_fpap(comps, n_comps, nav_pp);
		break;
	case 15:
		result = parse_ltp_gls(comps, n_comps, nav_pp);
		break;
	case 16:
		result = parse_ltp_gls(comps, n_comps, nav_pp);
		break;
	default:
		free_strlist(comps, n_comps);
		return (B_TRUE);
	}

	if (!result) {
		free_strlist(comps, n_comps);
		return (B_FALSE);
	}
	free_strlist(comps, n_comps);

	return (B_TRUE);
}

static bool_t
is_navaid_conflict(navaid_t *nav, navaid_t *nav2)
{
	enum {
	    BRG_MATCH = 10 /* deg */,
	    LOC_DIST_MATCH = 1000 /* m */,
	    GS_DIST_MATCH = 750 /* m */,
	    DME_DIST_MATCH = 500 /* m */
	};
	double dist;

	ASSERT(nav != NULL);
	ASSERT(nav2 != NULL);
	ASSERT3U(nav->type, ==, nav2->type);

	if (nav->freq != nav2->freq)
		return (B_FALSE);
	dist = vect3_abs(vect3_sub(nav->ecef, nav2->ecef));
	switch (nav->type) {
	case NAVAID_LOC:
		return (dist < LOC_DIST_MATCH &&
		    fabs(rel_hdg(nav->loc.brg, nav2->loc.brg)) < BRG_MATCH);
	case NAVAID_GS:
		return (dist < GS_DIST_MATCH &&
		    fabs(rel_hdg(nav->gs.brg, nav2->gs.brg)) < BRG_MATCH);
	case NAVAID_DME:
		return (dist < DME_DIST_MATCH);
	default:
		return (B_FALSE);
	}
}

/*
 * Checks if the navaiddb contains a duplicate navaid similar to `nav' at the
 * same airport and removes the duplicate. These duplicates arise primarily
 * as a result of an outdated entry in the Global Airports' hand-placed
 * localizers list with a different navaid identifier and a newer entry in
 * AIRAC data with. These navaids will be typically be pretty close together,
 * share the same frequency and be fairly similar in other parameters. Thus
 * a duplicate navaid is defined as:
 *
 * LOC: same frequency, bearing within 10 degrees of each other and
 *	less than 1000 meters apart
 * GS: same frequency, bearing within 10 degrees of each other and
 *	less than 750 meters apart
 * DME: same frequency and less than 500 meters apart
 *
 * No other navaids can be duplicate.
 */
static void
replace_arpt_navaid_duplicate(navaiddb_t *db, navaid_t *nav)
{
	const list_t *l;

	ASSERT(db != NULL);
	ASSERT(nav != NULL);

	l = htbl_lookup_multi(&db->by_arpt, &nav->type);
	if (l != NULL) {
		/* Remove any too-similar-looking navaids at this airport */
		for (void *v = list_head(l), *v_next = NULL;
		    v != NULL; v = v_next) {
			navaid_t *nav2 = HTBL_VALUE_MULTI(v);

			/*
			 * We could be removing this entry, so grab the next
			 * in line ahead of time.
			 */
			v_next = list_next(l, v);
			ASSERT3U(nav->type, ==, nav2->type);
			ASSERT0(strcmp(nav->icao, nav2->icao));

			if (is_navaid_conflict(nav, nav2)) {
				avl_remove(&db->by_id, nav2);
				avl_remove(&db->lat, nav2);
				avl_remove(&db->lon, nav2);
				list_remove(&db->navaids, nav2);
				htbl_remove_multi(&db->by_arpt, &nav2->type, v);
				free(nav2);
				/*
				 * Through the transitive property of
				 * equivalence we know that as soon as we
				 * have found one match, no other matches
				 * could present (since inserting the navaid
				 * we're about to remove would have removed
				 * them already).
				 */
				break;
			}
		}
	}
	htbl_set(&db->by_arpt, &nav->type, nav);
}

/*
 * Parses an earth_nav.dat file.
 * If `replace_freq' is true, if a duplicate navaid is found, we overwrite
 * its frequency with what we've read. This is to deal with the dickish
 * situation with hand-placed localizers, which override a localizer's
 * position but NOT its frequency. The frequencies are primarily in Custom
 * Data take priority.
 */
static bool_t
parse_earth_nav(navaiddb_t *db, const char *filename, bool_t replace_freq)
{
	FILE *fp = fopen(filename, "rb");
	int version = 0;
	char *line = NULL;
	size_t cap = 0;
	int line_nr = 3;

	if (fp == NULL) {
		logMsg("Error reading %s: %s", filename, strerror(errno));
		goto errout;
	}
	if (fscanf(fp, "I %d", &version) != 1 ||
	    version < EARTH_NAV_MIN_VERSION ||
	    version < EARTH_NAV_MAX_VERSION) {
		logMsg("Error reading %s: file malformed or version %d not "
		    "supported", filename, version);
		goto errout;
	}

	while (!feof(fp)) {
		if (fgetc(fp) == '\n')
			break;
	}

	while (getline(&line, &cap, fp) > 0) {
		navaid_t *nav;

		line_nr++;

		if (!parse_line(line, &nav))
			continue;

		if (nav != NULL) {
			navaid_t *other_nav;
			avl_index_t where_id, where_lat, where_lon;
			/*
			 * Due to airport naming and region naming
			 * inconsistencies, we might not find the
			 * navaid duplicated in the by-id database.
			 * So use the positional exclusion system
			 * as well. It's dumb, but those guys can
			 * go fix their database entries themselves.
			 */
			other_nav = avl_find(&db->by_id, nav, &where_id);
			if (other_nav == NULL)
				other_nav = avl_find(&db->lat, nav, &where_lat);
			if (other_nav == NULL)
				other_nav = avl_find(&db->lon, nav, &where_lon);
			if (other_nav != NULL) {
				/*
				 * Because of how garbage the X-Plane navaid
				 * database is, there can be localizers in the
				 * hand-placed localizer list with BAD freq's.
				 * So to work around that, we replace their
				 * frequencies from the navdata.
				 */
				if (replace_freq)
					other_nav->freq = nav->freq;
				free(nav);
				continue;
			}
			avl_insert(&db->by_id, nav, where_id);
			avl_insert(&db->lat, nav, where_lat);
			avl_insert(&db->lon, nav, where_lon);
			list_insert_tail(&db->navaids, nav);
			if (nav->icao[0] != '\0' &&
			    strcmp(nav->icao, "ENRT") != 0) {
				replace_arpt_navaid_duplicate(db, nav);
			}
		}
	}
	lacf_free(line);

	fclose(fp);
	return (B_TRUE);
errout:
	navaids_flush(db);
	if (fp != NULL)
		fclose(fp);
	return (B_FALSE);
}

navaiddb_t *
navaiddb_create(const char *xpdir, airportdb_t *adb)
{
	navaiddb_t *db = safe_calloc(1, sizeof (*db));
	char *path;
	bool_t parse_default = B_TRUE;
	bool_t is_dir;

	list_create(&db->navaids, sizeof (navaid_t), offsetof(navaid_t, node));
	avl_create(&db->lat, lat_compar,
	    sizeof (navaid_t), offsetof(navaid_t, lat_node));
	avl_create(&db->lon, lon_compar,
	    sizeof (navaid_t), offsetof(navaid_t, lon_node));
	avl_create(&db->by_id, id_compar,
	    sizeof (navaid_t), offsetof(navaid_t, id_node));
	htbl_create(&db->by_arpt, 1 << NAVAIDDB_HTBL_SHIFT,
	    /*
	     * Use the concatenated representation of the `type' and `icao'
	     * fields as the hash table key.
	     */
	    ((offsetof(navaid_t, icao) + NAVAIDDB_ICAO_LEN) -
	    offsetof(navaid_t, type)), B_TRUE);
	db->adb = adb;

	/*
	 * Since the first navaid candidate found wins here, we need to
	 * parse the files in order of user preference.
	 */

	/*
	 * First come the user's hand-placed navaids.
	 */
	path = mkpathname(xpdir, "Custom Data", "user_nav.dat", NULL);
	if (file_exists(path, &is_dir) && !is_dir)
		parse_earth_nav(db, path, B_FALSE);
	lacf_free(path);
	/*
	 * Next come the hand-placed localizers from the scenery gateway.
	 */
	path = mkpathname(xpdir, "Custom Scenery", "Global Airports",
	    "Earth nav data", "earth_nav.dat", NULL);
	if (file_exists(path, &is_dir) && !is_dir)
		parse_earth_nav(db, path, B_FALSE);
	lacf_free(path);
	/*
	 * Next try the custom data from data providers. If those exist,
	 * don't attempt to load the old data from X-Plane stock.
	 */
	path = mkpathname(xpdir, "Custom Data", "earth_nav.dat", NULL);
	if (file_exists(path, &is_dir) && !is_dir)
		parse_default = !parse_earth_nav(db, path, B_TRUE);
	lacf_free(path);
	if (parse_default) {
		path = mkpathname(xpdir, "Resources", "default data",
		    "earth_nav.dat", NULL);
		if (!parse_earth_nav(db, path, B_TRUE)) {
			/* No usable navaid source */
			lacf_free(path);
			navaiddb_destroy(db);
			return (NULL);
		}
		lacf_free(path);
	}

	return (db);
}

void
navaiddb_destroy(navaiddb_t *db)
{
	navaids_flush(db);

	avl_destroy(&db->lat);
	avl_destroy(&db->lon);
	avl_destroy(&db->by_id);
	htbl_destroy(&db->by_arpt);
	list_destroy(&db->navaids);
}

double
navaiddb_get_xp_elev(const navaid_t *nav_const)
{
	/*
	 * This is a blatant violation of the qualifier, but we are ok to do
	 * so, as the navaid is from our database, and hence we know we can
	 * write to it.
	 */
	navaid_t *nav = (navaid_t *)nav_const;
	XPLMProbeInfo_t info = { .structSize = sizeof (info) };
	XPLMProbeResult res;
	double x, y, z;
	XPLMProbeRef probe;

	if (!isnan(nav->xp_elev))
		return (nav->xp_elev);

	probe = XPLMCreateProbe(xplm_ProbeY);

	XPLMWorldToLocal(nav->pos.lat, nav->pos.lon, nav->pos.elev, &x, &y, &z);

	res = XPLMProbeTerrainXYZ(probe, x, 0, z, &info);
	if (res == xplm_ProbeHitTerrain) {
		double lat, lon;

		XPLMLocalToWorld(info.locationX, info.locationY,
		    info.locationZ, &lat, &lon, &nav->xp_elev);
	}

	XPLMDestroyProbe(probe);

	return (nav->xp_elev);
}

static inline bool_t
navaid_select(const navaid_t *nav, const char *id, uint64_t *freq,
    navaid_type_t *type)
{
	return ((id == NULL || strcmp(id, nav->id) == 0) &&
	    (freq == NULL || *freq == nav->freq) &&
	    (type == NULL || (nav->type & (*type)) != 0));
}

static unsigned
navaids_gather(navaiddb_t *db, geo_pos2_t center, const char *id,
    uint64_t *freq, navaid_type_t *type,
    double lat_spacing, double lon_spacing,
    avl_index_t where_lat, avl_index_t where_lon, navaid_list_t *list)
{
	unsigned count = 0;

	for (navaid_t *nav = avl_nearest(&db->lat, where_lat, AVL_BEFORE);
	    nav != NULL; nav = AVL_PREV(&db->lat, nav)) {
		if (ABS(center.lat - nav->pos.lat) > lat_spacing)
			break;
		if (navaid_select(nav, id, freq, type) &&
		    ABS(center.lon - nav->pos.lon) < lon_spacing) {
			if (list != NULL)
				list->navaids[count] = nav;
			count++;
		}
	}
	for (navaid_t *nav = avl_nearest(&db->lat, where_lat, AVL_AFTER);
	    nav != NULL; nav = AVL_NEXT(&db->lat, nav)) {
		if (ABS(center.lat - nav->pos.lat) > lat_spacing)
			break;
		if (navaid_select(nav, id, freq, type) &&
		    ABS(center.lon - nav->pos.lon) < lon_spacing) {
			if (list != NULL)
				list->navaids[count] = nav;
			count++;
		}
	}
	for (navaid_t *nav = avl_nearest(&db->lon, where_lon, AVL_BEFORE);
	    nav != NULL; nav = AVL_PREV(&db->lon, nav)) {
		if (ABS(center.lon - nav->pos.lon) > lon_spacing)
			break;
		if (navaid_select(nav, id, freq, type) &&
		    ABS(center.lat - nav->pos.lat) < lat_spacing) {
			if (list != NULL)
				list->navaids[count] = nav;
			count++;
		}
	}
	for (navaid_t *nav = avl_nearest(&db->lon, where_lon, AVL_AFTER);
	    nav != NULL; nav = AVL_NEXT(&db->lon, nav)) {
		if (ABS(center.lon - nav->pos.lon) > lon_spacing)
			break;
		if (navaid_select(nav, id, freq, type) &&
		    ABS(center.lat- nav->pos.lat) < lat_spacing) {
			if (list != NULL)
				list->navaids[count] = nav;
			count++;
		}
	}

	return (count);
}

static void
loc_align_with_rwy(navaiddb_t *db, navaid_t *nav)
{
	airport_t *arpt;
	runway_t *rwy;
	unsigned end;

	ASSERT(db != NULL);
	ASSERT(nav != NULL);
	ASSERT3U(nav->type, ==, NAVAID_LOC);
	ASSERT(!nav->loc.rwy_align_done);

	if (strcmp(nav->icao, "ENRT") == 0) {
		nav->loc.rwy_align_done = B_TRUE;
		return;
	}
	arpt = airport_lookup_global(db->adb, nav->icao);
	/* Correct the navaid bearing to the runway true heading. */
	if (arpt != NULL &&
	    airport_find_runway(arpt, nav->loc.rwy_id, &rwy, &end) &&
	    fabs(rel_hdg(nav->loc.brg, rwy->ends[end].hdg)) <= 1) {
		fpp_t fpp = gnomo_fpp_init(GEO3_TO_GEO2(rwy->ends[end].thr), 0,
		    NULL, B_TRUE);
		vect2_t thr2thr_v, cross_v, nav_v, c;
		geo_pos2_t p;

		thr2thr_v = geo2fpp(GEO3_TO_GEO2(rwy->ends[!end].thr), &fpp);
		nav_v = geo2fpp(GEO3_TO_GEO2(nav->pos), &fpp);
		cross_v = vect2_norm(thr2thr_v, B_TRUE);
		c = vect2vect_isect(cross_v, nav_v, thr2thr_v, ZERO_VECT2,
		    B_FALSE);
		p = fpp2geo(c, &fpp);
		nav->loc.corr_pos = GEO_POS3(p.lat, p.lon, nav->pos.elev);
		nav->loc.brg = dir2hdg(thr2thr_v);
		/*
		 * The distance from the localizer to the ILS reference datum
		 * (which we define as the runway threshold).
		 */
		nav->loc.ref_datum_dist = vect2_abs(nav_v);
		/*
		 * atan(106.9m / 1017m) = 6 degrees
		 * ICAO Annex 10 defines this as the maximum course sector
		 * width, so we clamp the reference datum distance at that.
		 */
		nav->loc.ref_datum_dist = MAX(nav->loc.ref_datum_dist, 1017);
	} else {
		nav->loc.corr_pos = nav->pos;
	}
	nav->loc.rwy_align_done = B_TRUE;
}

navaid_list_t *
navaiddb_query(navaiddb_t *db, geo_pos2_t center, double radius,
    const char *id, uint64_t *freq, navaid_type_t *type)
{
	navaid_t srch = { .pos = GEO2_TO_GEO3(center, 0) };
	avl_index_t where_lat, where_lon;
	double lat_spacing = (radius / (EARTH_MSL * 2 * M_PI)) * 360;
	double lon_spacing = (radius / (cos(DEG2RAD(center.lat)) *
	    (EARTH_MSL * 2 * M_PI))) * 360;
	navaid_list_t *list;

	VERIFY3P(avl_find(&db->lat, &srch, &where_lat), ==, NULL);
	VERIFY3P(avl_find(&db->lon, &srch, &where_lon), ==, NULL);

	list = safe_calloc(1, sizeof (*list));
	list->num_navaids = navaids_gather(db, center, id, freq, type,
	    lat_spacing, lon_spacing, where_lat, where_lon, NULL);
	list->navaids = safe_calloc(list->num_navaids, sizeof (*list->navaids));
	navaids_gather(db, center, id, freq, type, lat_spacing, lon_spacing,
	    where_lat, where_lon, list);

	airportdb_lock(db->adb);

	for (size_t i = 0; i < list->num_navaids; i++) {
		/* We can modify the navaid, since we own the database. */
		navaid_t *nav = (navaid_t *)list->navaids[i];
		if (nav->type == NAVAID_LOC && !nav->loc.rwy_align_done)
			loc_align_with_rwy(db, nav);
	}

	unload_distant_airport_tiles(db->adb, NULL_GEO_POS2);
	airportdb_unlock(db->adb);

	return (list);
}

const navaid_t *
navaiddb_find_conflict_same_arpt(navaiddb_t *db, const navaid_t *srch)
{
	const list_t *l;

	ASSERT(db != NULL);
	ASSERT(srch != NULL);

	l = htbl_lookup_multi(&db->by_arpt, &srch->type);
	if (l == NULL)
		return (NULL);
	for (void *v = list_head(l); v != NULL; v = list_next(l, v)) {
		const navaid_t *nav = HTBL_VALUE_MULTI(v);

		if (srch->freq == nav->freq && srch != nav)
			return (nav);
	}
	return (NULL);
}

void
navaiddb_list_free(navaid_list_t *list)
{
	free(list->navaids);
	free(list);
}

const char *
navaid_type2str(navaid_type_t type)
{
	switch (type) {
	case NAVAID_NDB:
		return ("NDB");
	case NAVAID_VOR:
		return ("VOR");
	case NAVAID_LOC:
		return ("LOC");
	case NAVAID_GS:
		return ("GS");
	case NAVAID_MRK:
		return ("MRK");
	case NAVAID_DME:
		return ("DME");
	case NAVAID_FPAP:
		return ("FPAP");
	case NAVAID_LTP:
		return ("TLS");
	case NAVAID_GLS:
		return ("GLS");
	default:
		VERIFY_MSG(0, "Invalid navaid type passed: %x\n", type);
	}
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

uint64_t
navaid_act_freq(navaid_type_t type, uint64_t ref_freq)
{
	switch (type) {
	case NAVAID_GS:
		return (332000000);
	case NAVAID_DME:
		return (vor2dme(ref_freq));
	default:
		return (ref_freq);
	}
}

geo_pos3_t
navaid_get_pos(const navaid_t *nav)
{
	if (nav->type == NAVAID_LOC)
		return (nav->loc.corr_pos);
	return (nav->pos);
}
