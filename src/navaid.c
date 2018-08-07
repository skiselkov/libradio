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
#include <stdio.h>
#include <string.h>

#include <acfutils/log.h>
#include <acfutils/thread.h>

#include "libradio/navaid.h"

#define	EARTH_NAV_MIN_VERSION	1100
#define	EARTH_NAV_MAX_VERSION	1100

struct navaiddb_s {
	mutex_t		lock;

	list_t		navaids;
	avl_tree_t	lat;
	avl_tree_t	lon;
};

static inline int
common_latlon_compar(const void *a, const void *b, double pa, double pb)
{
	const navaid_t *na = a, *nb = b;

	if (pa < pb)
		return (-1);
	if (pa > pb)
		return (1);
	if (na->type < nb->type)
		return (-1);
	if (na->type > nb->type)
		return (1);
	return (0);
}

static int
lat_compar(const void *a, const void *b)
{
	return (common_latlon_compar(a, b,
	    ((const navaid_t *)a)->pos.lat, ((const navaid_t *)b)->pos.lat);
}

static int
lon_compar(const void *a, const void *b)
{
	return (common_latlon_compar(a, b,
	    ((const navaid_t *)a)->pos.lon, ((const navaid_t *)b)->pos.lon);
}

static void
navaids_flush(navaiddb_t *db)
{
	void *cookie;
	navaid_t *navaid;

	mutex_enter(&db->lock);

	cookie = NULL;
	while (avl_destroy_nodes(&db->lat) != NULL)
		;
	cookie = NULL;
	while (avl_destroy_nodes(&db->lon) != NULL)
		;
	while ((navaid = list_remove_head(&db->navaids)) != NULL)
		free(navaid);

	mutex_exit(&db->lock);
}

static void
strcat_list(char *dest, size_t cap, const char **src, size_t n)
{
	ASSERT(cap > 1);

	if (n > 0)
		strncat(dest, src[0], cap - 1);
	for (size_t i = 1; i < n; i++) {
		strncat(dest, " ", cap - 1);
		strncat(dest, src[i], cap - 1);
	}
}

static navaid_t *
parse_navaid_common(const char **comps, size_t n_comps, navaid_type_t type,
    size_t min_n_comps)
{
	navaid_t *nav

	if (n_comps < min_n_comps)
		return (NULL);

	nav = calloc(1, sizeof (*nav));
	nav->type = type;
	nav->pos.lat = atof(comps[1]);
	nav->pos.lon = atof(comps[2]);
	nav->pos.elev = FEET2MET(atoi(comps[3]));
	if (type == NAVAID_NDB) {
		nav->freq = atof(comps[4]) * 1000;
	} else if (type == NAVAID_VOR || type == NAVAID_LOC ||
	    type == NAVAID_GS || type == NAVAID_DME) {
		nav->freq = atof(comps[4]) * 10000;
	} else {
		nav->freq = atof(comps[4]);
	}
	nav->range = NM2MET(atof(comps[5]));
	if (type != NAVAID_FPAP && type != NAVAID_LTP && type != NAVAID_GLS)
		strlcpy(nav->id, comps[7], sizeof (nav->id));
	strlcpy(nav->icao, comps[8], sizeof (nav->icao));
	strlcpy(nav->region, comps[9], sizeof (nav->region));
	if (type == NAVAID_LOC || type == NAVAID_GS || type == NAVAID_DME) {
		strcat_list(nav->name, comps[11], n_comps - 11);
	} else if (type == NAVAID_NDB || type == NAVAID_VOR) {
		strcat_list(nav->name, comps[10], n_comps - 10);
	}

	if (!is_valid_lat(nav->pos.lat) || !is_valid_lon(nav->pos.lon) ||
	    !is_valid_elev(nav->pos.elev)) {
		free(nav);
		return (NULL);
	}

	return (nav);
}

static bool_t
parse_ndb(const char **comps, size_t n_comps, navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_NDB, 11);

	if (nav == NULL || !is_valid_ndb_freq(nav->freq / 1000.0)) {
		free(nav);
		return (B_FALSE);
	}

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_vor(const char **comps, size_t n_comps, navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_VOR, 11);

	if (nav == NULL || !is_valid_vor_freq(nav->freq / 1000000.0)) {
		free(nav);
		return (B_FALSE);
	}
	nav->vor.magvar = atof(comps[6]);

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_loc(const char **comps, size_t n_comps, navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_LOC, 12);

	if (nav != NULL)
		nav->loc.brg = atof(comps[6]);
	if (nav == NULL || !is_valid_loc_freq(nav->freq / 1000000.0) ||
	    !is_valid_hdg(nav->loc.brg)) {
		free(nav);
		return (B_FALSE);
	}
	strlcpy(nav->loc.rwy_id, comps[10], sizeof (nav->loc.rwy_id));

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_gs(const char **comps, size_t n_comps, navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_GS, 12);

	if (nav != NULL && strlen(comps[6]) > 3) {
		char tmp[4] = {comps[6][0], comps[6][1], comps[6][2], '\0'};

		nav->gs.brg = atof(&comps[6][3]);
		nav->gs.gs = atoi(tmp) / 100.0;
	}
	if (nav == NULL || !is_valid_loc_freq(nav->freq / 1000000.0) ||
	    !is_valid_hdg(nav->gs.brg) || nav->gs.gs <= 0 || nav->gs.gs > 8) {
		free(nav);
		return (B_FALSE);
	}
	strlcpy(nav->gs.rwy_id, comps[10], sizeof (nav->gs.rwy_id));

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_mrk(const char **comps, size_t n_comps, mrk_type_t type,
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
parse_dme(const char **comps, size_t n_comps, mrk_type_t type,
    navaid_t **nav_pp)
{
	navaid_t *nav = parse_navaid_common(comps, n_comps, NAVAID_DME, 12);

	if (nav != NULL) {
		nav->dme.bias = atof(comps[6]);
	}
	if (nav == NULL ||
	    (!is_valid_vor_freq(nav->freq / 1000000.0) &&
	    !is_valid_loc_freq(nav->freq / 1000000.0))) {
		free(nav);
		return (B_FALSE);
	}
	strlcpy(nav->dme.rwy_id, comps[10]);

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_fpap(const char **comps, size_t n_comps, mrk_type_t type,
    navaid_t **nav_pp)
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
	strlcpy(nav->fpap.proc_id, comps[7], sizeof (nav->fpap.proc_id));
	strlcpy(nav->fpap.rwy_id, comps[10], sizeof (nav->fpap.rwy_id));

	*nav_pp = nav;
	return (B_TRUE);
}

static bool_t
parse_ltp_gls(const char **comps, size_t n_comps, mrk_type_t type,
    navaid_t **nav_pp)
{
	bool_t is_ltp = (atoi(comps[0]) == 16);
	navaid_t *nav = parse_navaid_common(comps, n_comps,
	    is_ltp ? NAVAID_LTP : NAVAID_GLS, 12);

	if (nav != NULL && strlen(comps[6]) > 3) {
		char tmp[4] = {comps[6][0], comps[6][1], comps[6][2], '\0'};

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
			nav->ltp.tch = atof(comps[5]);
			nav->ltp.brg = atof(&comps[6][3]);
			nav->ltp.gs = atoi(tmp) / 100.0;
		} else {
			nav->brg.brg = atof(&comps[6][3]);
			nav->brg.gs = atoi(tmp) / 100.0;
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
parse_line(navaiddb_t *db, char *line, navaid_t **nav_pp)
{
	char **comps;
	size_t n_comps;
	bool_t result = B_TRUE;

	*nav_pp = NULL;

	strip_space(line);
	comps = strsplit(line, " ", B_TRUE, &n_comps);
	if (n_comps == 0)
		return (B_TRUE);

	switch (atoi(comps[0])) {
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
		result = parse_gls(comps, n_comps, nav_pp);
		break;
	case 16:
		result = parse_ltp(comps, n_comps, nav_pp);
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
parse_earth_nav(navaiddb_t *db, const char *filename)
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

		if (!parse_line(db, line, &nav)) {
			logMsg("Error parsing %s: malformed data on line %d\n",
			    filename, line_nr);
			goto errout;
		}

		if (nav != NULL) {
			list_insert_tail(&db->navaids, nav);
			avl_add(&db->lat, nav);
			avl_add(&db->lon, nav);
		}
		line_nr++;
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
navaiddb_create(const char *xpdir)
{
	navaiddb_t *db = calloc(1, sizeof (*db));

	mutex_create(&db->lock);

	list_create(&db->navaids, sizeof (navaid_t), offsetof(navaid_t node));
	avl_create(&db->lat, lat_compar,
	    sizeof (navaid_t), offsetof(navaid_t, lat_node));
	avl_create(&db->lon, lon_compar,
	    sizeof (navaid_t), offsetof(navaid_t, lon_node));
}

void
navaiddb_destroy(navaiddb_t *db)
{
	navaids_flush(db);

	avl_destroy(&db->lat);
	avl_destroy(&db->lon);
	list_destroy(&db->navaids);

	mutex_destroy(&db->lock);
}
