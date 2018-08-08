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
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <acfutils/log.h>
#include <acfutils/helpers.h>
#include <acfutils/perf.h>
#include <acfutils/thread.h>

#include "libradio/navaiddb.h"

#define	EARTH_NAV_MIN_VERSION	1100
#define	EARTH_NAV_MAX_VERSION	1100

#define	HZ2MHZ(freq)	((freq / 1000) / 1000.0)
#define	HZ2KHZ(freq)	(freq / 1000)

struct navaiddb_s {
	list_t		navaids;
	avl_tree_t	lat;
	avl_tree_t	lon;
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
	while ((navaid = list_remove_head(&db->navaids)) != NULL)
		free(navaid);
}

static void
strcat_list(char *dest, size_t cap, char **src, size_t n)
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
parse_navaid_common(char **comps, size_t n_comps, navaid_type_t type,
    size_t min_n_comps)
{
	navaid_t *nav;

	if (n_comps < min_n_comps)
		return (NULL);

	nav = calloc(1, sizeof (*nav));
	nav->type = type;
	nav->pos.lat = atof(comps[1]);
	nav->pos.lon = atof(comps[2]);
	nav->pos.elev = FEET2MET(atoi(comps[3]));
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

	if (nav != NULL)
		nav->loc.brg = atof(comps[6]);
	if (nav == NULL || !is_valid_loc_freq(HZ2MHZ(nav->freq)) ||
	    !is_valid_hdg(nav->loc.brg)) {
		free(nav);
		return (B_FALSE);
	}
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

	if (n_comps >= 12 && strcmp(comps[n_comps - 2], "TACAN") == 0) {
		/* Skip TACAN stations, as they don't use regular VOR freq */
		free(nav);
		return (B_TRUE);
	}

	if (nav != NULL) {
		nav->dme.bias = atof(comps[6]);
	}
	if (nav == NULL ||
	    (!is_valid_vor_freq(HZ2MHZ(nav->freq)) &&
	    !is_valid_loc_freq(HZ2MHZ(nav->freq)))) {
		free(nav);
		return (B_FALSE);
	}
	strlcpy(nav->dme.rwy_id, comps[10], sizeof (nav->dme.rwy_id));

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
			nav->ltp.tch = atof(comps[5]);
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

		line_nr++;

		if (!parse_line(line, &nav)) {
			logMsg("Error parsing %s: malformed data on line %d\n",
			    filename, line_nr - 1);
			continue;
		}

		if (nav != NULL) {
			avl_index_t where_lat, where_lon;

			if (avl_find(&db->lat, nav, &where_lat) != NULL ||
			    avl_find(&db->lon, nav, &where_lon) != NULL) {
				logMsg("Error parsing %s:%d: duplicate navaid "
				    "%s (%s)", filename, line_nr, nav->id,
				    nav->name);
				free(nav);
				continue;
			}
			avl_insert(&db->lat, nav, where_lat);
			avl_insert(&db->lon, nav, where_lon);
			list_insert_tail(&db->navaids, nav);
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
navaiddb_create(const char *xpdir)
{
	navaiddb_t *db = calloc(1, sizeof (*db));
	char *path;
	bool_t parse_default = B_TRUE;

	list_create(&db->navaids, sizeof (navaid_t), offsetof(navaid_t, node));
	avl_create(&db->lat, lat_compar,
	    sizeof (navaid_t), offsetof(navaid_t, lat_node));
	avl_create(&db->lon, lon_compar,
	    sizeof (navaid_t), offsetof(navaid_t, lon_node));

	path = mkpathname(xpdir, "Custom Data", "earth_nav.dat", NULL);
	if (file_exists(path, NULL))
		parse_default = !parse_earth_nav(db, path);
	lacf_free(path);
	if (parse_default) {
		path = mkpathname(xpdir, "Resources", "default data",
		    "earth_nav.dat", NULL);
		if (!parse_earth_nav(db, path)) {
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
	list_destroy(&db->navaids);
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
		if (navaid_select(nav, id, freq, type) &&
		    center.lat - nav->pos.lat < lat_spacing) {
			if (list != NULL)
				list->navaids[count] = nav;
			count++;
		}
	}
	for (navaid_t *nav = avl_nearest(&db->lat, where_lat, AVL_AFTER);
	    nav != NULL; nav = AVL_PREV(&db->lat, nav)) {
		if (navaid_select(nav, id, freq, type) &&
		    nav->pos.lat - center.lat < lat_spacing) {
			if (list != NULL)
				list->navaids[count] = nav;
			count++;
		}
	}
	for (navaid_t *nav = avl_nearest(&db->lon, where_lon, AVL_BEFORE);
	    nav != NULL; nav = AVL_PREV(&db->lon, nav)) {
		if (navaid_select(nav, id, freq, type) &&
		    center.lon - nav->pos.lon < lon_spacing) {
			if (list != NULL)
				list->navaids[count] = nav;
			count++;
		}
	}
	for (navaid_t *nav = avl_nearest(&db->lon, where_lon, AVL_AFTER);
	    nav != NULL; nav = AVL_PREV(&db->lon, nav)) {
		if (navaid_select(nav, id, freq, type) &&
		    nav->pos.lon - center.lon < lon_spacing) {
			if (list != NULL)
				list->navaids[count] = nav;
			count++;
		}
	}

	return (count);
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

	list = calloc(1, sizeof (*list));
	list->num_navaids = navaids_gather(db, center, id, freq, type,
	    lat_spacing, lon_spacing, where_lat, where_lon, NULL);
	list->navaids = calloc(list->num_navaids, sizeof (*list->navaids));
	navaids_gather(db, center, id, freq, type, lat_spacing, lon_spacing,
	    where_lat, where_lon, list);

	return (list);
}

void
navaiddb_list_free(navaid_list_t *list)
{
	free(list->navaids);
	free(list);
}
