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
#include <string.h>

#if	LIN
#include <sys/types.h>
#include <dirent.h>
#endif	/* LIN */

#include <jni.h>

#include <shapefil.h>

#include <acfutils/geom.h>
#include <acfutils/helpers.h>
#include <acfutils/math.h>
#include <acfutils/mt_cairo_render.h>
#include <acfutils/perf.h>
#include <acfutils/png.h>
#include <acfutils/time.h>

#include "itm_c.h"
#include "com_vspro_util_RadioModel.h"

#define	NUM_LAT	18
#define	NUM_LON	36

/*
 * The X-Plane elevation data is stored in the alpha channel of the earth
 * orbit texture normal maps. The values range from 255 corresponding to
 * 418 meters BELOW sea level (lowest elevation on Earth) up to 0
 * corresponding to 8848 meters above sea level (highest elevation on Earth).
 * Yes, the range is inverted (for some reason). This range corresponds to
 * approximately 36.337 meters per elevation sample.
 */
#define	ELEV_SAMPLE2MET(sample)	((int)round(((255 - (sample)) * 36.337) - 418))

#define	LAT2TILE(lat)	(floor(((lat) + 90.0) / 10.0))
#define	LON2TILE(lon)	(floor(((lon) + 180.0) / 10.0))

#define	WATER_MASK_RES		2000	/* pixels */

/* #define	RELIEF_DEBUG */

typedef struct {
	unsigned	w;
	unsigned	h;

	int16_t		*points;

	cairo_surface_t	*water_surf;
	const uint8_t	*water_mask;
	int		water_mask_stride;
} tile_t;

static struct {
	bool_t		inited;
	unsigned	spacing;
	unsigned	max_pts;
	unsigned	max_dist;
	tile_t		tiles[NUM_LAT][NUM_LON];
} rm = { B_FALSE };

static void
throw(JNIEnv *env, const char *classname, const char *fmt, ...)
{
	jclass cls;
	char *msgbuf;
	int l;
	va_list ap;

	cls = (*env)->FindClass(env, classname);
	VERIFY(cls != NULL);

	va_start(ap, fmt);
	l = vsnprintf(NULL, 0, fmt, ap);
	va_end(ap);

	msgbuf = malloc(l + 1);

	va_start(ap, fmt);
	VERIFY3S(vsnprintf(msgbuf, l + 1, fmt, ap), ==, l);
	va_end(ap);

	(*env)->ThrowNew(env, cls, msgbuf);

	free(msgbuf);
}

static bool_t
init_test(JNIEnv *env)
{
	if (!rm.inited) {
		throw(env, "java/lang/ExceptionInInitializerError",
		    "RadioModel not initialized. You must call "
		    "RadioModel.init() first");
		return (B_FALSE);
	}
	return (B_TRUE);
}

static void
load_water_mask_file(const char *filename, cairo_t *cr, int lat, int lon)
{
	SHPHandle shp;
	int n_ent, shp_type;
	double lat_off = (lat / 10.0) - floor(lat / 10.0);
	double lon_off = (lon / 10.0) - floor(lon / 10.0);

	shp = SHPOpen(filename, "rb");
	if (shp == NULL)
		return;
	SHPGetInfo(shp, &n_ent, &shp_type, NULL, NULL);
	/* We only support polygons, given that that's what X-Plane uses. */
	if (shp_type != SHPT_POLYGON) {
		SHPClose(shp);
		return;
	}

	cairo_save(cr);
	cairo_translate(cr, 0, WATER_MASK_RES);
	cairo_scale(cr, 1, -1);
	cairo_translate(cr, round(lon_off * WATER_MASK_RES),
	    round(lat_off * WATER_MASK_RES));
	cairo_scale(cr, round(WATER_MASK_RES / 10),
	    round(WATER_MASK_RES / 10));

	/*
	 * Clear all data to mark the tile as being all terrain. The water
	 * mask will apply terrain where necessary.
	 */
	cairo_rectangle(cr, 0, 0, 1, 1);
	cairo_clip(cr);
	cairo_set_operator(cr, CAIRO_OPERATOR_CLEAR);
	cairo_paint(cr);
	cairo_set_operator(cr, CAIRO_OPERATOR_OVER);

	for (int i = 0; i < n_ent; i++) {
		SHPObject *obj = SHPReadObject(shp, i);

		if (obj == NULL)
			continue;

		cairo_new_path(cr);
		for (int j = 0; j < obj->nParts; j++) {
			int start_k, end_k;

			start_k = obj->panPartStart[j];
			if (j + 1 < obj->nParts)
				end_k = obj->panPartStart[j + 1];
			else
				end_k = obj->nVertices;

			cairo_new_sub_path(cr);
			/*
			 * Note that the rendered image will be "upside down"
			 * when viewed as a PNG. That's because cairo & PNG
			 * address the image from the top left, but our terrain
			 * coordinates start at the bottom left. So we flip the
			 * Y axis, so that increasing row numbers will
			 * correspond to increasing latitude.
			 */
			cairo_move_to(cr, obj->padfX[start_k] - lon,
			    obj->padfY[start_k] - lat);
			for (int k = start_k + 1; k < end_k; k++) {
				cairo_line_to(cr, obj->padfX[k] - lon,
				    obj->padfY[k] - lat);
			}
		}
		cairo_fill(cr);
		SHPDestroyObject(obj);
	}

	cairo_restore(cr);

	SHPClose(shp);
}

static bool_t
file_ext(const char *filename, const char *ext)
{
	const char *period = strrchr(filename, '.');
	return (period != NULL && strcmp(&period[1], ext) == 0);
}

static void
load_water_mask(tile_t *tile, const char *tile_path, unsigned tile_lat,
    unsigned tile_lon)
{
	char dname[64];
	char *path;
	DIR *dp;
	struct dirent *de;
	cairo_t *cr;
	bool_t dir;

	snprintf(dname, sizeof (dname), "%+03d%+04d", tile_lat, tile_lon);
	path = mkpathname(tile_path, dname, NULL);

	if (!file_exists(path, &dir) || !dir)
		goto out;
	dp = opendir(path);
	if (dp == NULL)
		goto out;

	ASSERT3P(tile->water_surf, ==, NULL);
	tile->water_surf = cairo_image_surface_create(CAIRO_FORMAT_A1,
	    WATER_MASK_RES, WATER_MASK_RES);
	tile->water_mask = cairo_image_surface_get_data(tile->water_surf);
	tile->water_mask_stride =
	    cairo_image_surface_get_stride(tile->water_surf);

	cr = cairo_create(tile->water_surf);
	/*
	 * Pre-fill everything with water, the tile will then apply terrain
	 * where necessary.
	 */
	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_paint(cr);
	cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
	cairo_set_source_rgb(cr, 1, 1, 1);

	while ((de = readdir(dp)) != NULL) {
		int subtile_lat, subtile_lon;
		char *filename;

		if (sscanf(de->d_name, "%d%d", &subtile_lat,
		    &subtile_lon) != 2 || !file_ext(de->d_name, "shp") ||
		    !is_valid_lat(subtile_lat) || !is_valid_lon(subtile_lon))
			continue;
		filename = mkpathname(tile_path, dname, de->d_name, NULL);
		load_water_mask_file(filename, cr, subtile_lat, subtile_lon);
		lacf_free(filename);
	}
	closedir(dp);
	cairo_surface_flush(tile->water_surf);
	cairo_destroy(cr);
out:
	lacf_free(path);
}

JNIEXPORT void JNICALL
Java_com_vspro_util_RadioModel_init(JNIEnv *env, jclass cls,
    jstring tile_path_str, jint spacing, jint max_pts, jint max_dist)
{
	enum { MIN_SPACING = 10, MAX_SPACING = 100000, DFL_SPACING = 500,
	    DFL_MAX_PTS = 500, DFL_MAX_DIST = 600000, MIN_MAX_DIST = 2000,
	    MAX_MAX_DIST = 1000000 };
	const char *tile_path = NULL;
	DIR *dp = NULL;
	struct dirent *de;

	UNUSED(cls);

	if (rm.inited) {
		throw(env, "java/lang/ExceptionInInitializerError",
		    "RadioModel.init() called twice without "
		    "calling RadioModel.fini() in between");
		return;
	}

	log_init((logfunc_t)puts, "libradio");

	if (spacing == 0)
		spacing = DFL_SPACING;
	if (spacing < MIN_SPACING || spacing > MAX_SPACING) {
		throw(env, "java/lang/IllegalArgumentException",
		    "Passed invalid `spacing' value (%d), must be an integer "
		    "between %d and %d (or 0 if you want the default value "
		    "of %d).", spacing, MIN_SPACING, MAX_SPACING, DFL_SPACING);
		return;
	}
	if (max_pts == 0)
		max_pts = DFL_MAX_PTS;
	if (max_pts < 2) {
		throw(env, "java/lang/IllegalArgumentException",
		    "Passed invalid `max_pts' value (%d), must be an integer "
		    "greater than or equal to 2 (or 0 if you want the default "
		    "value of %d).", max_pts, DFL_MAX_PTS);
		return;
	}
	if (max_dist == 0)
		max_dist = DFL_MAX_DIST;
	if (max_dist < MIN_MAX_DIST || max_dist > MAX_MAX_DIST) {
		throw(env, "java/lang/IllegalArgumentException",
		    "Passed invalid `max_dist' value (%d), must be an integer "
		    "greater than %d and lesser than %d).", max_dist,
		    MIN_MAX_DIST, MAX_MAX_DIST);
		return;
	}

	memset(&rm, 0, sizeof (rm));
	rm.inited = B_TRUE;
	rm.spacing = spacing;
	rm.max_pts = max_pts;
	rm.max_dist = max_dist;

	tile_path = (*env)->GetStringUTFChars(env, tile_path_str, NULL);

	dp = opendir(tile_path);
	if (dp == NULL) {
		throw(env, "java/lang/FileSystemNotFoundException",
		    "Can't open directory %s: %s", tile_path, strerror(errno));
		goto out;
	}
	while ((de = readdir(dp)) != NULL) {
		int tile_lat, tile_lon;
		int tile_lat_norm, tile_lon_norm;
		tile_t *tile;
		int w, h;
		uint8_t *pixels;
		char *path;

		if (sscanf(de->d_name, "%d%d", &tile_lat, &tile_lon) != 2 ||
		    strlen(de->d_name) != 15 || !file_ext(de->d_name, "png") ||
		    !is_valid_lat(tile_lat) || !is_valid_lon(tile_lon))
			continue;

		tile_lat_norm = floor(tile_lat / 10.0) + 9;
		tile_lon_norm = floor(tile_lon / 10.0) + 18;
		ASSERT3S(tile_lat_norm, >=, 0);
		ASSERT3S(tile_lat_norm, <, NUM_LAT);
		ASSERT3S(tile_lon_norm, >=, 0);
		ASSERT3S(tile_lon_norm, <, NUM_LON);

		tile = &(rm.tiles[tile_lat_norm][tile_lon_norm]);
		if (tile->points != NULL) {
			throw(env, "java/lang/ExceptionInInitializerError",
			    "Duplicate tile %s in tile path %s", de->d_name,
			    tile_path);
			goto out;
		}

		path = mkpathname(tile_path, de->d_name, NULL);
		pixels = png_load_from_file_rgba(path, &w, &h);
		if (pixels == NULL) {
			throw(env, "java/lang/ExceptionInInitializerError",
			    "Error reading PNG data from file %s", path);
			lacf_free(path);
			goto out;
		}
		lacf_free(path);
		ASSERT3S(w, >, 0);
		ASSERT3S(h, >, 0);

		tile->w = w;
		tile->h = h;
		tile->points =
		    malloc(tile->w * tile->h * sizeof (*tile->points));
		/*
		 * Copy over and keep only the alpha channel, since that's
		 * where our elevation data is. We'll pre-chew the samples
		 * to be whole meters, to make subsequent lookups faster.
		 */
		for (int i = 0; i < w * h; i++)
			tile->points[i] = ELEV_SAMPLE2MET(pixels[i * 4 + 3]);

		lacf_free(pixels);

		load_water_mask(tile, tile_path, tile_lat, tile_lon);
#ifdef	WATER_MASK_DEBUG
		if (tile->water_surf != NULL) {
			char *path = mkpathname("water", de->d_name, NULL);
			cairo_surface_write_to_png(tile->water_surf, path);
			lacf_free(path);
		}
#endif	/* WATER_MASK_DEBUG */
	}

out:
	if (dp != NULL)
		closedir(dp);
	(*env)->ReleaseStringUTFChars(env, tile_path_str, tile_path);
}

JNIEXPORT void JNICALL
Java_com_vspro_util_RadioModel_fini(JNIEnv *env, jclass cls)
{
	UNUSED(env);
	UNUSED(cls);

	if (!rm.inited)
		return;

	for (int lat = 0; lat < NUM_LAT; lat++) {
		for (int lon = 0; lon < NUM_LON; lon++) {
			tile_t *tile = &rm.tiles[lat][lon];
			free(tile->points);
			if (tile->water_surf != NULL)
				cairo_surface_destroy(tile->water_surf);
		}
	}

	memset(&rm, 0, sizeof (rm));
}

JNIEXPORT jint JNICALL
Java_com_vspro_util_RadioModel_countBytes(JNIEnv *env, jclass cls)
{
	int bytes = 0;

	UNUSED(env);
	UNUSED(cls);

	for (int lat = 0; lat < NUM_LAT; lat++) {
		for (int lon = 0; lon < NUM_LON; lon++) {
			tile_t *tile = &rm.tiles[lat][lon];

			bytes += sizeof (*tile->points) * tile->w * tile->h;
			bytes += tile->water_mask_stride * WATER_MASK_RES;
		}
	}

	return (bytes);
}

static inline double
elev_filter_lin(tile_t *tile, double fract_lat, double fract_lon)
{
	double x_f = clamp(fract_lon * tile->w, 0, tile->w - 1);
	/*
	 * In-tile the rows are stored from higher latitudes to lower.
	 */
	double y_f = clamp((1 - fract_lat) * (tile->h - 1), 0, tile->h - 1);
	unsigned x_lo = x_f, x_hi = MIN(x_lo + 1, tile->w - 1);
	unsigned y_lo = y_f, y_hi = MIN(y_lo + 1, tile->h - 1);
	double elev1, elev2, elev3, elev4;

	elev1 = tile->points[y_lo * tile->w + x_lo];
	elev2 = tile->points[y_lo * tile->w + x_hi];
	elev3 = tile->points[y_hi * tile->w + x_lo];
	elev4 = tile->points[y_hi * tile->w + x_hi];

	return (wavg(wavg(elev1, elev2, x_f - x_lo),
	    wavg(elev3, elev4, x_f - x_lo), y_f - y_lo));
}

static inline bool_t
tile_water_mask_read(tile_t *tile, double fract_lat, double fract_lon)
{
	unsigned x = clampi(round(fract_lon * (WATER_MASK_RES - 1)), 0,
	    WATER_MASK_RES - 1);
	unsigned y = clampi(round((1 - fract_lat) * (WATER_MASK_RES - 1)), 0,
	    WATER_MASK_RES - 1);
	uint32_t *row;
	unsigned row_slot, bit_slot;

	if (tile->water_mask == NULL)
		return (B_FALSE);

	row = (uint32_t *)(&tile->water_mask[y * tile->water_mask_stride]);
	row_slot = x / 32;
	bit_slot = x & 0x1f;

	return ((row[row_slot] >> bit_slot) & 1);
}

static inline double
tile_elev_read(geo_pos2_t p, bool_t *water)
{
	unsigned tile_lat = floor((p.lat + 90) / 10.0);
	unsigned tile_lon = floor((p.lon + 180) / 10.0);
	double tile_lat_fract = (((p.lat + 90) / 10) - tile_lat);
	double tile_lon_fract = (((p.lon + 180) / 10) - tile_lon);
	tile_t *tile;

	ASSERT3U(tile_lat, <, NUM_LAT);
	ASSERT3U(tile_lon, <, NUM_LON);
	tile = &rm.tiles[tile_lat][tile_lon];
	if (tile->points == NULL)
		return (0);

	ASSERT3F(tile_lat_fract, >=, 0.0);
	ASSERT3F(tile_lat_fract, <=, 1.0);
	ASSERT3F(tile_lon_fract, >=, 0.0);
	ASSERT3F(tile_lon_fract, <=, 1.0);

	if (water != NULL) {
		*water = tile_water_mask_read(tile, tile_lat_fract,
		    tile_lon_fract);
	}

	return (elev_filter_lin(tile, tile_lat_fract, tile_lon_fract));
}

static void
relief_construct(geo_pos2_t p1, geo_pos2_t p2, double *elev, bool_t *water,
    size_t num_pts)
{
	double d_lat = p2.lat - p1.lat;
	double d_lon = p2.lon - p1.lon;

	for (size_t i = 0; i < num_pts; i++) {
		double f = i / (double)(num_pts - 1);
		/*
		 * normalize_hdg here is used to make sure we handle
		 * longitude wrapping correctly.
		 */
		elev[i] = tile_elev_read(GEO_POS2(p1.lat + f * d_lat,
		    p1.lon + f * d_lon), &water[i]);
	}
}

static bool_t
validate_freq(JNIEnv *env, double freq_mhz)
{
	if (freq_mhz < 20 || freq_mhz > 20000) {
		throw(env, "java/lang/IllegalArgumentException",
		    "Invalid frequency passed (%f), must a number between "
		    "20 and 20000 (RadioModel is only applicable from "
		    "20 MHz to 20 GHz)", freq_mhz);
		return (B_FALSE);
	}
	return (B_TRUE);
}

#ifdef	RELIEF_DEBUG

static void
relief_debug(double sta1_elev, double sta2_elev, double *elev, size_t num_pts)
{
	enum { MULT = 5, HEIGHT = 350 };
	cairo_surface_t *surf = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
	    MULT * num_pts, HEIGHT);
	cairo_t *cr = cairo_create(surf);
	double max_elev = MAX(sta1_elev, sta2_elev);

	for (size_t i = 0; i < num_pts; i++)
		max_elev = MAX(max_elev, elev[i]);

	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_paint(cr);

	cairo_translate(cr, 0, HEIGHT);
	cairo_scale(cr, 1, -1);
	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_move_to(cr, 0, 0);
	for (size_t i = 0; i < num_pts; i++)
		cairo_line_to(cr, MULT * i, (elev[i] / max_elev) * HEIGHT);
	cairo_line_to(cr, MULT * num_pts, 0);
	cairo_fill(cr);

	cairo_surface_write_to_png(surf, "relief.png");

	cairo_destroy(cr);
	cairo_surface_destroy(surf);
}

#endif	/* RELIEF_DEBUG */

static double
p2p_impl(double freq_mhz, bool_t horiz_pol, double xmit_gain,
    double recv_min_gain, geo_pos3_t sta1_pos, geo_pos3_t sta2_pos,
    double *terr_elev, bool_t *water_p)
{
	enum {
	    MIN_DIST = 1000,	/* meters */
	    MIN_STA_HGT = 3	/* meters */
	};
	vect3_t v1 = geo2ecef_mtr(sta1_pos, &wgs84);
	vect3_t v2 = geo2ecef_mtr(sta2_pos, &wgs84);
	double dist =
	    clamp(vect3_abs(vect3_sub(v1, v2)), MIN_DIST, rm.max_dist);
	double sta1_hgt, sta2_hgt, dbloss;
	size_t num_pts;
	double *elev = NULL;
	bool_t *water = NULL;
	double water_sum = 0, water_fract = 0;
	double cond, dielec;

	/*
	 * Stations are too far apart, no chance of them seeing each other.
	 */
	if (dist >= rm.max_dist)
		goto errout;

	num_pts = clampi(dist / rm.spacing, 2, rm.max_pts);
	elev = malloc(num_pts * sizeof (*elev));
	water = malloc(num_pts * sizeof (*water));
	relief_construct(GEO3_TO_GEO2(sta1_pos), GEO3_TO_GEO2(sta2_pos),
	    elev, water, num_pts);

	for (size_t i = 0; i < num_pts; i++)
		water_sum += water[i];
	water_fract = clamp(water_sum / num_pts, 0, 1);
	cond = wavg(ITM_CONDUCT_GND_AVG, ITM_CONDUCT_WATER_FRESH, water_fract);
	dielec = wavg(ITM_DIELEC_GND_AVG, ITM_DIELEC_WATER_FRESH, water_fract);

	sta1_hgt = MAX(sta1_pos.elev - elev[0], MIN_STA_HGT);
	sta2_hgt = MAX(sta2_pos.elev - elev[num_pts - 1], MIN_STA_HGT);

#ifdef	RELIEF_DEBUG
	relief_debug(sta1_pos.elev, sta2_pos.elev, elev, num_pts);
#endif

	itm_point_to_pointMDH(elev, num_pts, dist,
	    sta1_hgt, sta2_hgt, dielec, cond, ITM_NS_AVG, freq_mhz,
	    ITM_ENV_CONTINENTAL_TEMPERATE,
	    horiz_pol ? ITM_POL_HORIZ : ITM_POL_VERT, ITM_ACCUR_MAX,
	    ITM_ACCUR_MAX, ITM_ACCUR_MAX, &dbloss, NULL, NULL);

	if (terr_elev != NULL) {
		*terr_elev = elev[0];
		*water_p = water[0];
	}

	free(elev);
	free(water);

	return (MAX(xmit_gain - dbloss, recv_min_gain));
errout:
	if (terr_elev != NULL) {
		ASSERT(water_p != NULL);
		*terr_elev = tile_elev_read(GEO3_TO_GEO2(sta1_pos), water_p);
	}
	free(elev);
	free(water);
	return (recv_min_gain);
}

JNIEXPORT jdouble JNICALL
Java_com_vspro_util_RadioModel_pointToPoint(JNIEnv *env, jclass cls,
    jdouble freq_mhz, jboolean horiz_pol, jdouble xmit_gain,
    jdouble recv_min_gain,
    jdouble sta1_lat, jdouble sta1_lon, jdouble sta1_elev,
    jdouble sta2_lat, jdouble sta2_lon, jdouble sta2_elev)
{
	UNUSED(cls);

	if (!init_test(env))
		return (0);
	if (!validate_freq(env, freq_mhz))
		return (0);
	if (!is_valid_lat(sta1_lat) || !is_valid_lon(sta1_lon) ||
	    !is_valid_elev(sta1_elev) || !is_valid_lat(sta2_lat) ||
	    !is_valid_lon(sta2_lon) || !is_valid_elev(sta2_elev)) {
		throw(env, "java/lang/IllegalArgumentException",
		    "Invalid aircraft or tower lat x lon x elevation passed "
		    "(coordinates must be in degrees x degrees x meters AMSL)");
		return (0);
	}

	return (p2p_impl(freq_mhz, horiz_pol, xmit_gain, recv_min_gain,
	    GEO_POS3(sta1_lat, sta1_lon, sta1_elev),
	    GEO_POS3(sta2_lat, sta2_lon, sta2_elev), NULL, NULL));
}

static inline uint32_t
get_terr_color(double elev)
{
	enum { NUM_TERR_COLORS = 22 };
	const uint32_t terr_colors[NUM_TERR_COLORS] = {
	    BE32(0x217f87ffu),	/* -1000 - 0 */
	    BE32(0x3da58cffu),	/* 0 - 1000 */
	    BE32(0x5fa485ffu),	/* 1000 - 2000 */
	    BE32(0x87993dffu),	/* 2000 - 3000 */
	    BE32(0xf1cf5affu),	/* 3000 - 4000 */
	    BE32(0xf1cf5affu),	/* 4000 - 5000 */
	    BE32(0xf6b358ffu),	/* 5000 - 6000 */
	    BE32(0xf6b358ffu),	/* 6000 - 7000 */
	    BE32(0xd18b44ffu),	/* 7000 - 8000 */
	    BE32(0xd18b44ffu),	/* 8000 - 9000 */
	    BE32(0xb96f33ffu),	/* 9000 - 10000 */
	    BE32(0xb96f33ffu),	/* 10000 - 11000 */
	    BE32(0xb96f33ffu),	/* 11000 - 12000 */
	    BE32(0xb45b29ffu),	/* 12000 - 13000 */
	    BE32(0xb45b29ffu),	/* 13000 - 14000 */
	    BE32(0xb45b29ffu),	/* 14000 - 15000 */
	    BE32(0xb45b29ffu),	/* 15000 - 16000 */
	    BE32(0xb45b29ffu),	/* 16000 - 17000 */
	    BE32(0xb45b29ffu),	/* 17000 - 18000 */
	    BE32(0xb45b29ffu),	/* 18000 - 19000 */
	    BE32(0xb45b29ffu),	/* 19000 - 20000 */
	    BE32(0x9c4c26ffu)	/* rest */
	};
	int idx = clampi((MET2FEET(elev) + 500) / 1000, 0, NUM_TERR_COLORS - 1);
	return (terr_colors[idx]);
}

static void
paint_impl(double freq_mhz, bool_t horiz_pol, double xmit_gain,
    double recv_min_gain, geo_pos3_t twr, geo_pos2_t ctr, double sta1_elev,
    bool_t sta1_agl,  int x, int y, int pixel_size, double deg_range,
    uint8_t *pixels)
{
	double lon = ctr.lon + ((x / (double)pixel_size) - 0.5) * deg_range;
	double lat = ctr.lat - ((y / (double)pixel_size) - 0.5) * deg_range;
	double elev, signal_db, signal_rel, signal_rel_quant;
	bool_t water;
	uint32_t terr_color;
	uint32_t pixel, rem;
	uint8_t r, g, b, rem_r, rem_g, rem_b;
	double d_lat = ABS(lat - twr.lat);
	double d_lon = ABS(lon - twr.lon);
	geo_pos3_t sta1_pos;

	if (sqrt(POW2(d_lat) + POW2(d_lon)) > 5)
		return;

	terr_color = *(uint32_t *)(&pixels[4 * (y * pixel_size + x)]);

	if (sta1_agl) {
		sta1_pos = GEO_POS3(lat, lon,
		    tile_elev_read(GEO_POS2(lat, lon), NULL) + sta1_elev);
	} else {
		sta1_pos = GEO_POS3(lat, lon, sta1_elev);
	}
	signal_db = p2p_impl(freq_mhz, horiz_pol, xmit_gain, recv_min_gain,
	    sta1_pos, twr, &elev, &water);

	signal_rel = iter_fract(signal_db, recv_min_gain,
	    xmit_gain - 90, B_TRUE);

	r = (terr_color & 0xff0000) >> 16;
	g = (terr_color & 0xff00) >> 8;
	b = (terr_color & 0xff);
	rem = 0xffffffffu - terr_color;
	rem_r = (rem & 0xff0000) >> 16;
	rem_g = (rem & 0xff00) >> 8;
	rem_b = (rem & 0xff);

	signal_rel_quant = floor(signal_rel * 10) / 10;
	if (ABS(signal_rel - signal_rel_quant) < 0.005 && signal_rel > 0.05) {
		r -= r / 8;
		g -= g / 8;
		b -= b / 8;
	}
	pixel = 0xff000000u |
	    (r + (uint32_t)(rem_r * signal_rel_quant)) << 16 |
	    (g + (uint32_t)(rem_g * signal_rel_quant)) << 8 |
	    (b + (uint32_t)(rem_b * signal_rel_quant));
	*(uint32_t *)(&pixels[4 * (y * pixel_size + x)]) = pixel;
}

JNIEXPORT void JNICALL
Java_com_vspro_util_RadioModel_paintMapMulti(JNIEnv *env, jclass cls,
    jdouble freq_mhz, jboolean horiz_pol, jdouble xmit_gain,
    jdouble recv_min_gain, jdouble sta1_elev, jboolean sta1_agl,
    jdoubleArray sta2_lats, jdoubleArray sta2_lons, jdoubleArray sta2_elevs,
    jdouble deg_range, jdouble ctr_lat, jdouble ctr_lon, jint pixel_size,
    jstring out_file_str)
{
	uint8_t *pixels;
	const char *out_file;
	unsigned n_lats = (*env)->GetArrayLength(env, sta2_lats);
	unsigned n_lons = (*env)->GetArrayLength(env, sta2_lons);
	unsigned n_elevs = (*env)->GetArrayLength(env, sta2_elevs);
	jdouble *lats, *lons, *elevs;

	UNUSED(cls);

	if (!init_test(env) || !validate_freq(env, freq_mhz))
		return;
	if (!is_valid_elev(sta1_elev) || !is_valid_lat(ctr_lat) ||
	    !is_valid_lon(ctr_lon)) {
		throw(env, "java/lang/IllegalArgumentException",
		    "Invalid aircraft elevation or center lat x lon passed "
		    "(%f, %f, %f). Coordinates must be in degrees or meters.",
		    sta1_elev, ctr_lat, ctr_lon);
		return;
	}
	if (n_lats != n_lons || n_lats != n_elevs) {
		throw(env, "java/lang/IllegalArgumentException",
		    "`twr_lat', `sta2_lons' and `sta2_elevs' arrays must "
		    "contain the same number of elements (%d, %d, %d).",
		    n_lats, n_lons, n_elevs);
		return;
	}
	if (deg_range <= 0 || deg_range >= 90) {
		throw(env, "java/lang/IllegalArgumentException",
		    "Invalid display range in degrees (%f). Must be a positive "
		    "number greater than 0 and less than 90", deg_range);
		return;
	}
	if (pixel_size <= 2 || pixel_size > 8192) {
		throw(env, "java/lang/IllegalArgumentException",
		    "Invalid pixel size specified (%d). Must be a positive "
		    "integer not greater than 8192", pixel_size);
		return;
	}
	if (-(90 - xmit_gain) < recv_min_gain) {
		throw(env, "java/lang/IllegalArgumentException",
		    "xmit_gain (%.1f dB) is too low to receive using "
		    "recv_min_gain (%.1f dB). Increase xmit_gain.",
		    xmit_gain, recv_min_gain);
		return;
	}
	lats = (*env)->GetDoubleArrayElements(env, sta2_lats, NULL);
	lons = (*env)->GetDoubleArrayElements(env, sta2_lons, NULL);
	elevs = (*env)->GetDoubleArrayElements(env, sta2_elevs, NULL);
	for (unsigned i = 0; i < n_lats; i++) {
		if (!is_valid_lat(lats[i]) || !is_valid_lon(lons[i]) ||
		    !is_valid_elev(elevs[i])) {
			throw(env, "java/lang/IllegalArgumentException",
			    "lat/lon/elev index %d (%f, %f, %f) invalid. "
			    "Must be degrees x degrees x meters.", i,
			    lats[i], lons[i], elevs[i]);
			(*env)->ReleaseDoubleArrayElements(env,
			    sta2_lats, lats, JNI_ABORT);
			(*env)->ReleaseDoubleArrayElements(env,
			    sta2_lons, lons, JNI_ABORT);
			(*env)->ReleaseDoubleArrayElements(env,
			    sta2_elevs, elevs, JNI_ABORT);
			return;
		}
	}

	out_file = (*env)->GetStringUTFChars(env, out_file_str, NULL);
	pixels = malloc(4 * pixel_size * pixel_size);

	for (int y = 0; y < pixel_size; y++) {
		for (int x = 0; x < pixel_size; x++) {
			double lon = ctr_lon +
			    ((x / (double)pixel_size) - 0.5) * deg_range;
			double lat = ctr_lat -
			    ((y / (double)pixel_size) - 0.5) * deg_range;
			bool_t water = B_FALSE;
			double elev = tile_elev_read(GEO_POS2(lat, lon),
			    &water);
			uint32_t terr_color;

			if (water)
				terr_color = 0xffff8f47u;
			else
				terr_color = get_terr_color(elev);
			*(uint32_t *)(&pixels[4 * (y * pixel_size + x)]) =
			    terr_color;
		}
	}

	for (unsigned i = 0; i < n_lats; i++) {
		geo_pos3_t sta2_pos = GEO_POS3(lats[i], lons[i], elevs[i]);

		for (int y = 0; y < pixel_size; y++) {
			for (int x = 0; x < pixel_size; x++) {
				paint_impl(freq_mhz, horiz_pol, xmit_gain,
				    recv_min_gain, sta2_pos,
				    GEO_POS2(ctr_lat, ctr_lon), sta1_elev,
				    sta1_agl, x, y, pixel_size, deg_range,
				    pixels);
			}
		}
	}
	if (!png_write_to_file_rgba(out_file, pixel_size, pixel_size, pixels)) {
		throw(env, "java/lang/IllegalArgumentException",
		    "Error writing png file %s", out_file);
	}
	free(pixels);
	(*env)->ReleaseStringUTFChars(env, out_file_str, out_file);
	(*env)->ReleaseDoubleArrayElements(env, sta2_lats, lats, JNI_ABORT);
	(*env)->ReleaseDoubleArrayElements(env, sta2_lons, lons, JNI_ABORT);
	(*env)->ReleaseDoubleArrayElements(env, sta2_elevs, elevs, JNI_ABORT);
}

JNIEXPORT jdouble JNICALL
Java_com_vspro_util_RadioModel_elevProbe(JNIEnv *env, jclass cls,
    jdouble lat, jdouble lon)
{
	bool_t water;

	UNUSED(cls);

	if (!init_test(env))
		return (0);
	if (!is_valid_lat(lat) || !is_valid_lon(lon)) {
		throw(env, "java/lang/IllegalArgumentException",
		    "Invalid lat x lon (%f, %f). Coordinates must be "
		    "in degrees", lat, lon);
		return (0);
	}

	return (tile_elev_read(GEO_POS2(lat, lon), &water));
}
