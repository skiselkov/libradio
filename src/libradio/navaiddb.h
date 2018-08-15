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

#ifndef	_LIBRADIO_NAVAIDDB_H_
#define	_LIBRADIO_NAVAIDDB_H_

#include <acfutils/avl.h>
#include <acfutils/geom.h>
#include <acfutils/list.h>

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct navaiddb_s navaiddb_t;

typedef enum {
	NAVAID_NDB =	1 << 0,
	NAVAID_VOR =	1 << 1,
	NAVAID_LOC =	1 << 2,
	NAVAID_GS =	1 << 3,
	NAVAID_MRK =	1 << 4,
	NAVAID_DME =	1 << 5,
	NAVAID_FPAP =	1 << 6,
	NAVAID_LTP =	1 << 7,
	NAVAID_GLS =	1 << 8
} navaid_type_t;

typedef enum {
	MRK_TYPE_OM,
	MRK_TYPE_MM,
	MRK_TYPE_IM
} mrk_type_t;

typedef enum {
	FPAP_PERF_LP,
	FPAP_PERF_LPV,
	FPAP_PERF_APV_II,
	FPAP_PERF_GLS
} fpap_perf_t;

typedef enum {
	LTP_PROV_WAAS,
	LTP_PROV_EGNOS,
	LTP_PROV_MSAS,
	LTP_PROV_GP
} ltp_prov_t;

typedef struct {
	navaid_type_t	type;

	geo_pos3_t	pos;
	vect3_t		ecef;
	uint64_t	freq;
	double		range;

	char		id[8];
	char		icao[8];
	char		region[8];
	char		name[32];

	union {
		struct {
			double		magvar;		/* degrees */
		} vor;
		struct {
			double		brg;		/* degrees */
			char		rwy_id[8];
		} loc;
		struct {
			double		brg;		/* degrees */
			double		gs;		/* degrees */
			char		rwy_id[8];
		} gs;
		struct {
			double		brg;		/* degrees */
			mrk_type_t	type;
			char		rwy_id[8];
		} mrk;
		struct {
			double		bias;		/* meters */
			char		arpt_id[8];
		} dme;
		struct {
			double		crs;		/* degrees */
			char		proc_id[8];
			char		rwy_id[8];
			fpap_perf_t	perf;
		} fpap;
		struct {
			double		tch;		/* meters */
			double		crs;		/* degrees */
			double		gs;		/* degrees */
			char		proc_id[8];
			char		rwy_id[8];
			ltp_prov_t	prov;
		} ltp;
		struct {
			double		crs;		/* degrees */
			double		gs;		/* degrees */
			char		proc_id[8];
			char		rwy_id[8];
		} gls;
	};

	list_node_t	node;
	avl_node_t	lat_node;
	avl_node_t	lon_node;
	avl_node_t	freq_node;
} navaid_t;

typedef struct {
	navaid_t	**navaids;
	size_t		num_navaids;
} navaid_list_t;

navaiddb_t *navaiddb_create(const char *xpdir);
void navaiddb_destroy(navaiddb_t *db);

navaid_list_t *navaiddb_query(navaiddb_t *db, geo_pos2_t center,
    double radius, const char *id, uint64_t *freq, navaid_type_t *type);
void navaiddb_list_free(navaid_list_t *list);

const char *navaid_type2str(navaid_type_t type);
uint64_t navaid_act_freq(navaid_type_t type, uint64_t ref_freq);

#ifdef	__cplusplus
}
#endif

#endif	/* _LIBRADIO_NAVAIDDB_H_ */
