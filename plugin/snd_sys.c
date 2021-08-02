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
 * Copyright 2021 Saso Kiselkov. All rights reserved.
 */

#include <XPLMProcessing.h>

#include <alc.h>
#include <alext.h>

#include <acfutils/dr.h>
#include <acfutils/helpers.h>
#include <acfutils/wav.h>
#include <acfutils/worker.h>
#include <acfutils/thread.h>

#include <libradio/navrad.h>

#include "libradio_plugin.h"
#include "snd_sys.h"

#define	WORKER_INTVAL		20000	/* us */
#define	CKPT_SPKR_POS		0.0, 0.9, -6.7
#define	CKPT_SPKR_REF_DIST	1

typedef struct {
	ALuint		buf;
	list_node_t	node;
} albuf_t;

typedef struct {
	bool		inited;
	ALuint		source;
	list_t		bufs;
	bool		squelch;
	unsigned	nr;
	navrad_type_t	type;
	unsigned	stream_id;
} radio_t;

typedef enum {
	SND_TGT_CKPT_SPKR = 1 << 0,
	SND_TGT_HEADSET = 1 << 1
} snd_tgt_t;

typedef struct {
	snd_tgt_t	tgt;
	unsigned	stream_id;
	char		*dev;
	alc_t		*alc;
	radio_t		nav[2];
	radio_t		adf[2];
	radio_t		dme;
} snd_sys_t;

static struct {
	dr_t	peX;
	dr_t	peY;
	dr_t	peZ;
	dr_t	view_pitch;
	dr_t	view_roll;
	dr_t	view_hdg;

	dr_t	paused;

	dr_t	master_volume;
	dr_t	radio_volume;
	dr_t	nav_volume[2];
	dr_t	nav_audio_on[2];
	dr_t	dme_volume[2];
	dr_t	dme_audio_on[2];
	dr_t	adf_volume[2];
	dr_t	adf_audio_on[2];
} drs;

static struct {
	mutex_t	lock;
	bool	paused;
	vect3_t	pos;
	vect3_t	orient;
	float	master_volume;
	float	radio_volume;
	float	nav_volume[2];
	bool	nav_audio_on[2];
	float	dme_volume[2];
	bool	dme_audio_on[2];
	float	adf_volume[2];
	bool	adf_audio_on[2];
} snd_state = {};

static bool		inited = false;
static snd_sys_t	main_sys = {};
static worker_t		main_sys_wk = {};

static float snd_sys_floop_cb(float unused1, float unused2, int unused,
    void *unused4);
static void snd_sys_worker_fini(void *userinfo);

static void
radio_init(radio_t *radio, unsigned nr, navrad_type_t type, unsigned stream_id)
{
	ASSERT(radio != NULL);

	alGenSources(1, &radio->source);
	alSourcei(radio->source, AL_LOOPING, 0);
	alSource3f(radio->source, AL_POSITION, CKPT_SPKR_POS);
	alSourcef(radio->source, AL_REFERENCE_DISTANCE, CKPT_SPKR_REF_DIST);
	alSourcef(radio->source, AL_ROLLOFF_FACTOR, 1);
	alSourcei(radio->source, AL_AIR_ABSORPTION_FACTOR, 1);
	list_create(&radio->bufs, sizeof (albuf_t), offsetof(albuf_t, node));
	radio->nr = nr;
	radio->type = type;
	radio->stream_id = stream_id;
	radio->inited = true;
}

static void
radio_fini(radio_t *radio)
{
	albuf_t *buf;

	if (!radio->inited)
		return;
	radio->inited = false;

	if (radio->source != 0)
		alDeleteSources(1, &radio->source);
	while ((buf = list_remove_head(&radio->bufs)) != NULL) {
		if (buf->buf != 0)
			alDeleteBuffers(1, &buf->buf);
		ZERO_FREE(buf);
	}
	list_destroy(&radio->bufs);

	memset(radio, 0, sizeof (*radio));
}

static bool_t
snd_sys_worker_init(void *userinfo)
{
	static const int attrs[] = {
		ALC_MONO_SOURCES, 2048,
		ALC_STEREO_SOURCES, 2048,
		ALC_HRTF_SOFT, 1,
		0	/* list terminator */
	};
	snd_sys_t *sys;

	ASSERT(userinfo);
	sys = userinfo;

	/* either no preferred device, or device open failed */
	sys->alc = openal_init2(sys->dev, false, attrs, true);
	if (sys->alc == NULL) {
		logMsg("openal-soft multi-threaded init failure");
		goto errout;
	}
	for (int i = 0; i < NUM_NAV_RADIOS; i++) {
		radio_init(&sys->nav[i], i, NAVRAD_TYPE_VLOC, sys->stream_id);
		radio_init(&sys->adf[i], i, NAVRAD_TYPE_ADF, sys->stream_id);
	}
	radio_init(&sys->dme, 0, NAVRAD_TYPE_DME, sys->stream_id);

	XPLMRegisterFlightLoopCallback(snd_sys_floop_cb, -1, NULL);

	return (true);
errout:
	snd_sys_worker_fini(sys);
	return (false);
}

static void
snd_sys_worker_fini(void *userinfo)
{
	snd_sys_t *sys;

	ASSERT(userinfo);
	sys = userinfo;

	XPLMUnregisterFlightLoopCallback(snd_sys_floop_cb, NULL);

	for (int i = 0; i < NUM_NAV_RADIOS; i++) {
		radio_fini(&sys->nav[i]);
		radio_fini(&sys->adf[i]);
	}
	radio_fini(&sys->dme);
	if (sys->alc != NULL)
		openal_fini(sys->alc);
	free(sys->dev);
	memset(sys, 0, sizeof (*sys));
}

static void
radio_snd_worker(const snd_sys_t *sys, radio_t *radio)
{
	int processed = 0, state = 0;
	bool_t play;
	float volume;

	ASSERT(sys != NULL);
	ASSERT(radio != NULL);
	ASSERT3U(radio->nr, <, 2);

	mutex_enter(&snd_state.lock);
	switch (radio->type) {
	case NAVRAD_TYPE_VLOC:
		volume = snd_state.master_volume * snd_state.radio_volume *
		    snd_state.nav_volume[radio->nr];
		break;
	case NAVRAD_TYPE_ADF:
		volume = snd_state.master_volume * snd_state.radio_volume *
		    snd_state.adf_volume[radio->nr];
		break;
	case NAVRAD_TYPE_DME:
		volume = snd_state.master_volume * snd_state.radio_volume *
		    snd_state.dme_volume[radio->nr];
		break;
	default:
		VERIFY_FAIL();
	}
	play = (volume != 0 && !snd_state.paused);
	mutex_exit(&snd_state.lock);

	if (!play && sys->stream_id == 0)
		navrad_sync_streams(radio->type, radio->nr);
	alGetSourcei(radio->source, AL_SOURCE_STATE, &state);
	if (!play && state == AL_PLAYING)
		alSourceStop(radio->source);

	if (play) {
		alGetSourcei(radio->source, AL_BUFFERS_PROCESSED, &processed);

		for (int i = 0; i < processed; i++) {
			albuf_t *buf = list_remove_head(&radio->bufs);

			ASSERT(buf != NULL);
			ASSERT(buf->buf != 0);
			alSourceUnqueueBuffers(radio->source, 1, &buf->buf);
			alDeleteBuffers(1, &buf->buf);
			ZERO_FREE(buf);
		}
		while (list_count(&radio->bufs) < 2) {
			size_t num_samples;
			int16_t *samples = navrad_get_audio_buf2(radio->type,
			    radio->nr, volume, radio->squelch, false,
			    radio->stream_id, &num_samples);
			albuf_t *buf;

			if (samples == NULL)
				break;
			buf = safe_calloc(1, sizeof (*buf));
			alGenBuffers(1, &buf->buf);
			alBufferData(buf->buf, AL_FORMAT_MONO16, samples,
			    num_samples * sizeof (*samples),
			    NAVRAD_AUDIO_SRATE);
			alSourceQueueBuffers(radio->source, 1, &buf->buf);
			list_insert_tail(&radio->bufs, buf);

			navrad_free_audio_buf(samples);
		}
	} else {
		albuf_t *buf;

		while ((buf = list_remove_head(&radio->bufs)) != NULL) {
			ASSERT(buf->buf != 0);
			alSourceUnqueueBuffers(radio->source, 1, &buf->buf);
			alDeleteBuffers(1, &buf->buf);
			ZERO_FREE(buf);
		}
	}

	if (play && state != AL_PLAYING)
		alSourcePlay(radio->source);

	/* Headset target. Always on. */
	alSourcei(radio->source, AL_SOURCE_SPATIALIZE_SOFT, 0);
	alSourcef(radio->source, AL_GAIN, 1);
}

static bool_t
snd_sys_worker(void *userinfo)
{
	snd_sys_t *sys;

	ASSERT(userinfo != NULL);
	sys = userinfo;

	if (sys->tgt & SND_TGT_CKPT_SPKR) {
		mutex_enter(&snd_state.lock);
		alc_listener_set_pos(sys->alc, snd_state.pos);
		alc_listener_set_orient(sys->alc, snd_state.orient);
		mutex_exit(&snd_state.lock);
	}
	for (unsigned i = 0; i < NUM_NAV_RADIOS; i++) {
		radio_snd_worker(sys, &sys->nav[i]);
		radio_snd_worker(sys, &sys->adf[i]);
	}
	radio_snd_worker(sys, &sys->dme);

	return (true);
}

bool
snd_sys_init(void)
{
	ASSERT(!inited);
	inited = true;

	fdr_find(&drs.peX, "sim/aircraft/view/acf_peX");
	fdr_find(&drs.peY, "sim/aircraft/view/acf_peY");
	fdr_find(&drs.peZ, "sim/aircraft/view/acf_peZ");
	fdr_find(&drs.view_pitch, "sim/graphics/view/cockpit_pitch");
	/* Yes, the hdg/roll drs are transposed on purpose - XP bug */
	fdr_find(&drs.view_roll, "sim/graphics/view/cockpit_heading");
	fdr_find(&drs.view_hdg, "sim/graphics/view/cockpit_roll");

	fdr_find(&drs.paused, "sim/time/paused");

	fdr_find(&drs.master_volume, "sim/operation/sound/master_volume_ratio");
	fdr_find(&drs.radio_volume, "sim/operation/sound/radio_volume_ratio");
	for (int i = 0; i < 2; i++) {
		fdr_find(&drs.nav_volume[i],
		    "sim/cockpit2/radios/actuators/audio_volume_nav%d", i + 1);
		fdr_find(&drs.nav_audio_on[i],
		    "sim/cockpit2/radios/actuators/audio_selection_nav%d",
		    i + 1);
		fdr_find(&drs.dme_volume[i],
		    "sim/cockpit2/radios/actuators/audio_volume_dme%d", i + 1);
		fdr_find(&drs.dme_audio_on[i],
		    "sim/cockpit2/radios/actuators/audio_selection_dme%d",
		    i + 1);
		fdr_find(&drs.adf_volume[i],
		    "sim/cockpit2/radios/actuators/audio_volume_adf%d", i + 1);
		fdr_find(&drs.adf_audio_on[i],
		    "sim/cockpit2/radios/actuators/audio_selection_adf%d",
		    i + 1);
	}

	mutex_init(&snd_state.lock);

	main_sys.tgt = SND_TGT_CKPT_SPKR | SND_TGT_HEADSET;
	main_sys.stream_id = 0;
	worker_init2(&main_sys_wk, snd_sys_worker_init, snd_sys_worker,
	    snd_sys_worker_fini, WORKER_INTVAL, &main_sys, "libradio_snd_wk");

	return (true);
}

void
snd_sys_fini(void)
{
	if (!inited)
		return;
	inited = false;

	worker_fini(&main_sys_wk);
	mutex_destroy(&snd_state.lock);
}

static float
snd_sys_floop_cb(float unused1, float unused2, int unused3, void *unused4)
{
	UNUSED(unused1);
	UNUSED(unused2);
	UNUSED(unused3);
	UNUSED(unused4);

	mutex_enter(&snd_state.lock);
	snd_state.paused = (dr_geti(&drs.paused) != 0);
	snd_state.pos = VECT3(dr_getf(&drs.peX), dr_getf(&drs.peY),
	    dr_getf(&drs.peZ));
	snd_state.orient = VECT3(dr_getf(&drs.view_pitch), dr_getf(&drs.view_hdg),
	    dr_getf(&drs.view_roll));
	snd_state.master_volume = dr_getf(&drs.master_volume);
	snd_state.radio_volume = dr_getf(&drs.radio_volume);
	for (int i = 0; i < 2; i++) {
		snd_state.nav_volume[i] = dr_getf(&drs.nav_volume[i]) *
		    dr_geti(&drs.nav_audio_on[i]);
		snd_state.dme_volume[i] = dr_getf(&drs.dme_volume[i]) *
		    dr_geti(&drs.dme_audio_on[i]);
		snd_state.adf_volume[i] = dr_getf(&drs.adf_volume[i]) *
		    dr_geti(&drs.adf_audio_on[i]);
	}

	mutex_exit(&snd_state.lock);

	return (-1);
}
