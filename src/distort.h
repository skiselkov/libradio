#ifndef	__DISTORT_H__
#define	__DISTORT_H__

#include <stdint.h>
#if	IBM
#include <windows.h>
#endif

#ifdef  __cplusplus
extern "C" {
#endif

#if	IBM
#define	DISTORT_API	__declspec(dllexport) __stdcall
#else
#define	DISTORT_API
#endif

typedef struct distort_s distort_t;

/*
 * Initializer and finalizer functions. Call to create & destroy context.
 * Sample rate must be one of: 44100 or 48000.
 */
DISTORT_API distort_t *distort_init(unsigned sample_rate);
DISTORT_API void distort_fini(distort_t *dis);

/*
 * Applies distortion to an input buffer. The buffer is replaced by the
 * distorted audio samples. If there isn't enough data in the input buffer
 * or cached data in the `dis' state object, the buffer will have silence
 * prepended to it to avoid outputting undistorted audio.
 *
 * Argument description:
 *
 * `dis': distortion control object previously created using distort_init().
 * `samples': an array of 16-bit signed PCM samples (single channel only!).
 * `num_samples': number of samples contained in `samples'.
 * `amplify': voice signal amplification level. You can pass 1.0 in here
 *	to not have the signal amplified at all, or a value less than 1.0
 *	to suppress the signal. This is not applied to the background noise
 *	generator, so you simulate a fading volume on the signal portion
 *	while keeping the noise volume constant.
 * `noise_level': background noise level from 0.0 to 1.0 (nothing but noise
 *	in the output). Typical values would be 0.02 for little background
 *	noise, 0.2 for moderate background noise, to 0.6 for heavy background
 *	noise (makes transmission almost unreadable).
 */
DISTORT_API void distort(distort_t *dis, int16_t *samples,
    size_t num_samples, double amplify, double noise_level);

/*
 * Clears any cached buffers from the distortion object. Call this between
 * audio transmissions to avoid any partially completed chunks being played
 * at the start of an unrelated transmission.
 */
DISTORT_API void distort_clear_buffers(distort_t *dis);

#ifdef  __cplusplus
}
#endif

#endif	/* __DISTORT_H__ */
