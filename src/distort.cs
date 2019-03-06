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
 * Copyright 2019 Saso Kiselkov. All rights reserved.
 */

using System.Runtime.InteropServices;

public class distort {

	/*
	 * Initializer and finalizer functions.
	 * Call to create & destroy context.
	 * Sample rate must be one of: 44100 or 48000.
	 */
	[DllImport("distort.dll"), EntryPoint = "distort_init@4"]
	public static extern IntPtr init(int sample_rate);

	[DllImport("distort.dll"), EntryPoint = "distort_fini@4"]
	public static extern void fini(IntPtr context);

	/*
	 * Applies distortion to an input buffer. The buffer is replaced by the
	 * distorted audio samples. If there isn't enough data in the input buffer
	 * or cached data in the `dis' state object, the buffer will have silence
	 * prepended to it to avoid outputting undistorted audio.
	 *
	 * Argument description:
	 *
	 * `context': distortion context previously created using
	 *	distort_init().
	 * `samples': an array of 16-bit signed PCM samples (single
	 *	channel only!). Please note that the native function
	 *	expects this to be a pointer to a simple array of samples,
	 *	not a complex class object or anything of the sort.
	 * `num_samples': number of samples contained in `samples'.
	 * `amplify': voice signal amplification level. You can pass 1.0
	 *	in here to not have the signal amplified at all, or a value
	 *	less than 1.0 to suppress the signal. This is not applied
	 *	to the background noise generator, so you simulate a fading
	 *	volume on the signal portion while keeping the noise volume
	 *	constant.
	 * `noise_level': background noise level from 0.0 to 1.0 (nothing
	 *	but noise in the output). Typical values would be 0.02 for
	 *	little background noise, 0.2 for moderate background noise,
	 *	to 0.6 for heavy background noise (makes transmission almost
	 *	unreadable).
	 */
	[DllImport("distort.dll"), EntryPoint = "distort@28"]
	public static extern void distort(IntPtr context, short[] samples,
	    long num_samples, double amplify, double noise_level);

	[DllImport("distort.dll"), EntryPoint = "distort_clear_buffers@4"]
	public static extern void clear_buffers(IntPtr context);
}
