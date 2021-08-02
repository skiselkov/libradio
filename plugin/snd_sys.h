/*
 * CONFIDENTIAL
 *
 * Copyright 2021 Saso Kiselkov. All rights reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property
 * of Saso Kiselkov. The intellectual and technical concepts contained
 * herein are proprietary to Saso Kiselkov and may be covered by U.S. and
 * Foreign Patents, patents in process, and are protected by trade secret
 * or copyright law. Dissemination of this information or reproduction of
 * this material is strictly forbidden unless prior written permission is
 * obtained from Saso Kiselkov.
 */

#ifndef	_SND_SYS_H_
#define	_SND_SYS_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool snd_sys_init(void);
void snd_sys_fini(void);

#ifdef __cplusplus
}
#endif

#endif	/* _SND_SYS_H_ */
