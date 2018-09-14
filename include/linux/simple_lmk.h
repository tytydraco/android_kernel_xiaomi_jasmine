/*
 * Copyright (C) 2018, Sultan Alsawaf <sultanxda@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _SIMPLE_LMK_H_
#define _SIMPLE_LMK_H_

/* Time until memory reclaim can be triggered again */
#define LMK_KILL_TIMEOUT (msecs_to_jiffies(CONFIG_ANDROID_SIMPLE_LMK_TIMEOUT))

void simple_lmk_mem_reclaim(void);

#endif /* _SIMPLE_LMK_H_ */
