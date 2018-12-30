// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Sultan Alsawaf <sultan@kerneltoast.com>.
 */
#ifndef _CPU_INPUT_BOOST_H_
#define _CPU_INPUT_BOOST_H_

#ifdef CONFIG_CPU_INPUT_BOOST
bool cpu_input_boost_within_timeout(unsigned int input_boost_timeout);
void cpu_input_boost_kick(void);
void cpu_general_boost_kick(unsigned int duration_ms);
void cpu_input_boost_kick_max(unsigned int duration_ms);
#else
static inline void cpu_input_boost_kick(void)
{
}
static inline void cpu_general_boost_kick(unsigned int duration_ms)
{
}
static inline void cpu_input_boost_kick_max(unsigned int duration_ms)
{
}
#endif

#endif /* _CPU_INPUT_BOOST_H_ */
