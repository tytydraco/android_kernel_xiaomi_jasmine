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

#define pr_fmt(fmt) "simple_lmk: " fmt

#include <linux/cpu_input_boost.h>
#include <linux/devfreq_boost.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/simple_lmk.h>

#define MIN_FREE_PAGES (CONFIG_ANDROID_SIMPLE_LMK_MINFREE * SZ_1M / PAGE_SIZE)

/* Duration to boost CPU and DDR bus to the max per memory reclaim event */
#define BOOST_DURATION_MS (1000)

/* Pulled from the Android framework */
static const short int adj_prio[] = {
	906, /* CACHED_APP_MAX_ADJ */
	900, /* CACHED_APP_MIN_ADJ */
	800, /* SERVICE_B_ADJ */
	700, /* PREVIOUS_APP_ADJ */
	600, /* HOME_APP_ADJ */
	500, /* SERVICE_ADJ */
	400, /* HEAVY_WEIGHT_APP_ADJ */
	300  /* BACKUP_APP_ADJ */
};

static DEFINE_SPINLOCK(reclaim_lock);
static unsigned long last_reclaim_expires;

static unsigned long scan_and_kill(int min_adj, int max_adj,
	unsigned long pages_needed)
{
	/* Boost priority of victim tasks so they can die quickly */
	static const struct sched_param param = {
		.sched_priority = MAX_RT_PRIO - 1
	};
	struct task_struct *tsk;
	unsigned long pages_freed = 0;

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *victim;
		unsigned long tasksize;
		short oom_score_adj;

		/* Don't commit suicide or kill kthreads */
		if (same_thread_group(tsk, current) || tsk->flags & PF_KTHREAD)
			continue;

		victim = find_lock_task_mm(tsk);
		if (!victim)
			continue;

		/* Don't kill tasks that have been killed or lack memory */
		if (victim->lmk_sigkill_sent ||
			test_tsk_thread_flag(victim, TIF_MEMDIE)) {
			task_unlock(victim);
			continue;
		}

		oom_score_adj = victim->signal->oom_score_adj;
		if (oom_score_adj < min_adj || oom_score_adj > max_adj) {
			task_unlock(victim);
			continue;
		}

		tasksize = get_mm_rss(victim->mm);
		task_unlock(victim);
		if (!tasksize)
			continue;

		get_task_struct(victim);
		if (do_send_sig_info(SIGKILL, SEND_SIG_NOINFO, victim, true)) {
			tasksize = 0;
		} else {
			victim->lmk_sigkill_sent = true;
			sched_setscheduler_nocheck(victim, SCHED_FIFO, &param);
		}
		put_task_struct(victim);

		pages_freed += tasksize;
		if (pages_freed >= pages_needed)
			break;
	}
	rcu_read_unlock();

	return pages_freed;
}

static void do_lmk_reclaim(unsigned long pages_needed)
{
	unsigned long pages_freed = 0;
	int i;

	for (i = 1; i < ARRAY_SIZE(adj_prio); i++) {
		pages_freed += scan_and_kill(adj_prio[i], adj_prio[i - 1],
					pages_needed - pages_freed);
		if (pages_freed >= pages_needed)
			break;
	}

	if (pages_freed)
		pr_info("freed %lu MiB\n", pages_freed * PAGE_SIZE / SZ_1M);
}

void simple_lmk_mem_reclaim(void)
{
	unsigned long flags;

	if (time_before(jiffies, last_reclaim_expires))
		return;

	/* Only one memory reclaim event can occur at a time */
	if (!spin_trylock_irqsave(&reclaim_lock, flags))
		return;

	last_reclaim_expires = jiffies + LMK_KILL_TIMEOUT;
	cpu_input_boost_kick_max(BOOST_DURATION_MS);
	devfreq_boost_kick_max(DEVFREQ_MSM_CPUBW, BOOST_DURATION_MS);
	do_lmk_reclaim(MIN_FREE_PAGES);
	spin_unlock_irqrestore(&reclaim_lock, flags);
}

/* Needed to prevent Android from thinking there's no LMK and thus rebooting */
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "lowmemorykiller."
static int minfree_unused;
module_param_named(minfree, minfree_unused, int, 0200);
