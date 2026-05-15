// SPDX-License-Identifier: GPL-2.0
/*
 * Reflex CPUFreq governor (based on schedutil)
 * Copyright (C) 2026 Masahito Suzuki
 *
 * schedutil + idle-time accounting based hispeed floor with PELT-
 * complementary exponential decay.
 *
 * Frequency scaling is identical to schedutil (including the 1.25×
 * DVFS headroom) except for the hispeed blend:
 *
 * On each observation window, real CPU busy% is measured from
 * kcpustat idle-time counters.  This is blended with PELT util:
 *
 *   blended = pelt + (hispeed - pelt) >> half_lives
 *
 * The blend decays with PELT's 32 ms half-life: every 32 ms, the
 * hispeed contribution halves while PELT fills the same gap,
 * keeping total coverage at ~100%.  After ~320 ms hispeed is
 * negligible and PELT-based proportional scaling takes full control.
 */

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/irq_work.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sched/cpufreq.h>
#include <linux/sched/topology.h>
#include <linux/slab.h>
#include <linux/tick.h>
#include <uapi/linux/sched/types.h>
#include <asm/topology.h>

#define CPUFREQ_REFLEX_PROGNAME "Reflex CPUFreq Governor"
#define CPUFREQ_REFLEX_AUTHOR   "Masahito Suzuki"
#define CPUFREQ_REFLEX_VERSION  "0.2.2-gki510"

#define CPUFREQ_REFLEX_DEFAULT_HISPEED_WINDOW_US    4000
#define CPUFREQ_REFLEX_DEFAULT_HISPEED_FILTER_SHIFT 1

#define SCHED_FLAG_SUGOV 0x10000000
#define IOWAIT_BOOST_MIN (SCHED_CAPACITY_SCALE / 8)

/* -----------------------------------------------------------------------
 * GKI 5.10 compatibility stubs
 * ----------------------------------------------------------------------- */

/* cpufreq_driver_test_flags: NOT in GKI 5.10 */
static inline bool rfx_driver_test_flags(u16 flags) { return false; }

/* cpufreq_driver_has_adjust_perf: NOT in GKI 5.10 */
static inline bool rfx_driver_has_adjust_perf(void) { return false; }

/* scx_switched_all: sched_ext not in GKI 5.10 */
static inline bool rfx_scx_switched_all(void) { return false; }

/* uclamp_rq_is_capped: NOT in GKI 5.10 */
static inline bool rfx_cpu_uclamp_capped(unsigned int cpu) { return false; }

/* get_capacity_ref_freq: NOT in GKI 5.10 → use policy->cpuinfo.max_freq */
static inline unsigned int rfx_get_ref_freq(struct cpufreq_policy *policy)
{
	return policy->cpuinfo.max_freq;
}

/*
 * [Fix2] to_gov_attr_set: NOT in GKI 5.10.
 * In newer kernels it is defined as:
 *   container_of(kobj, struct gov_attr_set, kobj)
 * We implement it inline here.
 */
static inline struct gov_attr_set *rfx_to_gov_attr_set(struct kobject *kobj)
{
	return container_of(kobj, struct gov_attr_set, kobj);
}

/* ----------------------------------------------------------------------- */

struct rfx_tunables {
	struct gov_attr_set attr_set;
	unsigned int rate_limit_us;
	unsigned int hispeed_window_us;
	unsigned int hispeed_filter_shift;
};

struct rfx_policy {
	struct cpufreq_policy *policy;
	struct rfx_tunables *tunables;
	struct list_head tunables_hook;
	raw_spinlock_t update_lock;
	u64 last_freq_update_time;
	s64 freq_update_delay_ns;
	unsigned int next_freq;
	unsigned int cached_raw_freq;
	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;
	bool limits_changed;
	bool need_freq_update;
};

struct rfx_cpu {
	struct update_util_data update_util;
	struct rfx_policy *rfx_policy;
	unsigned int cpu;
	bool iowait_boost_pending;
	unsigned int iowait_boost;
	u64 last_update;
	unsigned long util;
	unsigned long bw_min;
	u64 prev_idle_time;
	u64 prev_wall_time;
	unsigned int busy_pct;
	unsigned int filtered_busy_pct;
	bool hispeed_active;
	u64 hispeed_start_ns;
	unsigned int hispeed_idle_windows;
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct rfx_cpu, rfx_cpu);

static inline struct rfx_tunables *to_rfx_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct rfx_tunables, attr_set);
}

/************************ Governor internals ***********************/

static bool rfx_should_update_freq(struct rfx_policy *rfx_pol, u64 time)
{
	s64 delta_ns;

	if (!cpufreq_this_cpu_can_update(rfx_pol->policy))
		return false;

	if (unlikely(READ_ONCE(rfx_pol->limits_changed))) {
		WRITE_ONCE(rfx_pol->limits_changed, false);
		rfx_pol->need_freq_update = true;
		smp_mb();
		return true;
	} else if (rfx_pol->need_freq_update) {
		return true;
	}

	delta_ns = time - rfx_pol->last_freq_update_time;
	return delta_ns >= rfx_pol->freq_update_delay_ns;
}

static bool rfx_update_next_freq(struct rfx_policy *rfx_pol, u64 time,
				 unsigned int next_freq)
{
	if (rfx_pol->need_freq_update) {
		rfx_pol->need_freq_update = false;
		if (rfx_pol->next_freq == next_freq &&
		    false)
			return false;
	} else if (rfx_pol->next_freq == next_freq) {
		return false;
	}
	rfx_pol->next_freq = next_freq;
	rfx_pol->last_freq_update_time = time;
	return true;
}

static void rfx_deferred_update(struct rfx_policy *rfx_pol)
{
	if (!rfx_pol->work_in_progress) {
		rfx_pol->work_in_progress = true;
		irq_work_queue(&rfx_pol->irq_work);
	}
}

static unsigned int rfx_get_next_freq(struct rfx_policy *rfx_pol,
				      unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = rfx_pol->policy;
	unsigned int freq;

	freq = rfx_get_ref_freq(policy);
	freq = map_util_freq(util, freq, max);

	if (freq == rfx_pol->cached_raw_freq && !rfx_pol->need_freq_update)
		return rfx_pol->next_freq;

	rfx_pol->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(policy, freq);
}

/* [Fix4] Use rfx_get_util_gki510 exported from cpufreq_schedutil.c */
static void rfx_get_util(struct rfx_cpu *rfx_c, unsigned long boost)
{
	rfx_get_util_gki510(rfx_c->cpu, boost,
			    &rfx_c->util, &rfx_c->bw_min);
}

/************************ Hispeed accounting ***********************/

static void rfx_update_busy_pct(struct rfx_cpu *rfx_c, unsigned int window_us,
				unsigned int filter_shift, u64 time)
{
	u64 cur_idle, cur_wall;
	unsigned int wall_delta, idle_delta;

	cur_idle = get_cpu_idle_time(rfx_c->cpu, &cur_wall, 1);
	wall_delta = (unsigned int)(cur_wall - rfx_c->prev_wall_time);

	if (wall_delta >= window_us) {
		rfx_c->busy_pct = 0;
		rfx_c->hispeed_active = true;
		rfx_c->prev_idle_time = cur_idle;
		rfx_c->prev_wall_time = cur_wall;
		return;
	}
	if (!rfx_c->hispeed_active)
		return;
	rfx_c->hispeed_active = false;

	idle_delta = (cur_idle > rfx_c->prev_idle_time) ?
		     (unsigned int)(cur_idle - rfx_c->prev_idle_time) : 0;
	rfx_c->busy_pct = (wall_delta > idle_delta) ?
			  100 * (wall_delta - idle_delta) / wall_delta : 0;
	rfx_c->prev_idle_time = cur_idle;
	rfx_c->prev_wall_time = cur_wall;

	if (!filter_shift || rfx_c->busy_pct >= rfx_c->filtered_busy_pct) {
		rfx_c->filtered_busy_pct = rfx_c->busy_pct;
	} else {
		unsigned int step = (rfx_c->filtered_busy_pct - rfx_c->busy_pct)
				    >> filter_shift;
		if (step)
			rfx_c->filtered_busy_pct -= step;
		else
			rfx_c->filtered_busy_pct = rfx_c->busy_pct;
	}

	if (rfx_c->filtered_busy_pct > 0) {
		rfx_c->hispeed_idle_windows = 0;
		if (!rfx_c->hispeed_start_ns)
			rfx_c->hispeed_start_ns = time;
	} else {
		if (++rfx_c->hispeed_idle_windows >= 2) {
			rfx_c->hispeed_start_ns = 0;
			rfx_c->filtered_busy_pct = 0;
		}
	}
}

#define HISPEED_HALFLIFE_NS (32 * NSEC_PER_MSEC)

static unsigned long rfx_blend_util(struct rfx_cpu *rfx_c,
				    unsigned long pelt_util,
				    unsigned long max_cap, u64 time)
{
	unsigned long hispeed_util;
	unsigned int half_lives;

	if (!rfx_c->filtered_busy_pct || !rfx_c->hispeed_start_ns)
		return pelt_util;

	hispeed_util = max_cap * rfx_c->filtered_busy_pct / 100;
	if (hispeed_util <= pelt_util)
		return pelt_util;

	half_lives = (unsigned int)((time - rfx_c->hispeed_start_ns)
				    / HISPEED_HALFLIFE_NS);
	if (half_lives >= 10)
		return pelt_util;

	return min(pelt_util + ((hispeed_util - pelt_util) >> half_lives),
		   max_cap);
}

/************************ I/O wait boost ***********************/

static bool rfx_iowait_reset(struct rfx_cpu *rfx_c, u64 time,
			     bool set_iowait_boost)
{
	s64 delta_ns = time - rfx_c->last_update;
	if (delta_ns <= TICK_NSEC)
		return false;
	rfx_c->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	rfx_c->iowait_boost_pending = set_iowait_boost;
	return true;
}

static void rfx_iowait_boost(struct rfx_cpu *rfx_c, u64 time,
			     unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;
	if (rfx_c->iowait_boost && rfx_iowait_reset(rfx_c, time, set_iowait_boost))
		return;
	if (!set_iowait_boost)
		return;
	if (rfx_c->iowait_boost_pending)
		return;
	rfx_c->iowait_boost_pending = true;
	if (rfx_c->iowait_boost) {
		rfx_c->iowait_boost =
			min_t(unsigned int, rfx_c->iowait_boost << 1,
			      SCHED_CAPACITY_SCALE);
		return;
	}
	rfx_c->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long rfx_iowait_apply(struct rfx_cpu *rfx_c, u64 time,
				      unsigned long max_cap)
{
	if (!rfx_c->iowait_boost)
		return 0;
	if (rfx_iowait_reset(rfx_c, time, false))
		return 0;
	if (!rfx_c->iowait_boost_pending) {
		rfx_c->iowait_boost >>= 1;
		if (rfx_c->iowait_boost < IOWAIT_BOOST_MIN) {
			rfx_c->iowait_boost = 0;
			return 0;
		}
	}
	rfx_c->iowait_boost_pending = false;
	return (rfx_c->iowait_boost * max_cap) >> SCHED_CAPACITY_SHIFT;
}

/************************ Hold frequency ***********************/

#ifdef CONFIG_NO_HZ_COMMON
static bool rfx_hold_freq(struct rfx_cpu *rfx_c)
{
	unsigned long idle_calls;
	bool ret;

	if (rfx_scx_switched_all())
		return false;
	if (rfx_cpu_uclamp_capped(rfx_c->cpu))
		return false;

	idle_calls = tick_nohz_get_idle_calls_cpu(rfx_c->cpu);
	ret = idle_calls == rfx_c->saved_idle_calls;
	rfx_c->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool rfx_hold_freq(struct rfx_cpu *rfx_c) { return false; }
#endif

static inline void rfx_ignore_dl_rate_limit(struct rfx_cpu *rfx_c)
{
	/* [Fix4] use rfx_dl_bw_exceeded_gki510 */
	if (rfx_dl_bw_exceeded_gki510(rfx_c->cpu, rfx_c->bw_min))
		rfx_c->rfx_policy->need_freq_update = true;
}

/************************ Core update callbacks ***********************/

static void rfx_update_single_freq(struct update_util_data *hook, u64 time,
				   unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	struct rfx_tunables *tunables = rfx_pol->tunables;
	unsigned int cached_freq = rfx_pol->cached_raw_freq;
	unsigned long max_cap, boost, effective_util;
	unsigned int next_f;

	max_cap = arch_scale_cpu_capacity(NULL, rfx_c->cpu);
	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);

	if (!rfx_should_update_freq(rfx_pol, time))
		return;

	boost = rfx_iowait_apply(rfx_c, time, max_cap);
	rfx_get_util(rfx_c, boost);
	effective_util = max(rfx_c->util, boost);

	rfx_update_busy_pct(rfx_c, tunables->hispeed_window_us,
			    tunables->hispeed_filter_shift, time);
	effective_util = rfx_blend_util(rfx_c, effective_util, max_cap, time);

	next_f = rfx_get_next_freq(rfx_pol, effective_util, max_cap);

	if (rfx_hold_freq(rfx_c) && next_f < rfx_pol->next_freq &&
	    !rfx_pol->need_freq_update) {
		next_f = rfx_pol->next_freq;
		rfx_pol->cached_raw_freq = cached_freq;
	}

	if (!rfx_update_next_freq(rfx_pol, time, next_f))
		return;

	if (rfx_pol->policy->fast_switch_enabled)
		cpufreq_driver_fast_switch(rfx_pol->policy, next_f);
	else {
		raw_spin_lock(&rfx_pol->update_lock);
		rfx_deferred_update(rfx_pol);
		raw_spin_unlock(&rfx_pol->update_lock);
	}
}

/************************ Shared policy ***********************/

static unsigned int rfx_next_freq_shared(struct rfx_cpu *rfx_c, u64 time)
{
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	struct rfx_tunables *tunables = rfx_pol->tunables;
	struct cpufreq_policy *policy = rfx_pol->policy;
	unsigned long util = 0, max_cap;
	unsigned int j;

	max_cap = arch_scale_cpu_capacity(NULL, rfx_c->cpu);

	for_each_cpu(j, policy->cpus) {
		struct rfx_cpu *j_rfx_c = &per_cpu(rfx_cpu, j);
		unsigned long j_boost, j_util;

		j_boost = rfx_iowait_apply(j_rfx_c, time, max_cap);
		rfx_get_util(j_rfx_c, j_boost);
		j_util = max(j_rfx_c->util, j_boost);
		rfx_update_busy_pct(j_rfx_c, tunables->hispeed_window_us,
				    tunables->hispeed_filter_shift, time);
		j_util = rfx_blend_util(j_rfx_c, j_util, max_cap, time);
		util = max(j_util, util);
	}
	return rfx_get_next_freq(rfx_pol, util, max_cap);
}

static void rfx_update_shared(struct update_util_data *hook, u64 time,
			      unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	unsigned int next_f;

	raw_spin_lock(&rfx_pol->update_lock);
	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);

	if (rfx_should_update_freq(rfx_pol, time)) {
		next_f = rfx_next_freq_shared(rfx_c, time);
		if (!rfx_update_next_freq(rfx_pol, time, next_f))
			goto unlock;
		if (rfx_pol->policy->fast_switch_enabled)
			cpufreq_driver_fast_switch(rfx_pol->policy, next_f);
		else
			rfx_deferred_update(rfx_pol);
	}
unlock:
	raw_spin_unlock(&rfx_pol->update_lock);
}

/************************ Kthread ***********************/

static void rfx_work(struct kthread_work *work)
{
	struct rfx_policy *rfx_pol = container_of(work, struct rfx_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&rfx_pol->update_lock, flags);
	freq = rfx_pol->next_freq;
	rfx_pol->work_in_progress = false;
	raw_spin_unlock_irqrestore(&rfx_pol->update_lock, flags);

	mutex_lock(&rfx_pol->work_lock);
	__cpufreq_driver_target(rfx_pol->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&rfx_pol->work_lock);
}

static void rfx_irq_work(struct irq_work *irq_work)
{
	struct rfx_policy *rfx_pol =
		container_of(irq_work, struct rfx_policy, irq_work);
	kthread_queue_work(&rfx_pol->worker, &rfx_pol->work);
}

/************************** sysfs interface ************************/

static struct rfx_tunables *rfx_global_tunables;
static DEFINE_MUTEX(rfx_global_tunables_lock);

#define RFX_TUNABLE_UINT(name) \
static ssize_t name##_show(struct gov_attr_set *attr_set, char *buf) \
{ \
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->name); \
} \
static ssize_t \
name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{ \
	struct rfx_tunables *t = to_rfx_tunables(attr_set); \
	unsigned int val; \
	if (kstrtouint(buf, 10, &val)) \
		return -EINVAL; \
	t->name = val; \
	return count; \
} \
static struct governor_attr name = __ATTR_RW(name)

static ssize_t rfx_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->rate_limit_us);
}

static ssize_t rfx_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct rfx_tunables *tunables = to_rfx_tunables(attr_set);
	struct rfx_policy *rfx_pol;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;
	tunables->rate_limit_us = rate_limit_us;
	list_for_each_entry(rfx_pol, &attr_set->policy_list, tunables_hook)
		rfx_pol->freq_update_delay_ns = rate_limit_us * NSEC_PER_USEC;
	return count;
}

static struct governor_attr rfx_rate_limit_us =
	__ATTR(rate_limit_us, 0644, rfx_rate_limit_us_show, rfx_rate_limit_us_store);

static ssize_t version_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%s\n", CPUFREQ_REFLEX_VERSION);
}
static struct governor_attr version = __ATTR_RO(version);

RFX_TUNABLE_UINT(hispeed_window_us);
RFX_TUNABLE_UINT(hispeed_filter_shift);

static struct attribute *rfx_attrs[] = {
	&version.attr,
	&rfx_rate_limit_us.attr,
	&hispeed_window_us.attr,
	&hispeed_filter_shift.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx);

static void rfx_tunables_free(struct kobject *kobj)
{
	/* [Fix2] to_gov_attr_set → rfx_to_gov_attr_set (inline container_of) */
	kfree(to_rfx_tunables(rfx_to_gov_attr_set(kobj)));
}

/*
 * [Fix3] Non-const kobj_type — GKI 5.10 kobject_init_and_add() takes
 * struct kobj_type* (not const). Using const causes -Wincompatible-pointer-types.
 */
static struct kobj_type rfx_tunables_ktype = {
	.default_attrs = rfx_attrs,
	.sysfs_ops      = &governor_sysfs_ops,
	.release        = rfx_tunables_free,
};

static struct cpufreq_governor reflex_gov;

static struct rfx_policy *rfx_policy_alloc(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = kzalloc(sizeof(*rfx_pol), GFP_KERNEL);
	if (!rfx_pol)
		return NULL;
	rfx_pol->policy = policy;
	raw_spin_lock_init(&rfx_pol->update_lock);
	return rfx_pol;
}

static void rfx_policy_free(struct rfx_policy *rfx_pol) { kfree(rfx_pol); }

static int rfx_kthread_create(struct rfx_policy *rfx_pol)
{
	struct task_struct *thread;
	struct sched_attr attr = {
		.size           = sizeof(struct sched_attr),
		.sched_policy   = SCHED_DEADLINE,
		.sched_flags    = SCHED_FLAG_SUGOV,
		.sched_runtime  = NSEC_PER_MSEC,
		.sched_deadline = 10 * NSEC_PER_MSEC,
		.sched_period   = 10 * NSEC_PER_MSEC,
	};
	struct cpufreq_policy *policy = rfx_pol->policy;
	int ret;

	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&rfx_pol->work, rfx_work);
	kthread_init_worker(&rfx_pol->worker);
	thread = kthread_create(kthread_worker_fn, &rfx_pol->worker,
				"rfxgov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("reflex: kthread create failed: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}
	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_DEADLINE\n", __func__);
		return ret;
	}
	rfx_pol->thread = thread;
	if (policy->dvfs_possible_from_any_cpu)
		set_cpus_allowed_ptr(thread, policy->related_cpus);
	else
		kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&rfx_pol->irq_work, rfx_irq_work);
	mutex_init(&rfx_pol->work_lock);
	wake_up_process(thread);
	return 0;
}

static void rfx_kthread_stop(struct rfx_policy *rfx_pol)
{
	if (rfx_pol->policy->fast_switch_enabled)
		return;
	kthread_flush_worker(&rfx_pol->worker);
	kthread_stop(rfx_pol->thread);
	mutex_destroy(&rfx_pol->work_lock);
}

static struct rfx_tunables *rfx_tunables_alloc(struct rfx_policy *rfx_pol)
{
	struct rfx_tunables *tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &rfx_pol->tunables_hook);
		if (!have_governor_per_policy())
			rfx_global_tunables = tunables;
	}
	return tunables;
}

static void rfx_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		rfx_global_tunables = NULL;
}

static int rfx_init(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol;
	struct rfx_tunables *tunables;
	int ret = 0;

	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);
	rfx_pol = rfx_policy_alloc(policy);
	if (!rfx_pol) { ret = -ENOMEM; goto disable_fast_switch; }

	ret = rfx_kthread_create(rfx_pol);
	if (ret) goto free_rfx_pol;

	mutex_lock(&rfx_global_tunables_lock);

	if (rfx_global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL; goto stop_kthread;
		}
		policy->governor_data = rfx_pol;
		rfx_pol->tunables = rfx_global_tunables;
		gov_attr_set_get(&rfx_global_tunables->attr_set,
				 &rfx_pol->tunables_hook);
		goto out;
	}

	tunables = rfx_tunables_alloc(rfx_pol);
	if (!tunables) { ret = -ENOMEM; goto stop_kthread; }

	tunables->rate_limit_us        = cpufreq_policy_transition_delay_us(policy);
	tunables->hispeed_window_us    = CPUFREQ_REFLEX_DEFAULT_HISPEED_WINDOW_US;
	tunables->hispeed_filter_shift = CPUFREQ_REFLEX_DEFAULT_HISPEED_FILTER_SHIFT;

	policy->governor_data = rfx_pol;
	rfx_pol->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj,
				   &rfx_tunables_ktype,
				   get_governor_parent_kobj(policy),
				   "%s", reflex_gov.name);
	if (ret) goto fail;

out:
	mutex_unlock(&rfx_global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	rfx_clear_global_tunables();
stop_kthread:
	rfx_kthread_stop(rfx_pol);
	mutex_unlock(&rfx_global_tunables_lock);
free_rfx_pol:
	rfx_policy_free(rfx_pol);
disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	pr_err("reflex: init failed (error %d)\n", ret);
	return ret;
}

static void rfx_exit(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;
	struct rfx_tunables *tunables = rfx_pol->tunables;
	unsigned int count;

	mutex_lock(&rfx_global_tunables_lock);
	count = gov_attr_set_put(&tunables->attr_set, &rfx_pol->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		rfx_clear_global_tunables();
	mutex_unlock(&rfx_global_tunables_lock);

	rfx_kthread_stop(rfx_pol);
	rfx_policy_free(rfx_pol);
	cpufreq_disable_fast_switch(policy);
}

static int rfx_start(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;
	void (*uu)(struct update_util_data *data, u64 time, unsigned int flags);
	unsigned int cpu;

	rfx_pol->freq_update_delay_ns  = rfx_pol->tunables->rate_limit_us * NSEC_PER_USEC;
	rfx_pol->last_freq_update_time = 0;
	rfx_pol->next_freq             = 0;
	rfx_pol->work_in_progress      = false;
	rfx_pol->limits_changed        = false;
	rfx_pol->cached_raw_freq       = 0;
	rfx_pol->need_freq_update      = false;

	if (policy_is_shared(policy))
		uu = rfx_update_shared;
	else
		uu = rfx_update_single_freq;

	for_each_cpu(cpu, policy->cpus) {
		struct rfx_cpu *rfx_c = &per_cpu(rfx_cpu, cpu);
		memset(rfx_c, 0, sizeof(*rfx_c));
		rfx_c->cpu        = cpu;
		rfx_c->rfx_policy = rfx_pol;
		rfx_c->prev_idle_time = get_cpu_idle_time(cpu,
					&rfx_c->prev_wall_time, 1);
		cpufreq_add_update_util_hook(cpu, &rfx_c->update_util, uu);
	}
	return 0;
}

static void rfx_stop(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);
	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&rfx_pol->irq_work);
		kthread_cancel_work_sync(&rfx_pol->work);
	}
}

static void rfx_limits(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&rfx_pol->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&rfx_pol->work_lock);
	}
	smp_wmb();
	WRITE_ONCE(rfx_pol->limits_changed, true);
}

static struct cpufreq_governor reflex_gov = {
	.name   = "reflex",
	.owner  = THIS_MODULE,
	.flags  = 0,
	.init   = rfx_init,
	.exit   = rfx_exit,
	.start  = rfx_start,
	.stop   = rfx_stop,
	.limits = rfx_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_REFLEX
struct cpufreq_governor *cpufreq_default_governor(void) { return &reflex_gov; }
#endif

static int __init reflex_gov_init(void)
{
	pr_info("%s %s by %s\n", CPUFREQ_REFLEX_PROGNAME,
		CPUFREQ_REFLEX_VERSION, CPUFREQ_REFLEX_AUTHOR);
	return cpufreq_register_governor(&reflex_gov);
}

static void __exit reflex_gov_exit(void)
{
	cpufreq_unregister_governor(&reflex_gov);
}

module_init(reflex_gov_init);
module_exit(reflex_gov_exit);
MODULE_AUTHOR(CPUFREQ_REFLEX_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(CPUFREQ_REFLEX_PROGNAME);
