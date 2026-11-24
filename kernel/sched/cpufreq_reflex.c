// SPDX-License-Identifier: GPL-2.0
/*
 * Reflex CPUFreq governor, backported to Linux 4.19
 * Copyright (C) 2026 Masahito Suzuki
 */

#define pr_fmt(fmt) "reflex: " fmt

#include <linux/cpufreq.h>
#include <linux/energy_model.h>
#include <linux/irq_work.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/sched/boost_src.h>
#include <linux/sched/cpufreq.h>
#include <linux/slab.h>

#include "sched.h"

#define CPUFREQ_REFLEX_VERSION	"0.4.0"

#define IOWAIT_BOOST_MIN	(SCHED_CAPACITY_SCALE / 8)

#define RFX_FLOOR_N		65
#define RFX_LATENCY_WEIGHT_DEF	1000
#define RFX_LATENCY_WEIGHT_LIM	100000

extern u64 cpufreq_pelt_decay(u64 val, u64 delta_ns);

struct rfx_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		rate_limit_us;
	unsigned int		latency_weight;
};

struct rfx_policy {
	struct cpufreq_policy	*policy;

	struct rfx_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;
	s64			freq_update_delay_ns;
	s64			sample_delay_ns;
	unsigned int		alpha_us;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;

	unsigned int		floor[RFX_FLOOR_N];
#ifdef CONFIG_ENERGY_MODEL
	struct em_perf_domain	*em_pd;
#endif

	struct irq_work		irq_work;
	struct kthread_work	work;
	struct mutex		work_lock;
	struct kthread_worker	worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;
};

struct rfx_cpu {
	struct update_util_data	update_util;
	struct rfx_policy	*rfx_policy;
	unsigned int		cpu;

	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;

	unsigned long		bw_dl;

	u64			last_sample;
	u64			last_wall;
	u64			last_idle;
	unsigned long		H;
	unsigned int		N;
	bool			censored;
};

static DEFINE_PER_CPU(struct rfx_cpu, rfx_cpu);

static void rfx_update_rate_limit_us(struct rfx_policy *rfx_pol)
{
	rfx_pol->freq_update_delay_ns =
		(s64)rfx_pol->tunables->rate_limit_us * NSEC_PER_USEC;
	rfx_pol->sample_delay_ns = max_t(s64, rfx_pol->freq_update_delay_ns,
					 TICK_NSEC);
	rfx_pol->alpha_us = div_u64(rfx_pol->sample_delay_ns, NSEC_PER_USEC);
}

static inline unsigned int rfx_ref_freq(struct cpufreq_policy *policy)
{
	return arch_scale_freq_invariant() ? policy->cpuinfo.max_freq :
					     policy->cur;
}

static bool rfx_should_update_freq(struct rfx_policy *rfx_pol, u64 time)
{
	s64 delta_ns;

	if (!cpufreq_this_cpu_can_update(rfx_pol->policy))
		return false;

	if (rfx_pol->work_in_progress)
		return false;

	if (unlikely(READ_ONCE(rfx_pol->limits_changed))) {
		WRITE_ONCE(rfx_pol->limits_changed, false);
		rfx_pol->need_freq_update = true;
		smp_mb();
		return true;
	}

	delta_ns = time - rfx_pol->last_freq_update_time;

	return delta_ns >= rfx_pol->freq_update_delay_ns;
}

static bool rfx_update_next_freq(struct rfx_policy *rfx_pol, u64 time,
				 unsigned int next_freq)
{
	if (rfx_pol->next_freq == next_freq)
		return false;

	rfx_pol->next_freq = next_freq;
	rfx_pol->last_freq_update_time = time;

	return true;
}

static void rfx_fast_switch(struct rfx_policy *rfx_pol, u64 time,
			    unsigned int next_freq)
{
	struct cpufreq_policy *policy = rfx_pol->policy;

	if (!rfx_update_next_freq(rfx_pol, time, next_freq))
		return;

	next_freq = cpufreq_driver_fast_switch(policy, next_freq);
	if (!next_freq)
		return;

	policy->cur = next_freq;
}

static void rfx_deferred_update(struct rfx_policy *rfx_pol, u64 time,
				unsigned int next_freq)
{
	if (!rfx_update_next_freq(rfx_pol, time, next_freq))
		return;

	if (rfx_pol->work_in_progress)
		return;

	rfx_pol->work_in_progress = true;
	irq_work_queue(&rfx_pol->irq_work);
}

static unsigned int rfx_get_next_freq(struct rfx_policy *rfx_pol,
				      unsigned long util, unsigned long max,
				      unsigned int floor)
{
	struct cpufreq_policy *policy = rfx_pol->policy;
	unsigned int freq;

	freq = map_util_freq(util, rfx_ref_freq(policy), max);
	freq = max(freq, floor);

	if (freq == rfx_pol->cached_raw_freq && !rfx_pol->need_freq_update)
		return rfx_pol->next_freq;

	rfx_pol->need_freq_update = false;
	rfx_pol->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(policy, freq);
}

static unsigned long rfx_cpu_util(struct rfx_cpu *rfx_c, unsigned long max_cap)
{
	struct rq *rq = cpu_rq(rfx_c->cpu);

	rfx_c->bw_dl = cpu_bw_dl(rq);

	return schedutil_cpu_util(rfx_c->cpu, cpu_util_cfs(rq), max_cap,
				  FREQUENCY_UTIL, NULL);
}

static unsigned long rfx_cap(struct rfx_cpu *rfx_c, unsigned long max_cap)
{
#ifdef CONFIG_UCLAMP_TASK
	if (sched_boost_uclamp() && uclamp_is_used())
		return min_t(unsigned long, max_cap,
			     READ_ONCE(cpu_rq(rfx_c->cpu)->uclamp[UCLAMP_MAX].value));
#endif
	return max_cap;
}

static u64 rfx_icbrt(u64 n)
{
	u64 lo = 0, hi = 1ULL << 21;

	while (lo < hi) {
		u64 mid = (lo + hi + 1) >> 1;

		if (mid * mid * mid <= n)
			lo = mid;
		else
			hi = mid - 1;
	}
	return lo;
}

#ifdef CONFIG_ENERGY_MODEL
static bool rfx_em_floors(struct rfx_policy *rfx_pol, unsigned int weight)
{
	struct em_perf_domain *pd = rfx_pol->em_pd;
	struct em_cap_state *table, *top;
	u64 lmax = 0;
	int i, k, nr;

	if (!pd || pd->nr_cap_states < 1)
		return false;

	table = pd->table;
	nr = pd->nr_cap_states;
	top = &table[nr - 1];

	for (i = 0; i < nr - 1; i++) {
		u64 a = (u64)top->power * table[i].frequency;
		u64 b = (u64)table[i].power * top->frequency;

		if (table[i].frequency < top->frequency && a > b)
			lmax = max(lmax, DIV64_U64_ROUND_UP(a - b,
					top->frequency - table[i].frequency));
	}

	for (k = 0; k < RFX_FLOOR_N; k++) {
		u64 l = lmax * weight * k, best = U64_MAX;
		unsigned int freq = top->frequency;

		for (i = 0; i < nr; i++) {
			u64 score;

			if (!table[i].frequency)
				continue;
			score = div64_u64((u64)table[i].power * 1000 *
					  (RFX_FLOOR_N - 1) + l,
					  table[i].frequency);
			if (score <= best) {
				best = score;
				freq = table[i].frequency;
			}
		}
		WRITE_ONCE(rfx_pol->floor[k], freq);
	}

	return true;
}
#else
static inline bool rfx_em_floors(struct rfx_policy *rfx_pol,
				 unsigned int weight)
{
	return false;
}
#endif

static void rfx_cube_floors(struct rfx_policy *rfx_pol, unsigned int weight)
{
	unsigned int fmax = rfx_pol->policy->cpuinfo.max_freq;
	u64 den = 1000 * (RFX_FLOOR_N - 1);
	int k;

	for (k = 0; k < RFX_FLOOR_N; k++) {
		u64 num = (u64)weight * k;
		unsigned int freq = fmax;

		if (num < den)
			freq = ((u64)fmax *
				rfx_icbrt(div64_u64(num << 48, den))) >> 16;
		WRITE_ONCE(rfx_pol->floor[k], freq);
	}
}

static void rfx_speed_floors(struct rfx_policy *rfx_pol)
{
	unsigned int weight = READ_ONCE(rfx_pol->tunables->latency_weight);

	if (!rfx_em_floors(rfx_pol, weight))
		rfx_cube_floors(rfx_pol, weight);
}

static void rfx_sample(struct rfx_cpu *rfx_c, u64 time)
{
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	u64 idle, wall, dw, di;

	if (rfx_c->cpu != smp_processor_id() ||
	    (s64)(time - rfx_c->last_sample) < rfx_pol->sample_delay_ns)
		return;

	idle = get_cpu_idle_time(rfx_c->cpu, &wall, 0);
	dw = wall - rfx_c->last_wall;
	if (!dw)
		return;
	di = idle > rfx_c->last_idle ? min(idle - rfx_c->last_idle, dw) : 0;

	rfx_c->H = max_t(unsigned long,
			 div64_u64((dw - di) *
				   arch_scale_freq_capacity(rfx_c->cpu) *
				   arch_scale_cpu_capacity(NULL, rfx_c->cpu),
				   dw << SCHED_CAPACITY_SHIFT),
			 cpufreq_pelt_decay(rfx_c->H, dw * NSEC_PER_USEC));

	rfx_c->N = cpufreq_pelt_decay(rfx_c->N, dw * NSEC_PER_USEC) +
		   cpufreq_pelt_decay(dw - di, di * NSEC_PER_USEC);
	rfx_c->censored = !di;

	rfx_c->last_sample = time;
	rfx_c->last_wall = wall;
	rfx_c->last_idle = idle;
}

static inline bool rfx_active(struct rfx_cpu *rfx_c)
{
	return rfx_c->cpu == smp_processor_id() || !idle_cpu(rfx_c->cpu);
}

static unsigned long rfx_util(struct rfx_cpu *rfx_c, unsigned long max_cap,
			      unsigned long boost)
{
	unsigned long util = max(rfx_cpu_util(rfx_c, max_cap), boost);

	if (rfx_active(rfx_c))
		util = max(util, rfx_c->H);

	return min(util, rfx_cap(rfx_c, max_cap));
}

static unsigned int rfx_floor(struct rfx_cpu *rfx_c, unsigned long max_cap)
{
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	unsigned int floor, k = RFX_FLOOR_N - 1;
	unsigned long cap;

	if (!rfx_active(rfx_c))
		return 0;

	if (!rfx_c->censored) {
		unsigned int alpha = rfx_pol->alpha_us, den = rfx_c->N + alpha;

		k = (k * alpha + den / 2) / den;
	}
	floor = READ_ONCE(rfx_pol->floor[k]);

	cap = rfx_cap(rfx_c, max_cap);
	if (cap < max_cap)
		floor = min_t(unsigned int, floor,
			      div_u64((u64)rfx_ref_freq(rfx_pol->policy) * cap,
				      max_cap));
	return floor;
}

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

	if (rfx_c->iowait_boost &&
	    rfx_iowait_reset(rfx_c, time, set_iowait_boost))
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

static inline void rfx_ignore_dl_rate_limit(struct rfx_cpu *rfx_c)
{
	if (cpu_bw_dl(cpu_rq(rfx_c->cpu)) > rfx_c->bw_dl)
		WRITE_ONCE(rfx_c->rfx_policy->limits_changed, true);
}

static void rfx_update_single(struct update_util_data *hook, u64 time,
			      unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	unsigned long max_cap, boost, util;
	unsigned int next_f;

	if (flags & SCHED_CPUFREQ_PL)
		return;

	max_cap = arch_scale_cpu_capacity(NULL, rfx_c->cpu);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_sample(rfx_c, time);

	rfx_ignore_dl_rate_limit(rfx_c);

	if (!rfx_should_update_freq(rfx_pol, time))
		return;

	boost = rfx_iowait_apply(rfx_c, time, max_cap);
	util = rfx_util(rfx_c, max_cap, boost);
	next_f = rfx_get_next_freq(rfx_pol, util, max_cap,
				   rfx_floor(rfx_c, max_cap));

	if (rfx_pol->policy->fast_switch_enabled) {
		rfx_fast_switch(rfx_pol, time, next_f);
	} else {
		raw_spin_lock(&rfx_pol->update_lock);
		rfx_deferred_update(rfx_pol, time, next_f);
		raw_spin_unlock(&rfx_pol->update_lock);
	}
}

static unsigned int rfx_next_freq_shared(struct rfx_cpu *rfx_c, u64 time)
{
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	struct cpufreq_policy *policy = rfx_pol->policy;
	unsigned long util = 0, max_cap;
	unsigned int floor = 0, j;

	max_cap = arch_scale_cpu_capacity(NULL, rfx_c->cpu);

	for_each_cpu(j, policy->cpus) {
		struct rfx_cpu *j_rfx_c = &per_cpu(rfx_cpu, j);
		unsigned long j_boost;

		j_boost = rfx_iowait_apply(j_rfx_c, time, max_cap);
		util = max(util, rfx_util(j_rfx_c, max_cap, j_boost));
		floor = max(floor, rfx_floor(j_rfx_c, max_cap));
	}

	return rfx_get_next_freq(rfx_pol, util, max_cap, floor);
}

static void rfx_update_shared(struct update_util_data *hook, u64 time,
			      unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	unsigned int next_f;

	if (flags & SCHED_CPUFREQ_PL)
		return;

	raw_spin_lock(&rfx_pol->update_lock);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_sample(rfx_c, time);

	rfx_ignore_dl_rate_limit(rfx_c);

	if (rfx_should_update_freq(rfx_pol, time) &&
	    !(flags & SCHED_CPUFREQ_CONTINUE)) {
		next_f = rfx_next_freq_shared(rfx_c, time);

		if (rfx_pol->policy->fast_switch_enabled)
			rfx_fast_switch(rfx_pol, time, next_f);
		else
			rfx_deferred_update(rfx_pol, time, next_f);
	}

	raw_spin_unlock(&rfx_pol->update_lock);
}

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
	struct rfx_policy *rfx_pol;

	rfx_pol = container_of(irq_work, struct rfx_policy, irq_work);

	kthread_queue_work(&rfx_pol->worker, &rfx_pol->work);
}

static struct rfx_tunables *rfx_global_tunables;
static DEFINE_MUTEX(rfx_global_tunables_lock);

static inline struct rfx_tunables *to_rfx_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct rfx_tunables, attr_set);
}

static ssize_t rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct rfx_tunables *tunables = to_rfx_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->rate_limit_us);
}

static ssize_t rate_limit_us_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct rfx_tunables *tunables = to_rfx_tunables(attr_set);
	struct rfx_policy *rfx_pol;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->rate_limit_us = rate_limit_us;

	list_for_each_entry(rfx_pol, &attr_set->policy_list, tunables_hook)
		rfx_update_rate_limit_us(rfx_pol);

	return count;
}

static ssize_t latency_weight_show(struct gov_attr_set *attr_set, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_rfx_tunables(attr_set)->latency_weight);
}

static ssize_t latency_weight_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct rfx_tunables *tunables = to_rfx_tunables(attr_set);
	struct rfx_policy *rfx_pol;
	unsigned int weight;

	if (kstrtouint(buf, 10, &weight) || weight > RFX_LATENCY_WEIGHT_LIM)
		return -EINVAL;

	WRITE_ONCE(tunables->latency_weight, weight);

	list_for_each_entry(rfx_pol, &attr_set->policy_list, tunables_hook)
		rfx_speed_floors(rfx_pol);

	return count;
}

static ssize_t version_show(struct gov_attr_set *attr_set, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", CPUFREQ_REFLEX_VERSION);
}

static struct governor_attr rfx_rate_limit_us = __ATTR_RW(rate_limit_us);
static struct governor_attr rfx_latency_weight = __ATTR_RW(latency_weight);
static struct governor_attr rfx_version = __ATTR_RO(version);

static struct attribute *rfx_attributes[] = {
	&rfx_version.attr,
	&rfx_rate_limit_us.attr,
	&rfx_latency_weight.attr,
	NULL
};

static void rfx_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = container_of(kobj, struct gov_attr_set, kobj);

	kfree(to_rfx_tunables(attr_set));
}

static struct kobj_type rfx_tunables_ktype = {
	.default_attrs = rfx_attributes,
	.sysfs_ops = &governor_sysfs_ops,
	.release = &rfx_tunables_free,
};

static struct cpufreq_governor reflex_gov;

static struct rfx_policy *rfx_policy_alloc(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol;

	rfx_pol = kzalloc(sizeof(*rfx_pol), GFP_KERNEL);
	if (!rfx_pol)
		return NULL;

	rfx_pol->policy = policy;
	raw_spin_lock_init(&rfx_pol->update_lock);
	return rfx_pol;
}

static void rfx_policy_free(struct rfx_policy *rfx_pol)
{
	kfree(rfx_pol);
}

static int rfx_kthread_create(struct rfx_policy *rfx_pol)
{
	struct task_struct *thread;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
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
		pr_err("failed to create kthread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	rfx_pol->thread = thread;
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
	struct rfx_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
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
	if (!rfx_pol) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

#ifdef CONFIG_ENERGY_MODEL
	rfx_pol->em_pd = em_cpu_get(policy->cpu);
#endif

	ret = rfx_kthread_create(rfx_pol);
	if (ret)
		goto free_rfx_pol;

	mutex_lock(&rfx_global_tunables_lock);

	if (rfx_global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = rfx_pol;
		rfx_pol->tunables = rfx_global_tunables;

		gov_attr_set_get(&rfx_global_tunables->attr_set,
				 &rfx_pol->tunables_hook);
		goto out;
	}

	tunables = rfx_tunables_alloc(rfx_pol);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	tunables->rate_limit_us = cpufreq_policy_transition_delay_us(policy);
	tunables->latency_weight = RFX_LATENCY_WEIGHT_DEF;

	policy->governor_data = rfx_pol;
	rfx_pol->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj,
				   &rfx_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   reflex_gov.name);
	if (ret)
		goto fail;

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

	pr_err("initialization failed (error %d)\n", ret);
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
	unsigned int cpu;

	rfx_update_rate_limit_us(rfx_pol);
	rfx_speed_floors(rfx_pol);
	rfx_pol->last_freq_update_time	= 0;
	rfx_pol->next_freq		= 0;
	rfx_pol->work_in_progress	= false;
	rfx_pol->limits_changed		= false;
	rfx_pol->need_freq_update	= false;
	rfx_pol->cached_raw_freq	= 0;

	for_each_cpu(cpu, policy->cpus) {
		struct rfx_cpu *rfx_c = &per_cpu(rfx_cpu, cpu);

		memset(rfx_c, 0, sizeof(*rfx_c));
		rfx_c->cpu = cpu;
		rfx_c->rfx_policy = rfx_pol;
		rfx_c->last_idle = get_cpu_idle_time(cpu, &rfx_c->last_wall, 0);
	}

	for_each_cpu(cpu, policy->cpus) {
		struct rfx_cpu *rfx_c = &per_cpu(rfx_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &rfx_c->update_util,
					     policy_is_shared(policy) ?
						rfx_update_shared :
						rfx_update_single);
	}

	return 0;
}

static void rfx_stop(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_sched();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&rfx_pol->irq_work);
		kthread_cancel_work_sync(&rfx_pol->work);
	}
}

static void rfx_limits(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;
	unsigned long flags, now;
	unsigned int freq;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&rfx_pol->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&rfx_pol->work_lock);
	} else {
		raw_spin_lock_irqsave(&rfx_pol->update_lock, flags);
		freq = policy->cur;
		now = ktime_get_ns();

		freq = cpufreq_driver_resolve_freq(policy, freq);
		rfx_pol->cached_raw_freq = freq;
		rfx_fast_switch(rfx_pol, now, freq);
		raw_spin_unlock_irqrestore(&rfx_pol->update_lock, flags);
	}

	rfx_speed_floors(rfx_pol);

	smp_wmb();

	WRITE_ONCE(rfx_pol->limits_changed, true);
}

static struct cpufreq_governor reflex_gov = {
	.name			= "reflex",
	.owner			= THIS_MODULE,
	.dynamic_switching	= true,
	.init			= rfx_init,
	.exit			= rfx_exit,
	.start			= rfx_start,
	.stop			= rfx_stop,
	.limits			= rfx_limits,
};

cpufreq_governor_init(reflex_gov);
