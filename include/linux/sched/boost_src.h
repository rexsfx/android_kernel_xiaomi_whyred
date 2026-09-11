#ifndef _LINUX_SCHED_BOOST_SRC_H
#define _LINUX_SCHED_BOOST_SRC_H

#include <linux/compiler.h>

enum sched_boost_src {
	SCHED_BOOST_STUNE  = 1,
	SCHED_BOOST_UCLAMP = 2,
	SCHED_BOOST_HYBRID = 3,
};

extern int sysctl_sched_boost_src;
extern int sysctl_sched_boost_src_min;
extern int sysctl_sched_boost_src_max;

static inline int sched_boost_src(void)
{
	int m = READ_ONCE(sysctl_sched_boost_src);

	if (m < SCHED_BOOST_STUNE || m > SCHED_BOOST_HYBRID)
		return SCHED_BOOST_HYBRID;
	return m;
}

static inline bool sched_boost_stune(void)
{
	return sched_boost_src() != SCHED_BOOST_UCLAMP;
}

static inline bool sched_boost_uclamp(void)
{
	return sched_boost_src() != SCHED_BOOST_STUNE;
}

#endif /* _LINUX_SCHED_BOOST_SRC_H */
