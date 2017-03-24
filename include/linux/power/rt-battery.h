/*
 *  include/linux/power/rt-battery.h
 *  Include header file for Richtek Battery Driver
 *
 *  Copyright (C) 2015 Richtek Technology Corp.
 *  Jeff Chang <jeff_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef __LINUX_RT_BATTERY_H
#define __LINUX_RT_BATTERY_H

#include <linux/power_supply.h>

#define RT_BATT_NAME	"battery"

static inline struct power_supply *get_power_supply_by_name(char *name)
{
	if (!name)
		return (struct power_supply *)NULL;
	return power_supply_get_by_name(name);
}

#define psy_do_property(name, func, property, value) \
{	\
	struct power_supply *psy;	\
	int ret;	\
	psy = get_power_supply_by_name((name));	\
	if (!psy) {	\
		pr_err("%s: Fail to "#func" psy (%s)\n",	\
			__func__, (name));	\
		value.intval = 0;	\
	} else {	\
		if (psy->func##_property != NULL) { \
			ret = psy->func##_property(psy,	\
						(property), &(value)); \
			if (ret < 0) {	\
				pr_err("Fail to "#name" "#func" (%d=>%d)\n", \
					(property), ret);	\
				value.intval = 0;	\
			}	\
		}	\
	}	\
}

const char *rt_battery_status_txt[] = {
	"Unknown", "Charging", "DisCharging", "Not Charging", "Full",
};

#endif /* #ifndef __LINUX_RT_BATTERY_H */

