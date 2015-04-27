/*
 * TPA2028D1 Audio Amplifier Driver
 *
 * Copyright (C) 2014  Rudolf Tammekivi <rtammekivi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __TPA2028D1_H__
#define __TPA2028D1_H__

#ifdef __KERNEL__
/* struct tpa2028d1_config
 * All of the members must be initialized according to the datasheet.
 *
 * @ng_enabled		- noise gate function
 * @atk_time		- the minimum time between two gain decrements
 * @rel_time		- the minimum time between two gain increments
 * @hold_time		- the time it takes for the very first gain increment
 			  after the input signal amplitude decreases
 * @fixed_gain		- the normal gain of the device when the AGC is inactive
 * @out_lim_disabled	- disables output limiter
 * @ng_threshold	- below this value, the AGC holds the gain to prevent
 			  breathing effects
 * @out_lim_level	- the value that sets the maximum allowed output
 			  amplitude
 * @max_gain		- the gain at the lower end of the compression region
 * @comp_ratio		- the relation between input and output voltage
 */
struct tpa2028d1_config {
	bool ng_enabled;
	int atk_time;
	int rel_time;
	int hold_time;
	int fixed_gain;
	bool out_lim_disabled;
	int ng_threshold;
	int out_lim_level;
	int max_gain;
	int comp_ratio;
};

struct tpa2028d1_callbacks {
	int (*enable)(struct tpa2028d1_callbacks *cb, bool enable);
};

/* struct tpa2028d1_platform_data
 * @amp_en_gpio - amplifier enable GPIO (EN). -1 if not enabled
 * @config	- tpa2028d1 config
 */
struct tpa2028d1_platform_data {
	int amp_en_gpio;
	const struct tpa2028d1_config config;

	void (*register_callbacks)(struct tpa2028d1_callbacks *cb);
	void (*unregister_callbacks)(void);
};
#endif

#endif /* __TPA2028D1_H__ */
