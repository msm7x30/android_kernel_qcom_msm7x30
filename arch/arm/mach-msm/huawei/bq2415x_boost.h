/*
 * bq2415x charger driver
 *
 * Copyright (C) 2011-2012  Pali Rohár <pali.rohar@gmail.com>
 * Copyright (C) 2015  Rudolf Tammekivi <rtammekivi@gmail.com>
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

#ifndef __BQ2415X_BOOST_H__
#define __BQ2415X_BOOST_H__

enum bq2415x_mode {
	BQ2415X_MODE_BOOST_OFF,
	BQ2415X_MODE_BOOST_ON,
};

#ifdef CONFIG_BQ2415X_BOOST
int bq2415x_set_mode(enum bq2415x_mode mode);
#else
static inline int bq2415x_set_mode(enum bq2415x_mode mode)
{
	return -ENOENT;
}
#endif
#endif /* __BQ2415X_BOOST_H__ */
