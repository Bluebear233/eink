/*
 * File      : rtc.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-10-10     aozima       first version.
 */

#ifndef RTC_H_INCLUDED
#define RTC_H_INCLUDED

#include "../../../kernel/includes/rtthread.h"

/**
 * RTC get power time.
 */
rt_uint32_t rtc_time(void);

void rtc_wakeup_counter_start(rt_uint16_t cnt);
void rtc_wakeup_init(void);

/**
 * RTC Wake-Up counter stop.
 */
void rtc_wakeup_counter_stop(void);

#endif // RTC_H_INCLUDED
