/*
 * board.h
 *
 *  Created on: 2017年1月1日
 *      Author: peng
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f10x.h"

#include "../../device_drivers/includes/rtdevice.h"

#define RT_USING_UART1

void rt_hw_board_init(void);
void rt_hw_us_delay(rt_uint32_t us);

#endif /* BOARD_H_ */
