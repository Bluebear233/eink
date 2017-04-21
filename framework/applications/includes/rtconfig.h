/*
 * rtthread.h
 *
 *  Created on: 2017年1月1日
 *      Author: peng
 */

#ifndef __RTTHREAD_H__
#define __RTTHREAD_H__

#define RT_DEBUG
#define RT_DEBUG
#define RT_THREAD_DEBUG

#define RT_USING_HEAP
#define RT_USING_SMALL_MEM

#define RT_USING_HOOK

// 使用rt-thread自带的组件初始化
#define RT_USING_COMPONENTS_INIT
#ifdef RT_USING_COMPONENTS_INIT
// 使用rt-thread自带的主函数
#define RT_USING_USER_MAIN
#endif

#define RT_NAME_MAX                 8
#define RT_TICK_PER_SECOND          100
#define RT_THREAD_PRIORITY_MAX      32
#define RT_ALIGN_SIZE               4

#define RT_USING_EVENT
#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_MAILBOX

// 使用rt-thread设备驱动
#define RT_USING_DEVICE

// 使用rt-thread串口驱动
#define RT_USING_SERIAL
#define RT_USING_CONSOLE
#define RT_CONSOLE_DEVICE_NAME      "UART1"
// 配置终端缓冲区大小
#define RT_CONSOLEBUF_SIZE          128

// 使用SPI驱动框架
#define RT_USING_SPI

// 使用I2C驱动框架
#define RT_USING_I2C
#define RT_USING_I2C_BITOPS

#define RT_USING_PIN

/* 使用finsh */
#define RT_USING_FINSH

/* 使用制表符 */
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION

#define FINSH_USING_MSH
#define FINSH_USING_MSH_ONLY

extern struct rt_thread *rt_current_thread;

#endif /* RTTHREAD_H_ */
