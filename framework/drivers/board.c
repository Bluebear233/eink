/*
 * board.c
 *
 *  Created on: 2017年2月9日
 *      Author: rd5p
 */

#include "board.h"

#include "rtthread.h"
#include "stm32_uart.h"

void rt_hw_board_init(void)
{
    /* Configure the system clock @ 72 Mhz */
    SystemInit();

//    rt_hw_rtc_init();

    /* Set NVIC PriorityGroup 4 */
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);

    /* Configure the systemtick have interrupt in 10ms time basis */
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    rt_hw_uart_init();

#ifdef RT_USING_HEAP
    extern int __bss_end;
    extern int _estack;
    rt_system_heap_init((void *)&__bss_end, (void *)&_estack);
#endif

#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
      stm32_hw_pin_init();

//    rt_hw_spi_init();

//    rt_hw_i2c_bus_init();

//    rt_hw_i2c_software_bus_init();

//    rt_hw_adc_init();

//    rt_hw_ntc_init();

//    rt_hw_w25q64_init();

//    rt_hw_sht20_init();

//    rt_hw_md85rc04v_init();

//    rt_hw_ir_init();

//    rt_hw_ir_ctrl_init();

//    rt_hw_key_init();

//    rt_hw_di_init();

//    rt_hw_do_init();

//    rt_hw_rn7302_init();

//    rt_hw_buzzer_init();
}

void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * @brief 软件延迟
 * @param us : 毫秒
 */
void rt_hw_us_delay(rt_uint32_t us)
{
	uint32_t current_delay;
	uint32_t delta;
	uint32_t temp;

	current_delay = 0;
	delta = 0;
	temp = 0;

	// 2MHZ的周期为0.5us,那么变相计数*2
	us *= 72;

	/* 获得当前时间 */
	delta = SysTick->VAL;

	do
	{
		temp = SysTick->VAL;

		if (delta > temp)
		{
			current_delay += delta - temp;
		}
		else
		{
			current_delay += SysTick->LOAD - temp + delta;
		}

		delta = temp;
	} while (current_delay < us);
}

