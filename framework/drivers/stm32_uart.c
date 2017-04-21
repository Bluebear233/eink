/**
 * file    uart.c
 * STM32 UART驱动，支持UART1(包括映射), UART2, UART3(包括映射), UART4
 *
 * Change Logs
 * Data           Author          Notes
 * 2017-2-25      luzhipeng       第一版
 * 2017-2-27      luzhipeng       增加UART2和UART3支持
 * 2017-3-16      luzhipeng       修正中断函数名称
 */

#include <stm32_uart.h>
#include "stm32f10x.h"
#include "rtdevice.h"

/* Private Define ---------------------------------------------------------------*/
#define USE_STM32_UART1               //Tx:PA9  Rx:PA10
//#define USE_STM32_UART1_REMAP         //Tx:PB6  Rx:PB7
//#define USE_STM32_UART2               //Tx:PA2  Rx:PA3
//#define USE_STM32_UART3               //Tx:PB10 Rx:PB11
//#define USE_STM32_UART3_FUll_REMAP    //Tx:PD8  Rx:PD9
//#define USE_STM32_UART4               //Tx:PC10 Rx:PC11

/* Private Typedef --------------------------------------------------------------*/
typedef struct
{
	rt_serial_t dev;
	USART_TypeDef* usart_base;
} rt_stm32_uart_t;

/* Private functions ------------------------------------------------------------*/
static rt_err_t stm32_uart_gpio_configure(struct rt_serial_device *serial);
static rt_err_t stm32_uart_nvic_configure(struct rt_serial_device *serial);
static rt_err_t stm32_uart_configure(struct rt_serial_device *serial,
		struct serial_configure *cfg);
static rt_err_t stm32_uart_control(struct rt_serial_device *serial, int cmd,
		void *arg);
static int stm32_uart_send(struct rt_serial_device *serial, char c);
static int stm32_uart_receive(struct rt_serial_device *serial);
static void stm32_uart_isr(struct rt_serial_device *serial);

/* Private Variables ------------------------------------------------------------*/
static const struct rt_uart_ops stm32_uart_ops =
{ stm32_uart_configure, stm32_uart_control, stm32_uart_send, stm32_uart_receive,
RT_NULL };
static const struct serial_configure stm32_uart_default_config =
		RT_SERIAL_CONFIG_DEFAULT;

#ifdef USE_STM32_UART1
static rt_stm32_uart_t uart1;
#endif

#ifdef USE_STM32_UART2
static rt_stm32_uart_t uart2;
#endif

#ifdef USE_STM32_UART3
static rt_stm32_uart_t uart3;
#endif

#ifdef USE_STM32_UART4
static rt_stm32_uart_t uart4;
#endif

/**
 * 配置端口
 */
static rt_err_t stm32_uart_gpio_configure(struct rt_serial_device *serial)
{
	USART_TypeDef* uart = ((rt_stm32_uart_t*) serial)->usart_base;

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_TypeDef* GPIOx;
	rt_uint32_t RCC_APB2Periph_GPIOx;
	rt_uint32_t GPIO_Pin_Tx;
	rt_uint32_t GPIO_Pin_Rx;
	rt_uint32_t GPIO_Remap;

	switch ((rt_uint32_t) uart)
	{
#ifdef USE_STM32_UART1
	case (rt_uint32_t) USART1:
#ifndef USE_STM32_UART1_REMAP
		RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOA;
		GPIOx = GPIOA;
		GPIO_Pin_Tx = GPIO_Pin_9;
		GPIO_Pin_Rx = GPIO_Pin_10;
		GPIO_Remap = RT_NULL;
#else
		RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOB;
		GPIOx = GPIOB;
		GPIO_Pin_Tx = GPIO_Pin_6;
		GPIO_Pin_Rx = GPIO_Pin_7;
		GPIO_Remap = GPIO_Remap_USART1;
#endif
		break;
#endif

#ifdef USE_STM32_UART2
	case (rt_uint32_t) USART2:
		RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOD;
		GPIOx = GPIOD;
		GPIO_Pin_Tx = GPIO_Pin_5;
		GPIO_Pin_Rx = GPIO_Pin_6;
		GPIO_Remap = RT_NULL;
		break;
#endif

#ifdef USE_STM32_UART3
	case (rt_uint32_t) USART3:
#ifndef USE_STM32_UART3_FUll_REMAP
		RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOB;
		GPIOx = GPIOB;
		GPIO_Pin_Tx = GPIO_Pin_10;
		GPIO_Pin_Rx = GPIO_Pin_11;
		GPIO_Remap = RT_NULL;
#else
		RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOD;
		GPIOx = GPIOD;
		GPIO_Pin_Tx = GPIO_Pin_8;
		GPIO_Pin_Rx = GPIO_Pin_9;
		GPIO_Remap = GPIO_FullRemap_USART3;
#endif
		break;
#endif

#ifdef USE_STM32_UART4
	case (rt_uint32_t) UART4:
		RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOC;
		GPIOx = GPIOC;
		GPIO_Pin_Tx = GPIO_Pin_10;
		GPIO_Pin_Rx = GPIO_Pin_11;
		GPIO_Remap = RT_NULL;
		break;
#endif

	default:
		RT_ASSERT(0)
		;
	}

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* Configure UART Tx&Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Tx;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Rx;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

	if (GPIO_Remap != RT_NULL)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

		GPIO_PinRemapConfig(GPIO_Remap, ENABLE);
	}

	return RT_EOK;
}

/**
 * 配置NVIC
 */
static rt_err_t stm32_uart_nvic_configure(struct rt_serial_device *serial)
{
	USART_TypeDef* uart = ((rt_stm32_uart_t*) serial)->usart_base;

	NVIC_InitTypeDef NVIC_InitStructure;

	switch ((rt_uint32_t) uart)
	{
#ifdef USE_STM32_UART1
	case (rt_uint32_t) USART1:
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		break;
#endif
#ifdef USE_STM32_UART2
	case (rt_uint32_t) USART2:
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		break;
#endif
#ifdef USE_STM32_UART3
	case (rt_uint32_t) USART3:
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		break;
#endif
#ifdef USE_STM32_UART4
	case (rt_uint32_t) UART4:
		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
		break;
#endif
	default:
		RT_ASSERT(0)
		;
	}

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	return RT_EOK;
}

/**
 * 串口配置
 */
static rt_err_t stm32_uart_configure(struct rt_serial_device *serial,
		struct serial_configure *cfg)
{
	USART_TypeDef* uart = ((rt_stm32_uart_t*) serial)->usart_base;

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	void (*RCC_APBxPeriphClockCmd)(uint32_t, FunctionalState);
	uint32_t RCC_APBxPeriph_UARTx;

	// 配置UART GPIO
	stm32_uart_gpio_configure(serial);

	// 配置NVIC
	stm32_uart_nvic_configure(serial);

	switch ((rt_uint32_t) uart)
	{
#ifdef USE_STM32_UART1
	case (rt_uint32_t) USART1:
		RCC_APBxPeriphClockCmd = &RCC_APB2PeriphClockCmd;
		RCC_APBxPeriph_UARTx = RCC_APB2Periph_USART1;
		break;
#endif
#ifdef USE_STM32_UART2
	case (rt_uint32_t) USART2:
		RCC_APBxPeriphClockCmd = &RCC_APB1PeriphClockCmd;
		RCC_APBxPeriph_UARTx = RCC_APB1Periph_USART2;
		break;
#endif
#ifdef USE_STM32_UART3
	case (rt_uint32_t) USART3:
		RCC_APBxPeriphClockCmd = &RCC_APB1PeriphClockCmd;
		RCC_APBxPeriph_UARTx = RCC_APB1Periph_USART3;
		break;
#endif
#ifdef USE_STM32_UART4
	case (rt_uint32_t) UART4:
		RCC_APBxPeriphClockCmd = &RCC_APB1PeriphClockCmd;
		RCC_APBxPeriph_UARTx = RCC_APB1Periph_UART4;
		break;
#endif
	default:
		RT_ASSERT(0)
		;
	}

	switch (cfg->baud_rate)
	{
	case BAUD_RATE_9600:
		USART_InitStructure.USART_BaudRate = 9600;
		break;
	case BAUD_RATE_115200:
		USART_InitStructure.USART_BaudRate = 115200;
		break;
	default:
		RT_ASSERT(0)
		;
	}

	switch (cfg->data_bits)
	{
	case DATA_BITS_8:
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		break;
	default:
		RT_ASSERT(0)
		;
	}

	switch (cfg->stop_bits)
	{
	case STOP_BITS_1:
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		break;
	default:
		RT_ASSERT(0)
		;
	}

	switch (cfg->parity)
	{
	case PARITY_NONE:
		USART_InitStructure.USART_Parity = USART_Parity_No;
		break;
	default:
		RT_ASSERT(0)
		;
		return RT_ERROR;
	}

	/* Enable UART clock */
	RCC_APBxPeriphClockCmd(RCC_APBxPeriph_UARTx, ENABLE);

	/* USART configuration */
	USART_Init(uart, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(uart, ENABLE);

	return RT_EOK;
}

/**
 * 串口中断控制
 */
static rt_err_t stm32_uart_control(struct rt_serial_device *serial, int cmd,
		void *arg)
{
	rt_err_t result = RT_EOK;

	USART_TypeDef* uart = ((rt_stm32_uart_t*) serial)->usart_base;

	switch ((uint32_t) arg)
	{
	case RT_DEVICE_FLAG_INT_RX:
		switch (cmd)
		{
		case RT_DEVICE_CTRL_CLR_INT:
			USART_ITConfig(uart, USART_IT_RXNE, DISABLE);
			break;
		case RT_DEVICE_CTRL_SET_INT:
			USART_ITConfig(uart, USART_IT_RXNE, ENABLE);
			break;
		default:
			RT_ASSERT(0)
			;
		}
		break;
	default:
		RT_ASSERT(0)
		;
	}

	return result;
}

/**
 * 串口发送函数
 */
static int stm32_uart_send(struct rt_serial_device *serial, char c)
{
	USART_TypeDef* uart = ((rt_stm32_uart_t*) serial)->usart_base;

	while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET)
			;

	USART_SendData(uart, c);

	return RT_EOK;
}

/**
 * 串口接收函数
 */
static int stm32_uart_receive(struct rt_serial_device *serial)
{
	USART_TypeDef* uart = ((rt_stm32_uart_t*) serial)->usart_base;
	int ch;

	if (USART_GetFlagStatus(uart, USART_FLAG_RXNE) == RESET)
		return -1;

	USART_ClearFlag(uart, USART_FLAG_RXNE);
	ch = USART_ReceiveData(uart);

	return ch;
}

/**
 * 串口中断函数
 */
static void stm32_uart_isr(struct rt_serial_device *serial)
{
	USART_TypeDef* uart = ((rt_stm32_uart_t*) serial)->usart_base;

	RT_ASSERT(uart != RT_NULL);

	if (USART_GetITStatus(uart, USART_IT_RXNE) != RESET)
	{
		rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
		/* clear interrupt */
		USART_ClearITPendingBit(uart, USART_IT_RXNE);
	}
}

#ifdef USE_STM32_UART1
/**
 * 串口1中断函数
 */
void USART1_IRQHandler(void)
{
	stm32_uart_isr(&uart1.dev);
}
#endif

#ifdef USE_STM32_UART2
/**
 * 串口2中断函数
 */
void USART2_IRQHandler(void)
{
	stm32_uart_isr(&uart2.dev);
}
#endif

#ifdef USE_STM32_UART3
/**
 * 串口3中断函数
 */
void USART3_IRQHandler(void)
{
	stm32_uart_isr(&uart3.dev);
}
#endif

#ifdef USE_STM32_UART4
/**
 * 串口3中断函数
 */
void UART4_IRQHandler(void)
{
	stm32_uart_isr(&uart4.dev);
}
#endif

/**
 * @brief 串口设备注册
 * @param uart      : STM32 UART设备结构体
 * @param uart_base : STM32 UART外设基地址
 * @param name      : STM32 UART设备名
 */
static void rt_hw_uart_register(rt_stm32_uart_t *uart, USART_TypeDef *uart_base, char *name)
{
	RT_ASSERT(uart != RT_NULL);
	RT_ASSERT(uart_base != RT_NULL);

	// 没有定义对应的硬件I2C
	if (!(uart_base == USART1 || uart_base == USART2 || uart_base == USART3 || uart_base == UART4 || uart_base == UART5))
	{
		RT_ASSERT(0);
	}

	uart->usart_base = uart_base;
	uart->dev.ops = &stm32_uart_ops;
	uart->dev.config = stm32_uart_default_config;

	rt_hw_serial_register(&uart->dev, name,
			RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, RT_NULL);
}

/**
 * 串口注册
 */
void rt_hw_uart_init(void)
{
#ifdef USE_STM32_UART1
	rt_hw_uart_register(&uart1, USART1, "UART1");
#endif
#ifdef USE_STM32_UART2
	rt_hw_uart_register(&uart2, USART2, "UART2");
#endif
#ifdef USE_STM32_UART3
	rt_hw_uart_register(&uart3, USART3, "UART3");
#endif
#ifdef USE_STM32_UART4
	rt_hw_uart_register(&uart4, UART4, "UART4");
#endif
}
