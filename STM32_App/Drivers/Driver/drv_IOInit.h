/*
 * drv_IOInit.h
 *
 *  Created on: 2016/01/11
 *      Author: Arvin
 */

#ifndef ECGPB_DRIVERS_DRV_IOINIT_H_
#define ECGPB_DRIVERS_DRV_IOINIT_H_

#define   DUMMY_BYTE    			(uint8_t)0xA5		/* 伪命令 for SPI，可以为任意值，用于读操作 */

/*-----------------------------------Shell Uart  ----------------------------------*/
/* Definition for USARTx clock resources */
#define SHELL_UART                           		USART1
#define SHELL_UART_CLK_EN()   			__USART1_CLK_ENABLE()
#define SHELL_UART_CLK_DISABLE()               	__USART1_CLK_DISABLE()
#define SHELL_UARTDMAx_CLK_ENABLE()           __DMA2_CLK_ENABLE()

/* Definition for USARTx Pins */
#define SHELL_UART_TX_PIN                    		GPIO_PIN_6

#define SHELL_UART_TX_GPIO_PORT              	    GPIOB
#define SHELL_UART_TX_AF                     		GPIO_AF7_USART1
#define SHELL_UART_RX_PIN                    		GPIO_PIN_7
#define SHELL_UART_RX_GPIO_PORT              	    GPIOB
#define SHELL_UART_RX_AF                     		GPIO_AF7_USART1

/* Definition for USARTx's DMA */
#define SHELL_UART_TX_DMA_STREAM                    DMA2_Stream7
#define SHELL_UART_TX_DMA_CH              		    DMA_CHANNEL_4
#define SHELL_UART_RX_DMA_STREAM
#define SHELL_UART_RX_DMA_CH
/* Definition for USARTx's NVIC */
#define SHELL_UART_DMA_TX_IRQn                	    DMA2_Stream7_IRQn
#define SHELL_UART_DMA_RX_IRQn

#define  SHELL_UART_IRQn                            USART1_IRQn

/*-----------------------------------TIME BASE  ----------------------------------*/
//#define SYS_BASE_TIMER            			TIM2      /*TIM2_OC3---pps Input   OC1=ETR=GPS10MHz*/
/* -----------------------------------------------------------------------
 ----------------------------------------------------------------------- */
/*-------------------------------------------SPI_ROM------------------------------------------*/
#define SPI_ROM_SPI_INS      					SPI3
#define SPI_ROM_MISO_PIN   					GPIO_PIN_11
#define SPI_ROM_MISO_PORT   					GPIOC
#define SPI_ROM_MOSI_PIN   					GPIO_PIN_5
#define SPI_ROM_MOSI_PORT   					GPIOB
#define SPI_ROM_MI_MO_AF     					GPIO_AF6_SPI3
#define SPI_ROM_SCK_PIN   					GPIO_PIN_10
#define SPI_ROM_SCK_PORT   					GPIOC
#define SPI_ROM_SCK_AF     					GPIO_AF6_SPI3
#define SPI_ROM_CLK_EN()					__SPI3_CLK_ENABLE ()

#define SPI_ROM_NSS_PIN   					GPIO_PIN_1
#define SPI_ROM_NSS_PORT   					GPIOB

#define GPIO_LED0_PIN					GPIO_PIN_6
#define GPIO_LED0_IO_PORT				GPIOG
#define GPIO_LED1_PIN					GPIO_PIN_7
#define GPIO_LED1_IO_PORT				GPIOG

#define GPIO_LED3_PIN					GPIO_PIN_12
#define GPIO_LED3_IO_PORT				GPIOC
#define GPIO_LED4_PIN					GPIO_PIN_13
#define GPIO_LED4_IO_PORT				GPIOC

/*-------------------------------------------SPI_BLE------------------------------------------*/
#define SPI_BLE_SPI_INS      			    SPI2
#define SPI_BLE_MISO_PIN   					GPIO_PIN_2
#define SPI_BLE_MISO_PORT   				GPIOC
#define SPI_BLE_MOSI_PIN   					GPIO_PIN_3
#define SPI_BLE_MOSI_PORT   				GPIOC
#define SPI_BLE_MI_MO_AF     				GPIO_AF5_SPI2
#define SPI_BLE_SCK_PIN   					GPIO_PIN_13
#define SPI_BLE_SCK_PORT   					GPIOB
#define SPI_BLE_SCK_AF     					GPIO_AF5_SPI2

#define SPI_BLE_NSS_PIN   					GPIO_PIN_2
#define SPI_BLE_NSS_PORT   					GPIOB
#define SPI_BLE_NSS_PORT_CLK()         	    __GPIOB_CLK_ENABLE()

#define SPI_BLE_CLK_ENABLE()                __SPI2_CLK_ENABLE()
#define SPI_BLE_GPIO_CLK_ENABLE()       	__GPIOC_CLK_ENABLE(); __GPIOB_CLK_ENABLE()/*enable Clk port c and port b*/

#define PT_RCC_AHB_GPIO_BTREQN()      		__GPIOB_CLK_ENABLE()
#define PT_GPIO_PIN_BTREQN                  GPIO_PIN_0
#define PT_GPIOBTREQN                    	GPIOB

#define PT_RCC_AHB_GPIO_BTSYNC0()     	    __GPIOB_CLK_ENABLE()
#define PT_GPIO_Pin_BTSYNC0               	GPIO_PIN_3
#define PT_GPIOBTSYNC0                    	GPIOB

#define PT_RCC_AHBPeriph_GPIO_BTRESET()     __GPIOB_CLK_ENABLE()
#define PT_GPIO_PIN_BTRESET               	GPIO_PIN_14
#define PT_GPIOBTRESET                    	GPIOB

#define  BLE_BTREQN_EXIT_IO_PIN             GPIO_PIN_4
#define  BLE_BTREQN_EXIT_IO_PORT		    GPIOB
#define  BLE_BTREQN_EXIT_IT_LINE    		EXTI4_IRQn

#define  BLE_BTSYNC0_EXIT_IO_PIN            GPIO_PIN_3
#define  BLE_BTSYNC0_EXIT_IO_PORT		    GPIOB
#define  BLE_BTSYNC0_EXIT_IT_LINE    		EXTI3_IRQn

#endif /* ECGPB_DRIVERS_DRV_IOINIT_H_ */

