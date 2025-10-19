/**
 * @file       disco_bsp.h
 * @brief      
 * @date       2025/10/04
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    
 * @ref        
 * @copyright  Copyright (c) 2025 RoboTun
*/
#ifndef _DISCO_BSP_H_
#define _DISCO_BSP_H_
#ifdef __cplusplus
extern "C"
{
#endif

/* CODE */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#define METHOD 2

#define MCU_GPIO_PIN_0					0U
#define MCU_GPIO_PIN_1					1U
#define MCU_GPIO_PIN_2					2U
#define MCU_GPIO_PIN_3					3U
#define MCU_GPIO_PIN_4					4U
#define MCU_GPIO_PIN_5					5U
#define MCU_GPIO_PIN_6					6U
#define MCU_GPIO_PIN_7					7U
#define MCU_GPIO_PIN_8					8U
#define MCU_GPIO_PIN_9					9U
#define MCU_GPIO_PIN_10					10U
#define MCU_GPIO_PIN_11					11U
#define MCU_GPIO_PIN_12					12U
#define MCU_GPIO_PIN_13					13U
#define MCU_GPIO_PIN_14					14U
#define MCU_GPIO_PIN_15					15U

/* SPI1 and Accelerometer*/

/* DISCO SPI */
#define DISCO_SPIx                      SPI1

#define DISCO_SPIx_AF                   GPIO_AF5_SPI1
#define DISCO_SPIx_CLK_Enable()         __HAL_RCC_SPI1_CLK_ENABLE()
#define DISCO_SPIx_port                 GPIOA
#define DISCO_SPIx_GPIO_CLK_Enable()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define DISCO_SPIx_SCK                  GPIO_PIN_5
#define DISCO_SPIx_MISO                 GPIO_PIN_6
#define DISCO_SPIx_MOSI                 GPIO_PIN_7          
#define DISCO_SPIx_IRQn                 SPI1_IRQn
#define DISCO_SPIx_IRQn_Handler         SPI1_IRQHandler

/* ACCELEROMETER */
#define ACCE_port               GPIOE
#define ACCE_GPIO_CLK_Enable()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define ACCE_CS                 GPIO_PIN_3
#define ACCE_INT1               GPIO_PIN_0
#define ACCE_INI2               GPIO_PIN_1



/* LCD */
#if (METHOD == 1)
#define SPI						SPI2
#define LCD_SCL_PIN				MCU_GPIO_PIN_13
#define LCD_SCL_PORT			GPIOB
#define LCD_SDI_PIN				MCU_GPIO_PIN_15
#define LCD_SDI_PORT			GPIOB
#define LCD_SDO_PIN				MCU_GPIO_PIN_2
#define LCD_SDO_PORT			GPIOC
#define LCD_RESX_PIN			MCU_GPIO_PIN_10
#define LCD_RESX_PORT			GPIOD
#define LCD_DCX_PIN				MCU_GPIO_PIN_9
#define LCD_DCX_PORT			GPIOD
#define LCD_CSX_PORT			GPIOD
#define LCD_CSX_PIN				MCU_GPIO_PIN_11
#else
#define SPI						SPI2
#define LCD_SCL_PIN				MCU_GPIO_PIN_10     /* GPIOB 10 */
#define LCD_SCL_PORT			GPIOB
#define LCD_SDI_PIN				MCU_GPIO_PIN_15     /* GPIOB 15 */
#define LCD_SDI_PORT			GPIOB
#define LCD_SDO_PIN				MCU_GPIO_PIN_2      /* GPIOC 2 */
#define LCD_SDO_PORT			GPIOC
#define LCD_RESX_PIN			MCU_GPIO_PIN_1      /* GPIOC 1 */
#define LCD_RESX_PORT			GPIOC
#define LCD_DCX_PIN				MCU_GPIO_PIN_4      /* GPIOC 4 */
#define LCD_DCX_PORT			GPIOC
#define LCD_CSX_PIN				MCU_GPIO_PIN_0      /* GPIOC 0 */
#define LCD_CSX_PORT			GPIOC

#endif 

#ifdef __cplusplus
}
#endif
#endif
