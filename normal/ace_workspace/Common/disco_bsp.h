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


#ifdef __cplusplus
}
#endif
#endif
