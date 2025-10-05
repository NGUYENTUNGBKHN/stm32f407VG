/**
 * @file       lis3dsh.h
 * @brief      
 * @date       2025/10/04
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    
 * @ref        
 * @copyright  Copyright (c) 2025 RoboTun
*/
#ifndef _LIS3DSH_H_
#define _LIS3DSH_H_
#ifdef __cplusplus
extern "C"
{
#endif

/* CODE */
#include "common.h"
#include "stm32f4xx_hal_conf.h"
#include "disco_bsp.h"

typedef struct LIS3DSH_S lis3dsh_t;
struct LIS3DSH_S
{
    SPI_HandleTypeDef SpiHandle;
    void (*init)(lis3dsh_t *self);
    void (*write)(lis3dsh_t *self, uint8_t addr, uint8_t data);
    void (*read)(lis3dsh_t *self, uint8_t addr, uint8_t *data);
};


extern lis3dsh_t *lis3dsh_create();

#ifdef __cplusplus
}
#endif
#endif
