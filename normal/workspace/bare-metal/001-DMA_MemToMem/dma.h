/**
 * @file       dma.h
 * @brief      
 * @date       2025/08/01
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    
 * @ref        
 * @copyright  Copyright (c) 2025 RoboTun
*/
#ifndef _DMA_H_
#define _DMA_H_
#ifdef __cplusplus
extern "C"
{
#endif

/* CODE */
#include "stm32f407xx.h"

void dma_mem_to_mem_init();
void dma_mem_transfer(uint32_t src, uint32_t dest, uint16_t size);

#ifdef __cplusplus
}
#endif
#endif




