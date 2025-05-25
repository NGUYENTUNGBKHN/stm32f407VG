/*
 * dma.h
 *
 *  Created on: May 18, 2025
 *      Author: nguyen
 */

#ifndef DMA_H_
#define DMA_H_

#include <stdint.h>
#include "stm32f4xx.h"

#define LISR_TCIF0    (1U << 5)
#define LIFCR_CTCIF0    (1U << 5)

#define LISR_TEIF0		(1U << 3)
#define LIFCR_CTEIF0		(1U << 3)


extern void dma_config();
extern void dma_transfer_start(uint32_t src_buff, uint32_t dest_buff, uint32_t len);

#endif /* DMA_H_ */
