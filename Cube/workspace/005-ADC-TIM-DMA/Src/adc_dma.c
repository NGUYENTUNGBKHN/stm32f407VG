/**
 *
 */
#include "adc_dma.h"

#define GPIOAEN 		(1U<<0)
#define ADC1EN			(1U<<8)

#define CR1_SCAN		(1U<<8)
#define CR2_DMA			(1U<<8)
#define CR2_DDS			(1U<<9)
#define CR2_CONT		(1U<<1)

#define CR2_ADCON		(1U<<0)
#define CR2_SWSTART		(1U<<30)

/* DMA */
#define DMA2EN			(1U << 22)
#define DMA_SCR_EN		(1U << 0)
#define DMA_SCR_MINC	(1U<<10)
#define DMA_SCR_PINC	(1U<<9)
#define DMA_SCR_TCIE	(1U<<4)
#define DMA_SCR_TEIE	(1U<<2)
#define DMA_SFCR_DMDIS	(1U<<2)
#define DMA_SCR_CIRC	(1U<<8)


#define TIM2EN			(1U << 0)
#define CR1_CEN			(1U << 0)

uint16_t adc_raw_data[NUM_OF_SAMPLES];

void adc_tim_dma_init()
{
	/*1 GPIO Configuration*/
	/*1.1. Enable clock access to ADC GPIO Pin's Port*/
	RCC->AHB1ENR |= GPIOAEN;

	/*1.2. Set PA0 mode to analog mode*/
	GPIOA->MODER |= (1U<<0);
	GPIOA->MODER |= (1U<<1);

//	GPIOA->MODER |= (1U<<2);
//	GPIOA->MODER |= (1U<<3);

	/*2 ADC Configuration*/
	/*2.1. Enable clock access to ADC*/
	RCC->APB2ENR |= ADC1EN;
//	/*2.2. Set Sequence length*/
//	ADC1->SQR1 |= (1UL<<20);
//	ADC1->SQR1 &= ~(1U<<21);
//	ADC1->SQR1 &= ~(1U<<22);
//	ADC1->SQR1 &= ~(1U<<23);
//	/*2.3. Set Sequence*/
//	ADC1->SQR3 = (0U<<0) | (1U <<5);
//	/*2.4. Enable scan mode*/
//	ADC1->CR1 = CR1_SCAN;
	/*2.5. Select to use DMA*/
	ADC1->CR2 |= CR2_DMA | CR2_DDS;

	/*Select external trigger on rising egde*/
	ADC1->CR2 |= (1U << 28);
	ADC1->CR2 &= ~(1U << 29);

	/* Select ZTimer 2 TRGO event*/
	ADC1->CR2 &= ~(1U<<24);
	ADC1->CR2 |= (1U<<25);
	ADC1->CR2 |= (1U<<26);
	ADC1->CR2 &= ~(1U<<27);

	/*3 DMA Configuration*/
	/*3.1. Enable clock access to DMA*/
	RCC->AHB1ENR |= DMA2EN;
	/*3.2. Disable DMA*/
	DMA2_Stream0->CR &= ~DMA_SCR_EN;
	/*3.3. Wait until DMA is disabled*/
	while((DMA2_Stream0->CR & DMA_SCR_EN)){}
	/*3.4. Enable circular mode*/
	DMA2_Stream0->CR |= DMA_SCR_CIRC;
	/*3.5. Set memory transfer size*/
	DMA2_Stream0->CR |= (1U << 13);
	DMA2_Stream0->CR &= ~(1U << 14);
	/*3.6. Set peripheral transfer size*/
	DMA2_Stream0->CR |= (1U << 11);
	DMA2_Stream0->CR &= ~(1U << 12);
	/*3.7. Enable memory increment*/
	DMA2_Stream0->CR |= DMA_SCR_MINC;

	/* Enable interrupt */
	DMA2_Stream0->CR |= DMA_SCR_TCIE;
//	DMA2_Stream0->CR |= DMA_SCR_TEIE;

	/*3.8. Set peripheral address*/
	DMA2_Stream0->PAR = (uint32_t)(&(ADC1->DR));
	/*3.9. Set memory address*/
	DMA2_Stream0->M0AR = (uint32_t)(&adc_raw_data);
	/*3.10. Set number of transfer*/
	DMA2_Stream0->NDTR = (uint16_t)NUM_OF_SAMPLES;

	/* Timer Configuration */

	/* Configure timer period to 100Hx ADC is going
	 * to be sampling at a 100GHz rate, every 10ms*/
	/* Enable clock access to TIM2*/
	RCC->APB1ENR |= TIM2EN;
	/* Set TIm prescaler value*/
	TIM2->PSC = 16000 - 1;
	/* Set TIm auto-reload value*/
	TIM2->ARR = 10 -1;
	/* Configure master mode selection bits*/
	TIM2->CR2 &= ~(1U << 4);
	TIM2->CR2 |= (1U << 5);
	TIM2->CR2 &= ~(1U << 6);

	/*4 ADC Configuration*/
	/*4.1. Enable ADC*/
	ADC1->CR2 |= CR2_ADCON;
//	/*4.2. Start ADC*/
//	ADC1->CR2 |= CR2_SWSTART;

	/*3.11. Enable DMA stream*/
	DMA2_Stream0->CR |= DMA_SCR_EN;

	/* Enable Timer */
	TIM2->CR1 |= CR1_CEN;

	/* Enable DMA interrupt in NVIC*/
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}


void adc_dma_init()
{
	/*1 GPIO Configuration*/
	/*1.1. Enable clock access to ADC GPIO Pin's Port*/
	RCC->AHB1ENR |= GPIOAEN;

	/*1.2. Set PA0 and PA1 mode to analog mode*/
	GPIOA->MODER |= (1U<<0);
	GPIOA->MODER |= (1U<<1);

	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);

	/*2 ADC Configuration*/
	/*2.1. Enable clock access to ADC*/
	RCC->APB2ENR |= ADC1EN;
	/*2.2. Set Sequence length*/
	ADC1->SQR1 |= (1UL<<20);
	ADC1->SQR1 &= ~(1U<<21);
	ADC1->SQR1 &= ~(1U<<22);
	ADC1->SQR1 &= ~(1U<<23);
	/*2.3. Set Sequence*/
	ADC1->SQR3 = (0U<<0) | (1U <<5);
	/*2.4. Enable scan mode*/
	ADC1->CR1 = CR1_SCAN;
	/*2.5. Select to use DMA*/
	ADC1->CR2 |= CR2_CONT | CR2_DMA | CR2_DDS;

	/*3 DMA Configuration*/
	/*3.1. Enable clock access to DMA*/
	RCC->AHB1ENR |= DMA2EN;
	/*3.2. Disable DMA*/
	DMA2_Stream0->CR &= ~DMA_SCR_EN;
	/*3.3. Wait until DMA is disabled*/
	while((DMA2_Stream0->CR & DMA_SCR_EN)){}
	/*3.4. Enable circular mode*/
	DMA2_Stream0->CR |= DMA_SCR_CIRC;
	/*3.5. Set memory transfer size*/
	DMA2_Stream0->CR |= (1U << 13);
	DMA2_Stream0->CR &= ~(1U << 14);
	/*3.6. Set peripheral transfer size*/
	DMA2_Stream0->CR |= (1U << 11);
	DMA2_Stream0->CR &= ~(1U << 12);
	/*3.7. Enable memory increment*/
	DMA2_Stream0->CR |= DMA_SCR_MINC;
	/*3.8. Set peripheral address*/
	DMA2_Stream0->PAR = (uint32_t)(&(ADC1->DR));
	/*3.9. Set memory address*/
	DMA2_Stream0->M0AR = (uint32_t)(&adc_raw_data);
	/*3.10. Set number of transfer*/
	DMA2_Stream0->NDTR = (uint16_t)NUM_OF_CHNANELS;
	/*3.11. Enable DMA stream*/
	DMA2_Stream0->CR |= DMA_SCR_EN;
	/*4 ADC Configuration*/
	/*4.1. Enable ADC*/
	ADC1->CR2 |= CR2_ADCON;
	/*4.2. Start ADC*/
	ADC1->CR2 |= CR2_SWSTART;

}


