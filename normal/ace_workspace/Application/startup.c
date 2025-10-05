/**
 * @file       boot_startup.c
 * @brief      
 * @date       2025/09/30
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    
 * @ref        
 * @copyright  Copyright (c) 2025 RoboTun
*/

/*******************************************************************************
**                                INCLUDES
*******************************************************************************/
#include <stdint.h>
#include "system_stm32f4xx.h"
/*******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/


/*******************************************************************************
**                      COMMON VARIABLE DEFINITIONS
*******************************************************************************/


/*******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/


/*******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/


/*******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/

extern uint32_t _estack;
extern int main();
// extern void __libc_init_array();

extern uint32_t _sidata;

extern uint32_t _sdata;
extern uint32_t _edata;

extern uint32_t _sbss;
extern uint32_t _ebss;


void Reset_Handler()
{
    volatile uint32_t *start = &_sdata;
    volatile uint32_t *end = &_edata;
    volatile uint32_t *data = &_sidata;
    volatile uint32_t *bss_start = &_sbss;
    volatile uint32_t *bss_end = &_ebss;
    SystemInit();
    while (start < end)
    {
        /* code */
        *start++ = *data++;
    }
   
    /* setup bss data = 0 */
    while (bss_start < bss_end)
    {
        /* code */
        *bss_start++ = 0;
    }


    /* Jump to __libc_init_array */
    // __libc_init_array();
    
    main();
}

void default_func()
{
    while (1)
    {
        /* code */
    }
    
}

void NMI_IRQHandler(void) __attribute((weak, alias("default_func")));
void HardFault_IRQHandler(void) __attribute((weak, alias("default_func")));
void MemManage_IRQHandler(void) __attribute((weak, alias("default_func")));
void BusFault_IRQHandler(void) __attribute((weak, alias("default_func")));
void UsageFault_IRQHandler(void) __attribute((weak, alias("default_func")));
void SVC_IRQHandler(void) __attribute((weak, alias("default_func")));
void DebugMonitor_IRQHandler(void) __attribute((weak, alias("default_func")));
void PendSV_IRQHandler(void) __attribute((weak, alias("default_func")));
void SysTick_IRQHandler(void) __attribute((weak, alias("default_func")));
void WWDG_IRQHandler (void) __attribute((weak, alias("default_func")));            
void PVD_IRQHandler (void) __attribute((weak, alias("default_func")));             
void TAMP_STAMP_IRQHandler  (void) __attribute((weak, alias("default_func")));     
void RTC_WKUP_IRQHandler (void) __attribute((weak, alias("default_func")));                            
void RCC_IRQHandler(void) __attribute((weak, alias("default_func")));              
void EXTI0_IRQHandler(void) __attribute((weak, alias("default_func")));            
void EXTI1_IRQHandler(void) __attribute((weak, alias("default_func")));            
void EXTI2_IRQHandler(void) __attribute((weak, alias("default_func")));            
void EXTI3_IRQHandler(void) __attribute((weak, alias("default_func")));            
void EXTI4_IRQHandler(void) __attribute((weak, alias("default_func")));            
void DMA1_Stream0_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA1_Stream1_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA1_Stream2_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA1_Stream3_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA1_Stream4_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA1_Stream5_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA1_Stream6_IRQHandler(void) __attribute((weak, alias("default_func")));     
void ADC_IRQHandler(void) __attribute((weak, alias("default_func")));              
void CAN1_TX_IRQHandler(void) __attribute((weak, alias("default_func")));          
void CAN1_RX0_IRQHandler(void) __attribute((weak, alias("default_func")));         
void CAN1_RX1_IRQHandler(void) __attribute((weak, alias("default_func")));         
void CAN1_SCE_IRQHandler(void) __attribute((weak, alias("default_func")));         
void EXTI9_5_IRQHandler(void) __attribute((weak, alias("default_func")));          
void TIM1_BRK_TIM9_IRQHandler(void) __attribute((weak, alias("default_func")));    
void TIM1_UP_TIM10_IRQHandler(void) __attribute((weak, alias("default_func")));    
void TIM1_TRG_COM_TIM11_IRQHandler(void) __attribute((weak, alias("default_func"))); 
void TIM1_CC_IRQHandler(void) __attribute((weak, alias("default_func")));          
void TIM2_IRQHandler(void) __attribute((weak, alias("default_func")));             
void TIM3_IRQHandler(void) __attribute((weak, alias("default_func")));             
void TIM4_IRQHandler(void) __attribute((weak, alias("default_func")));             
void I2C1_EV_IRQHandler(void) __attribute((weak, alias("default_func")));          
void I2C1_ER_IRQHandler(void) __attribute((weak, alias("default_func")));          
void I2C2_EV_IRQHandler(void) __attribute((weak, alias("default_func")));          
void I2C2_ER_IRQHandler(void) __attribute((weak, alias("default_func")));          
void SPI1_IRQHandler(void) __attribute((weak, alias("default_func")));             
void SPI2_IRQHandler(void) __attribute((weak, alias("default_func")));             
void USART1_IRQHandler(void) __attribute((weak, alias("default_func")));           
void USART2_IRQHandler(void) __attribute((weak, alias("default_func")));           
void USART3_IRQHandler(void) __attribute((weak, alias("default_func")));           
void EXTI15_10_IRQHandler(void) __attribute((weak, alias("default_func")));        
void RTC_Alarm_IRQHandler(void) __attribute((weak, alias("default_func")));        
void OTG_FS_WKUP_IRQHandler(void) __attribute((weak, alias("default_func")));      
void TIM8_BRK_TIM12_IRQHandler(void) __attribute((weak, alias("default_func")));   
void TIM8_UP_TIM13_IRQHandler(void) __attribute((weak, alias("default_func")));    
void TIM8_TRG_COM_TIM14_IRQHandler(void) __attribute((weak, alias("default_func"))); 
void TIM8_CC_IRQHandler(void) __attribute((weak, alias("default_func")));          
void DMA1_Stream7_IRQHandler(void) __attribute((weak, alias("default_func")));     
void FSMC_IRQHandler(void) __attribute((weak, alias("default_func")));             
void SDIO_IRQHandler(void) __attribute((weak, alias("default_func")));             
void TIM5_IRQHandler(void) __attribute((weak, alias("default_func")));             
void SPI3_IRQHandler(void) __attribute((weak, alias("default_func")));             
void UART4_IRQHandler(void) __attribute((weak, alias("default_func")));            
void UART5_IRQHandler(void) __attribute((weak, alias("default_func")));            
void TIM6_DAC_IRQHandler(void) __attribute((weak, alias("default_func")));         
void TIM7_IRQHandler(void) __attribute((weak, alias("default_func")));             
void DMA2_Stream0_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA2_Stream1_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA2_Stream2_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA2_Stream3_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA2_Stream4_IRQHandler(void) __attribute((weak, alias("default_func")));     
void ETH_IRQHandler(void) __attribute((weak, alias("default_func")));              
void ETH_WKUP_IRQHandler(void) __attribute((weak, alias("default_func")));         
void CAN2_TX_IRQHandler(void) __attribute((weak, alias("default_func")));          
void CAN2_RX0_IRQHandler(void) __attribute((weak, alias("default_func")));         
void CAN2_RX1_IRQHandler(void) __attribute((weak, alias("default_func")));         
void CAN2_SCE_IRQHandler(void) __attribute((weak, alias("default_func")));         
void OTG_FS_IRQHandler(void) __attribute((weak, alias("default_func")));           
void DMA2_Stream5_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA2_Stream6_IRQHandler(void) __attribute((weak, alias("default_func")));     
void DMA2_Stream7_IRQHandler(void) __attribute((weak, alias("default_func")));     
void USART6_IRQHandler(void) __attribute((weak, alias("default_func")));           
void I2C3_EV_IRQHandler(void) __attribute((weak, alias("default_func")));          
void I2C3_ER_IRQHandler(void) __attribute((weak, alias("default_func")));          
void OTG_HS_EP1_OUT_IRQHandler(void) __attribute((weak, alias("default_func")));   
void OTG_HS_EP1_IN_IRQHandler(void) __attribute((weak, alias("default_func")));    
void OTG_HS_WKUP_IRQHandler(void) __attribute((weak, alias("default_func")));      
void OTG_HS_IRQHandler(void) __attribute((weak, alias("default_func")));           
void DCMI_IRQHandler(void) __attribute((weak, alias("default_func")));             
void CRYP_IRQHandler(void) __attribute((weak, alias("default_func")));             
void HASH_RNG_IRQHandler(void) __attribute((weak, alias("default_func")));         
void FPU_IRQHandler(void) __attribute((weak, alias("default_func")));              



__attribute((section(".isr_vector")))
uint32_t *_isr_vector[] = {
    (uint32_t*) &_estack,
    (uint32_t*) Reset_Handler,
    (uint32_t*) NMI_IRQHandler,
    (uint32_t*) HardFault_IRQHandler,
    (uint32_t*) MemManage_IRQHandler,
    (uint32_t*) BusFault_IRQHandler,
    (uint32_t*) UsageFault_IRQHandler,
    (uint32_t*) 0,
    (uint32_t*) 0,
    (uint32_t*) 0,
    (uint32_t*) 0,
    (uint32_t*) SVC_IRQHandler,
    (uint32_t*) DebugMonitor_IRQHandler,
    (uint32_t*) 0,
    (uint32_t*) PendSV_IRQHandler,
    (uint32_t*) SysTick_IRQHandler,
    (uint32_t*) WWDG_IRQHandler ,             			/* Window Watchdog interrupt                                          */
    (uint32_t*) PVD_IRQHandler ,              			/* PVD through EXTI line detection interrupt                          */
    (uint32_t*) TAMP_STAMP_IRQHandler  ,      			/* Tamper and TimeStamp interrupts through the EXTI line              */
    (uint32_t*) RTC_WKUP_IRQHandler ,         			/* RTC Wakeup interrupt through the EXTI line                         */
    (uint32_t*) 0 ,                           			/* Reserved                                                           */
    (uint32_t*) RCC_IRQHandler,               			/* RCC global interrupt                                               */
    (uint32_t*) EXTI0_IRQHandler,             			/* EXTI Line0 interrupt                                               */
    (uint32_t*) EXTI1_IRQHandler,             			/* EXTI Line1 interrupt                                               */
    (uint32_t*) EXTI2_IRQHandler,             			/* EXTI Line2 interrupt                                               */
    (uint32_t*) EXTI3_IRQHandler,             			/* EXTI Line3 interrupt                                               */
    (uint32_t*) EXTI4_IRQHandler,             			/* EXTI Line4 interrupt                                               */
    (uint32_t*) DMA1_Stream0_IRQHandler,      			/* DMA1 Stream0 global interrupt                                      */
    (uint32_t*) DMA1_Stream1_IRQHandler,      			/* DMA1 Stream1 global interrupt                                      */
    (uint32_t*) DMA1_Stream2_IRQHandler,      			/* DMA1 Stream2 global interrupt                                      */
    (uint32_t*) DMA1_Stream3_IRQHandler,      			/* DMA1 Stream3 global interrupt                                      */
    (uint32_t*) DMA1_Stream4_IRQHandler,      			/* DMA1 Stream4 global interrupt                                      */
    (uint32_t*) DMA1_Stream5_IRQHandler,      			/* DMA1 Stream5 global interrupt                                      */
    (uint32_t*) DMA1_Stream6_IRQHandler,      			/* DMA1 Stream6 global interrupt                                      */
    (uint32_t*) ADC_IRQHandler,               			/* ADC1, ADC2 and ADC3 global interrupts                              */
    (uint32_t*) CAN1_TX_IRQHandler,           			/* CAN1 TX interrupts                                                 */
    (uint32_t*) CAN1_RX0_IRQHandler,          			/* CAN1 RX0 interrupts                                                */
    (uint32_t*) CAN1_RX1_IRQHandler,          			/* CAN1 RX1 interrupts                                                */
    (uint32_t*) CAN1_SCE_IRQHandler,          			/* CAN1 SCE interrupt                                                 */
    (uint32_t*) EXTI9_5_IRQHandler,           			/* EXTI Line[9:5] interrupts                                          */
    (uint32_t*) TIM1_BRK_TIM9_IRQHandler,     			/* TIM1 Break interrupt and TIM9 global interrupt                     */
    (uint32_t*) TIM1_UP_TIM10_IRQHandler,     			/* TIM1 Update interrupt and TIM10 global interrupt                   */
    (uint32_t*) TIM1_TRG_COM_TIM11_IRQHandler,			/* TIM1 Trigger and Commutation interrupts and TIM11 global interrupt */
    (uint32_t*) TIM1_CC_IRQHandler,           			/* TIM1 Capture Compare interrupt                                     */
    (uint32_t*) TIM2_IRQHandler,              			/* TIM2 global interrupt                                              */
    (uint32_t*) TIM3_IRQHandler,              			/* TIM3 global interrupt                                              */
    (uint32_t*) TIM4_IRQHandler,              			/* TIM4 global interrupt                                              */
    (uint32_t*) I2C1_EV_IRQHandler,           			/* I2C1 event interrupt                                               */
    (uint32_t*) I2C1_ER_IRQHandler,           			/* I2C1 error interrupt                                               */
    (uint32_t*) I2C2_EV_IRQHandler,           			/* I2C2 event interrupt                                               */
    (uint32_t*) I2C2_ER_IRQHandler,           			/* I2C2 error interrupt                                               */
    (uint32_t*) SPI1_IRQHandler,              			/* SPI1 global interrupt                                              */
    (uint32_t*) SPI2_IRQHandler,              			/* SPI2 global interrupt                                              */
    (uint32_t*) USART1_IRQHandler,            			/* USART1 global interrupt                                            */
    (uint32_t*) USART2_IRQHandler,            			/* USART2 global interrupt                                            */
    (uint32_t*) USART3_IRQHandler,            			/* USART3 global interrupt                                            */
    (uint32_t*) EXTI15_10_IRQHandler,         			/* EXTI Line[15:10] interrupts                                        */
    (uint32_t*) RTC_Alarm_IRQHandler,         			/* RTC Alarms (A and B) through EXTI line interrupt                   */
    (uint32_t*) OTG_FS_WKUP_IRQHandler,       			/* USB On-The-Go FS Wakeup through EXTI line interrupt                */
    (uint32_t*) TIM8_BRK_TIM12_IRQHandler,    			/* TIM8 Break interrupt and TIM12 global interrupt                    */
    (uint32_t*) TIM8_UP_TIM13_IRQHandler,     			/* TIM8 Update interrupt and TIM13 global interrupt                   */
    (uint32_t*) TIM8_TRG_COM_TIM14_IRQHandler,			/* TIM8 Trigger and Commutation interrupts and TIM14 global interrupt */
    (uint32_t*) TIM8_CC_IRQHandler,           			/* TIM8 Capture Compare interrupt                                     */
    (uint32_t*) DMA1_Stream7_IRQHandler,      			/* DMA1 Stream7 global interrupt                                      */
    (uint32_t*) FSMC_IRQHandler,              			/* FSMC global interrupt                                              */
    (uint32_t*) SDIO_IRQHandler,              			/* SDIO global interrupt                                              */
    (uint32_t*) TIM5_IRQHandler,              			/* TIM5 global interrupt                                              */
    (uint32_t*) SPI3_IRQHandler,              			/* SPI3 global interrupt                                              */
    (uint32_t*) UART4_IRQHandler,             			/* UART4 global interrupt                                             */
    (uint32_t*) UART5_IRQHandler,             			/* UART5 global interrupt                                             */
    (uint32_t*) TIM6_DAC_IRQHandler,          			/* TIM6 global interrupt, DAC1 and DAC2 underrun error interrupt      */
    (uint32_t*) TIM7_IRQHandler,              			/* TIM7 global interrupt                                              */
    (uint32_t*) DMA2_Stream0_IRQHandler,      			/* DMA2 Stream0 global interrupt                                      */
    (uint32_t*) DMA2_Stream1_IRQHandler,      			/* DMA2 Stream1 global interrupt                                      */
    (uint32_t*) DMA2_Stream2_IRQHandler,      			/* DMA2 Stream2 global interrupt                                      */
    (uint32_t*) DMA2_Stream3_IRQHandler,      			/* DMA2 Stream3 global interrupt                                      */
    (uint32_t*) DMA2_Stream4_IRQHandler,      			/* DMA2 Stream4 global interrupt                                      */
    (uint32_t*) ETH_IRQHandler,               			/* Ethernet global interrupt                                          */
    (uint32_t*) ETH_WKUP_IRQHandler,          			/* Ethernet Wakeup through EXTI line interrupt                        */
    (uint32_t*) CAN2_TX_IRQHandler,           			/* CAN2 TX interrupts                                                 */
    (uint32_t*) CAN2_RX0_IRQHandler,          			/* CAN2 RX0 interrupts                                                */
    (uint32_t*) CAN2_RX1_IRQHandler,          			/* CAN2 RX1 interrupts                                                */
    (uint32_t*) CAN2_SCE_IRQHandler,          			/* CAN2 SCE interrupt                                                 */
    (uint32_t*) OTG_FS_IRQHandler,            			/* USB On The Go FS global interrupt                                  */
    (uint32_t*) DMA2_Stream5_IRQHandler,      			/* DMA2 Stream5 global interrupt                                      */
    (uint32_t*) DMA2_Stream6_IRQHandler,      			/* DMA2 Stream6 global interrupt                                      */
    (uint32_t*) DMA2_Stream7_IRQHandler,      			/* DMA2 Stream7 global interrupt                                      */
    (uint32_t*) USART6_IRQHandler,            			/* USART6 global interrupt                                            */
    (uint32_t*) I2C3_EV_IRQHandler,           			/* I2C3 event interrupt                                               */
    (uint32_t*) I2C3_ER_IRQHandler,           			/* I2C3 error interrupt                                               */
    (uint32_t*) OTG_HS_EP1_OUT_IRQHandler,    			/* USB On The Go HS End Point 1 Out global interrupt                  */
    (uint32_t*) OTG_HS_EP1_IN_IRQHandler,     			/* USB On The Go HS End Point 1 In global interrupt                   */
    (uint32_t*) OTG_HS_WKUP_IRQHandler,       			/* USB On The Go HS Wakeup through EXTI interrupt                     */
    (uint32_t*) OTG_HS_IRQHandler,            			/* USB On The Go HS global interrupt                                  */
    (uint32_t*) DCMI_IRQHandler,              			/* DCMI global interrupt                                              */
    (uint32_t*) CRYP_IRQHandler,              			/* CRYP crypto global interrupt                                       */
    (uint32_t*) HASH_RNG_IRQHandler,          			/* Hash and Rng global interrupt                                      */
    (uint32_t*) FPU_IRQHandler,               			/* FPU interrupt                                                      */
};


/******************************** End of file *********************************/


