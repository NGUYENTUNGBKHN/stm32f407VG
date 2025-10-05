/**
 * @file       lis3dsh.c
 * @brief      
 * @date       2025/10/04
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    
 * @ref        
 * @copyright  Copyright (c) 2025 RoboTun
*/
/*******************************************************************************
**                                INCLUDES
*******************************************************************************/
#include "lis3dsh.h"
#include <stdlib.h>
#include "string.h"
/*******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/


/*******************************************************************************
**                      COMMON VARIABLE DEFINITIONS
*******************************************************************************/
#define ADDRCMD_MASTER_READ                         ((uint16_t)0x1234)
#define ADDRCMD_MASTER_WRITE                        ((uint16_t)0x5678)
#define CMD_LENGTH                                  ((uint16_t)0x0004)
#define DATA_LENGTH                                 ((uint16_t)0x0020)

#undef INTER_USE

#define SPI_MASTER_SYNBYTE                          0xAC
#define lis3dsh_cs_low()          HAL_GPIO_WritePin(ACCE_port, ACCE_CS, GPIO_PIN_RESET);
#define lis3dsh_cs_high()           HAL_GPIO_WritePin(ACCE_port, ACCE_CS, GPIO_PIN_SET);
/*******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/


/*******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
uint8_t addrcmd[CMD_LENGTH] = {0};
lis3dsh_t *lis;
uint32_t SpixTimeout = 0x1000;    /*<! Value of Timeout when SPI communication fails */
/*******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/


void SPIx_MspInit()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable the SPI peripheral */
    DISCO_SPIx_CLK_Enable();

    /* Enable SCK, MOSI and MISO GPIO clocks */
    DISCO_SPIx_GPIO_CLK_Enable();

    /* SPI SCK, MOSI, MISO pin configuration */
    GPIO_InitStructure.Pin = (DISCO_SPIx_SCK | DISCO_SPIx_MISO | DISCO_SPIx_MOSI);
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Alternate = DISCO_SPIx_AF;
    HAL_GPIO_Init(DISCO_SPIx_port, &GPIO_InitStructure);

#if defined(INTER_USE)
    HAL_NVIC_SetPriority((IRQn_Type)DISCO_SPIx_IRQn, 0x01, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)DISCO_SPIx_IRQn);
#endif 
}

static void lis3dsh_init(struct LIS3DSH_S *self)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure the Accelerometer Control pins --------------------------------*/
    /* Enable CS GPIO clock and configure GPIO pin for Accelerometer Chip select */
    ACCE_GPIO_CLK_Enable();

    /* Configure GPIO PIN for LIS Chip select */
    GPIO_InitStructure.Pin = ACCE_CS;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(ACCE_port, &GPIO_InitStructure);

    lis3dsh_cs_high();
    HAL_Delay(100);
    // lis3dsh_cs_low();
    // HAL_Delay(100);
    // lis3dsh_cs_high();

    if(HAL_SPI_GetState(&self->SpiHandle) == HAL_SPI_STATE_RESET)
    {
        /* SPI configuration -----------------------------------------------------*/
        self->SpiHandle.Instance = DISCO_SPIx;
        self->SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        self->SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
        self->SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
        self->SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
        self->SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
        self->SpiHandle.Init.CRCPolynomial = 10;
        self->SpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
        self->SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
        self->SpiHandle.Init.NSS = SPI_NSS_SOFT;
        self->SpiHandle.Init.TIMode = SPI_TIMODE_DISABLED;
        self->SpiHandle.Init.Mode = SPI_MODE_MASTER;

        SPIx_MspInit();
        HAL_SPI_Init(&self->SpiHandle);
        
    }
}

static void lis3sh_write(struct LIS3DSH_S *self, uint8_t addr, uint8_t data)
{
    lis3dsh_cs_low();
#if defined(INTER_USE)
    if(HAL_SPI_Transmit_IT(&self->SpiHandle, &addr, 1) != HAL_OK)
    {
        ERROR("SPI send fail.\n");
    }
    while(HAL_SPI_GetState(&self->SpiHandle) != HAL_SPI_STATE_READY)
    {}

    if(HAL_SPI_Transmit_IT(&self->SpiHandle, &data, 1) != HAL_OK)
    {
        ERROR("SPI send fail.\n");
    }
    while(HAL_SPI_GetState(&self->SpiHandle) != HAL_SPI_STATE_READY)
    {}
#else
    if(HAL_SPI_Transmit(&self->SpiHandle, &addr, 1, SpixTimeout) != HAL_OK)
    {
        ERROR("SPI send fail.\n");
    }

    if(HAL_SPI_Transmit(&self->SpiHandle, &data, 1, SpixTimeout) != HAL_OK)
    {
        ERROR("SPI send fail.\n");
    }
#endif 
    lis3dsh_cs_high();
}

static void lis3dsh_read(struct LIS3DSH_S *self, uint8_t addr, uint8_t *data)
{
    uint8_t test = 0;
    test = addr | 0x80;
    lis3dsh_cs_low();
#if defined(INTER_USE)
    if (HAL_SPI_TransmitReceive_IT(&self->SpiHandle, (uint8_t*) &test, (uint8_t*) data, 1) != HAL_OK)
    {
        ERROR("SPI received fail.\n");
    }
    while(HAL_SPI_GetState(&self->SpiHandle) != HAL_SPI_STATE_READY)
    {}
#else
    if(HAL_SPI_Transmit(&self->SpiHandle, &test, 1, SpixTimeout) != HAL_OK)
    {
        ERROR("SPI send fail.\n");
    }

    if(HAL_SPI_Receive(&self->SpiHandle, data, 1, SpixTimeout) != HAL_OK)
    {
        ERROR("SPI send fail.\n");
    }
#endif
    lis3dsh_cs_high();
}

lis3dsh_t *lis3dsh_create()
{
    lis = (lis3dsh_t*)malloc(sizeof(lis3dsh_t));

    if (lis == NULL)
    {
        return NULL;
    }
    memset(lis, 0, sizeof(lis3dsh_t));

    lis->init = lis3dsh_init;
    lis->write = lis3sh_write;
    lis->read = lis3dsh_read;
    
    return lis;
}

/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void DISCO_SPIx_IRQn_Handler(void)
{
    HAL_SPI_IRQHandler(&lis->SpiHandle);
}

/******************************** End of file *********************************/


