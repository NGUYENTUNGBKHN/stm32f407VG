/**
 * @file       lcd.c
 * @brief      
 * @date       2025/10/10
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    
 * @ref        
 * @copyright  Copyright (c) 2025 RoboTun
*/
/*******************************************************************************
**                                INCLUDES
*******************************************************************************/
#include "lcd.h"
/*******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define BARE_METAL

#define LCD_RESX_LOW()				    REG_CLR_BIT(LCD_RESX_PORT->ODR, LCD_RESX_PIN)
#define LCD_RESX_HIGH()				    REG_SET_BIT(LCD_RESX_PORT->ODR, LCD_RESX_PIN)

#define LCD_DCX_LOW()                   REG_CLR_BIT(LCD_DCX_PORT->ODR, LCD_DCX_PIN)
#define LCD_DCX_HIGH()                  REG_SET_BIT(LCD_DCX_PORT->ODR, LCD_DCX_PIN)

#define LCD_CSX_LOW()                   REG_CLR_BIT(LCD_CSX_PORT->ODR, LCD_CSX_PIN)
#define LCD_CSX_HIGH()                  REG_SET_BIT(LCD_CSX_PORT->ODR, LCD_CSX_PIN)

#define LCD_LED_LOW()                   REG_CLR_BIT(LCD_LED_PORT->ODR, LCD_LED_PIN)
#define LCD_LED_HIGH()                  REG_SET_BIT(LCD_LED_PORT->ODR, LCD_LED_PIN)

#define __disable_spi_ssm()           	REG_CLR_BIT(SPI->CR1,SPI_CR1_SSM_Pos)
#define __enable_spi_ssoe()				REG_SET_BIT(SPI->CR2,SPI_CR2_SSOE_Pos)
#define __spi_set_dff_8bit()  			REG_CLR_BIT(SPI->CR1,SPI_CR1_DFF_Pos)
#define __spi_set_dff_16bit()			REG_SET_BIT(SPI->CR1,SPI_CR1_DFF_Pos)
#define __enable_spi()					REG_SET_BIT(SPI->CR1,SPI_CR1_SPE_Pos)
#define __disable_spi()					do{while(REG_READ_BIT(SPI->SR,SPI_SR_BSY_Pos)); \
										   REG_CLR_BIT(SPI->CR1,SPI_CR1_SPE_Pos);}while(0)

#define __enable_irq()					do{\
												uint32_t priMask = 0;\
												__asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");}while(0)

#define __disable_irq()					do{\
												uint32_t priMask = 1;\
												__asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");}while(0)


#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

#define HIGH_16(x)     					((((uint16_t)x) >> 8U) & 0xFFU)
#define LOW_16(x)      					((((uint16_t)x) >> 0U) & 0xFFU)
/*******************************************************************************
**                      COMMON VARIABLE DEFINITIONS
*******************************************************************************/
extern const uint8_t t1_320x240_map[]; /* pixel byte array of frame 1 */

/*******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
SPI_HandleTypeDef m_spi;
uint32_t Spi2Timeout = 0x1000;    /*<! Value of Timeout when SPI communication fails */
/*******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void lcd_gpio_init();
static void lcd_spi_init();
static void lcd_reset();
static void lcd_config();
static void lcd_spi_enable();

void lcd_write_cmd(uint8_t cmd);
void lcd_write_data(uint8_t *buffer,uint32_t len);

void lcd_set_orientation(uint8_t orientation);
void lcd_set_display_area(lcd_area_t *area);
void lcd_buffer_init(lcd_t *lcd);
void lcd_set_background_color(uint32_t rgb888);
void bsp_lcd_set_display_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);


lcd_t lcd_handle;

lcd_t *hlcd = &lcd_handle;

#define DB_SIZE 					(10UL * 1024UL)
uint8_t bsp_db[DB_SIZE];
uint8_t bsp_wb[DB_SIZE];
/*******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
#define RGB888(r,g,b)  (((r) << 16) | ((g) << 8) | (b))
#define YELLOW   	RGB888(255,255,0)

void bsp_lcd_write(uint8_t *buffer, uint32_t nbytes)
{
	uint16_t *buff_ptr;

	__disable_spi();
	__spi_set_dff_16bit();
	__enable_spi();

	LCD_CSX_LOW();

	buff_ptr = (uint16_t*)buffer;
	while(nbytes){
		while(!REG_READ_BIT(SPI->SR,SPI_SR_TXE_Pos));
		REG_WRITE(SPI->DR,*buff_ptr);
		++buff_ptr;
		nbytes -= 2;
	}

	__disable_spi();
	LCD_CSX_HIGH();
	__spi_set_dff_8bit();
	__enable_spi();

}

void bsp_lcd_send_cmd_mem_write(void)
{
	lcd_write_cmd(ILI9341_GRAM);
}

void write_frame(uint8_t *fb_addr, uint32_t nbytes)
{
	bsp_lcd_set_display_area(0, LCD_ACTIVE_WIDTH-1, 0, LCD_ACTIVE_HEIGHT-1);
	bsp_lcd_send_cmd_mem_write();
	bsp_lcd_write(fb_addr, nbytes);
}

void lcd_init()
{
#if defined(BARE_METAL)
    lcd_gpio_init();
#endif 
    lcd_spi_init();
    lcd_spi_enable();
    lcd_handle.orientation = LCD_ORIENTATION;
	lcd_handle.pixel_format = LCD_PIXEL_FMT;
    lcd_reset();
    lcd_config();
    hlcd->area.x1 = 0;
	hlcd->area.x2 = LCD_ACTIVE_WIDTH-1;
	hlcd->area.y1 = 0;
	hlcd->area.y2 = LCD_ACTIVE_HEIGHT-1;
    lcd_set_display_area(&hlcd->area);
	lcd_set_orientation(hlcd->orientation);
	lcd_buffer_init(hlcd);
    HAL_Delay(10);
    lcd_set_background_color(YELLOW);
    while (1)
    {
        /* code */
        // write_frame((uint8_t*)t1_320x240_map, (320 * 240 * 2));
        // HAL_Delay(100);

    }
    
}
#if !defined(BARE_METAL)
static void lcd_cs_low()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
}

static void lcd_cs_high()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
}

static void lcd_reset_low()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

static void lcd_reset_high()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

static void lcd_dcx_low()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}

static void lcd_dcx_high()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}
#endif 
static void lcd_gpio_init()
{
#if defined(BARE_METAL)
    RCC_TypeDef *pRCC = RCC;

	/* Enable the clock for GPIOB, GPIOC, GPIOD  peripherals */
	REG_SET_BIT(pRCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Pos);
	REG_SET_BIT(pRCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Pos);
	REG_SET_BIT(pRCC->AHB1ENR, RCC_AHB1ENR_GPIODEN_Pos);

	/* RESET */
	REG_SET_VAL(LCD_RESX_PORT->MODER ,0x1U, 0x3, (LCD_RESX_PIN * 2U)); 			/*mode*/
	REG_CLR_BIT(LCD_RESX_PORT->OTYPER, LCD_RESX_PIN); 							/*Output type*/
	REG_SET_VAL(LCD_RESX_PORT->OSPEEDR, 2U, 0x3U, (LCD_RESX_PIN * 2U)); 		/*speed*/

	/* DC/SX */
	REG_SET_VAL(LCD_DCX_PORT->MODER, 0x1U, 0x3, (LCD_DCX_PIN * 2U)); 			/*mode*/
	REG_CLR_BIT(LCD_DCX_PORT->OTYPER, LCD_DCX_PIN); 							/*Output type*/
	REG_SET_VAL(LCD_DCX_PORT->OSPEEDR, 2U, 0x3U, (LCD_DCX_PIN * 2U)); 			/*speed*/

	/* SCL is SCK GPIOB 10 */
	REG_SET_VAL(LCD_SCL_PORT->MODER, 2U ,0x3U, (LCD_SCL_PIN * 2U));
	REG_CLR_BIT(LCD_SCL_PORT->OTYPER, LCD_SCL_PIN);
	REG_SET_VAL(LCD_SCL_PORT->OSPEEDR, 2U, 0x3U, (LCD_SCL_PIN * 2U));
	REG_SET_VAL(LCD_SCL_PORT->AFR[1], 5U, 0xFU, ((LCD_SCL_PIN %8) * 4U));

	/* SDI is MOSI GPIOBB 15 */
	REG_SET_VAL(LCD_SDI_PORT->MODER, 2U, 0x3U, (LCD_SDI_PIN * 2U));
	REG_CLR_BIT(LCD_SDI_PORT->OTYPER, LCD_SDI_PIN);
	REG_SET_VAL(LCD_SDI_PORT->OSPEEDR, 2U, 0x3U, (LCD_SDI_PIN * 2U));
	REG_SET_VAL(LCD_SDI_PORT->AFR[1], 5U, 0xFU, ((LCD_SDI_PIN % 8) * 4U));

	/* SDO is MISO GPIOC 2 */
	REG_SET_VAL(LCD_SDO_PORT->MODER, 2U, 0x3U, (LCD_SDO_PIN * 2U));
	REG_CLR_BIT(LCD_SDO_PORT->OTYPER, LCD_SDO_PIN);
	REG_SET_VAL(LCD_SDO_PORT->OSPEEDR, 2U, 0x3U, (LCD_SDO_PIN * 2U));
	REG_SET_VAL(LCD_SDO_PORT->AFR[0], 5U, 0xFU, (LCD_SDO_PIN * 4U));

	/* CS GPIOC 0 */
	REG_SET_VAL(LCD_CSX_PORT->MODER, 0x1U, 0x3, (LCD_CSX_PIN * 2U)); 		/*mode*/
	REG_CLR_BIT(LCD_CSX_PORT->OTYPER, LCD_CSX_PIN); 						/*Output type*/
	REG_SET_VAL(LCD_CSX_PORT->OSPEEDR, 2U, 0x3U, (LCD_CSX_PIN * 2U)); 		/*speed*/
	//CSX = HIGH
	REG_SET_BIT(LCD_CSX_PORT->ODR,LCD_CSX_PIN);


	//RESX = HIGH
	REG_SET_BIT(LCD_RESX_PORT->ODR,LCD_RESX_PIN);
	//D/CX = HIGH
	REG_SET_BIT(LCD_DCX_PORT->ODR,LCD_DCX_PIN);

#else
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable the SPI peripheral */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* SPI MOSI, MISO pin configuration */
    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
    /* SPI SCK */
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* SPI MOSI */
    GPIO_InitStructure.Pin = GPIO_PIN_15;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* RESET and DC and LED */
    GPIO_InitStructure.Pin = (GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

#endif 

}

static void lcd_spi_init()
{
#if defined(BARE_METAL)
    SPI_TypeDef *pSPI = SPI;
	RCC_TypeDef *pRCC = RCC;

	/* Enable CLK APB1 */
	REG_SET_BIT(pRCC->APB1ENR, RCC_APB1ENR_SPI2EN_Pos);

	/* 1. SET CPHA = 0 and CPOL = 0  */
	REG_CLR_BIT(pSPI->CR1, SPI_CR1_CPHA_Pos);
	REG_CLR_BIT(pSPI->CR1, SPI_CR1_CPOL_Pos);

	/* 2. MSTR Master selection */
	REG_SET_BIT(pSPI->CR1, SPI_CR1_MSTR_Pos);

	/* 3. Baud rate control - flck/2 42MHz/2  = 21MHz*/
	REG_SET_VAL(pSPI->CR1, 0x00U, 0x7U, SPI_CR1_BR_Pos);

	/* 4. LSBFIRST: Frame format */
	REG_CLR_BIT(pSPI->CR1, SPI_CR1_LSBFIRST_Pos);

	/* 5. SSM Software slave management
		  SSI Internal slave select */
	REG_SET_BIT(pSPI->CR1, SPI_CR1_SSM_Pos);
	REG_SET_BIT(pSPI->CR1, SPI_CR1_SSI_Pos);

	/* 6. RXONLY: Receive only */
	// REG_CLR_BIT(pSPI->CR1, SPI_CR1_RXONLY_Pos);

	/* 7. DFF: Data frame format set 8bit */
	REG_CLR_BIT(pSPI->CR1, SPI_CR1_DFF_Pos);

	/* 8. BIDIMODE: Bidirectional data mode enable */
	REG_CLR_BIT(pSPI->CR1, SPI_CR1_BIDIMODE_Pos);

	/* 9. FRF: Frame format set SPI Motorola mode */
	REG_CLR_BIT(pSPI->CR1, SPI_CR2_FRF_Pos);
#else
    GPIO_InitTypeDef GPIO_InitStructure;
    

    /* Configure the Accelerometer Control pins --------------------------------*/
    /* Enable CS GPIO clock and configure GPIO pin for Accelerometer Chip select */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    /* Configure GPIO PIN for LIS Chip select */
    GPIO_InitStructure.Pin = GPIO_PIN_0;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    lcd_cs_high();
    lcd_dcx_high();

    if(HAL_SPI_GetState(&m_spi) == HAL_SPI_STATE_RESET)
    {
        /* SPI configuration -----------------------------------------------------*/
        m_spi.Instance = SPI2;
        m_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        m_spi.Init.Direction = SPI_DIRECTION_2LINES;
        m_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
        m_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
        m_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
        m_spi.Init.CRCPolynomial = 10;
        m_spi.Init.DataSize = SPI_DATASIZE_8BIT;
        m_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
        m_spi.Init.NSS = SPI_NSS_SOFT;
        m_spi.Init.TIMode = SPI_TIMODE_DISABLED;
        m_spi.Init.Mode = SPI_MODE_MASTER;

        lcd_gpio_init();
        HAL_SPI_Init(&m_spi);
        
    }
#endif 
}

static void lcd_config()
{
#if defined(BARE_METAL)
    uint8_t params[15];
	/* 1. Reset software command */
	lcd_write_cmd(ILI9341_SWRESET);

	lcd_write_cmd(ILI9341_POWERB);
	params[0] = 0x00;
	params[1] = 0xD9;
	params[2] = 0x30;
	lcd_write_data(params, 3);

	lcd_write_cmd(ILI9341_POWER_SEQ);
	params[0]= 0x64;
	params[1]= 0x03;
	params[2]= 0X12;
	params[3]= 0X81;
	lcd_write_data(params, 4);

	lcd_write_cmd(ILI9341_DTCA);
	params[0]= 0x85;
	params[1]= 0x10;
	params[2]= 0x7A;
	lcd_write_data(params, 3);

	lcd_write_cmd(ILI9341_POWERA);
	params[0]= 0x39;
	params[1]= 0x2C;
	params[2]= 0x00;
	params[3]= 0x34;
	params[4]= 0x02;
	lcd_write_data(params, 5);

	lcd_write_cmd(ILI9341_PRC);
	params[0]= 0x20;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_DTCB);
	params[0]= 0x00;
	params[1]= 0x00;
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_POWER1);
	params[0]= 0x1B;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_POWER2);
	params[0]= 0x12;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_VCOM1);
	params[0]= 0x08;
	params[1]= 0x26;
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_VCOM2);
	params[0]= 0XB7;
	lcd_write_data(params, 1);


	lcd_write_cmd(ILI9341_PIXEL_FORMAT);
	params[0]= 0x55; //select RGB565
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_FRMCTR1);
	params[0]= 0x00;
	params[1]= 0x1B;//frame rate = 70
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_DFC);    // Display Function Control
	params[0]= 0x0A;
	params[1]= 0xA2;
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_3GAMMA_EN);    // 3Gamma Function Disable
	params[0]= 0x02; //LCD_WR_DATA(0x00);
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_GAMMA);
	params[0]= 0x01;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_PGAMMA);    //Set Gamma
	params[0]= 0x0F;
	params[1]= 0x1D;
	params[2]= 0x1A;
	params[3]= 0x0A;
	params[4]= 0x0D;
	params[5]= 0x07;
	params[6]= 0x49;
	params[7]= 0X66;
	params[8]= 0x3B;
	params[9]= 0x07;
	params[10]= 0x11;
	params[11]= 0x01;
	params[12]= 0x09;
	params[13]= 0x05;
	params[14]= 0x04;
	lcd_write_data(params, 15);

	lcd_write_cmd(ILI9341_NGAMMA);
	params[0]= 0x00;
	params[1]= 0x18;
	params[2]= 0x1D;
	params[3]= 0x02;
	params[4]= 0x0F;
	params[5]= 0x04;
	params[6]= 0x36;
	params[7]= 0x13;
	params[8]= 0x4C;
	params[9]= 0x07;
	params[10]= 0x13;
	params[11]= 0x0F;
	params[12]= 0x2E;
	params[13]= 0x2F;
	params[14]= 0x05;
	lcd_write_data(params, 15);

	lcd_write_cmd(ILI9341_SLEEP_OUT); //Exit Sleep
	HAL_Delay(100);
	lcd_write_cmd(ILI9341_DISPLAY_ON); //display on
#else
    lcd_write_cmd(ILI9341_SWRESET);
#endif 
}

void lcd_write_cmd(uint8_t cmd)
{
#if defined(BARE_METAL)
	SPI_TypeDef *pSPI = SPI2;
	LCD_CSX_LOW();
	LCD_DCX_LOW(); //DCX = 0 , for command
	while(!REG_READ_BIT(pSPI->SR,SPI_SR_TXE_Pos));
	REG_WRITE(pSPI->DR,cmd);
	while(!REG_READ_BIT(pSPI->SR,SPI_SR_TXE_Pos));
	while(REG_READ_BIT(pSPI->SR,SPI_SR_BSY_Pos));
	LCD_DCX_HIGH();
	LCD_CSX_HIGH();
#else
    lcd_cs_low();
    lcd_dcx_low();
    if(HAL_SPI_Transmit(&m_spi, &cmd, 1, Spi2Timeout) != HAL_OK)
    {
        ERROR("SPI send fail.\n");
    }
    lcd_dcx_high();
    lcd_cs_high();
#endif 
}

void bsp_lcd_set_display_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    lcd_area_t area;
    area.x1 = x1;
    area.x2 = x2;
    area.y1 = y1;
    area.y2 = y2;
    lcd_set_display_area(&area);
}

void lcd_set_display_area(lcd_area_t *area)
{
	uint8_t params[4];
	/*Column address set(2Ah) */
	params[0] = HIGH_16(area->x1);
	params[1] = LOW_16(area->x1);
	params[2] = HIGH_16(area->x2);
	params[3] = LOW_16(area->x2);
	lcd_write_cmd(ILI9341_CASET);
	lcd_write_data(params, 4);

	params[0] = HIGH_16(area->y1);
	params[1] = LOW_16(area->y1);
	params[2] = HIGH_16(area->y2);
	params[3] = LOW_16(area->y2);
	lcd_write_cmd(ILI9341_RASET);
	lcd_write_data(params, 4);

}

void lcd_buffer_init(lcd_t *lcd)
{
	lcd->draw_buffer1 = bsp_db;
	lcd->draw_buffer2 = bsp_wb;
	lcd->buff_to_draw = NULL;
	lcd->buff_to_flush = NULL;
}

void lcd_set_orientation(uint8_t orientation)
{
	uint8_t param = 0;

	if(orientation == LANDSCAPE)
    {
		param = MADCTL_MV | MADCTL_MY | MADCTL_BGR; /*Memory Access Control <Landscape setting>*/
	}else if(orientation == PORTRAIT){
		param = MADCTL_MY| MADCTL_MX| MADCTL_BGR;  /* Memory Access Control <portrait setting> */
	}

	lcd_write_cmd(ILI9341_MAC);    // Memory Access Control command
	lcd_write_data(&param, 1);
}

uint16_t convert_rgb888_to_rgb565(uint32_t rgb888)
{
    uint16_t r,g,b;
	r = (rgb888 >> 19) & 0x1FU;
	g = (rgb888 >> 10) & 0x3FU;
	b = (rgb888 >> 3)  & 0x1FU;
	return (uint16_t)((r << 11) | (g << 5) | b);
}

uint32_t bytes_to_pixels(uint32_t nbytes, uint8_t pixel_format)
{
	UNUSED(pixel_format);
	return nbytes/2;

}

uint32_t pixels_to_bytes(uint32_t pixels, uint8_t pixel_format)
{
	UNUSED(pixel_format);
	return pixels * 2UL;
}

 void lcd_flush(lcd_t *hlcd)
{
	lcd_set_display_area(&hlcd->area);
	bsp_lcd_send_cmd_mem_write();
#if (USE_DMA == 0)
	bsp_lcd_write(hlcd->buff_to_flush,hlcd->write_length);
	hlcd->buff_to_flush = NULL;
#else
	lcd_write_dma((uint32_t)hlcd->buff_to_flush,hlcd->write_length);
#endif
}

uint8_t *get_buff(lcd_t *hlcd)
{
	uint32_t buf1 = (uint32_t)hlcd->draw_buffer1;
	uint32_t buf2 = (uint32_t)hlcd->draw_buffer2;

	__disable_irq();
	if(hlcd->buff_to_draw == NULL && hlcd->buff_to_flush == NULL){
		return  hlcd->draw_buffer1;
	}else if((uint32_t)hlcd->buff_to_flush == buf1 && hlcd->buff_to_draw == NULL ){
		return  hlcd->draw_buffer2;
	}else if ((uint32_t)hlcd->buff_to_flush == buf2 && hlcd->buff_to_draw == NULL){
		return  hlcd->draw_buffer1;
	}
	__enable_irq();

	return NULL;
}

static uint8_t is_lcd_write_allowed(lcd_t *hlcd)
{
	__disable_irq();
	if(!hlcd->buff_to_flush)
		return 1;
	__enable_irq();

	return 0;
}

uint32_t copy_to_draw_buffer( lcd_t *hlcd,uint32_t nbytes,uint32_t rgb888)
{
	uint16_t *fb_ptr = NULL;
	uint32_t npixels;
	hlcd->buff_to_draw = get_buff(hlcd);
	fb_ptr = (uint16_t*)hlcd->buff_to_draw;
	nbytes =  ((nbytes > DB_SIZE)?DB_SIZE:nbytes);
	npixels= bytes_to_pixels(nbytes,hlcd->pixel_format);
	if(hlcd->buff_to_draw != NULL)
	{
		for(uint32_t i = 0 ; i < npixels ;i++){
			*fb_ptr = convert_rgb888_to_rgb565(rgb888);
			fb_ptr++;
		}
		hlcd->write_length = pixels_to_bytes(npixels,hlcd->pixel_format);
		while(!is_lcd_write_allowed(hlcd));
		hlcd->buff_to_flush = hlcd->buff_to_draw;
		hlcd->buff_to_draw = NULL;
		lcd_flush(hlcd);
		return pixels_to_bytes(npixels,hlcd->pixel_format);
	}

	return 0;
}

void make_area(lcd_area_t *area,uint32_t x_start, uint32_t x_width,uint32_t y_start,uint32_t y_height){

	uint16_t lcd_total_width,lcd_total_height;

	lcd_total_width =  LCD_ACTIVE_WIDTH-1;
	lcd_total_height = LCD_ACTIVE_HEIGHT -1;

	area->x1 = x_start;
	area->x2 = x_start + x_width -1;
	area->y1 = y_start;
	area->y2 = y_start + y_height -1;

	area->x2 = (area->x2 > lcd_total_width) ? lcd_total_width :area->x2;
	area->y2 = (area->y2 > lcd_total_height) ? lcd_total_height : area->y2;

}

uint32_t get_total_bytes(lcd_t *hlcd, uint32_t w , uint32_t h)
{
	uint8_t bytes_per_pixel = 2;
	if(hlcd->pixel_format == LCD_PIXEL_FMT_RGB565)
	{
		bytes_per_pixel = 2;
	}
	return (w * h * bytes_per_pixel);
}

void lcd_fill_rect(uint32_t rgb888, uint32_t x_start, uint32_t x_width, uint32_t y_start, uint32_t y_height)
{

    uint32_t total_bytes_to_write = 0;
    uint32_t bytes_sent_so_far = 0;
    uint32_t remaining_bytes = 0;

    uint32_t npix;
    uint32_t pixels_sent = 0;
    uint32_t x1, y1;
    uint32_t pixel_per_line = x_width;

    if ((x_start + x_width) > LCD_ACTIVE_WIDTH)
        return;
    if ((y_start + y_height) > LCD_ACTIVE_HEIGHT)
        return;

    // 1. calculate total number of bytes written in to DB
    total_bytes_to_write = get_total_bytes(hlcd, x_width, y_height);
    remaining_bytes = total_bytes_to_write;
    while (remaining_bytes)
    {
        x1 = x_start + (pixels_sent % pixel_per_line);
        y1 = y_start + (pixels_sent / pixel_per_line);

        make_area(&hlcd->area, x1, x_width, y1, y_height);

        if (x1 != x_start)
        {
            npix = x_start + x_width - x1;
        }
        else
        {
            npix = bytes_to_pixels(remaining_bytes, hlcd->pixel_format);
        }

        bytes_sent_so_far += copy_to_draw_buffer(hlcd, pixels_to_bytes(npix, hlcd->pixel_format), rgb888);
        pixels_sent = bytes_to_pixels(bytes_sent_so_far, hlcd->pixel_format);
        remaining_bytes = total_bytes_to_write - bytes_sent_so_far;
    }
}

void lcd_set_background_color(uint32_t rgb888)
{
	lcd_fill_rect(rgb888,0,(LCD_ACTIVE_WIDTH),0,(LCD_ACTIVE_HEIGHT));
}

 void lcd_write_data(uint8_t *buffer,uint32_t len)
{
#if defined(BARE_METAL)
	SPI_TypeDef *pSPI = SPI2;
	LCD_CSX_LOW();
	for(uint32_t i = 0 ; i < len ;i++){
		while(!REG_READ_BIT(pSPI->SR,SPI_SR_TXE_Pos));
		REG_WRITE(pSPI->DR,buffer[i]);
	}
	while(!REG_READ_BIT(pSPI->SR,SPI_SR_TXE_Pos));
	while(REG_READ_BIT(pSPI->SR,SPI_SR_BSY_Pos));
	LCD_CSX_HIGH();
#else
#endif 
}

static void lcd_spi_enable()
{
#if defined(BARE_METAL)
	/* Enable SPI module */
    REG_SET_BIT(SPI2->CR1, SPI_CR1_SPE_Pos);
#endif 
}


static void lcd_reset()
{
#if defined(BARE_METAL)
    LCD_RESX_LOW();
    HAL_Delay(50);
    LCD_RESX_HIGH();
    HAL_Delay(50);
#else
    lcd_reset_low();
    HAL_Delay(50);
    lcd_reset_high();
    HAL_Delay(50);
#endif
}

/******************************** End of file *********************************/

