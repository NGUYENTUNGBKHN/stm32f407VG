/**
 * @file       lcd.h
 * @brief      
 * @date       2025/10/10
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    
 * @ref        
 * @copyright  Copyright (c) 2025 RoboTun
*/
#ifndef _LCD_H_
#define _LCD_H_
#ifdef __cplusplus
extern "C"
{
#endif

/* CODE */
#include "stm32f407xx.h"
#include "disco_bsp.h"
#include "reg_util.h"
#include "ili9341_reg.h"
#include "common.h"

#define LCD_WIDTH       240     /* 240 pixel */
#define LCD_HEIGHT      320     /* 320 pixel */

/* Select pixel format */
#define LCD_PIXEL_FMT_L8        1
#define LCD_PIXEL_FMT_RGB565    2
#define LCD_PIXEL_FMT_RGB666    3
#define LCD_PIXEL_FMT_RGB888    4
#define LCD_PIXEL_FMT           LCD_PIXEL_FMT_RGB565

/* Select orientation */
#define PORTRAIT            0
#define LANDSCAPE           1
#define LCD_ORIENTATION     LANDSCAPE

#if (LCD_ORIENTATION == PORTRAIT)
    #define LCD_ACTIVE_WIDTH        LCD_WIDTH
    #define LCD_ACTIVE_HEIGHT       LCD_HEIGHT
#elif(LCD_ORIENTATION == LANDSCAPE)
    #define LCD_ACTIVE_WIDTH        LCD_HEIGHT
    #define LCD_ACTIVE_HEIGHT       LCD_WIDTH
#endif 

#define AUTO            1
#define MANUAL          0
#define LCD_CS_MANAGE   MANUAL

#define USE_DMA 0

typedef struct
{
    uint16_t x1;
    uint16_t x2;
    uint16_t y1;
    uint16_t y2;
} lcd_area_t;

struct bsp_lcd;

typedef void (*bsp_lcd_dma_cplt_cb_t)(struct bsp_lcd*);
typedef void (*bsp_lcd_dma_err_cb_t)(struct bsp_lcd*);

typedef struct
{
    uint8_t orientation;
    uint8_t pixel_format;
    uint8_t *draw_buffer1;
    uint8_t *draw_buffer2;
    uint32_t write_length;
    uint8_t *buff_to_draw;
    uint8_t *buff_to_flush;
    lcd_area_t area;
    bsp_lcd_dma_cplt_cb_t dma_cplt_cb;
    bsp_lcd_dma_err_cb_t dma_err_cb;
} lcd_t;

typedef struct LCD_DRV_S lcd_drv_t;
struct LCD_DRV_S
{
    void (*init)(lcd_drv_t *self);
    void (*reset)(lcd_drv_t *self);
    void (*config)(lcd_drv_t *self);
};

extern void lcd_init();

#ifdef __cplusplus
}
#endif
#endif


