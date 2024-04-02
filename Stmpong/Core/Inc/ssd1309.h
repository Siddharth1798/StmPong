#ifndef SSD1306_H
#define SSD1306_H 100


#include "stm32f072xb.h"
#include "stdlib.h"
#include "string.h"



/* I2C address */
#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR         0x78
#endif

/* SSD1306 settings */
/* SSD1306 width in pixels */
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH            128
#endif
/* SSD1306 LCD height in pixels */
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT           64
#endif

void ssd1306_I2C_Init(void);