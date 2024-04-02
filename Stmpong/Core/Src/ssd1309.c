#include "ssd1309.h"


/* Initialize SSD1306 on I2C2 */
void ssd1306_I2C_Init(void) {
    
    // Enable I2C2 Clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN

	// Set PB11 - AF1
	GPIOB->MODER |= (GPIO_MODER_MODER11_1);          
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_11);            
	GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL11_Pos); 
	
	// Set PB13 - AF5
	GPIOB->MODER |= (GPIO_MODER_MODER13_1);          
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_13);            
	GPIOB->AFR[1] |= (0x5 << GPIO_AFRH_AFSEL13_Pos); 
	
	// Set PB14 - initialize high
	GPIOB->MODER |= (GPIO_MODER_MODER14_0);              
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);  
	GPIOB->ODR |= (1<<14);	

	// Set PC0 - initialize high
	GPIOC->MODER |= (GPIO_MODER_MODER0_0);              
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_0);               
	GPIOC->ODR |= (1<<0);

    // Set I2C2 to 400Khz
	I2C2->TIMINGR |= (0x0  << I2C_TIMINGR_PRESC_Pos);  
	I2C2->TIMINGR |= (0x09 << I2C_TIMINGR_SCLL_Pos);   
	I2C2->TIMINGR |= (0x03 << I2C_TIMINGR_SCLH_Pos);   
	I2C2->TIMINGR |= (0x01  << I2C_TIMINGR_SDADEL_Pos); 
	I2C2->TIMINGR |= (0x03  << I2C_TIMINGR_SCLDEL_Pos); 
	
	
	I2C2->CR1 |= I2C_CR1_PE; 
	
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
}

void ssd1306_Init_Display() {
}

void SSD1306_WRITECOMMAND(uint8_t command){
	ssd1306_I2C_Write(display_addr, 0x00,  command);
}

void ssd1306_I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data) {
	
}