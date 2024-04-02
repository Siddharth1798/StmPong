/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "stm32f072xb.h"
#include "ssd1309.h"
#include "stdlib.h"
#include "string.h"

#define display_width = 128
#define display_height = 32




uint8_t SET_CONTRAST = (0x81);
uint8_t SET_ENTIRE_ON = (0xA4);
uint8_t SET_NORM_INV = (0xA6);
uint8_t SET_DISP = (0xAE);
uint8_t SET_MEM_ADDR = (0x20);
uint8_t SET_COL_ADDR = (0x21);
uint8_t SET_PAGE_ADDR = (0x22);
uint8_t SET_DISP_START_LINE = (0x40);
uint8_t SET_SEG_REMAP = (0xA0);
uint8_t SET_MUX_RATIO = (0xA8);
uint8_t SET_IREF_SELECT = (0xAD);
uint8_t SET_COM_OUT_DIR = (0xC0);
uint8_t SET_DISP_OFFSET = (0xD3);
uint8_t SET_COM_PIN_CFG = (0xDA);
uint8_t SET_DISP_CLK_DIV = (0xD5);
uint8_t SET_PRECHARGE = (0xD9);
uint8_t SET_VCOM_DESEL = (0xDB);
uint8_t SET_CHARGE_PUMP = (0x8D);





volatile uint8_t newdata = 0; 
volatile uint8_t led = NULL , operation = NULL;
volatile uint8_t display_addr = 0x3C<<1; 



void transmitToI2C2(uint8_t slvAddr, uint8_t noOfBits, uint8_t txdrData);
uint8_t receiveFromI2C2(uint8_t slvAddr, uint8_t noOfBits);
void initDisplay(void);
void initLEDs(void);
void initI2C2(void);
void part1(void);
void initGyro(void);
void senseGyro(void);
void SystemClock_Config(void);
void UART3_SendStr(char str[]);
static uint8_t SSD1306_Buffer[128 * 32 / 8];


void ssd1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t data);
void ssd1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count);

void SSD1306_WRITECOMMAND(uint8_t command){
	ssd1306_I2C_Write(display_addr, 0x00,  command);
}

void SSD1306_UpdateScreen(void) {
	UART3_SendStr("Updating Display");
	uint8_t m;
	
	for (m = 0; m < 8; m++) {
		SSD1306_WRITECOMMAND(0xB0 + 1);
		SSD1306_WRITECOMMAND(0x00);
		SSD1306_WRITECOMMAND(0x10);
		
		/* Write multi data */
		ssd1306_I2C_WriteMulti(display_addr, 0x40, &SSD1306_Buffer[128 * m], 128);
		GPIOC->ODR ^= (1<<7);
		HAL_Delay(200);
	}
}



void SSD1306_Fill(uint8_t color) {
	UART3_SendStr("Filling Display");
	int c = 128 * 32 / 8;
	for(int i=0;i<c;i++){
		SSD1306_Buffer[i] = 0xFF;
	}
	//memset(SSD1306_Buffer, (color == 0) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}



void UART3_SendStr(char str[]){
    uint8_t send = 0;
		while(str[send] != '\0'){
			if ((USART3->ISR & USART_ISR_TC) == USART_ISR_TC){
				USART3->TDR = str[send++];
			}
		}
		USART3->TDR = '\n';
	USART3->ICR |= USART_ICR_TCCF;
}

void USART3_4_IRQHandler(void){
	uint8_t rx_val = (uint8_t)(USART3->RDR); /* Receive data, clear flag */
		if(led == NULL && rx_val >=65 && rx_val <= 122){
			led = rx_val;
			UART3_SendStr("\nCMD:: ");
		}else if(rx_val >= 48 && rx_val <= 50){
		  operation	= rx_val;
			newdata = 1;
		}	 
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN | RCC_APB1ENR_USART3EN;
  RCC->AHBENR |= (RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOBEN);
	
	
	USART3->BRR |= 69;
	USART3->CR1 |= ( USART_CR1_TE | USART_CR1_UE);
	USART3->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE;
	
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn,1);

  GPIOC->MODER |= ((1<<9)|(1<<11));
	GPIOC->AFR[0] |= ((1<<16)|(1<<20));
	
		
	UART3_SendStr("hELLO\n");
	
	// initialize LEDs
	initLEDs();
	// initialize I2C2
	ssd1306_I2C_Init();
	//initI2C2();	
	
	initDisplay();
	
  while (1)
  {
  }
}







void transmitCmdToI2C2(uint8_t slvAddr , uint8_t reg , uint8_t data){
	// Clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	I2C2->CR2 |= (slvAddr << I2C_CR2_SADD_Pos);   	// Set the L3GD20 slave address = slvAddr
	I2C2->CR2 |= (0x2  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = noOfBits
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);        			// Set the RD_WRN to write operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          			// Set START bit
	
	
	
	// Wait until TXIS or NACKF flags are set
	while(1) {
		// Continue if TXIS flag is set
		if (I2C2->ISR & I2C_ISR_TXIS) {
			I2C2->TXDR = reg;
			break;
		}
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if (I2C2->ISR & I2C_ISR_NACKF) {
		}
	}
	
	
	// Wait again until TXIS or NACKF flags are set (2)
	while(1) {
		// Continue if TXIS flag is set
		if (I2C2->ISR & I2C_ISR_TXIS) {
			I2C2->TXDR = data;
			break;
		}
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if (I2C2->ISR & I2C_ISR_NACKF) {
		}
	}
	
	// Wait for TC flag is set
	/*while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			break;
		}	
	}*/
}










void initDisplay(void){
	UART3_SendStr("Initializing Display");
	SSD1306_WRITECOMMAND(0xAE); //display off
	SSD1306_WRITECOMMAND(0x20); //Set Memory Addressing Mode   
	SSD1306_WRITECOMMAND(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	SSD1306_WRITECOMMAND(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	SSD1306_WRITECOMMAND(0xC8); //Set COM Output Scan Direction
	SSD1306_WRITECOMMAND(0x00); //---set low column address
	SSD1306_WRITECOMMAND(0x10); //---set high column address
	SSD1306_WRITECOMMAND(0x40); //--set start line address
	SSD1306_WRITECOMMAND(0x81); //--set contrast control register
	SSD1306_WRITECOMMAND(0xFF);
	SSD1306_WRITECOMMAND(0xA1); //--set segment re-map 0 to 127
	SSD1306_WRITECOMMAND(0xA6); //--set normal display
	SSD1306_WRITECOMMAND(0xA8); //--set multiplex ratio(1 to 64)
	SSD1306_WRITECOMMAND(0x3F); //
	SSD1306_WRITECOMMAND(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	SSD1306_WRITECOMMAND(0xD3); //-set display offset
	SSD1306_WRITECOMMAND(0x00); //-not offset
	SSD1306_WRITECOMMAND(0xD5); //--set display clock divide ratio/oscillator frequency
	SSD1306_WRITECOMMAND(0xF0); //--set divide ratio
	SSD1306_WRITECOMMAND(0xD9); //--set pre-charge period
	SSD1306_WRITECOMMAND(0x22); //
	SSD1306_WRITECOMMAND(0xDA); //--set com pins hardware configuration
	SSD1306_WRITECOMMAND(0x12);
	SSD1306_WRITECOMMAND(0xDB); //--set vcomh
	SSD1306_WRITECOMMAND(0x20); //0x20,0.77xVcc
	SSD1306_WRITECOMMAND(0x8D); //--set DC-DC enable
	SSD1306_WRITECOMMAND(0x14); //
	//SSD1306_WRITECOMMAND(0xA5); //
	SSD1306_WRITECOMMAND(0xAF); //--turn on SSD1306 panel
	
	HAL_Delay(3000);
	SSD1306_WRITECOMMAND(0xA7);
	
	//SSD1306_Fill(1);
	//SSD1306_UpdateScreen();
	
}



/*void ssd1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	uint8_t i;
	transmitToI2C2(address, 0x1, reg);
	for (i = 0; i < count; i++) {
		transmitToI2C2(address, 0x1, data[i]);
		//USART3->TDR = data[i];
	}
	USART3->TDR = data[i];
	USART3->TDR = 0xFF;
}*/




void ssd1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	// Clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	I2C2->CR2 |= (address << I2C_CR2_SADD_Pos);   	// Set the L3GD20 slave address = slvAddr
	I2C2->CR2 |= (count  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = noOfBits
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);        			// Set the RD_WRN to write operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          			// Set START bit
	
	
	// Wait until TXIS or NACKF flags are set
	while(1) {
		// Continue if TXIS flag is set
		if (I2C2->ISR & I2C_ISR_TXIS) {
			I2C2->TXDR = reg;
			break;
		}
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if (I2C2->ISR & I2C_ISR_NACKF) {
		}
	}
	
	GPIOC->ODR ^= (1<<7);
	for(int j=0;j<count-5;j++){
		// Wait again until TXIS or NACKF flags are set (2)
		while(1) {
			// Continue if TXIS flag is set
			if (I2C2->ISR & I2C_ISR_TXIS) {
				I2C2->TXDR = data[j];
				break;
			}
			// Light ORANGE LED if NACKF flag is set (slave didn't respond)
			if (I2C2->ISR & I2C_ISR_NACKF) {
				
			}
		}
		GPIOC->ODR ^= (1<<9);
		HAL_Delay(100);
	}
	GPIOC->ODR ^= (1<<6);
	// Wait for TC flag is set
	/*while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			GPIOC->ODR |= (1<<7);
			HAL_Delay(200);
			break;
		}	
	}*/
	
}





void ssd1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t data) {
	// Clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	I2C2->CR2 |= (address << I2C_CR2_SADD_Pos);   	// Set the L3GD20 slave address = slvAddr
	I2C2->CR2 |= (0x2  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = noOfBits
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);        			// Set the RD_WRN to write operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          			// Set START bit
	
	
	
	// Wait until TXIS or NACKF flags are set
	while(1) {
		// Continue if TXIS flag is set
		if (I2C2->ISR & I2C_ISR_TXIS) {
			I2C2->TXDR = reg;
			break;
		}
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if (I2C2->ISR & I2C_ISR_NACKF) {
		}
	}
	
	
	// Wait again until TXIS or NACKF flags are set (2)
	while(1) {
		// Continue if TXIS flag is set
		if (I2C2->ISR & I2C_ISR_TXIS) {
			I2C2->TXDR = data;
			break;
		}
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if (I2C2->ISR & I2C_ISR_NACKF) {
		}
	}
	
	// Wait for TC flag is set
	/*while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			break;
		}	
	}*/
}


uint8_t receiveFromI2C2(uint8_t slvAddr, uint8_t noOfBits){
		
	// Clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	I2C2->CR2 |= (slvAddr << I2C_CR2_SADD_Pos);   	// Set the L3GD20 slave address = slvAddr
	I2C2->CR2 |= (noOfBits  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = noOfBits
	I2C2->CR2 |= (I2C_CR2_RD_WRN_Msk);         			// Set the RD_WRN to read operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          			// Set START bit
	
	// Wait until RXNE or NACKF flags are set
	while(1) {
		// Continue if RXNE flag is set
		if ((I2C2->ISR & I2C_ISR_RXNE)) {
			break;
		}
		
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if ((I2C2->ISR & I2C_ISR_NACKF)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			continue;
		}
	}
	// Wait for TC flag is set
	/*while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			break;
		}		
	}*/
	  
	return I2C2->RXDR;;
}


void transmitToI2C2(uint8_t slvAddr, uint8_t noOfBits, uint8_t txdrData){
	// Clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	I2C2->CR2 |= (slvAddr << I2C_CR2_SADD_Pos);   	// Set the L3GD20 slave address = slvAddr
	I2C2->CR2 |= (noOfBits  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = noOfBits
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);        			// Set the RD_WRN to write operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          			// Set START bit

	// Wait until TXIS or NACKF flags are set
	while(1) {
		// Continue if TXIS flag is set
		if ((I2C2->ISR & I2C_ISR_TXIS)) {
			I2C2->TXDR = txdrData; // Set I2C2->TXDR = txdrData
			//UART3_SendStr("DATA SENT");
			break;
		}
		
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if ((I2C2->ISR & I2C_ISR_NACKF)) {
			//UART3_SendStr("NACKF");
			continue;
		}
	}
	
	// Wait for TC flag is set
	/*while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			UART3_SendStr("TCFLAG");
			break;
		}
	}*/
	
}


/* Initialize all configured peripherals */
void initI2C2(void) {
	
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

	
	I2C2->TIMINGR |= (0x0  << I2C_TIMINGR_PRESC_Pos);  
	I2C2->TIMINGR |= (0x09 << I2C_TIMINGR_SCLL_Pos);   
	I2C2->TIMINGR |= (0x03 << I2C_TIMINGR_SCLH_Pos);   
	I2C2->TIMINGR |= (0x01  << I2C_TIMINGR_SDADEL_Pos); 
	I2C2->TIMINGR |= (0x03  << I2C_TIMINGR_SCLDEL_Pos); 
	
	
	I2C2->CR1 |= I2C_CR1_PE; 
	
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
}



/* Initialize the LEDs*/
void initLEDs(void) {	
	
	GPIOC->MODER |= ((1 << 12)|(1 << 14) | (1 << 16) | (1 << 18));
	GPIOC->MODER &= ~((1 << 13)|(1 << 15) | (1 << 17) | (1 << 19));
	
	GPIOC->OTYPER &= ~((1 << 6)|(1 << 7) | (1 << 8) | (1 << 9));
	
	GPIOC->OSPEEDR &= ~((1 << 12)|(1 << 14)|(1 << 16)|(1 << 18 ));
	
	GPIOC->PUPDR &= ~((1 << 13)|(1 << 15)|(1 << 12)|(1 << 14)|(1 << 16) | (1 << 18)|(1 << 17) | (1 << 19));
	
	GPIOC->ODR &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));

}


void part1(void) {
	UART3_SendStr("I2C2 Started::\n");
	//uint8_t receivedValue = receiveFromI2C2(0x3C, 0x1);
	
	transmitToI2C2(0x78, 0x1, 0xA5);
	transmitToI2C2(0x78, 0x1, 0xA7);
	transmitToI2C2(0x78, 0x1, 0xAF);
	/*transmitToI2C2(0x78, 0x1, 0xA5);
	transmitToI2C2(0x78, 0x1, 0xA5);
	transmitToI2C2(0x78, 0x1, 0xA5);
	transmitToI2C2(0x78, 0x1, 0xA5);
	transmitToI2C2(0x78, 0x1, 0xA5);
	transmitToI2C2(0x78, 0x1, 0xA5);
	transmitToI2C2(0x78, 0x1, 0xA5);*/
	

	
	
	I2C2->CR2 |= (I2C_CR2_STOP);
} 


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
