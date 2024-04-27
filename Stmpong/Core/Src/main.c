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
void initBuzzer(void);
void initButtons(void);	
void drawPaddle(void);
void startSequence(void);
int dir = -3;
int A = 1103515245,C = 12345;
long long int M = 2147483648;
int generateRandom(int min, int max) {
    static unsigned int seed = 0;
    seed = (A * seed + C) % M;
    return min + (seed % (max - min + 1));
}
int x = 63,y=32,x1=-1,y1=-1,Px=5,Py=32,Pheight=8, maxSpeed=2,point=3, gameStatus = -1, gameComplete = 0;

int ifHitPaddle(int x, int y) {
		if((x-3<=Px+2)&&(y<Py+8)&&(y>Py-8)&&(x1<0)){
			GPIOC->ODR ^= (1<<7);
			point++;
			if(point>2){
				maxSpeed=3;
			}
			if(point>9){
				SSD1306_Fill(0);
				SSD1306_GotoXY(40,27);
				SSD1306_Puts("YOU WON", NULL, 1);   
				gameComplete = 1;
				gameStatus = 1;
			}
			return 1;
	  }
		if(((y>Py+6)||(y<Py-6))&&(x-3<=3)&&(x1<0)
		){
			point--;
			if(point<1){
				gameComplete = 1;
				gameStatus = 0;
			}
			return 1;
		}
		return 0;
}

void EXTI4_15_IRQHandler(void){
	
	GPIOC->ODR ^= (1<<6);
	if (EXTI->PR & (1<<4)){
		//Erase The Current Paddle
		SSD1306_DrawLine(Px-1,Py-Pheight,Px-1,Py+Pheight,0);
		SSD1306_DrawLine(Px,Py-Pheight,Px,Py+Pheight,0);
		//Decrement The Paddle Position
		Py=Py-8;
		if((Py-8)<=0){
			Py=8;
		}
		//Draw New Paddle
		SSD1306_DrawLine(Px-1,Py-Pheight,Px-1,Py+Pheight,1);
		SSD1306_DrawLine(Px,Py-Pheight,Px,Py+Pheight,1);
		EXTI->PR |= (1<<4);		// Clearing the pending register
	}
	if (EXTI->PR & (1<<5)){
		//Erase The Current Paddle
		SSD1306_DrawLine(Px-1,Py-Pheight,Px-1,Py+Pheight,0);
		SSD1306_DrawLine(Px,Py-Pheight,Px,Py+Pheight,0);
		//Increment The Paddle Position
		Py=Py+8;
		if((Py+8)>=64){
			Py=56;
		}
		//Draw New Paddle
		SSD1306_DrawLine(Px-1,Py-Pheight,Px-1,Py+Pheight,1);
		SSD1306_DrawLine(Px,Py-Pheight,Px,Py+Pheight,1);
		EXTI->PR |= (1<<5);   // Clearing the pending register
	}
}

void EXTI0_1_IRQHandler(void){
	if (EXTI->PR & (1<<0)){
		//Erase The Current Paddle
		SSD1306_DrawLine(Px-1,Py-Pheight,Px-1,Py+Pheight,0);
		SSD1306_DrawLine(Px,Py-Pheight,Px,Py+Pheight,0);
		//Increment The Paddle Position
		Py=Py+8;
		if((Py+8)>=64){
			Py=56;
		}
		//Draw New Paddle
		SSD1306_DrawLine(Px-1,Py-Pheight,Px-1,Py+Pheight,1);
		SSD1306_DrawLine(Px,Py-Pheight,Px,Py+Pheight,1);
		EXTI->PR |= (1<<0);   // Clearing the pending register
	}
}

volatile uint8_t isUpdate = 0; 

void TIM2_IRQHandler(void){
	GPIOC->ODR ^= (1 << 8)|(1 << 9);		//Timer 2 HeartBeat
	SSD1306_DrawCircle(x-x1,y-y1,2,0);	//Erase The Previous Ball
	SSD1306_DrawCircle(x,y,2,1);				//Draw New Ball
	drawPaddle();												//Draw Paddle
	//Change The Bounce Angle And Speed Randomly
	if(y>=64-3){												
		y1=-generateRandom(2,maxSpeed);
	}else if(y<=4){
		y1=generateRandom(2,maxSpeed);
	}
	y=y+y1;
	if(x>=128-5){
		x1=-generateRandom(2,maxSpeed);
	}else if((x<=3) || ifHitPaddle(x,y)){
		x1=generateRandom(2,maxSpeed);
	}
	x=x+x1; 
	
	//Print The Score
	SSD1306_GotoXY(59,50);
	SSD1306_Putc('0'+point,NULL,1);
	if(!gameComplete){
		isUpdate = 1;
	}
	TIM2->SR &= ~(1<<0);		//Clear The Interrupt Status Register Bit
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN | RCC_APB1ENR_USART3EN | RCC_APB1ENR_TIM2EN;		//Enable APB1ENR Periphs
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN;			//Enable APBENR Periphs
	
	//Enable Timer 2 Interrupt At 30Hz
	TIM2->PSC = 2666;		//Set PreScaler to 2666
	TIM2->ARR = 100;		//Set ARR to 100
	
	TIM2->DIER |= (1 << 0); 
	
	NVIC_SetPriority(TIM2_IRQn, 1);		//Set Priority Of The Timer 2 to 1
	TIM2->EGR |= (1<<0);
	TIM2->CR1 |= (1<<0);
	
	//Set Up USART3 For Debugging 
	USART3->BRR |= 69;	//Set Baud Rate To 115200
	USART3->CR1 |= ( USART_CR1_TE | USART_CR1_UE);
	USART3->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE;
	
	NVIC_EnableIRQ(USART3_4_IRQn);			//Enable USART3 Interrupt
	NVIC_SetPriority(USART3_4_IRQn,1);	//Set Priority Of The USART to 1

  GPIOC->MODER |= ((1<<9)|(1<<11));
	GPIOC->AFR[0] |= ((1<<16)|(1<<20));
			
	initLEDs();				//Init LEDs For Debugging
	initButtons();		//Init Buttons With Interrupt
	ssd1306_Init_Display();		//Init Oled Display
	
	startSequence();		//Display Start Sequence
	NVIC_EnableIRQ( TIM2_IRQn );			//Enable Timer 2 Interrupt

	drawPaddle();							//Draw Paddle
	SSD1306_UpdateScreen();		//Update Screen
	GPIOC->ODR |= (1<<2);			
  while (1)
  {
		if(isUpdate==1 && gameComplete==0){
			SSD1306_UpdateScreen();		//Updating Screen Every 30Hz
			isUpdate = 0;
		}else if(gameComplete == 1){
			NVIC_DisableIRQ( TIM2_IRQn );
			SSD1306_Fill(0);
			//Print Game Status
			if(gameStatus == 1){
				SSD1306_GotoXY(40,27);
				SSD1306_Puts("YOU WON", NULL, 1);
			}else{
				SSD1306_GotoXY(33,27);
				SSD1306_Puts("GAME OVER", NULL, 1);
			}
			SSD1306_UpdateScreen();				//Update Screen
			gameComplete = 0;
		}
  }
}

/* Start Sequence For Game */
void startSequence(void){
		SSD1306_GotoXY(61,28);
		SSD1306_Puts("3", NULL, 1);
		SSD1306_UpdateScreen();
		SSD1306_Fill(0);
		HAL_Delay(1000);
		SSD1306_GotoXY(61,28);
		SSD1306_Puts("2", NULL, 1);
		SSD1306_UpdateScreen();
		SSD1306_Fill(0);
		HAL_Delay(1000);
		SSD1306_GotoXY(61,28);
		SSD1306_Puts("1", NULL, 1);
		SSD1306_UpdateScreen();
		SSD1306_Fill(0);
		HAL_Delay(1000);
		SSD1306_GotoXY(46,28);
		SSD1306_Puts("START", NULL, 1);
		SSD1306_UpdateScreen();
		SSD1306_Fill(0);
		HAL_Delay(1000);
}

//Method To Draw Paddle 
void drawPaddle(void){
	SSD1306_DrawLine(Px-1,Py-Pheight,Px-1,Py+Pheight,1);
	SSD1306_DrawLine(Px,Py-Pheight,Px,Py+Pheight,1);
}



/* Initialize The LEDs*/
void initLEDs(void) {	
	
	GPIOC->MODER |= ((1 << 12)|(1 << 14) | (1 << 16) | (1 << 18));
	GPIOC->MODER &= ~((1 << 13)|(1 << 15) | (1 << 17) | (1 << 19));
	
	GPIOC->OTYPER &= ~((1 << 6)|(1 << 7) | (1 << 8) | (1 << 9));
	
	GPIOC->OSPEEDR &= ~((1 << 12)|(1 << 14)|(1 << 16)|(1 << 18 ));
	
	GPIOC->PUPDR &= ~((1 << 13)|(1 << 15)|(1 << 12)|(1 << 14)|(1 << 16) | (1 << 18)|(1 << 17) | (1 << 19));
	
	GPIOC->ODR &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));

}

/*Inialize The Buttons*/
void initButtons(void){
	
	// Setting up rising edge interrupt for PA4
	GPIOA->MODER &= ~((1 << 8) & (1 << 9)); // Setting PA4 as input
	GPIOA->OSPEEDR &= ~((1 << 8)); // Setting PA4 to low speed
	GPIOA->PUPDR &= ~((1 << 8));   // Enabling Pull Down resistor for PA4
	GPIOA->PUPDR |= ((1 << 9));
	
	EXTI->IMR |= (1 << 4); 
	EXTI->FTSR &= ~(1 << 4);
	EXTI->RTSR |= (1 << 4);
	SYSCFG->EXTICR[0] = 0;
	
	
	// Setting up rising edge interrupt for PA7
	GPIOA->MODER &= ~((1 << 14) & (1 << 15)); // Setting PA7 as input
	GPIOA->OSPEEDR &= ~((1 << 14)); // Setting PA7 to low speed
	GPIOA->PUPDR &= ~((1 << 14));   // Enabling Pull Down resistor for PA7
	GPIOA->PUPDR |= ((1 << 15));
	
	EXTI->IMR |= (1 << 7); 
	EXTI->FTSR &= ~(1 << 7);
	EXTI->RTSR |= (1 << 7);
	/*SYSCFG->EXTICR[1] = 0;
	SYSCFG->EXTICR[2] = 0;
	SYSCFG->EXTICR[3] = 0;*/
	
	
	
	GPIOA->MODER &= ~((1 << 0) & (1 << 1)); // Setting PC0 as input
	GPIOA->OSPEEDR &= ~((1 << 0)); // Setting PC0 to low speed
	GPIOA->PUPDR &= ~((1 << 0));   // Enabling Pull Down resistor for PA0
	GPIOA->PUPDR |= ((1 << 1));
	
	
	
	// Setting up rising edge interrupt for PA5
	GPIOA->MODER &= ~((1 << 10) & (1 << 11)); // Setting PA5 as input
	GPIOA->OSPEEDR &= ~((1 << 10)); // Setting PA5 to low speed
	GPIOA->PUPDR &= ~((1 << 10));   // Enabling Pull Down resistor for PA5
	GPIOA->PUPDR |= ((1 << 11));
	
	EXTI->IMR |= (1 << 5); 
	EXTI->FTSR &= ~(1 << 5);
	EXTI->RTSR |= (1 << 5);
	SYSCFG->EXTICR[1] = 0;
	SYSCFG->EXTICR[2] = 0;
	SYSCFG->EXTICR[3] = 0;	
	
	
	GPIOA->MODER &= ~((1 << 0) & (1 << 1)); // Setting PC0 as input
	GPIOA->OSPEEDR &= ~((1 << 0)); // Setting PC0 to low speed
	GPIOA->PUPDR &= ~((1 << 0));   // Enabling Pull Down resistor for PA0
	GPIOA->PUPDR |= ((1 << 1));
	
	
	
	// Setting up rising edge interrupt for PA0 
	EXTI->IMR |= (1 << 0); 
	EXTI->FTSR &= ~(1 << 0);
	EXTI->RTSR |= (1 << 0);
	SYSCFG->EXTICR[0] = 0;
	
	//NVIC_SetPriority(EXTI0_1_IRQn, 3); // Setting priority for EXTI0 interrupt
	//NVIC_EnableIRQ( EXTI0_1_IRQn ); // Enabling EXTI0 interrupt
	NVIC_SetPriority(EXTI4_15_IRQn, 3); // Setting Priority For EXTI4 Interrupt
	NVIC_EnableIRQ( EXTI4_15_IRQn ); // Enabling EXTI4 Interrupt
}


/*Initialize Buzzer*/
void initBuzzer(void) {	
	GPIOB->MODER |= (1 << 4);
	GPIOB->MODER &= ~(1 << 5);
	
	GPIOB->OTYPER &= ~(1 << 2);
	
	GPIOB->OSPEEDR &= ~(1 << 4);
	
	GPIOB->PUPDR &= ~(1 << 5);
	
	GPIOB->ODR &= ~(1<<2);
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
