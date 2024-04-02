#include "uart.h"


void UART3_SendChar(char c){
	USART3->TDR = c;  
	//while (!(USART3->ISR & USART_ISR_TC));
}

void UART3_SendStr(char str[]){
    uint8_t send = 0;
		while(str[send] != '\0'){
			if ((USART3->ISR & USART_ISR_TC) == USART_ISR_TC){
				USART3->TDR = str[send++];
			}
		}
	USART3->ICR |= USART_ICR_TCCF;
}

void USART3_4_IRQHandler(uint8_t data){
	uint8_t rx_val = (uint8_t)(USART3->RDR); /* Receive data, clear flag */
		if(data == NULL && rx_val >=65 && rx_val <= 122){
			led = rx_val;
			UART3_SendStr("\nCMD:: ");
		}else if(rx_val >= 48 && rx_val <= 50){
		  operation	= rx_val;
			newdata = 1;
		}	 
}


void USART1_IRQHandler(void){
	char rx_val1[]; /* Receive data, clear flag */
	uint8_t send1 = 0;
		while(rx_val1[send1] != '\0'){
			if ((USART3->ISR & USART_ISR_TC) == USART_ISR_TC){
				USART3->TDR = rx_val1[send1++];
			}
		}
	USART3->ICR |= USART_ICR_TCCF;
}

void usart_init(void)
{
    USART1->BRR |= 69;
    USART1->CR1 |= ( USART_CR1_TE | USART_CR1_UE);
    USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE;

    USART3->BRR |= 69;
    USART3->CR1 |= ( USART_CR1_TE | USART_CR1_UE);
    USART3->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE;

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn,1);

    NVIC_EnableIRQ(USART3_4_IRQn);
    NVIC_SetPriority(USART3_4_IRQn,1);

    GPIOC->MODER |= (GPIO_MODER_MODER4 | GPIO_MODER_MODER5); //usart3 pins PC4 & PC5 

    GPIOB->MODER |= (GPIO_MODER_MODER6 | GPIO_MODER_MODER7); //usart1 pins PB6 & PB7 

    GPIOC->AFR[0] |= ((1<<16)|(1<<20));


    uint32_t led_pin = 0;
    UART3_SendStr("AT");
    UART3_SendStr("AT+CIFSR");
    UART3_SendStr("AT+CWMODE=1");
    UART3_SendStr("AT+CWJAP=\"Elevate417\",\"qwerty@asdfg\"");
    UART3_SendStr("AT+CIPMUX=1");
    UART3_SendStr("AT+CIPSTART=0,\"TCP\",\"192.168.4.1\",8888");
    UART3_SendStr("AT+CIPSEND=0,10");
    UART3_SendStr("A");

    uint8_t rx_val = (uint8_t)(USART3->RDR);
    UART3_SendStr(rx_val1);
    /*while (1)
    {		
        //USART3->TDR = (1<<6);
        if(newdata == 1){
                USART3->TDR = operation;
                USART3->TDR = led;
                switch(led){
                case 82:
                    if(operation == 48){
                        GPIOC->ODR &= ~(1<<6);
                    }else if(operation == 49){
                        GPIOC->ODR |= (1<<6);
                    }else if(operation == 50){
                        GPIOC->ODR ^= (1<<6);
                    }
                    break;
                case 66:
                    if(operation == 48){
                        GPIOC->ODR &= ~(1<<7);
                    }else if(operation == 49){
                        GPIOC->ODR |= (1<<7);
                    }else if(operation == 50){
                        GPIOC->ODR ^= (1<<7);
                    }
                    break;
                case 71:
                    if(operation == 48){
                        GPIOC->ODR &= ~(1<<9);
                    }else if(operation == 49){
                        GPIOC->ODR |= (1<<9);
                    }else if(operation == 50){
                        GPIOC->ODR ^= (1<<9);
                    }
                    break;
                case 79:
                    if(operation == 48){
                        GPIOC->ODR &= ~(1<<8);
                    }else if(operation == 49){
                        GPIOC->ODR |= (1<<8);
                    }else if(operation == 50){
                        GPIOC->ODR ^= (1<<8);
                    }
                    break;
                default:
                    if(led >= 65 && led <= 122 ){
                        UART3_SendStr("Invalid Input\n");
                    }
                    break;
                }
            
                UART3_SendStr("\nCMD:: ");
                newdata = NULL;
                led = NULL;
                operation = NULL;
        }				
            
    }*/

}
