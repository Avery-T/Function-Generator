/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lookup_table.h"
#include <stdbool.h>
void keypad_init(void);

uint32_t get_key_from_keypad(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Internal_clock_Intit();
void DAC_init();
void DAC_write();
void Timer2_Init();
void change_function_gens_output(uint32_t key);
//CONSTS for keypad config  
const int COL_OFFSET = 4;
const int ROW_OFFSET = 0;
const int16_t NUM_ROWS = 4;
const uint16_t NUM_COLS = 4;
const uint16_t NUM_OF_SAMPLES = 600;

uint32_t val = 0;

const uint16_t keys[4][4] = {
    {1, 2, 3, 10},
    {4, 5, 6, 11},
    {7, 8, 9, 12},
    {14, 0, 15, 13}
};


// FLAGS 
 volatile int sawtooth = 0, sine = 0, square = 1,triangle =0;
 volatile int table_index = 0, square_wave_duty = 5, 
		 	   frequency_set = 1, freq_cnt = 0;
//
 
//UART_HandleTypeDef huart2;
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  Internal_clock_Intit();
  DAC_init();
  Timer2_Init();
  keypad_init();
   
  uint32_t key = -1;
  
  while(1)
  {
	  key = get_key_from_keypad(); 
	  change_function_gens_output(key);
	  key = -1; //resets the prev key
  }
}

void TIM2_IRQHandler(void)
{
	  TIM2->SR &= ~TIM_DIER_UIE;
	  if(table_index >= NUM_OF_SAMPLES){
		table_index=0;
	  }
	  table_index += frequency_set;
      DAC_write();
}
void DAC_write()
{
	 while(!(SPI1->SR & SPI_SR_TXE));		// ensure room in TXFIFO before writing

	 if(sine){
		 SPI1->DR = (0x3000 | SINE_WAVE[table_index]);
		 return;
	 }
	  if(sawtooth){
		 SPI1->DR = (0x3000 | SAWTOOTH_WAVE[table_index]);
		 return;
	  }
	 if(triangle){
		 SPI1->DR = (0x3000 | TRIANGLE_WAVE[table_index]);
		 return;
	 }
	SPI1->DR = (0x3000 | SQUARE_WAVE[square_wave_duty][table_index]);
}

uint32_t get_key_from_keypad(void)
{
	uint32_t key = -1;
    bool row_match = false;
        GPIOC->ODR |= (0x0FU << COL_OFFSET);
        for(uint32_t row = 0; row < NUM_ROWS; row++){
            for(uint32_t col = 0; col < NUM_COLS; col++){
                GPIOC->BSRR |= (1U << (COL_OFFSET + col + 16));
                row_match = !(GPIOC->IDR & (1U << (ROW_OFFSET + row)));
                if(row_match){
                    key = keys[row][col];
                }
                GPIOC->BSRR = (1U << (COL_OFFSET + col));
            }
        }
        return key;
}
void keypad_init(void)
{
    //initialize clock to ports B and C
    RCC->AHB2ENR |=  RCC_AHB2ENR_GPIOCEN;
    //set mode to input for pins 0-3 in port C, rows
    GPIOC->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
    //set mode to output for pins 12-15 in port B, columns
    GPIOC->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5| GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOC->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0);
    //pull up rows to high
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3);
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0);
}
void change_function_gens_output(uint32_t key)
 {
 	HAL_Delay(100); //debouncer
 	if(key==-1) return; //if its not a valid key dont change anyof the flags
 	sine = 0; sawtooth =0; triangle =0; square =0;
 	switch(key){
 		case 1:
		frequency_set =1;
 		case 1:
 				frequency_set =1;
 		case 2:
 				frequency_set = 2;
 		case 3:
 				frequency_set = 3;
 		case 4:
 				frequency_set = 4;
 		case 5:
 				frequency_set = 5;
 		case 6:
 				frequency_set = 1;
 		case 7:
 				frequency_set = 1;
 		case 8:
 				frequency_set = 1;
 		case 9:
 				frequency_set = 1;
 		case 14:
 				if( key == 14 && square_wave_duty != 0)
 					square_wave_duty--;		
 		case 15:
 			 	if(square_wave_duty !=8)
 			 		square_wave_duty++;
 		default: return;
 	}
 	
 }  	

void DAC_init()
{
	 /*
		   * Configure SPI Pins		PA4 - SPI_1_NSS		PA5 - SPI_1_SCK
		   * 						PA6 - SPI_1_MISO	PA7 - SPI_1_MOSI
		   * follow order of configuring registers AFR, OTYPER, PUPDR, OSPEEDR, MODDER
		   * to avoid a glitch is created on the output pin
		   */
		  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN); // need to generate a clock for spi because the clock is syncronized to the cpu clock

		  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 |		// mask AF selection
				  	  	  	  GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
		  GPIOA->AFR[0] |= ((5 << GPIO_AFRL_AFSEL4_Pos) |				// select SPI_1 (AF5)
		  		  	  	    (5 << GPIO_AFRL_AFSEL5_Pos) |
						    (5 << GPIO_AFRL_AFSEL6_Pos) |
						    (5 << GPIO_AFRL_AFSEL7_Pos));
		  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 |		// push-pull output
				  	  	  	  GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);

		  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 |		// no pull ups or pull downs
				  	  	  	GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
		  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED4 | 					// low speed
				  	  	  	  GPIO_OSPEEDR_OSPEED5 |
							  GPIO_OSPEEDR_OSPEED6 |
							  GPIO_OSPEEDR_OSPEED7);

		  GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 |		// mask function
				  	  	  	GPIO_MODER_MODE6 | GPIO_MODER_MODE7);

		  GPIOA->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 |	// enable alternate function
				  	  	   GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
		  RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);		// enable SPI1 clock
		   SPI1->CR1 = (SPI_CR1_MSTR);				// enable master mode, fck/2, hardware CS, MSB first, full duplex
		   SPI1->CR2 = (SPI_CR2_SSOE |				// enable CS output
		 		  	   SPI_CR2_NSSP |				// create CS pulse
		 			   (0xF << SPI_CR2_DS_Pos));	// 16-bit data frames
		   SPI1->CR1 |= (SPI_CR1_SPE);				// enable SPI

}

void Internal_clock_Intit()
{
	// Enable MCO, select MSI (4 MHz source)
	RCC->CFGR = ((RCC->CFGR & ~(RCC_CFGR_MCOSEL)) | (RCC_CFGR_MCOSEL_0));
	// Configure MCO output on PA8
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER   &= ~(GPIO_MODER_MODE8);		// alternate function mode
	GPIOA->MODER   |=  (2 << GPIO_MODER_MODE8_Pos);
	GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT8);		// Push-pull output
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD8);		// no resistor
	GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8);		// high speed
	GPIOA->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);		// select MCO function

}

void Timer2_Init()
{
    // Enable the peripheral clock for TIM2
	RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOCEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    // Configure the GPIO pin for PWM output (change these values for your specific pin)
    // Configure TIM2 as PWM mode
    TIM2->ARR =  320000/600;  //32 megahurtz/600 samples
    TIM2->DIER |= TIM_DIER_UIE; //flag for arr overflow inturupt
    NVIC_SetPriority(TIM2_IRQn, 0); // Set priority (adjust as needed)
    NVIC_EnableIRQ(TIM2_IRQn); // Enable the interrupt    // Set the PWM period (ARR register)
    // Set the initial PWM duty cycle (CCR1 register)
    TIM2->PSC = 0;
    //TIM2->CCR1 = 399; // For a 50% duty cycle, assuming a 1 kHz PWM frequency
    TIM2->CR1 |= TIM_CR1_CEN;
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  //RCC_OscInitStruct.MSIState = RCC_MSI_ON;  //datasheet says NOT to turn on the MSI then change the frequency.
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
	/* from stm32l4xx_hal_rcc.h
	#define RCC_MSIRANGE_0                 MSI = 100 KHz
	#define RCC_MSIRANGE_1                 MSI = 200 KHz
	#define RCC_MSIRANGE_2                 MSI = 400 KHz
	#define RCC_MSIRANGE_3                 MSI = 800 KHz
	#define RCC_MSIRANGE_4                 MSI = 1 MHz
	#define RCC_MSIRANGE_5                 MSI = 2 MHz
	#define RCC_MSIRANGE_6                 MSI = 4 MHz
	#define RCC_MSIRANGE_7                 MSI = 8 MHz
	#define RCC_MSIRANGE_8                 MSI = 16 MHz
	#define RCC_MSIRANGE_9                 MSI = 24 MHz
	#define RCC_MSIRANGE_10                MSI = 32 MHz
	#define RCC_MSIRANGE_11                MSI = 48 MHz   dont use this one*/
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;  //datasheet says NOT to turn on the MSI then change the frequency.
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
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
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
