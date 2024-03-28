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
  *This is potentiometer control code for controlling an RC car with Nintendo Nunchuck controller.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
volatile uint8_t txBuffer[6];
volatile int txCounter = 0;
volatile int txAmount = 0;
volatile uint8_t rdBuffer[6];
volatile uint8_t nunchuckData[6];
volatile int rdCounter = 0;
volatile int rdAmount = 0;

/**
	*@brief waits for Stop.
**/
void waitStop(){
	  while (1)
  {
		if((I2C2->ISR & (1<<5))){
			I2C2->ICR |=(1<<5);
			break;
		}
  }
}

/**
	*@brief handler for I2C event interrupt.
**/
void I2C2_IRQHandler(){

	//Transmit ready
	if(I2C2->ISR & I2C_ISR_TXIS){
		
		I2C2->TXDR = txBuffer[txCounter];
		
		if(txCounter < txAmount - 1){
			txCounter++;
		}
		else{
			txCounter = 0;
		}
	}
	
	//read ready
	if(I2C2->ISR & I2C_ISR_RXNE){

		rdBuffer[rdCounter] = I2C2->RXDR;
		
		if(rdCounter < rdAmount -1){
			rdCounter++;
			
		}
		else{

			rdCounter = 0;
		}
	}
	
	//transfer complete
	if((I2C2->ISR & I2C_ISR_TC)){
		
	//stop 
	I2C2->CR2 |=(1<<14);
		}
	
}
/**
	*@brief Initialize Nunchuck to not Encrypt data.
**/
void nunchuckInitialize(){
//Initialize nunchuck to not use encription
//set slave address
I2C2->CR2 |= (0x52<<1); 
//set n bytes
I2C2->CR2 |= (2<<16); 
//RD WRN
I2C2->CR2 &=~(1<<10); 
//START
I2C2->CR2 |=(1<<13); 


	//Send 0xF0, 0x55, stop

	
	txBuffer[0] = 0xF0;
	txBuffer[1] = 0x55;
	txAmount = 2;
			//START
	I2C2->CR2 |=(1<<13); 

	waitStop();
	

		//START
	I2C2->CR2 |=(1<<13);
	//send 0xFB, 0x00, stop
	txBuffer[0] = 0xFB;
	txBuffer[1] = 0x00;
	txAmount = 2;
	



	return ;
}
/**
	*@brief Collects input from the nunchuck.
**/
void nunchuckDataCollect(){
	//write 0x00 to collect data
		//Set slave Address
	waitStop();
	I2C2->CR2 &= ~(0xFF << 1);  // Clear bits 23-16
	I2C2->CR2 |= (0x52 << 1); 
	

	//set 1 byte
	I2C2->CR2 &=~ (0xFF<<16); 
	I2C2->CR2 |= (1<<16); 
	
	//Set to write
	I2C2->CR2 &=~(1<<10);
	
	//set 0x0 to then read data
	txBuffer[0] = 0x0;
	//set one byte
	txAmount = 1;
	
			//START
	I2C2->CR2 |=(1<<13);
	
	
	//read data and append to the nunchuck data
	
   //wait for transmission before reading
	waitStop();

	//set 6 bytes
	I2C2->CR2 &= ~(0xFF << 16);  // Clear bits 23-16
	I2C2->CR2 |= (0x6 << 16);       // Set 6 bytes transfer
	
	//Set to read
	I2C2->CR2 |=(1<<10);
	
	rdAmount = 6;
	
		//START
	I2C2->CR2 |=(1<<13);
	
	nunchuckData[0] = rdBuffer[0];
	nunchuckData[1] = rdBuffer[1];
	nunchuckData[2] = rdBuffer[2];
	nunchuckData[3] = rdBuffer[3];
	nunchuckData[4] = rdBuffer[4];
	nunchuckData[5] = rdBuffer[5];
	
	return ;
}



/**
  * @brief  This method sends a signal to the drive potentiometer to drive the car.
	* @input this is the input Char value to change the potentiometer to steer car
  */
void steerCar(char input){
	//address 0x28
	//write 0x00 then value
	waitStop();
	//GPIOC->ODR ^=(1<<9);
	I2C2->CR2 &= ~(0xFF << 1);  // Clear bits 23-16
	I2C2->CR2 |= (0x28 << 1); 
	

	//set 2 byte
	I2C2->CR2 &=~ (0xFF<<16); 
	I2C2->CR2 |= (0x2<<16); 
	
	//Set to write
	I2C2->CR2 &=~(1<<10);
	
	//set 0x0 to then read data
	txBuffer[0] = 0x00;
	txBuffer[1] = input;
	//set one byte
	txAmount = 2;
			//START
	I2C2->CR2 |=(1<<13);
	
	
}

/**
  * @brief  This method sends a signal to the drive potentiometer to drive the car.
	* @input this is the input Char value to change the potentiometer
  */
void driveCar(char input){
	//address 0x29
	//write 0x00 then value
	waitStop();
	//GPIOC->ODR ^=(1<<9);
	I2C2->CR2 &= ~(0xFF << 1);  // Clear bits 23-16
	I2C2->CR2 |= (0x29 << 1); 
	

	//set 2 byte
	I2C2->CR2 &=~ (0xFF<<16); 
	I2C2->CR2 |= (0x2<<16); 
	
	//Set to write
	I2C2->CR2 &=~(1<<10);
	
	//set 0x0 to then read data
	txBuffer[0] = 0x00;
	txBuffer[1] = input;
	//set one byte
	txAmount = 2;
			//START
	I2C2->CR2 |=(1<<13);
	
	
}

/**
  * @brief  This method determines the direction to drive the car.
  */
void determineDrive(){
		if(nunchuckData[5] == 0x02){
			GPIOC->ODR |=(1<<9);
			driveCar(0x3c);
		}
		else	if(nunchuckData[5] == 0x01){
			GPIOC->ODR |=(1<<8);
			driveCar(0x41);
		}
		else{
			GPIOC->ODR &=~(1<<9);
			GPIOC->ODR &=~(1<<8);
			driveCar(0x3f);
		}
}
/**
  * @brief  This method determines the direction the car should turn it's wheels.
  */
void determineDirection(){
	//check if left or right
 //future turn values 0x36 0x39 0x3c
			if(nunchuckData[0] > 0x80){
			GPIOC->ODR |=(1<<7);
			steerCar(0x33);
		}
				//future turn values 0x42 0x45 0x48
		else if(nunchuckData[0] < 0x80){
			GPIOC->ODR |=(1<<6);
			
			steerCar(0x4b);
		}
		

		else{
			GPIOC->ODR &=~(1<<6);
			GPIOC->ODR &=~(1<<7);
			steerCar(0x3f);
		}
}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	//setup RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	//setup LED lights for debugging


	//configure LEDS
	GPIOC->MODER &=~(1<<19); 
	GPIOC->MODER |=(1<<18);
	GPIOC->MODER &=~(1<<17); 
	GPIOC->MODER |=(1<<16);
	GPIOC->MODER &=~(1<<15); 
	GPIOC->MODER |=(1<<14); 
	GPIOC->MODER &=~(1<<13); 
	GPIOC->MODER |=(1<<12); 
	
	GPIOC->OTYPER &=~(1<<6);
	GPIOC->OTYPER &=~(1<<7);
	GPIOC->OTYPER &=~(1<<8);
	GPIOC->OTYPER &=~(1<<9);
	
	GPIOC->OSPEEDR &=~(1<<18); 
	GPIOC->OSPEEDR &=~(1<<16);
	GPIOC->OSPEEDR &=~(1<<14);
	GPIOC->OSPEEDR &=~(1<<12); 
 
	
	GPIOC->PUPDR &=~(1<<19); 
	GPIOC->PUPDR &=~(1<<18);
	GPIOC->PUPDR &=~(1<<17); 
	GPIOC->PUPDR &=~(1<<16);
	GPIOC->PUPDR &=~(1<<15); 
	GPIOC->PUPDR &=~(1<<14);
	GPIOC->PUPDR &=~(1<<13); 
	GPIOC->PUPDR &=~(1<<12); 

	GPIOC->ODR &=~(1<<9);
	GPIOC->ODR &=~(1<<8);
	GPIOC->ODR &=~(1<<7);
	GPIOC->ODR &=~(1<<6);
	
	//begin setup of I2C2
	
	// set pb11 to alternate function
	GPIOB->MODER &=~(1<<22); 
	GPIOB->MODER |=(1<<23);

	GPIOB->OTYPER |=(1<<11);
	
	//set afr
	GPIOB->AFR[1] |=(1<<12);

	// set pb13 to alternate function
	GPIOB->MODER &=~(1<<26); 
	GPIOB->MODER |=(1<<27);

	GPIOB->OTYPER |=(1<<13);
	
	//set afr
	GPIOB->AFR[1] |=(1<<22);
	GPIOB->AFR[1] |=(1<<20);

	//set pb14
	GPIOB->MODER |=(1<<28);
	GPIOB->OTYPER &=~(1<<14);
	//set pb14 high
	GPIOB->ODR |=(1<<14);

	//set pc0
	GPIOC->MODER |=(1<<0);
	GPIOC->OTYPER &=~(1<<0);
	//set pb14 high
	GPIOC->ODR |=(1<<0);

	//set to 100kHz
	I2C2->TIMINGR |= 0x13;
	I2C2->TIMINGR |= (0xF<<8);
	I2C2->TIMINGR |= (0x2<<16);
	I2C2->TIMINGR |= (0x4<<20);
	I2C2->TIMINGR |= (0x1<<28);

	//i2c enable
	I2C2->CR1 |=(1<<0);
	
	//enable interrupts
	I2C2->CR1 |=(1<<1);
	I2C2->CR1 |=(1<<6);
	I2C2->CR1 |=(1<<2);

	//enable interrupts in the NVIC
	NVIC_EnableIRQ(I2C2_IRQn);
	NVIC_SetPriority(I2C2_IRQn,2);
	
	//initialize the nunchuck
	nunchuckInitialize();
  while (1)
  {
		HAL_Delay(10);

		nunchuckDataCollect();
		determineDrive();
		determineDirection();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
