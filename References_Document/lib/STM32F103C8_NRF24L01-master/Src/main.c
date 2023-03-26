
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

#include "nrf24l01mbal.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t one_byte_buf[ 1 ];
uint8_t tmp_char;

//NRF24L01_ports_TypeDef nrf_tx;
//NRF24L01_ports_TypeDef nrf_rx;
NRF24L01_config_TypeDef nrf_tx_cfg;
NRF24L01_config_TypeDef nrf_rx_cfg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void int8_to_bin( int8_t x, int8_t *_buf );

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_SET );

  //nrf_tx_cfg.RX_TX           = 0; //0 - TX
  nrf_tx_cfg.CE_pin          = GPIO_PIN_0;
  nrf_tx_cfg.CE_port         = GPIOA;
  nrf_tx_cfg.CSN_pin         = GPIO_PIN_1;
  nrf_tx_cfg.CSN_port        = GPIOA;
  nrf_tx_cfg.SPI             = &hspi1;
  nrf_tx_cfg.radio_channel   = 15;
  nrf_tx_cfg.baud_rate       = TM_NRF24L01_DataRate_1M;
  nrf_tx_cfg.payload_len     = 1;
  nrf_tx_cfg.crc_len         = 1;
  nrf_rx_cfg.output_power    = TM_NRF24L01_OutputPower_0dBm;
  nrf_tx_cfg.rx_address[ 0 ] = 0xE7;
  nrf_tx_cfg.rx_address[ 1 ] = 0xE7;
  nrf_tx_cfg.rx_address[ 2 ] = 0xE7;
  nrf_tx_cfg.rx_address[ 3 ] = 0xE7;
  nrf_tx_cfg.rx_address[ 4 ] = 0xE7;
  nrf_tx_cfg.tx_address[ 0 ] = 0x7E;
  nrf_tx_cfg.tx_address[ 1 ] = 0x7E;
  nrf_tx_cfg.tx_address[ 2 ] = 0x7E;
  nrf_tx_cfg.tx_address[ 3 ] = 0x7E;
  nrf_tx_cfg.tx_address[ 4 ] = 0x7E;

  //nrf_rx_cfg.RX_TX           = 1; //RX
  nrf_rx_cfg.CE_pin          = GPIO_PIN_8;
  nrf_rx_cfg.CE_port         = GPIOA;
  nrf_rx_cfg.CSN_pin         = GPIO_PIN_9;
  nrf_rx_cfg.CSN_port        = GPIOA;
  nrf_rx_cfg.SPI             = &hspi2;
  nrf_rx_cfg.radio_channel   = 15;
  nrf_rx_cfg.baud_rate       = TM_NRF24L01_DataRate_1M;
  nrf_rx_cfg.payload_len     = 1;
  nrf_rx_cfg.crc_len         = 1;
  nrf_rx_cfg.output_power    = TM_NRF24L01_OutputPower_0dBm;
  nrf_rx_cfg.rx_address[ 0 ] = 0x7E;
  nrf_rx_cfg.rx_address[ 1 ] = 0x7E;
  nrf_rx_cfg.rx_address[ 2 ] = 0x7E;
  nrf_rx_cfg.rx_address[ 3 ] = 0x7E;
  nrf_rx_cfg.rx_address[ 4 ] = 0x7E;
  nrf_rx_cfg.tx_address[ 0 ] = 0xE7;
  nrf_rx_cfg.tx_address[ 1 ] = 0xE7;
  nrf_rx_cfg.tx_address[ 2 ] = 0xE7;
  nrf_rx_cfg.tx_address[ 3 ] = 0xE7;
  nrf_rx_cfg.tx_address[ 4 ] = 0xE7;

  /* Data received and data for send */
  uint8_t dataOut[32], dataIn[32], config_bytes[ 100 ], text_to_show[ 100 ];
  uint8_t cnt = 0;

  /* NRF transmission status */
  TM_NRF24L01_Transmit_Status_t transmissionStatus;

  /* Buffer for strings */
  //char str[40];

  mbal_NRF24L01_Init( &nrf_rx_cfg );
  //mbal_NRF24L01_PowerUpTx( &nrf_rx_cfg );
  HAL_Delay( 2000 );
  mbal_NRF24L01_Init( &nrf_tx_cfg );
  //mbal_NRF24L01_PowerUpRx( &nrf_tx_cfg );
  HAL_Delay( 2000 );

  mbal_NRF24L01_Clear_Interrupts( &nrf_tx_cfg );
  mbal_NRF24L01_Clear_Interrupts( &nrf_rx_cfg );
  HAL_Delay( 200 );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay( 100 );
	  if( one_byte_buf[ 0 ] != 0 ) {
		  dataOut[ 0 ] = one_byte_buf[ 0 ];
		  tmp_char = one_byte_buf[ 0 ];
		  one_byte_buf[ 0 ] = 0;
		  mbal_NRF24L01_Transmit( &nrf_tx_cfg, dataOut );
		  /* Wait for data to be sent */
		  do {
			/* Get transmission status */
			transmissionStatus = mbal_NRF24L01_GetTransmissionStatus( &nrf_tx_cfg );
		  } while( transmissionStatus == TM_NRF24L01_Transmit_Status_Sending );
		  mbal_NRF24L01_PowerUpRx( &nrf_tx_cfg );
			dataIn[ 1 ] = 13;
		    dataIn[ 2 ] = 10;
		    CDC_Transmit_FS( dataIn, 3 );
		    HAL_Delay( 2 );

		  if( tmp_char != '\r' ) {
			  CDC_Transmit_FS( &tmp_char, 1 );
			  HAL_Delay( 5 );
		  }

		  if( tmp_char == 'H' ) {
			  HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_SET );
		  }
		  else if( tmp_char == 'L' ) {
			  HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_RESET );
		  }
		  else if( tmp_char == 't' ) {
			  dataIn[ 0 ] = 13;
			  dataIn[ 1 ] = 10;
			  CDC_Transmit_FS( dataIn, 2 );
			  HAL_Delay( 5 );
			  mbal_NRF24L01_ReadConfig( &nrf_tx_cfg, config_bytes );
			  for( int i = 0; i < 10; i++ ) { //38
				  int8_to_bin( config_bytes[ i ], text_to_show );
				  CDC_Transmit_FS( text_to_show, 10 );
				  HAL_Delay( 1 );
			  }
		  }
		  else if( tmp_char == 'r' ) {
			  dataIn[ 0 ] = 13;
			  dataIn[ 1 ] = 10;
			  CDC_Transmit_FS( dataIn, 2 );
			  HAL_Delay( 5 );
			  mbal_NRF24L01_ReadConfig( &nrf_rx_cfg, config_bytes );
			  for( int i = 0; i < 10; i++ ) {
				  int8_to_bin( config_bytes[ i ], text_to_show );
				  CDC_Transmit_FS( text_to_show, 10 );
				  HAL_Delay( 1 );
			  }
		  }
		  else if( tmp_char == '\r' ) {
			  dataIn[ 0 ] = '\r';
			  dataIn[ 1 ] = '\n';
			  CDC_Transmit_FS( dataIn, 2 );
		  }
		  //one_byte_buf[ 0 ] = 0;
	  }

	  if( mbal_NRF24L01_DataReady( &nrf_rx_cfg )) {
	    /* Get data from NRF24L01+ */
	    mbal_NRF24L01_GetData( &nrf_rx_cfg, dataIn );
	    tmp_char = dataIn[ 0 ];
	    //mbal_NRF24L01_Clear_Interrupts( &nrf_rx_cfg );
	    dataIn[ 0 ] = '\r';
	    dataIn[ 1 ] = '\n';
	    dataIn[ 2 ] = 'R';
	    dataIn[ 3 ] = ':';
	    dataIn[ 4 ] = tmp_char; //one_byte_buf[ 0 ];
	    dataIn[ 5 ] = '\r';
	    dataIn[ 6 ] = '\n';
	    CDC_Transmit_FS( dataIn, 7 );
	    //one_byte_buf[ 0 ] = 0;
	    tmp_char = '9';
		mbal_NRF24L01_Transmit( &nrf_rx_cfg, &tmp_char );
		/* Wait for data to be sent */
		do {
		  /* Get transmission status */
		  transmissionStatus = mbal_NRF24L01_GetTransmissionStatus( &nrf_rx_cfg );
		} while( transmissionStatus == TM_NRF24L01_Transmit_Status_Sending );
		mbal_NRF24L01_PowerUpRx( &nrf_rx_cfg );
	  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  if( mbal_NRF24L01_DataReady( &nrf_tx_cfg )) {
	  	    /* Get data from NRF24L01+ */
	  	    mbal_NRF24L01_GetData( &nrf_tx_cfg, dataIn );
	  	  mbal_NRF24L01_Clear_Interrupts( &nrf_tx_cfg );
	  	    tmp_char = dataIn[ 0 ];
	  	    //mbal_NRF24L01_Clear_Interrupts( &nrf_rx );
	  	    dataIn[ 0 ] = '\r';
	  	    dataIn[ 1 ] = '\n';
	  	    dataIn[ 2 ] = 'R';
	  	    dataIn[ 3 ] = '2';
	  	    dataIn[ 4 ] = ':';
	  	    dataIn[ 5 ] = tmp_char; //one_byte_buf[ 0 ];
	  	    dataIn[ 6 ] = '\r';
	  	    dataIn[ 7 ] = '\n';
	  	    CDC_Transmit_FS( dataIn, 8 );
	  	    //one_byte_buf[ 0 ] = 0;
//	  		mbal_NRF24L01_Transmit( &nrf_tx_cfg, &tmp_char );
//	  		/* Wait for data to be sent */
//	  		do {
//	  		  /* Get transmission status */
//	  		  transmissionStatus = mbal_NRF24L01_GetTransmissionStatus( &nrf_tx_cfg );
//	  		} while( transmissionStatus == TM_NRF24L01_Transmit_Status_Sending );
//	  		mbal_NRF24L01_PowerUpRx( &nrf_tx_cfg );
	  	  }

	  cnt++;
	  if( cnt >= 30 ) {
		  cnt = 0;
		  tmp_char = 'X';
			mbal_NRF24L01_Transmit( &nrf_rx_cfg, &tmp_char );
			/* Wait for data to be sent */
			do {
			  /* Get transmission status */
			  transmissionStatus = mbal_NRF24L01_GetTransmissionStatus( &nrf_rx_cfg );
			} while( transmissionStatus == TM_NRF24L01_Transmit_Status_Sending );
			mbal_NRF24L01_PowerUpRx( &nrf_rx_cfg );
	  }

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void int8_to_bin( int8_t x, int8_t *_buf ) {
	for( int loop = 0; loop < 8; loop++ ) {
		_buf[ loop ] = x & (0x80 >> loop) ? '1' : '0';
	}
	_buf[ 8 ] = '\n';
	_buf[ 9 ] = '\r';
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
