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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UART2_BASE (0x40004400)
#define UART_BRR (0x08)
#define UART_SR (0x00)
#define UART_DR (0x04)
#define UART_CR1 (0x0C)
#define GPIOA_BASE_ADD (0x40020000)
#define GPIOA_MODER (0x00)
#define GPIO_AFRL (0x20)

int rx_index = 0;
char rx_buf[32]={0};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void alternate_fc_pin()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* MODER = (uint32_t*)(GPIOA_BASE_ADD);
	*MODER |= (0b10 << 4) | (0b10 << 6);

	uint32_t* AFRL = (uint32_t*)(GPIOA_BASE_ADD + GPIO_AFRL);
	*AFRL |= (0b0111 << 8) | (0b0111 << 12);
}

void UART_init()
{
	//todo: enable RCC UART
	__HAL_RCC_USART2_CLK_ENABLE();	//16Mhz clock for UART2

	//todo: set baudrate 9600bps

	/*
	 * baurate = fck/(16*UARTDIV)
	 * ---> UARTDIV = 104.1875
	 * man = 104
	 * franc = 0.1875*16 = 3
	 * */
	uint32_t* baudrate = (uint32_t*)(UART2_BASE + UART_BRR);
	*baudrate = (104 << 4) | (3 << 0);

	//todo: set frame data: 8bit data none parity
	uint32_t* control_CR1 = (uint32_t*)(UART2_BASE + UART_CR1);
	*control_CR1 &= ~(1 << 12);				//word length
	*control_CR1 &= ~(1 << 10);				//patity control

	//todo: enable interrupt to RXNE
	*control_CR1 |= (1 << 5);

	//todo: enable UART
	*control_CR1 |= (1 << 3);				//Tx
	*control_CR1 |= (1 << 2);				//Rx
	*control_CR1 |= (1 << 13);				// ENABLE UART
}

void NVIC_init()
{
	uint32_t* ISER1 = (uint32_t*)(0xE000E104);
	*ISER1 |= (1 << 6);
}

void UART_Send_byte(char _data)
{
	// todo: write "data" to DR (data register)
	uint32_t* data_register = (uint32_t*)(UART2_BASE + UART_DR);
	*data_register = _data;
	//check state data Tx complete -> read bit 6 in SR
	uint32_t* check_Tx = (uint32_t*)(UART2_BASE + UART_SR);
	while(((*check_Tx >> 7)&1) == 0);		// check TxE data register empty
	uint32_t time = 0;
	while(((*check_Tx >> 6)&1) == 0);
//	{
//		if(time++ > 1000) break;
//		HAL_Delay(1);				//--->remember set timeout
//	}
	*check_Tx &= ~(1 << 6);

}

uint8_t UART_Recv_byte()
{
	uint32_t* check_Tx = (uint32_t*)(UART2_BASE + UART_SR);
	uint32_t* data_register = (uint32_t*)(UART2_BASE + UART_DR);
	uint8_t data_recv = 0;
	while(((*check_Tx >> 5)&1)==0);
	data_recv = *data_register;
	return data_recv;
}

uint32_t systick_cnt = 0;

void SysTick_Handler()
{
	systick_cnt++;
}
void system_tick_init()
{
	uint32_t* CSR = (uint32_t* )0xe000e010;
	uint32_t* RVR = (uint32_t* )0xe000e014;
	*RVR = 15999;
	*CSR |= (1<<1)|(1<<0)|(1<<2);
}


void Custom_delay(uint32_t mSec)
{
	systick_cnt = 0;
	while(systick_cnt < mSec);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void USART2_IRQHandler()
{
	uint32_t* data_register = (uint32_t*)(UART2_BASE + UART_DR);
	  rx_buf[rx_index] = *data_register;
	  if(rx_index++ >= sizeof(rx_buf))
		  rx_index = 0;
}


int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  alternate_fc_pin();
  system_tick_init();
  UART_init();
  NVIC_init();
//  Custom_delay(10);
  char msg[] ="Hello PC!!\r\n";
  for(int i = 0; i < sizeof(msg) - 1; i++)
  {
	  UART_Send_byte(msg[i]);
//	  Custom_delay(10);
  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
