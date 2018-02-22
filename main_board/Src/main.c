/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;//UART2 NOT ACTIVATED
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//char rx_indx1, uart2.rx_indx, uart3.rx_indx, uart4.rx_indx, rx_indx5, uart6.rx_indx, rx_indx7, rx_indx8;
// unsigned char uart1.rx_data[6], rx_buffer1[10], transfer_cplt1;   //UART1
// unsigned char uart2.rx_data[6], uart2.rx_buffer[10], transfer_cplt2;   //UART2
// unsigned char uart3.rx_data[6], uart3.rx_buffer[10], transfer_cplt3;   //UART3
// unsigned char uart4.rx_data[6], uart4.rx_buffer[10], transfer_cplt4;   //UART4
// unsigned char rx_data5[6], rx_buffer5[10], transfer_cplt5;   //UART5
// unsigned char uart6.rx_data[20], uart4.rx_buffer[50], transfer_cplt6;  //UART6
// unsigned char rx_data7[20], rx_buffer7[50], transfer_cplt7;  //UART7
// unsigned char rx_data8[20], rx_buffer8[100], transfer_cplt8; //UART8
// unsigned char print_buffer1[10], print_buffer2[10], print_buffer3[10], print_buffer4[10];
// unsigned char print_buffer5[10], print_buffer6[50], print_buffer7[50], print_buffer8[100];
// unsigned char rx_data[20][8];
// int print_size1,print_size2,print_size3,print_size4;
// int print_size5,print_size6,print_size7,print_size8;

struct uarts
{
  char rx_indx;
  unsigned char rx_data[20];
  unsigned char print_buffer[50];
  unsigned char rx_buffer[100];
  unsigned char transfer_cplt;
  int print_size;
};

struct motors
{
  int measured_direction;
  char measured_speed[3];
  int desired_direction;
  char desired_speed[3];
};

uint8_t string_size_mtr = 6 , string_size_tx2 = 18;


int counter = 0;

struct uarts uart1, uart2, uart3, uart4, uart5, uart6, uart7, uart8;
struct motors mtr_FrntRight, mtr_RearRight, mtr_FrntLeft, mtr_RearLeft;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART7_UART_Init(void);
static void MX_USART8_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallBack(UART_HandleTypeDef *huart);
void sendData(void);
void transferComplete(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_TIM_Base_Init(&htim3);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HAL_UART_Receive_IT(&huart1, uart1.rx_data, 1);
//	HAL_UART_Receive_IT(&huart2, uart2.rx_data, 1);
	HAL_UART_Receive_IT(&huart3, uart3.rx_data, 1);
	HAL_UART_Receive_IT(&huart4, uart4.rx_data, 1);
	HAL_UART_Receive_IT(&huart5, uart5.rx_data, 1);
	HAL_UART_Receive_IT(&huart6, uart6.rx_data, 1);
	HAL_UART_Receive_IT(&huart7, uart7.rx_data, 1);
	HAL_UART_Receive_IT(&huart8, uart8.rx_data, 1);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();
  MX_USART7_UART_Init();
  MX_USART8_UART_Init();

  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
		transferComplete();
		sendData();

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
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

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART4 init function */
static void MX_USART4_UART_Init(void)
{

  huart4.Instance = USART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART5 init function */
static void MX_USART5_UART_Init(void)
{

  huart5.Instance = USART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART7 init function */
static void MX_USART7_UART_Init(void)
{

  huart7.Instance = USART7;
  huart7.Init.BaudRate = 9600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART8 init function */
static void MX_USART8_UART_Init(void)
{

  huart8.Instance = USART8;
  huart8.Init.BaudRate = 9600;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallBack(UART_HandleTypeDef *huart)
{
  /*Receive Interrupt
    UART1 Front-right
    UART2 Rear-right // until to revision of main board it is UART5
    UART3 Front-left
    UART4 Rear-left
    UART5
    UART6 NVIDIA TX2
    UART7
    UART8 GNSS
  */

  if (huart->Instance == USART1)
  {
    if (uart1.rx_indx == 0)
    {
      for (int i = 0; i < 6; i++)
        uart1.rx_data[i] = 0;
    }
    if (uart1.rx_data[0] == 'S') //If data that comes equal to 'S', that means index equals to 0
    {
      uart1.rx_indx = 0;
    }
    if (uart1.rx_data[0] > 0x80)
    {
      uart1.rx_data[0] = uart1.rx_data[0] - 0x80;
    }
    if (uart1.rx_data[0] != 0x0A) //Receive data that comes until it is equal to "/r"
    {
      uart1.rx_buffer[uart1.rx_indx++] = uart1.rx_data[0];
    }
    else
    {
      uart1.rx_indx = 0;
      uart1.transfer_cplt = 1; //Receive completed
    }
		HAL_UART_Receive_IT(&huart1, uart1.rx_data, 1);
  }
  if (huart->Instance == USART2)
  {
    // if (uart2.rx_indx == 0)
    // {
    //   for (int i = 0; i < 6; i++)
    //     uart2.rx_buffer[i] = 0;
    // }
    // if (uart2.rx_data[0] == 'S') //If data that comes equal to 'S', that means index equals to 0
    // {
    //   uart2.rx_indx = 0
    // }
    // if (uart2.rx_data[0] > 0x80)
    // {
    //   uart2.rx_data[0] = uart2.rx_data[0] - 0x80;
    // }
    // if (uart2.rx_data[0] != 0x0A) //Receive data that comes until it is equal to "/r"
    // {
    //   uart2.rx_buffer[uart2.rx_indx++] = uart2.rx_data[0];
    // }
    // else
    // {
    //   uart2.rx_indx = 0;
    //   uart2.transfer_cplt = 1; //Receive completed
    // }
//		HAL_UART_Receive_IT(&huart2, uart2.rx_data, 1);
  }
  if (huart->Instance == USART3)
  {
    if (uart3.rx_indx == 0)
    {
      for (int i = 0; i < 6; i++)
        uart3.rx_buffer[i] = 0;
    }
    if (uart3.rx_data[0] == 'S') //If data that comes equal to 'S', that means index equals to 0
    {
      uart3.rx_indx = 0;
    }
    if (uart3.rx_data[0] > 0x80)
    {
      uart3.rx_data[0] = uart3.rx_data[0] - 0x80;
    }
    if (uart3.rx_data[0] != 0x0A) //Receive data that comes until it is equal to "/r"
    {
      uart3.rx_buffer[uart3.rx_indx++] = uart3.rx_data[0];
    }
    else
    {
      uart3.rx_indx = 0;
      uart3.transfer_cplt = 1; //Receive completed
    }
		HAL_UART_Receive_IT(&huart3, uart3.rx_data, 1);
  }
  if (huart->Instance == USART4)
  {
    if (uart4.rx_indx == 0)
    {
      for (int i = 0; i < 6; i++)
        uart4.rx_buffer[i] = 0;
    }
    if (uart4.rx_data[0] == 'S') //If data that comes equal to 'S', that means index equals to 0
    {
      uart4.rx_indx = 0;
    }
    if (uart4.rx_data[0] > 0x80)
    {
      uart4.rx_data[0] = uart4.rx_data[0] - 0x80;
    }
    if (uart4.rx_data[0] != 0x0A) //Recieve datas that comes until it is equal to "/r"
    {
      uart4.rx_buffer[uart4.rx_indx++] = uart4.rx_data[0];
    }
    else
    {
      uart4.rx_indx = 0;
      uart4.transfer_cplt = 1; //Receive completed
    }
		HAL_UART_Receive_IT(&huart4, uart4.rx_data, 1);
  }
  if (huart->Instance == USART5)
  { // Temporarily transfered to uart2
        if (uart5.rx_indx == 0)
    {
      for (int i = 0; i < 6; i++)
        uart5.rx_buffer[i] = 0;
    }
    if (uart5.rx_data[0] == 'S') //If data that comes equal to 'S', that means index equals to 0
    {
      uart5.rx_indx = 0;
    }
    if (uart5.rx_data[0] > 0x80)
    {
      uart5.rx_data[0] = uart5.rx_data[0] - 0x80;
    }
    if (uart5.rx_data[0] != 0x0A) //Receive data that comes until it is equal to "/r"
    {
      uart5.rx_buffer[uart5.rx_indx++] = uart5.rx_data[0];
    }
    else
    {
      uart5.rx_indx = 0;
      uart5.transfer_cplt = 1; //Receive completed
    }
		HAL_UART_Receive_IT(&huart5, uart1.rx_data, 1);
  }
  if (huart->Instance == USART6)
  {
    if (uart6.rx_indx == 0)
    {
      for (int i = 0; i < 20; i++)
        uart6.rx_buffer[i] = 0;
    }
    if (uart6.rx_data[0] == 'S')
    {
      uart6.rx_indx = 0;
    }
    if (uart6.rx_data[0] > 0x80)
    {
      uart6.rx_data[0] = uart6.rx_data[0] - 0x80;
    }
    if (uart6.rx_data[0] != 0x0A)
    {
      uart6.rx_buffer[uart6.rx_indx++] = uart6.rx_data[0];
    }
    else
    {
      uart6.rx_indx = 0;
      uart6.transfer_cplt = 1;
    }
		HAL_UART_Receive_IT(&huart6, uart6.rx_data, 1);
  }
  if (huart->Instance == USART7)
  {
  }
  if (huart->Instance == USART8)
  {
       if (uart8.rx_indx == 0)
    {
      for (int i = 0; i < 20; i++)
        uart8.rx_buffer[i] = 0;
    }
    if (uart8.rx_data[0] == 'S')
    {
      uart8.rx_indx = 0;
    }
    if (uart8.rx_data[0] > 0x80)
    {
      uart8.rx_data[0] = uart8.rx_data[0] - 0x80;
    }
    if (uart8.rx_data[0] != 0x0A)
    {
      uart8.rx_buffer[uart8.rx_indx++] = uart8.rx_data[0];
    }
    else
    {
      uart8.rx_indx = 0;
      uart8.transfer_cplt = 1;
    }
		HAL_UART_Receive_IT(&huart8, uart8.rx_data, 1);
  }
}
void TIM3_IRQHandler(void)
{	//Timer Counter 1ms 
  /* USER CODE BEGIN TIM3_IRQn 0 */
  counter++;
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

void sendData()
{
  if (counter >= 15)
  {
    uart1.print_size = sprintf((char*)uart1.print_buffer, "S%d%sF", mtr_FrntRight.desired_direction, mtr_FrntRight.desired_speed);
		HAL_UART_Transmit_IT(&huart1,uart1.print_buffer,uart1.print_size);
	
//		uart2.print_size = sprintf(uart2.print_buffer, "S%d%dF", mtr_RearRight.desired_direction, mtr_RearRight.desired_speed);
//		HAL_UART_Transmit_IT(&huart2,uart2.print_buffer,uart2.print_size);
		
		uart3.print_size = sprintf((char*)uart3.print_buffer, "S%d%sF", mtr_FrntLeft.desired_direction, mtr_FrntLeft.desired_speed);
		HAL_UART_Transmit_IT(&huart3,uart3.print_buffer,uart3.print_size);
		
		uart4.print_size = sprintf((char*)uart4.print_buffer, "S%d%sF", mtr_RearLeft.desired_direction, mtr_RearLeft.desired_speed);
		HAL_UART_Transmit_IT(&huart4,uart4.print_buffer,uart4.print_size);
		
		uart5.print_size = sprintf((char*)uart5.print_buffer, "S%d%sF", mtr_RearRight.desired_direction, mtr_RearRight.desired_speed);
		HAL_UART_Transmit_IT(&huart5,uart5.print_buffer,uart5.print_size);
	
		uart6.print_size = sprintf((char*)uart6.print_buffer,"S%d%s%d%s%d%s%d%sF",mtr_FrntLeft.measured_direction,mtr_FrntLeft.measured_speed,mtr_RearLeft.measured_direction,mtr_RearLeft.measured_speed,mtr_FrntRight.measured_direction,mtr_FrntRight.measured_speed,mtr_RearRight.measured_direction,mtr_RearRight.measured_speed);
				
		if(uart6.print_size == 18)
		{
				HAL_UART_Transmit_IT(&huart6,uart6.print_buffer,uart6.print_size);
		}
		
  }
}

void transferComplete()
{
  /*
    UART1 Front-right
    UART2 Rear-right // until to revision of main board it is UART5
    UART3 Front-left
    UART4 Rear-left
  */
  if (uart1.transfer_cplt) //UART1 FRONT-RIGHT
  {
    /*"SdrpmF" six character message that come from motor controller 
    check that if ends with "F" which is equal to hex 0x46 */
    if (uart1.rx_buffer[5] == 0x46) //0x46 = 'F' (F correspond to finish)
    {
      //Message that motor controller 1 (front-right) sends include measured direction and speed in rpm
      mtr_FrntRight.measured_direction = uart1.rx_buffer[1] - '0';
			sprintf(mtr_FrntRight.measured_speed,"%c%c%c",uart1.rx_buffer[2],uart1.rx_buffer[3],uart1.rx_buffer[4]);
    }
  }
  if (uart2.transfer_cplt) //UART2 REAR RIGHT
  {
    if (uart2.rx_buffer[5] == 0x46) //0x46 = 'F' (F corresponds to finish)
    {
      //Message that motor controller 2 (rear-right) sends include measured direction and speed in rpm
      //mtr_RearRight.measured_direction= uart2.rx_buffer[1] - '0
      //sprintf(mtr_RearRight.measured_speed,"%c%c%c",uart2.rx_buffer[2],uart2.rx_buffer[3],uart2.rx_buffer[4]);

      //it is temporarily transfered to UART5
    }
  }
  if (uart3.transfer_cplt) // UART3 FRONT LEFT
  {
    if (uart3.rx_buffer[5] == 0x46) //0x46 = 'F' (F corresponds to finish)
    {
      //Message that motor controller 3 (front-left) sends include measured direction and speed in rpm
      mtr_FrntLeft.measured_direction = uart3.rx_buffer[1] - '0';
			sprintf(mtr_FrntLeft.measured_speed,"%c%c%c",uart3.rx_buffer[2],uart3.rx_buffer[3],uart3.rx_buffer[4]);

    }
  }
  if (uart4.transfer_cplt) // UART4 REAR LEFT
  {
    if (uart4.rx_buffer[5] == 0x46) //0x46 = 'F' (F corresponds to finish)
    {
      //Message that motor controller 4 (rear-left) sends include measured direction and speed in rpm
      mtr_RearRight.measured_direction = uart4.rx_buffer[1] - '0';
			sprintf(mtr_RearLeft.measured_speed,"%c%c%c",uart4.rx_buffer[2],uart4.rx_buffer[3],uart4.rx_buffer[4]);

    }
  }
  if (uart5.transfer_cplt) //UART2 REAR RIGHT (TEMPORARY)
  {
    if (uart5.rx_buffer[5] == 0x46) //0x46 = 'F' (F corresponds to finish)
    {
      //Message that motor controller 2 (rear-right) sends include measured direction and speed in rpm
			mtr_RearRight.measured_direction= uart5.rx_buffer[1] - '0';
			sprintf(mtr_RearRight.measured_speed,"%c%c%c",uart5.rx_buffer[2],uart5.rx_buffer[3],uart5.rx_buffer[4]);

    }
  }
  if (uart6.transfer_cplt) // SERIAL COMM WITH TX2
  {
    if (uart6.rx_buffer[17] == 0x46) //0x46 = 'F' (F corresponds to finish)
    {                               /* message come from tx2 S1999199919991999F 
      the message for motors encoded to four decimal in a block (ex.'1999')
      block's first decimal is direction (ex."'1'999"), 
      other decimals define the motor's rpms (ex."1'999'"), rpms are up to 450 rpm
      First block defines front-left
      Second block defines rear-left
      Third block defines front-right
      Fourth block defines rear-right 
      Because of shifting sides, implementation that runs in below is needed.
      */			
			mtr_FrntLeft.desired_direction = uart6.rx_buffer[1]-'0';
//			mtr_FrntLeft.desired_speed = ((uart6.rx_buffer[2] - '0') * 100) + ((uart6.rx_buffer[3] - '0') * 10) + (uart6.rx_buffer[4] - '0');
			sprintf(mtr_FrntLeft.desired_speed,"%c%c%c",uart6.rx_buffer[2],uart6.rx_buffer[3],uart6.rx_buffer[4]);

			mtr_RearLeft.desired_direction = uart6.rx_buffer[5]-'0';
//			mtr_RearLeft.desired_speed = ((uart6.rx_buffer[6] - '0') * 100) + ((uart6.rx_buffer[7] - '0') * 10) + (uart6.rx_buffer[8] - '0');
			sprintf(mtr_RearLeft.desired_speed,"%c%c%c",uart6.rx_buffer[6],uart6.rx_buffer[7],uart6.rx_buffer[8]);

			
			mtr_FrntRight.desired_direction = uart6.rx_buffer[9]-'0';
//			mtr_FrntRight.desired_speed = ((uart6.rx_buffer[10] - '0') * 100) + ((uart6.rx_buffer[11] - '0') * 10) + (uart6.rx_buffer[12] - '0');
			sprintf(mtr_FrntRight.desired_speed,"%c%c%c",uart6.rx_buffer[10],uart6.rx_buffer[11],uart6.rx_buffer[12]);

			
			mtr_RearRight.desired_direction = uart6.rx_buffer[13]-'0';
//			mtr_RearRight.desired_speed = ((uart6.rx_buffer[14] - '0') * 100) + ((uart6.rx_buffer[15] - '0') * 10) + (uart6.rx_buffer[16] - '0');
			sprintf(mtr_RearRight.desired_speed,"%c%c%c",uart6.rx_buffer[14],uart6.rx_buffer[15],uart6.rx_buffer[16]);

    }
  }
  if (uart7.transfer_cplt)
  {
    //Empty
  }
  if (uart8.transfer_cplt)
  {
    //GNSS UART
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
