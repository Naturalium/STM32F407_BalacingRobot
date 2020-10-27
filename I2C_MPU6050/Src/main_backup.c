/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

osThreadId defaultTaskHandle;
osThreadId GYROHandle;
osMessageQId myQueue01Handle;
osTimerId myTimer01Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ACC_DIV 1

const uint16_t addw = 208;
uint8_t data;
static uint8_t check;
//int8_t* pldata;
static int8_t pdata[14];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void StartGYRO(void const * argument);
void Callback01(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
//  uint16_t addw = 0x68;
                   //0x68 * 2 ????????????????????
  
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
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  //HAL_I2C_Master_Transmit(&hi2c1, addw, (uint8_t *)'S', 1, 50);
  //HAL_I2C_Master_Transmit(&hi2c1, addw, &addw, 1, 50);
  //HAL_I2C_Master_Transmit(&hi2c1, addw, (uint8_t *)'0x3B', 8, 50);

  
  
  //data = 0x80;
  //HAL_I2C_Mem_Write(&hi2c1, addw, 0x6B, 1, &data, 1, 1000);
  data = 0x80;          //reset register
  //addw = 208;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  
  
  //data = 0x07;
  //HAL_I2C_Mem_Write(&hi2c1, addw, 0x19, 1, &data, 1, 1000);
  
  //data = 0x00;
  //HAL_I2C_Mem_Write(&hi2c1, addw, 0x1B, 1, &data, 1, 1000);
  //data = 0x00;
  //HAL_I2C_Mem_Write(&hi2c1, addw, 0x1C, 1, &data, 1, 1000);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of GYRO */
  osThreadDef(GYRO, StartGYRO, osPriorityAboveNormal, 0, 200);
  GYROHandle = osThreadCreate(osThread(GYRO), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 14, uint32_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  while(check != 0x68)
  {
      data = 0x80;
      HAL_I2C_Mem_Write(&hi2c1, addw, 0x6B, 1, &data, 1, 50);
      osDelay(50);
      HAL_I2C_Mem_Read(&hi2c1, addw, 0x75, 1, &check, 1, 50);
      //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
      printf("can't connect with IMU\n");
      osDelay(500);
      //printf("rebooting...\n");
      //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
      //__HAL_I2C_DISABLE(&hi2c1);
      //HAL_Delay(500);
      //__HAL_I2C_ENABLE(&hi2c1);
      //HAL_I2C_Mem_Write(&hi2c1, addw, 0x6B, 1, &data, 1, 1000);
      //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
      //HAL_Delay(500);
      //printf("check: %x add: %x\n", check, addw);
  }
  printf("connect success.\n");
  
  data = 0x00;          //gyro samplerate
  HAL_I2C_Mem_Write(&hi2c1, addw, 0x19, 1, &data, 1, 50);
  
  data = 0x06;        //DLF filter
  HAL_I2C_Mem_Write(&hi2c1, addw, 0x1A, 1, &data, 1, 50);
  
  data = 0x10;          //Accel scale bit[4, 3]
  //HAL_I2C_Mem_Write(&hi2c1, addw, 0x1C, 1, &data, 1, 50);
  
  data = 0x08;          //Gyro scale bit[4, 3]
  //HAL_I2C_Mem_Write(&hi2c1, addw, 0x1B, 1, &data, 1, 50);
  
  data = 0;             //Standby
  HAL_I2C_Mem_Write(&hi2c1, addw, 0x6B, 1, &data, 1, 50);
  osTimerStart(myTimer01Handle, 10);
  /* Infinite loop */
  for(;;)
  {
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* StartGYRO function */
void StartGYRO(void const * argument)
{
  /* USER CODE BEGIN StartGYRO */
  static osEvent R_buffer;
  static uint8_t ldata[14];
  static int16_t ax,ay,az,gx,gy,gz,CCCC;
  int8_t* pldata;
  /*
  pldata is anounced as global variable, why it is must be global Var??
  or, just JTAG data tracing is just about core so no permission?
  maybe just beacause i gave wrong value for address bit size
  
  
  */
  //uint8_t check = 1;
  
  /* Infinite loop */
  for(;;)
  {
    R_buffer = osMessageGet(myQueue01Handle, osWaitForever);
    pldata = R_buffer.value.p;
    for(int i = 0; i < 14; i++)
    {
      ldata[i] = (int8_t)*pldata;
      pldata++;
    }
    
    
    ax = (int16_t)(ldata[0] << 8 | ldata[1]);
    ay = (int16_t)(ldata[2] << 8 | ldata[3]);
    az = (int16_t)(ldata[4] << 8 | ldata[5]);
    CCCC = (int16_t)(ldata[6] << 8 | ldata[7]);
    gx = (int16_t)(ldata[8] << 8 | ldata[9]);
    gy = (int16_t)(ldata[10] << 8 | ldata[11]);
    gz = (int16_t)(ldata[12] << 8 | ldata[13]);
    
    ax /= ACC_DIV;
    ay /= ACC_DIV;
    az /= ACC_DIV;
    
    printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", ax, ay, az, gx, gy, gz);
    //printf("%d   %d   %d\r\n", gx, gy, gz);
    //printf("%d   %d   %d\r\n", ax, ay, az);
    if(ax != 0 | ay != 0 | az != 0| CCCC != 0| gx != 0| gy != 0 | gz != 0)
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    osDelay(100);
  }
  /* USER CODE END StartGYRO */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
  
  data = 0;             //Standby
  HAL_I2C_Mem_Write(&hi2c1, addw, 0x6B, 1, &data, 1, 50);
  HAL_I2C_Mem_Read(&hi2c1, addw, 0x3B, 1, (uint8_t*)pdata, 14, 50);
  osMessagePut(myQueue01Handle, (uint32_t)pdata , 0);
  
  /* USER CODE END Callback01 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
