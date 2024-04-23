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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <stdarg.h>


#define FW_NAME "Hauler"
#define FW_VERSION "1.00"
#define TAG "main"

#define SYS_CLOCK_FREQ_50_MHZ  50
#define SYS_CLOCK_FREQ_84_MHZ  84
#define SYS_CLOCK_FREQ_120_MHZ 120
#define SYS_CLOCK_FREQ_180_MHZ 180

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_RESET   "\x1b[0m"


#define true  1
#define false 0

#define OSC_MODE_INT 0
#define OSC_MODE_EXT 1



int dutyX =50;
int  dutyY =68;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t recievd_time ;
uint32_t timeout =2000;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

CAN_RxHeaderTypeDef RxHeader;

int msg=0;





typedef struct{
	uint32_t id;
	uint8_t msg[8];
	uint32_t length;
	uint8_t isRxcvd;
}Rx_CAN_Info;

Rx_CAN_Info rx_can_info;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_USART2_UART_Init(void);
void UART1_Init(void);
void UART2_Init(void);
void Error_handler(void);
int con(int);

//Logging functions using in-built USB
void LogD(const char *tag,const char *format, ...);//Debug
void LogI(const char *tag,const char *format, ...);//Info
void LogW(const char *tag,const char *format, ...);//Warning
void LogE(const char *tag,const char *format, ...);//Error
void LogArray(unsigned char *arr, int length,const char *tag, const char *format, ...);
void byte2HexNbl(unsigned char *dest, unsigned char *data, int start, int end, char separator, int append);
void CAN1_Init(void);
void CAN2_Init(void);
void CAN_Filter_Config(void);
void CAN1_Send(uint32_t id,uint8_t msg[],uint32_t len);
void CAN2_Send(uint32_t id,uint8_t msg[],uint32_t len);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
HAL_StatusTypeDef CAN1_Start(void);
HAL_StatusTypeDef CAN2_Start(void);



uint8_t txmsg[] = {0x65,0x00,0x00,0x00,0x00,0x00,0x00,0x00};



void TIMER6_Init(void);
void TIMER7_Init(void);
//static void MX_TIM6_Init(void);
//static void MX_TIM7_Init(void);
int con(int);
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
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
//  MX_USART2_UART_Init();
  UART2_Init();
  CAN1_Init();
  CAN2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  TIMER6_Init();



  TIMER7_Init();
  LogI(TAG, (const char *)"Welcome to NAVCON Hauler Application: %s-V%s",FW_NAME,FW_VERSION);
 	LogD(TAG, (const char *)"SYSCLK : %ldHz",HAL_RCC_GetSysClockFreq());
 	LogD(TAG, (const char *)"HCLK   : %ldHz",HAL_RCC_GetHCLKFreq());
 	LogD(TAG, (const char *)"PCLK1  : %ldHz",HAL_RCC_GetPCLK1Freq());
 	LogD(TAG, (const char *)"PCLK2  : %ldHz",HAL_RCC_GetPCLK2Freq());
 	 CAN_Filter_Config();
 	if (CAN1_Start() != HAL_OK)
 	{  // CAN 1 START
 	  	Error_Handler();
 	  	}
 	if (CAN2_Start() != HAL_OK)
 	 	{  // CAN 1 START
 	 	  	Error_Handler();
 	 	  	}


 	  TIM6->SR = 0;
 	  	HAL_TIM_Base_Start_IT(&htim6); // TIMER 6 START

 	  	TIM7->SR = 0;
 	  		HAL_TIM_Base_Start_IT(&htim7); // TIMER 7 START
//  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_SET_AUTORELOAD(&htim2,2000);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_AUTORELOAD(&htim3,2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	   if ((HAL_GetTick() - recievd_time) > timeout)
	        {
	            // Timeout occurred, set variable to default value
	           dutyX = 50;
	           dutyY = 68;
//	           NVIC_SystemReset();
	        }
	   else

    /* USER CODE BEGIN 3 */
//	  CAN1_Send(0x101, txmsg, 8);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,con(dutyX));

	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,con(dutyY));
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM6){	// TIMER 6 THREAD
		uint8_t txmsg[] = {0x65,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//		LogD(TAG, (const char *)"TIm6 working ");
//		CAN1_Send(0x100, txmsg, 8);


	}
	else if ((htim->Instance == TIM7)) // TIMER 7 THREAD
	{
//		LogD(TAG, (const char *)" TIm7 working  ");
	}
}


/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 450-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);


}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 450-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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

void UART1_Init(void){
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if ( HAL_UART_Init(&huart1) != HAL_OK )
	{
		//There is a problem
		Error_handler();
	}
}

void UART2_Init(void){
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	if ( HAL_UART_Init(&huart2) != HAL_OK )
	{
		//There is a problem
		Error_handler();
	}
}

void Error_handler(void){

	while(1);
}

void LogD(const char *tag,const char *format, ...){
	va_list arg_ptr;
	char logstr[255],lineBuff[300];
	va_start(arg_ptr, format);
	vsnprintf(logstr, 255, format, arg_ptr);
	va_end(arg_ptr);
	sprintf(lineBuff,"%s %lu [%s] %s %s\r\n",ANSI_COLOR_BLUE,HAL_GetTick(),tag,logstr,ANSI_COLOR_RESET);
	HAL_UART_Transmit(&huart2,(uint8_t*)lineBuff,strlen(lineBuff),HAL_MAX_DELAY);
}

void LogI(const char *tag,const char *format, ...){
	va_list arg_ptr;
	char logstr[255],lineBuff[300];
	va_start(arg_ptr, format);
	vsnprintf(logstr, 255, format, arg_ptr);
	va_end(arg_ptr);
	sprintf(lineBuff,"%s %lu [%s] %s %s\r\n",ANSI_COLOR_GREEN,HAL_GetTick(),tag,logstr,ANSI_COLOR_RESET);
	HAL_UART_Transmit(&huart2,(uint8_t*)lineBuff,strlen(lineBuff),HAL_MAX_DELAY);
}

void LogW(const char *tag,const char *format, ...){
	va_list arg_ptr;
	char logstr[255],lineBuff[300];
	va_start(arg_ptr, format);
	vsnprintf(logstr, 255, format, arg_ptr);
	va_end(arg_ptr);
	sprintf(lineBuff,"%s %lu [%s] %s %s\r\n",ANSI_COLOR_YELLOW,HAL_GetTick(),tag,logstr,ANSI_COLOR_RESET);
	HAL_UART_Transmit(&huart2,(uint8_t*)lineBuff,strlen(lineBuff),HAL_MAX_DELAY);
}

void LogE(const char *tag,const char *format, ...){
	va_list arg_ptr;
	char logstr[255],lineBuff[300];
	va_start(arg_ptr, format);
	vsnprintf(logstr, 255, format, arg_ptr);
	va_end(arg_ptr);
	sprintf(lineBuff,"%s %lu [%s] %s %s\r\n",ANSI_COLOR_RED,HAL_GetTick(),tag,logstr,ANSI_COLOR_RESET);
	HAL_UART_Transmit(&huart2,(uint8_t*)lineBuff,strlen(lineBuff),HAL_MAX_DELAY);
}

void LogArray(unsigned char *arr, int length,const char *tag, const char *format, ...){
	va_list arg_ptr;
	char logstr[255],lineBuff[300];
	if(length>200){
		LogE((const char *)"DBG",(const char *)"print array length(%d) avoided to prevent segmentation error", length);
		return;
	}
	va_start(arg_ptr, format);
	vsnprintf(logstr, 255, format, arg_ptr);
	va_end(arg_ptr);
	byte2HexNbl((unsigned char *) logstr, (unsigned char *) arr, 0, length, ' ', 1);
	sprintf(lineBuff,"%s %lu [%s] %s %s\r\n",ANSI_COLOR_GREEN,HAL_GetTick(),tag,logstr,ANSI_COLOR_RESET);
	HAL_UART_Transmit(&huart2,(uint8_t*)lineBuff,strlen(lineBuff),HAL_MAX_DELAY);
}
void TIMER6_Init(void){
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 1024;
	htim6.Init.Period = 8780.48780487805-1;
	if( HAL_TIM_Base_Init(&htim6) != HAL_OK ){
		LogE(TAG,(const char*)"Error while initializing TIM6");
		Error_handler();
	}
	LogI(TAG,(const char*)"TIM6_Initialized ");
}

void TIMER7_Init(void)
{

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1024;
  htim7.Init.Period = 8780.48780487805-1;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
	  LogE(TAG,(const char*)"Error while initializing TIM6");
    Error_handler();
  }
  LogI(TAG,(const char*)"TIM7_Initialized ");

}


void CAN1_Init(void){
	//	memset(&rx_can_info,0,sizeof(rx_can_info));
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	//Settings related to CAN bit timings
	hcan1.Init.Prescaler = 5;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;

	if ( HAL_CAN_Init (&hcan1) != HAL_OK){
		LogE(TAG,(const char *)"Error while Initializing" );
		Error_handler();
	}else
		LogI(TAG,(const char *)"Initialized Successfully" );

}

void CAN_Filter_Config(void){
	CAN_FilterTypeDef can1_filter_init;
	CAN_FilterTypeDef can2_filter_init;


	can1_filter_init.FilterActivation = ENABLE;
	can1_filter_init.FilterBank  = 0;
	can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
	can1_filter_init.FilterIdHigh = 0;
	can1_filter_init.FilterIdLow = 0;
	can1_filter_init.FilterMaskIdHigh = 0;
	can1_filter_init.FilterMaskIdLow = 0;
	can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	can1_filter_init.FilterScale = CAN_FILTERSCALE_16BIT;
	can1_filter_init.SlaveStartFilterBank = 14;

	if( HAL_CAN_ConfigFilter(&hcan1,&can1_filter_init) != HAL_OK){
		LogE(TAG,(const char *)"Error while ConfigFilter CAN1 " );
		Error_handler();
	}else
		LogI(TAG,(const char *)"Configured Filter for CAN1" );

	can2_filter_init.FilterActivation = ENABLE;
	can2_filter_init.FilterBank  = 14;
	can2_filter_init.FilterFIFOAssignment = CAN_RX_FIFO1;
	can2_filter_init.FilterIdHigh = 0;
	can2_filter_init.FilterIdLow = 0;
	can2_filter_init.FilterMaskIdHigh = 0;
	can2_filter_init.FilterMaskIdLow = 0;
	can2_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	can2_filter_init.FilterScale = CAN_FILTERSCALE_16BIT;
	can2_filter_init.SlaveStartFilterBank = 14;

	if( HAL_CAN_ConfigFilter(&hcan2,&can2_filter_init) != HAL_OK){
		LogE(TAG,(const char *)"Error while ConfigFilter CAN2" );
		Error_handler();
	}else
		LogI(TAG,(const char *)"Configured Filter for CAN2" );
}

HAL_StatusTypeDef CAN1_Start(void){

	HAL_StatusTypeDef ret_stats=HAL_OK;
	if((ret_stats = HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF))!= HAL_OK){
		LogE(TAG,(const char *)"Error:%d HAL_CAN_ActivateNotification",ret_stats );
		return ret_stats;
	}
	if((ret_stats = HAL_CAN_Start(&hcan1)) != HAL_OK){
		LogE(TAG,(const char *)"Error:%d HAL_CAN_Start",ret_stats );
		return ret_stats;
	}
	LogI(TAG,(const char *)"CAN 1 Started Successfully");
	return ret_stats;
}

void CAN1_Send(uint32_t id,uint8_t msg[],uint32_t len){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.DLC = len;
	TxHeader.StdId = id;
	TxHeader.IDE   = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,msg,&TxMailbox) != HAL_OK){
		LogE(TAG,(const char *)"Error while Adding Message:%#x",id );
		//Error_handler();
	}else{
		LogI(TAG,(const char *)"Added Message:%#x",id );
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	LogD(TAG,(const char *)"CAN Message Transmitted:M0");
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	LogD(TAG,(const char *)"CAN Message Transmitted:M1");
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	LogD(TAG,(const char *)"CAN Message Transmitted:M2");
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rcvd_msg[8];
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxHeader,rcvd_msg) != HAL_OK){
		Error_handler();
	}
	uint32_t recievd_time = HAL_GetTick();
	rx_can_info.id = RxHeader.ExtId;
	rx_can_info.length = RxHeader.DLC;
	memcpy(rx_can_info.msg,rcvd_msg,RxHeader.DLC);
	rx_can_info.isRxcvd = true;
	LogArray((unsigned char *)rcvd_msg,RxHeader.DLC, TAG,(const char *)"Received CAN ID : %#x message status %x:",rx_can_info.id,rcvd_msg);
		LogArray((unsigned char *)rcvd_msg,RxHeader.DLC, TAG,(const char *)"%d",recievd_time);

	   if(rx_can_info.id == 0x100)
	{
	dutyX = rcvd_msg[0];
	dutyY = rcvd_msg[1];
	LogArray((unsigned char *)rcvd_msg,RxHeader.DLC, TAG,(const char *)"Received CAN ID : %#x message status: :",rx_can_info.id,rcvd_msg);


	}

}



void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rcvd_msg[8];
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO1,&RxHeader,rcvd_msg) != HAL_OK){
			Error_handler();
		}
	rx_can_info.id = RxHeader.ExtId;
	rx_can_info.length = RxHeader.DLC;
	memcpy(rx_can_info.msg,rcvd_msg,RxHeader.DLC);
	rx_can_info.isRxcvd = true;

	LogArray((unsigned char *)rcvd_msg,RxHeader.DLC, TAG,(const char *)"Received CAN ID : %#x message status %x:",rx_can_info.id,rcvd_msg);
 if(RxHeader.ExtId==0x933)
 {
	 CAN1_Send(0x200, rcvd_msg, 8);
 }

}


int con(int input) {
    return 1800 - (input*2);
}


void CAN2_Init(void)
{

	hcan2.Instance = CAN2;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.AutoBusOff = ENABLE;
	hcan2.Init.AutoRetransmission = ENABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
	//Settings related to CAN bit timings
	hcan2.Init.Prescaler = 5;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_15TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;

	if ( HAL_CAN_Init (&hcan2) != HAL_OK){
		LogE(TAG,(const char *)"Error while Initializing" );
		Error_handler();
	}else
		LogI(TAG,(const char *)"Initialized Successfully" );

}

HAL_StatusTypeDef CAN2_Start(void){

	HAL_StatusTypeDef ret_stats=HAL_OK;
	if((ret_stats = HAL_CAN_ActivateNotification(&hcan2,CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO1_MSG_PENDING|CAN_IT_BUSOFF))!= HAL_OK){
		LogE(TAG,(const char *)"Error:%d HAL_CAN_ActivateNotification",ret_stats );
		return ret_stats;
	}
	if((ret_stats = HAL_CAN_Start(&hcan2)) != HAL_OK){
		LogE(TAG,(const char *)"Error:%d HAL_CAN_Start",ret_stats );
		return ret_stats;
	}
	LogI(TAG,(const char *)"CAN Started Successfully");
	return ret_stats;
}


void CAN2_Send(uint32_t id,uint8_t msg[],uint32_t len){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.DLC = len;
	TxHeader.ExtId = id;
	TxHeader.IDE   = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;

	if( HAL_CAN_AddTxMessage(&hcan2,&TxHeader,msg,&TxMailbox) != HAL_OK){
		LogE(TAG,(const char *)"Error while Adding Message CAN2:%#x",id );
		//Error_handler();
	}else{
		LogI(TAG,(const char *)"Added Message CAN2:%#x",id );
	}
}


void byte2HexNbl(unsigned char *dest, unsigned char *data, int start, int end, char separator, int append){
	int i, j;
	const char* pNibbleHex = {"0123456789ABCDEF"};
	int ndxMul = 2;
	int sepLen = (separator == '\0') ? 1 : end - start;
	int nLength = ((end - start) * 2) + sepLen;
	char pBuffer[255];
	pBuffer[0] = 0;
	pBuffer[nLength] = 0;
	if (sepLen > 1)
		ndxMul = 3;
	for (i = start, j = 0; i < end; i++, j++) {
		// divide by 16
		int nNibble = *(data + i) >> 4;
		pBuffer[ndxMul * j] = pNibbleHex[nNibble];

		nNibble = *(data + i) & 0x0F;
		pBuffer[ndxMul * j + 1] = pNibbleHex[nNibble];

		if (sepLen > 1)
			pBuffer[ndxMul * j + 2] = separator;
	}
	if (append == 1)
		strcat((char*) dest, (char*) pBuffer);

	else
		strcpy((char*) dest, (char*) pBuffer);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */


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
