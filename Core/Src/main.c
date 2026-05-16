/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

COMP_HandleTypeDef hcomp4;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_COMP4_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int32_t count_1 = 0;
volatile int32_t count_2 = 0;

uint8_t TxData[8];
void CAN_TX(uint32_t recipient)
{
  // 送信用インスタンス等
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;

  // 送信メールボックスに空きがあったら送信開始
  if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan))
  {
    // 送信用インスタンスの設定
    TxHeader.StdId = recipient; // 受取手のCANのID
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8; // データ長を8byteに設定
    TxHeader.TransmitGlobalTime = DISABLE;
    // 各データ
    TxData[0] = (count_1 >> 24) & 0xFF;
    TxData[1] = (count_1 >> 16) & 0xFF;
    TxData[2] = (count_1 >> 8) & 0xFF;
    TxData[3] = count_1 & 0xFF;
    TxData[4] = (count_2 >> 24) & 0xFF;
    TxData[5] = (count_2 >> 16) & 0xFF;
    TxData[6] = (count_2 >> 8) & 0xFF;
    TxData[7] = count_2 & 0xFF;
    // CANメッセージを送信
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

typedef struct
{
  uint8_t data[8];
} CanFrame;

volatile CanFrame rx_queue[8];
volatile uint8_t head;
volatile uint8_t tail;

volatile uint32_t id;
// volatile uint8_t use_data[8];
volatile uint8_t can_buf[8];
volatile uint8_t can_updated = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader; // 受信メッセージの情報が格納されるインスタンス
  uint8_t RxData[8];            // 受信したデータを一時保存する配列
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    id = RxHeader.StdId; // RxHeaderの中に入っているidを取り出す
    if (id == 0x101)
    {
      for (int i = 0; i <= 7; i++)
      {
        can_buf[i] = RxData[i];
      }
      can_updated = 1;
    }
  }
}

volatile int8_t flag = 0;
uint32_t uwIncrementValue = 15000;
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  // CH4 (CC4) の割り込みか判定
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    // 現在のCCR4の値を取得
    uint32_t current_ccr = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

    // 次に割り込みを発生させるカウント数を計算（オーバーフロー対策）
    uint32_t next_ccr = (current_ccr + uwIncrementValue) & 0xFFFF;

    // 次の比較値を設定
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, next_ccr);
    // ここにTIM3_CH4の割り込み処理を記述する
    flag = 1;
  }
}

volatile uint8_t prev_state_1 = 0;
volatile uint8_t prev_state_2 = 0;

void __attribute__((section(".ramfunc"))) Encodercounter_1()
{
  uint32_t gpioa = GPIOA->IDR;
  uint8_t a = (gpioa & GPIO_PIN_3) ? 1 : 0;
  uint8_t b = (gpioa & GPIO_PIN_10) ? 1 : 0;

  uint8_t curr_state = (a << 1) | b;

  uint8_t transition = (prev_state_1 << 2) | curr_state; //+1-1を決める

  // 正しい遷移のみカウント
  switch (transition)
  {
  // 正回転
  case 0b0001:
  case 0b0111:
  case 0b1110:
  case 0b1000:
    count_1++;
    break;

  // 逆回転
  case 0b0010:
  case 0b0100:
  case 0b1101:
  case 0b1011:
    count_1--;
    break;

  default:
    // ノイズなど（無視）
    break;
  }
  prev_state_1 = curr_state;
}

void __attribute__((section(".ramfunc"))) Encodercounter_2()
{
  uint32_t gpioa = GPIOA->IDR;
  uint32_t gpiob = GPIOB->IDR;
  uint8_t a = (gpioa & GPIO_PIN_7) ? 1 : 0;
  uint8_t b = (gpiob & GPIO_PIN_3) ? 1 : 0;

  uint8_t curr_state = (a << 1) | b;

  uint8_t transition = (prev_state_2 << 2) | curr_state; //+1-1を決める

  // 正しい遷移のみカウント
  switch (transition)
  {
  // 正回転
  case 0b0001:
  case 0b0111:
  case 0b1110:
  case 0b1000:
    count_2++;
    break;

  // 逆回転
  case 0b0010:
  case 0b0100:
  case 0b1101:
  case 0b1011:
    count_2--;
    break;

  default:
    // ノイズなど（無視）
    break;
  }
  prev_state_2 = curr_state;
}

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
  return len;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  setbuf(stdout, NULL);
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
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_COMP4_Init();
  MX_DAC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_COMP_Start(&hcomp4); // COMP4を起動
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2122); // 1.71v

  // CANスタート
  HAL_CAN_Start(&hcan);
  // 割り込み有効
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);

  uint8_t last_state_1 = GPIO_PIN_RESET;
  uint8_t last_state_2 = GPIO_PIN_RESET;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t local_buf[8];

    if (can_updated)
    {
      __disable_irq();
      memcpy(local_buf, can_buf, 8);
      can_updated = 0;
      __enable_irq();

      int16_t v1, v2, v3, v4;
      // int8_t dir1, dir2, dir3, dir4;

      v1 = (int16_t)((local_buf[0] << 8) | local_buf[1]);
      v2 = (int16_t)((local_buf[2] << 8) | local_buf[3]);
      v3 = (int16_t)((local_buf[4] << 8) | local_buf[5]);
      v4 = (int16_t)((local_buf[6] << 8) | local_buf[7]);

      int pwm1 = abs(v1);
      int pwm2 = abs(v2);
      int pwm3 = abs(v3);
      int pwm4 = abs(v4);

      if (pwm1 > 2999)
        pwm1 = 2999;
      if (pwm2 > 2999)
        pwm2 = 2999;
      if (pwm3 > 2999)
        pwm3 = 2999;
      if (pwm4 > 2999)
        pwm4 = 2999;

      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm2);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm3);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm4);

      GPIO_PinState dir1 = v1 < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
      GPIO_PinState dir2 = v2 < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
      GPIO_PinState dir3 = v3 < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
      GPIO_PinState dir4 = v4 < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, dir1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, dir2);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, dir3);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, dir4);

      // デバック出力
      /* printf("CNT=%lu CCR1=%lu\n",
                   TIM3->CNT,
                   TIM3->CCR1);
            TIM3->CCR1 = 1500;
            HAL_Delay(500);*/

      printf("pwm1:%d pwm2:%d pwm3:%d pwm4:%d\n", pwm1, pwm2, pwm3, pwm4);
      printf("dir1:%d dir2:%d dir3:%d dir4:%d\n", dir1, dir2, dir3, dir4);

      can_updated = 0;
    }

    if (flag == 1)
    {
      flag = 0;
      CAN_TX(0x100); // stmのid
    }
    /*uint8_t current_state_1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);

        // 前回がLow(Reset)で、今回がHigh(Set)なら立ち上がり
        if (last_state_1 == GPIO_PIN_RESET && current_state_1 == GPIO_PIN_SET)
        {
          // 立ち上がり検出時の処理
          Encodercounter_1();
        }
        last_state_1 = current_state_1;

        uint8_t current_state_2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);

        // 前回がLow(Reset)で、今回がHigh(Set)なら立ち上がり
        if (last_state_2 == GPIO_PIN_RESET && current_state_2 == GPIO_PIN_SET)
        {
          // 立ち上がり検出時の処理
          Encodercounter_2();
        }
        last_state_2 = current_state_2;

        // 少し待機（チャタリング対策やCPU負荷低減）
        HAL_Delay(10);*/

    /* //  比較結果を取得 (1: INP > INM, 0: INP < INM ※極性設定による)
        if (HAL_COMP_GetOutputLevel(&hcomp4) == COMP_OUTPUTLEVEL_HIGH)
        {
          // ハイレベル時の処理
          printf("HIGH");
          printf("\n");
        }
        else
        {
          // ローレベル時の処理
          printf("LOW");
          printf("\n");
        }

        HAL_Delay(10);*/
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL15;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  HAL_NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);

  CAN_FilterTypeDef filter;
  filter.FilterIdHigh = 0x101 << 5;
  filter.FilterMaskIdHigh = 0x7FF << 5;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;     // 16モード
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // FIFO0へ格納
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.SlaveStartFilterBank = 14;
  filter.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &filter);
  /* USER CODE END CAN_Init 2 */
}

/**
 * @brief COMP4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */

  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */

  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH2;
  hcomp4.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp4.Init.Output = COMP_OUTPUT_NONE;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */

  /* USER CODE END COMP4_Init 2 */
}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
   */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
   */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  // sConfig.DAC_OutputSwitch = DAC_OUTPUTSWITCH_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /*
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  ↓ 下に変える

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
*/

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_3)
  {
    // onInterrupt_1();
    Encodercounter_1();
  }
  if (GPIO_Pin == GPIO_PIN_7)
  {
    // onInterrupt_2();
    Encodercounter_2();
  }
}
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
#ifdef USE_FULL_ASSERT
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
