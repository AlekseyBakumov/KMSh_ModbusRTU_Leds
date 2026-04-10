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
#include "Modbus_RTU_my.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RECEIVE_TIMEOUT         8999 // 5000 ticks at 25/5 MHz = 0,0002 sec.
#define MODBUS_DEVICE_ID        0x01

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define HAL_TIMEOUT_VALUE 0xFFFFFFFF
#define countof(a) (sizeof(a) / sizeof(*(a)))

volatile uint8_t ticks_timer;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
ModbusRTU_Handle_t hmodbus1;

uint8_t test_response[16]  = {0x01, 0x03, 0x02, 0x00, 0x19, 0x79, 0x8E};
uint8_t rx_temp;
uint8_t uart_buffer[256];
uint16_t uart_index = 0;

ModbusRTU_Package_t received;

uint8_t enabled_leds[8] = {0,0,0,0,0,0,0,0};
uint8_t blinking[8]     = {0,0,0,0,0,0,0,0};

uint16_t brightness[8] = {100, 100, 100, 100, 100, 100, 100, 100};
uint16_t intervals[8]  = {1000,1000,1000,1000,1000,1000,1000,1000};

uint8_t pwm_counter = 0;
uint32_t blink_counter = 0;

uint16_t LED_PIN[8] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13};
GPIO_TypeDef* LED_PORT[8] = {GPIOA, GPIOA, GPIOA, GPIOB, GPIOB, GPIOF, GPIOF, GPIOF};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Modbus_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    char msg[64];
    uint32_t err = huart->ErrorCode;

    if (huart->Instance == USART1) {
        if (err & HAL_UART_ERROR_ORE) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
            HAL_UART_Transmit(huart, (uint8_t*)"Err: Overrun (ORE)\r\n", 20, 100);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
            
            __HAL_UART_CLEAR_OREFLAG(huart);
        }
        if (err & HAL_UART_ERROR_NE) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
            HAL_UART_Transmit(huart, (uint8_t*)"Err: Noise (NE)\r\n", 17, 100);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
            
            __HAL_UART_CLEAR_NEFLAG(huart);
        }
        if (err & HAL_UART_ERROR_FE) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
            HAL_UART_Transmit(huart, (uint8_t*)"Err: Framing (FE)\r\n", 19, 100);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
            
            __HAL_UART_CLEAR_FEFLAG(huart);
        }
        if (err & HAL_UART_ERROR_PE) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
            HAL_UART_Transmit(huart, (uint8_t*)"Err: Parity (PE)\r\n", 18, 100);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
            
            __HAL_UART_CLEAR_PEFLAG(huart);
        }
    }
}

void UartTimer_Prolong() {
    
    __HAL_TIM_SET_AUTORELOAD(&htim1, RECEIVE_TIMEOUT);
    
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    
    SET_BIT(htim1.Instance->DIER, TIM_DIER_UIE);
    
    SET_BIT(htim1.Instance->CR1, TIM_CR1_CEN);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
      
        UartTimer_Prolong();
      
        uart_buffer[uart_index++] = rx_temp;
        
        HAL_UART_Receive_IT(huart, &rx_temp, 1);
        
        HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, GPIO_PIN_SET);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
      load_uart_message(&hmodbus1, uart_buffer, uart_index);
      
      if (process_Request(&hmodbus1) == PROCESS_OK) {
      
        uint8_t out_len;
        uint8_t* output_buffer = get_Output(&hmodbus1, &out_len);
      
      
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
            
      for(volatile int i=0; i<500; i++);    
      //HAL_UART_Transmit(&huart1, uart_buffer, uart_index, 100); // debug echo in uart
      HAL_UART_Transmit(&huart1, output_buffer, out_len, 100);
      //HAL_UART_Transmit(&huart1, test_response, 7, 100);
        
      while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
            
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
           
      } else {
        // ?
      }
      
      uart_index = 0;
            
      HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOK, GPIO_PIN_0, GPIO_PIN_SET);
    }
    
    if (htim->Instance == TIM3) {
      pwm_counter++;
      if (pwm_counter > 100) pwm_counter = 0;
      blink_counter++;
      if (blink_counter > 100000) blink_counter = 0;
      
      for (int i = 0; i < 8; i++) {
        if (enabled_leds[i] == 0) {
          LED_PORT[i]->BSRR = (uint32_t)LED_PIN[i] << 16;
          continue;
        }
        
        if (blinking[i]) {
          if (intervals[i] > ((blink_counter/10) % (2 * intervals[i]))) {
            LED_PORT[i]->BSRR = (uint32_t)LED_PIN[i] << 16;
            continue;
          }
        }
        
        if (pwm_counter < brightness[i]) {
          LED_PORT[i]->BSRR = LED_PIN[i];
        } else {
          LED_PORT[i]->BSRR = (uint32_t)LED_PIN[i] << 16;
        }
      }
    }
}

uint8_t registers_Mapping_Read(uint16_t addr, uint16_t* val, uint8_t mode) {
  switch(mode) {
  case READ_MODE_COILS: // 0x01
    
    // Example of switch-case mapping:
    /*
    switch(addr) {
    case 0x0000: 
      *val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
      return 0;
    case 0x0001: 
      *val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
      return 0;
    case 0x0002: 
      *val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
      return 0;
    case 0x0003: 
      *val = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
      return 0;
    case 0x0004: 
      *val = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
      return 0;
    case 0x0005:
      *val = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11);
      return 0;
    case 0x0006:
      *val = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12);
      return 0;
    case 0x0007:
      *val = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13);
      return 0;
    */
    
      // Array mapping:
      if (addr >= 0x0000 && addr <= 0x0007) {
        *val = enabled_leds[addr];
        return 0;
      }
      if (addr >= 0x0010 && addr <= 0x0017) {
        *val = blinking[addr-16];
        return 0;
      }
      return 2;
      
  case READ_MODE_DISCRETE_INP: // 0x02
    switch(addr) {
    case 0x0000: 
      *val = GPIO_PIN_SET; // Testing input
      return 0;
    default:
      return 2;
    }
    
  case READ_MODE_HOLDING_REGS: // 0x03
    if (addr >= 0x0000 && addr <= 0x0007) {
        *val = brightness[addr];
        return 0;
      }
      if (addr >= 0x0010 && addr <= 0x0017) {
        *val = intervals[addr-16];
        return 0;
      }
      return 2;
    
  case READ_MODE_INPUT_REGS: // 0x04
    switch(addr) {
    case 0x0000: 
      *val = 12345; // Also testing
      return 0;
    case 0x0001: 
      *val = 54321;
      return 0;
    default: return 2;
    }
  }
  return 5;
}

uint8_t registers_Mapping_Write(uint16_t addr, uint16_t val, uint8_t mode) {
  switch(mode) {
  case WRITE_MODE_SINGLE_COIL: // 0x05
    
    /*
    if (val != 0xFF00 && val != 0x0000) return 3;
    switch(addr) {
    case 0x0000: 
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, val == 0xFF00 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      return 0;
    case 0x0001: 
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, val == 0xFF00 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      return 0;
    case 0x0002: 
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, val == 0xFF00 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      return 0;
    case 0x0003: 
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, val == 0xFF00 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      return 0;
    case 0x0004: 
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, val == 0xFF00 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      return 0;
    case 0x0005:
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, val == 0xFF00 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      return 0;
    case 0x0006:
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, val == 0xFF00 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      return 0;
    case 0x0007:
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, val == 0xFF00 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      return 0;
    default: return 2;
  }
  */
    if (addr >= 0x0000 && addr <= 0x0007) {
        enabled_leds[addr] = val == 0xFF00;
        return 0;
      }
      if (addr >= 0x0010 && addr <= 0x0017) {
        blinking[addr-16] = val == 0xFF00;
        return 0;
      }
      return 2;
    
  case WRITE_MODE_SINGLE_REG: // 0x06
    if (addr >= 0x0000 && addr <= 0x0007) {
        if (val > 100) return 3;
        brightness[addr] = val;
        return 0;
      }
      if (addr >= 0x0010 && addr <= 0x0017) {
        if (val > 5000) return 3;
        intervals[addr-16] = val;
        return 0;
      }
      return 2;
    
  case WRITE_MODE_MULT_COILS: // 0x0f
    // Actually, I don`t need those two. Multiple or single write is processed in
    // library, in case of multiple write it`ll just call single write multiple times.
    break;
    
  case WRITE_MODE_MULT_REGS: // 0x10
    
    break;
  }
  
  return 5;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  //while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  //if ( timeout < 0 )
  //{
  //Error_Handler();
  //}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Modbus_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
//while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
//if ( timeout < 0 )
//{
//Error_Handler();
//}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, GPIO_PIN_RESET);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
    
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PF11 PF12 PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PK0 PK1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Modbus_Init(void)
{
  init_ModbusRTU_Handle(&hmodbus1, MODBUS_DEVICE_ID);
  set_Read_Handler(&hmodbus1, &registers_Mapping_Read);
  set_Write_Handler(&hmodbus1, &registers_Mapping_Write);
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
