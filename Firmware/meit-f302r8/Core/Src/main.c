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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "meit.h"
#include "mux.h"
#include "probe.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum 
{
  TERM_CMD_NONE,
  TERM_CMD_UNKNOWN,

  TERM_CMD_SET,
  TERM_CMD_GET,

  TERM_CMD_FSAMPLING,

  TERM_CMD_MEASUREMENT,
  TERM_CMD_NEGATIVE,
  TERM_CMD_POSITIVE,
  TERM_CMD_REFERENCE,

  TERM_CMD_GAIN,
  TERM_CMD_INCREASE,
  TERM_CMD_DECREASE,

  TERM_CMD_CALIBRATION,
} TermCommand_Type;

typedef volatile struct
{
  TermCommand_Type action;    // GET or SET
  TermCommand_Type subject;   // The specific command
  // Optional argument, used for commands that require a numerical parameter
  uint32_t arg1;
  uint32_t arg2;
} TermHandlerFlag_Type;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_SIZE 1000
#define UART_RX_BUFFER_SIZE 40 
#define PROBES_N 1 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// main flags definition
// type should be volatile as they are accessed in interrupt callbacks
volatile uint32_t adc_buffer[ADC_BUFFER_SIZE];
volatile uint8_t adc_complete_flag = 0;

volatile uint8_t uart_buffer[UART_RX_BUFFER_SIZE];
TermHandlerFlag_Type term_handler_flag = {TERM_CMD_NONE, TERM_CMD_NONE, 0};

// meit modules declaration
Mux_TypeDef mux1;
Probe_TypeDef probes[PROBES_N];

// trecking currenty selected probe/probe with MEASUREMENT mode
extern int probe_mode_numbers[];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef LOG_SEMIHOSTING
extern void initialise_monitor_handles(void);
#endif

HAL_StatusTypeDef adc_read(void);
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
#ifdef LOG_SEMIHOSTING
  // semihosting init
  initialise_monitor_handles();
#endif
  log("__________ System initializing... __________\n");
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
  MX_SPI3_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // multiplexer module initialization
  mux_init(&mux1, 0);
  // mux_select(&mux1, PROBE_MODE_MEASUREMENT, 0, PROBE_MODE_RESET);

  // probe initialization
  for (size_t i = 0; i < (sizeof(probes) / sizeof(probes[0])); i++)
  {
    probe_init(&probes[i], &hspi3, &mux1, i);
    // probe_error_print(&probes[i]);
  }
  
  // start the uart interface
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)uart_buffer, UART_RX_BUFFER_SIZE);
  HAL_TIM_Base_Start_IT(&htim6);

  // DEBUG
  probe_mode(&probes[0], PROBE_MODE_MEASUREMENT, PROBE_MODE_SET);
  // probe_gain_specific(&probes[0], PROBE_GAINSET_1, PROBE_RANGESET_20V);
  probe_gain(&probes[0], 14);

  log("__________ System initialized __________\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (term_handler_flag.action != TERM_CMD_NONE)
    {
      // Buffer to hold each formatted adc value plus comma and possible null-terminator
      char txBuffer[UART_RX_BUFFER_SIZE]; 

      switch (term_handler_flag.action)
      {
        case TERM_CMD_SET:
          switch (term_handler_flag.subject)
          {
            case TERM_CMD_FSAMPLING:
              log("UART INPUT: SET FSAMPLING %ld\r\n", term_handler_flag.arg1);
              HAL_TIM_Base_Stop(&htim6);
              htim6.Instance->PSC = term_handler_flag.arg1;
            break;

            case TERM_CMD_MEASUREMENT:
              log("UART INPUT: SET MEASUREMENT\r\n");
              if (term_handler_flag.arg1 <= PROBES_N)
                probe_mode(&probes[term_handler_flag.arg1], PROBE_MODE_MEASUREMENT, PROBE_MODE_SET);
              else
                log("ERROR: Probe with number:%ld is out of reach\r\n", term_handler_flag.arg1);
            break;

            case TERM_CMD_NEGATIVE:
              log("UART INPUT: SET NEGATIVE\r\n");
              if (term_handler_flag.arg1 <= PROBES_N)
                probe_mode(&probes[term_handler_flag.arg1], PROBE_MODE_CURRENT_NEGATIVE, PROBE_MODE_SET);
              else
                log("ERROR: Probe with number:%ld is out of reach\r\n", term_handler_flag.arg1);
            break;

            case TERM_CMD_POSITIVE:
              log("UART INPUT: SET POSITIVE\r\n");
              if (term_handler_flag.arg1 <= PROBES_N)
                probe_mode(&probes[term_handler_flag.arg1], PROBE_MODE_CURRENT_POSITIVE, PROBE_MODE_SET);
              else
                log("ERROR: Probe with number:%ld is out of reach\r\n", term_handler_flag.arg1);
            break;

            case TERM_CMD_REFERENCE:
              log("UART INPUT: SET REFERENCE\r\n");
              if (term_handler_flag.arg1 <= PROBES_N)
                probe_mode(&probes[term_handler_flag.arg1], PROBE_MODE_REFERENCE, PROBE_MODE_SET);
              else
                log("ERROR: Probe with number:%ld is out of reach\r\n", term_handler_flag.arg1);
            break;

            case TERM_CMD_GAIN:
              log("UART INPUT: SET GAIN\r\n");
              if(probe_mode_numbers[PROBE_MODE_MEASUREMENT] < 0)
                log("ERROR: No probe in MEASUREMENT mode.\r\n");
              else
                probe_gain(&probes[probe_mode_numbers[PROBE_MODE_MEASUREMENT]], term_handler_flag.arg1);
            break;

            case TERM_CMD_INCREASE:
              log("UART INPUT: SET INCREASE\r\n");
              if(probe_mode_numbers[PROBE_MODE_MEASUREMENT] < 0)
                log("ERROR: No probe in MEASUREMENT mode.\r\n");
              else
                probe_gain_increase(&probes[probe_mode_numbers[PROBE_MODE_MEASUREMENT]]);
            break;

            case TERM_CMD_DECREASE:
              log("UART INPUT: SET DECREASE\r\n");
              if(probe_mode_numbers[PROBE_MODE_MEASUREMENT] < 0)
                log("ERROR: No probe in MEASUREMENT mode.\r\n");
              else
                probe_gain_decrease(&probes[probe_mode_numbers[PROBE_MODE_MEASUREMENT]]);
            break;

            case TERM_CMD_CALIBRATION:
              log("UART INPUT: SET CALIBRATION\r\n");
              if(probe_mode_numbers[PROBE_MODE_MEASUREMENT] < 0)
                log("ERROR: No probe in MEASUREMENT mode.\r\n");
              else if (term_handler_flag.arg1 >= (2^8))
                log("ERROR: Probe gain index:%ld is out of reach\r\n", term_handler_flag.arg1);
              else
                probe_calibrate(&probes[probe_mode_numbers[PROBE_MODE_MEASUREMENT]], (float)term_handler_flag.arg2, term_handler_flag.arg1);
            break;

            default:
              log("UART ERROR: Unknown Command\r\n");
            break;
          }
          break;

        case TERM_CMD_GET:
          switch (term_handler_flag.subject)
          {
            case TERM_CMD_FSAMPLING:
              log("UART INPUT: GET FSAMPLING\r\n");

              // Calculate ADC sampling frequency, ADC is triggered by Timer 6
              // uint32_t adcSamplingFreq = HAL_RCC_GetPCLK1Freq() / (TIM6->PSC + 1) / (TIM6->ARR + 1);
              uint8_t n_chanels = 2;
              float clock = (float)HAL_RCC_GetPCLK1Freq() * 2.0f;
              float adcSamplingFreq = clock / (float)((1.5f + 12.5f) * (TIM6->PSC + 1) * (TIM6->ARR + 1) * n_chanels);
              // Format ADC values
              sprintf(txBuffer, "%0.4f", adcSamplingFreq);

              if (HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), 10) != HAL_OK)
                log("ERROR: UART Can't transmit\r\n");

              txBuffer[0] = '\n';
              if (HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, sizeof(char), 10) != HAL_OK)
                log("ERROR: UART Can't transmite\r\n");
            break;

            case TERM_CMD_MEASUREMENT:
              log("UART INPUT: GET VOLTAGE\r\n");
              if (adc_read() != HAL_OK) break;
              
              for (size_t i = 0; i < ADC_BUFFER_SIZE; i++)
              {
                float voltage;
                float current;
                uint8_t separator = ',';
                if(i % 2 == 0)
                {
                  // voltage values on first ADC channel
                  voltage = probe_voltage(&probes[0], adc_buffer[i]);
                  // Format ADC values
                  sprintf(txBuffer, "%f", voltage);
                }
                else
                {
                  // current values on second ADC channel
                  // Format ADC values
                  current = (float)((3.3f * (adc_buffer[i] / 4096.0f) - 1.65f));
                  sprintf(txBuffer, "%f", current);
                }
                // Transmit formatted data
                if (HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), 10) != HAL_OK)
                  log("ERROR: UART Can't transmit adc value\r\n");
                // Transmit separator
                if (i != ADC_BUFFER_SIZE - 1)
                {
                  if (HAL_UART_Transmit(&huart2, &separator, sizeof(uint8_t), 10) != HAL_OK)
                    log("ERROR: UART Can't transmit adc value\r\n");
                }
              }

              txBuffer[0] = '\n';
              if (HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, sizeof(char), 10) != HAL_OK)
                log("ERROR: UART Can't transmit adc value\r\n");

            break;

            default:
              log("UART ERROR: Unknown Command\r\n");
            break;
          }
          break;

        default:
          log("Unknown Command");
          break;
      }

      // Restart the reception for the next command
      term_handler_flag.action = TERM_CMD_NONE;
      HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)uart_buffer, UART_RX_BUFFER_SIZE);
      __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }

/*
    HAL_Delay(100);

    for (uint8_t i = 0; i < 4; i++)
    {
      // turn on mounted leds one by one
      probe_mode(&probes[0], i);
      HAL_Delay(200);
    }

    uint32_t delay = 1000;
    uint32_t list_errors = 0;
    probe_mode(&probes[0], PROBE_MODE_MEASUREMENT);
    probe_error_print(&probes[0]);

    log("\n");
    log("INFO: swithing all ranges of probe:%d\n", probes[0].number);
    HAL_Delay(delay);

    // PROBE_RANGE_13V,
    // PROBE_RANGE_15V,
    // PROBE_RANGE_17V,
    // PROBE_RANGE_20V,
    // PROBE_RANGE_25V,
    // PROBE_RANGE_33V,
    // PROBE_RANGE_48V,
    for (size_t i = 8; i > 0; i--)
    {
      probe_gain_specific(&probes[0], PROBE_GAIN_1, i);
      log("   INFO: probe:%d range = %d\n", probes[0].number, probes[0].range);
      if(list_errors) probe_error_print(&probes[0]);
      HAL_Delay(delay);
    }

    // PROBE_GAIN_0125 = 0,
    // PROBE_GAIN_025,
    // PROBE_GAIN_05,
    // PROBE_GAIN_1,
    // PROBE_GAIN_2,
    // PROBE_GAIN_4,
    // PROBE_GAIN_8,
    // PROBE_GAIN_16,
    // PROBE_GAIN_32,
    // PROBE_GAIN_64,
    // PROBE_GAIN_128,
    log("INFO: swithing all gains of probe:%d\n", probes[0].number);
    for (size_t i = 0; i < 11; i++)
    {
      probe_gain_specific(&probes[0], i, PROBE_RANGE_13V);
      log("   INFO: probe:%d gain = %d\n", probes[0].number, probes[0].gain);
      if(list_errors) probe_error_print(&probes[0]);
      HAL_Delay(delay);
    }
*/
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // stop DMA channel when ADC conversion completed
  HAL_ADC_Stop_DMA(hadc);
  // timer 6 stop for ADC triggering
  HAL_TIM_Base_Stop(&htim6);
  // update flag, conversion complete indication
  adc_complete_flag = 1;
}

HAL_StatusTypeDef adc_read(void)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  // start ADC conversion utilizing DMA
  adc_complete_flag = 0;
  // timer 6 start for ADC triggering
  hal_status = HAL_TIM_Base_Start(&htim6);
  hal_status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);
  if (hal_status != HAL_OK)
  {
    log("ERROR: HAL ERROR while staring ADC + DMA conversion\r\n");
    return hal_status;
  }

  // wait until the ADC buffer is filled or timeout occurs, timeout is in ms
  size_t timeout = 100 * (htim6.Instance->PSC + 1);
  int i;
  for (i = 0; i < timeout; i++)
  {
    if (adc_complete_flag == 1)
      break;
    HAL_Delay(1);
  }
  if (i == timeout)
  {
    log("ERROR: ADC buffer not filled, timeout occurs\r\n");
    return HAL_ERROR;
  }

  return hal_status;
}

TermCommand_Type term_get_command_enum(const char* command)
{
  if (strcmp(command, "SET") == 0) return TERM_CMD_SET;
  if (strcmp(command, "GET") == 0) return TERM_CMD_GET;

  if (strcmp(command, "FSAMPLING") == 0) return TERM_CMD_FSAMPLING;

  if (strcmp(command, "MEASUREMENT") == 0) return TERM_CMD_MEASUREMENT;
  if (strcmp(command, "NEGATIVE") == 0) return TERM_CMD_NEGATIVE;
  if (strcmp(command, "POSITIVE") == 0) return TERM_CMD_POSITIVE;
  if (strcmp(command, "REFERENCE") == 0) return TERM_CMD_REFERENCE;

  if (strcmp(command, "GAIN") == 0) return TERM_CMD_GAIN;
  if (strcmp(command, "INCREASE") == 0) return TERM_CMD_INCREASE;
  if (strcmp(command, "DECREASE") == 0) return TERM_CMD_DECREASE;

  if (strcmp(command, "CALIBRATION") == 0) return TERM_CMD_CALIBRATION;
  return TERM_CMD_UNKNOWN;
}

TermHandlerFlag_Type parse_input_command(const char *input)
{
  TermHandlerFlag_Type command = {TERM_CMD_UNKNOWN, TERM_CMD_UNKNOWN, 0};

  // copy inpout content to 
  char buffer[UART_RX_BUFFER_SIZE];
  strncpy(buffer, input, sizeof(buffer));
  // ensure null-termination
  buffer[sizeof(buffer) - 1] = '\0';

  // tokenize the input string to separate the command parts
  // first token should be SET or GET, or it could be a special command like HELP
  char *token = strtok(buffer, " \r\n");
  if (token == NULL) return command;
  command.action = term_get_command_enum(token);
  if (command.action == TERM_CMD_UNKNOWN) return command;

  // get the next part of the command
  token = strtok(NULL, " \r\n");
  if (token == NULL) return command;
  command.subject = term_get_command_enum(token);
  if (command.subject == TERM_CMD_UNKNOWN) return command;

  // If the next token is a number
  token = strtok(NULL, " \r\n");
  if (token == NULL) return command;
  command.arg1 = (uint32_t)atoi(token);

  // If the next token is a number
  token = strtok(NULL, " \r\n");
  if (token == NULL) return command;
  command.arg2 = (float)atof(token);

  return command;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) 
{
  if (huart->Instance == USART2)
  {
    // Parse the buffer
    term_handler_flag = parse_input_command((char *)uart_buffer);
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
  log("Wrong parameters value: file %s on line %ld\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
