/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "freeRTOS.h"
#include "string.h"
#include "stdio.h"
#include "task.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* voltage threshold below which low power mode is turned ON */
#define VOLTAGE_THRESHOLD_LOW 0.50f

/* External interrupt line number for the button */
#define BUTTON_EXTI_IRQn   EXTI4_15_IRQn
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* ADC handle for voltage reading */
ADC_HandleTypeDef hadc;
/* UART handle for printing logs to serial monitor */
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
/* queue to pass input for logging task */
QueueHandle_t q_print;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* FreeRTOS task handlers */
static void voltagetask_handler(void* parameters);
static void logtask_handler(void* parameters);
static void lowpowertask_handler(void* parameters);
/* Utility: Convert integer to string with fixed number of digits */
static void intToStr(int val, char *buf , int digits);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BaseType_t status; // Stores return values of task creations
TaskHandle_t voltagetask_handle;
TaskHandle_t logtask_handle;
TaskHandle_t lowpowertask_handle;
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

  /* Configure system clock (HSE, PLL settings handled inside) */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* ---------------- FreeRTOS Initialization ---------------- */
  /* Create a queue to hold voltage strings (20 bytes each), max 10 items */
 q_print = xQueueCreate(10,sizeof(char[20]));
 configASSERT(q_print != NULL);

 status = xTaskCreate(voltagetask_handler, "voltage task", 128, NULL, 2, &voltagetask_handle);
 configASSERT(status == pdPASS);
 status = xTaskCreate(logtask_handler, "log task", 128, NULL, 2, &logtask_handle);
 configASSERT(status == pdPASS);
 status = xTaskCreate(lowpowertask_handler, "low power task", 128, NULL, 3, &lowpowertask_handle);
 configASSERT(status == pdPASS);
 /* Start the FreeRTOS kernel (will not return unless error occurs) */
 vTaskStartScheduler();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(BUTTON_EXTI_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(BUTTON_EXTI_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void lowpowertask_handler(void* parameters){
	char msg[20];
	while(1){
		/* Block this task until notified  */
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		/* Log entry into sleep mode */
		strncpy(msg, "Entering sleep mode\r\n", sizeof(msg));
		msg[sizeof(msg) - 1] = '\0';// Ensure null-termination
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		/* Prepare for external wakeup via button EXTI */
		__HAL_GPIO_EXTI_CLEAR_IT(button_Pin);
		HAL_NVIC_EnableIRQ(BUTTON_EXTI_IRQn);// Re-enable button interrupt
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		/* Re-initialize HAL and system clock after waking up */
		HAL_ResumeTick();
		HAL_NVIC_DisableIRQ(BUTTON_EXTI_IRQn);
		/* Resume the voltage monitoring task */
		vTaskResume(voltagetask_handle);
	}
}


static void voltagetask_handler(void* parameters){
	float voltage = 0;
	char msg[20];
	uint16_t adcVal;
		while (1) {
			 /* Start ADC conversion and wait for result */
		    HAL_ADC_Start(&hadc);
		    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		    adcVal = HAL_ADC_GetValue(&hadc);
		    /* Convert ADC value to voltage (Assuming 3.3V ref and 12-bit ADC) */
		    voltage = (adcVal * 3.3f) / 4095.0f;
		    vTaskDelay(pdMS_TO_TICKS(1000));
		    /* Check if voltage falls below critical threshold */
		    if(voltage < VOLTAGE_THRESHOLD_LOW){
		    	xTaskNotifyGive(lowpowertask_handle);
		    	vTaskSuspend(NULL);
		    }
		    		int intPart = (int)voltage;
		    		int decPart = (int)((voltage - intPart) * 1000);
		    		// Convert integer part to string
		    		 int idx = 0;
		    		        if (intPart >= 100)
		    		        {
		    		            msg[idx++] = '0' + (intPart / 100);
		    		            intPart %= 100;
		    		            msg[idx++] = '0' + (intPart / 10);
		    		            intPart %= 10;
		    		            msg[idx++] = '0' + intPart;
		    		        }
		    		        else if (intPart >= 10)
		    		        {
		    		            msg[idx++] = '0' + (intPart / 10);
		    		            intPart %= 10;
		    		            msg[idx++] = '0' + intPart;
		    		        }
		    		        else
		    		        {
		    		            msg[idx++] = '0' + intPart;
		    		        }
		    		        msg[idx++] = '.';
		    		        intToStr(decPart,&msg[idx],3);
		    		        idx+=3;
		    		        msg[idx++] = ' ';
		    		        msg[idx++] = 'V';
		    		        msg[idx++] = '\r';
		    		        msg[idx++] = '\n';
		    		        msg[idx] = '\0';
		    /* Send voltage message to queue for UART transmission */
		    if(xQueueSend(q_print,msg,pdMS_TO_TICKS(100)) != pdPASS){
		    	HAL_UART_Transmit(&huart2, (uint8_t*)"send failed\r\n", strlen("send failed\r\n"), HAL_MAX_DELAY);
		    }
		    /* Notify the log task to handle UART transmission */
		    xTaskNotifyGive(logtask_handle);
		}
}


static void logtask_handler(void* parameters){

	char msg[20];
	while(1){
		/* Block until notified */
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	/* Retrieve voltage string from queue and transmit via UART */
	if( xQueueReceive(q_print, msg, 100) == pdPASS){
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
	}

}

static void intToStr(int val, char *buf , int digits){
	for(int i = digits-1; i>=0; i--){
		buf[i]='0'+(val%10);// Extract last digit
		val/=10;// Shift right by one digit
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == button_Pin) {

    }
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
