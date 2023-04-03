#include "main.h"
#include "cmsis_os.h"

I2C_HandleTypeDef hi2c1;
osThreadId defaultTaskHandle;
osThreadId counterTaskHandle;

SemaphoreHandle_t xSemaphore = NULL;
volatile int counter = 0;

static void main_rtos_init(void),
			MX_GPIO_Init(void),
			MX_I2C1_Init(void),
			Error_Handler(void),
			SystemClock_Config(void),
			wait_for_sem(void),
			blink_led(uint8_t, uint16_t);

int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();

  main_rtos_init();
  osKernelStart();

  // We should never get here as control is taken by the scheduler
  while (1);
}

static void main_rtos_init(void) {
  if ((xSemaphore = xSemaphoreCreateMutex()) == NULL) {
	  counter = -1;
	  while (1); //not enough space, NOP forever loop
  }

  // Be careful with stack size
  // Too large of size will cause thread to fail
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 1, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(counterTask, StartCounterTask, osPriorityNormal, 1, 256);
  counterTaskHandle = osThreadCreate(osThread(counterTask), NULL);
}

void StartDefaultTask(void const * argument) {
  while (1)  {
	  if(xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
		  blink_led(1, 500);
		  xSemaphoreGive(xSemaphore);
		  vTaskDelay(100);
	  }
	  // Give up some cycles, coutner task has semaphore
	  else vTaskDelay(500);
  }
}

void StartCounterTask(void const * argument) {
  while (1)  {
	  vTaskDelay(500);
	  if (counter >= 0xFFFFFFFF) counter = 0;
	  counter++;
	  if (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET) {
		  counter += 100;
		  wait_for_sem();
		  blink_led(100, 65);
		  xSemaphoreGive(xSemaphore);
	  }
	  vTaskDelay(500);
  }
}

static void blink_led(uint8_t num_blinks, uint16_t delay) {
	  for (uint8_t i = 0; i < num_blinks; i++) {
		  HAL_GPIO_WritePin(GPIOC, LED_Pin, GPIO_PIN_RESET);
		  vTaskDelay(delay);
		  HAL_GPIO_WritePin(GPIOC, LED_Pin, GPIO_PIN_SET);
		  vTaskDelay(delay);
	  }
}

static void wait_for_sem(void) {
  while (1) {
	  if(xSemaphoreTake(xSemaphore, (TickType_t)0) == pdTRUE)
		  return;
  }
}

static void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : LED_Pin RESET_Pin */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|RESET_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED_Pin|RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

}

static void Error_Handler(void) {
  __disable_irq();
  while (1);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
	while (1);
}
#endif /* USE_FULL_ASSERT */
