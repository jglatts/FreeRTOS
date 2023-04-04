/* Main app includes */
#include "main.h"
#include "ST7528i.h"
#include "fonts.h"
#include "bit_maps.h"
#include "cmsis_os.h"

/* Global app variables */
I2C_HandleTypeDef hi2c1;
osThreadId defaultTaskHandle;
osThreadId counterTaskHandle;

/* FreeRTOS variables */
SemaphoreHandle_t xSemaphore = NULL;
volatile int counter = 0;

/* Private functions */
static void main_rtos_init(void),
			MX_GPIO_Init(void),
			MX_I2C1_Init(void),
			Error_Handler(void),
			SystemClock_Config(void),
			wait_for_sem(void),
			move(void),
			blink_led(uint8_t, uint16_t);

/*	App entry point */
int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();

  // Anything running from ST75 lib will hardfault
  // Forsure has something to do with RTOS task size
  /*
  ST7528i_InitGPIO(&hi2c1);
  ST7528i_Init();
  //move();
  */

  // Init RTOS tasks and vars
  // Start the scheduler
  main_rtos_init();
  osKernelStart();

  // We should never get here as control is now taken by the scheduler
  while (1);
  return 0;
}

/* Init FreeRTOS tasks and semaphore */
static void main_rtos_init(void) {
  if ((xSemaphore = xSemaphoreCreateMutex()) == NULL) {
	  counter = -1;
	  while (1); //not enough space, NOP forever loop
  }

  // Be careful with stack size
  // Too large of size will cause thread to fail
  const osThreadDef_t default_task =  { "defaultTask", StartDefaultTask, osPriorityNormal, 1, 256 };
  defaultTaskHandle = osThreadCreate(&default_task, NULL);

  const osThreadDef_t counter_task =  { "counterTask", StartCounterTask, osPriorityNormal, 1, 256 };
  defaultTaskHandle = osThreadCreate(&counter_task, NULL);
}

/*
 * Main task for the app
 * Blinks a shared resource, LED, in a forever loop
 */
void StartDefaultTask(void const* argument) {
  while (1)  {
	  if(xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
		  blink_led(1, 500);
		  xSemaphoreGive(xSemaphore);
		  vTaskDelay(100);
	  }
	  // Give up some cycles, counter task has semaphore
	  else vTaskDelay(500);
  }
}

/*
 * Second task for the app
 * Reads an input pin and checks for a state change
 * Will use the shared resource, LED, if a change is found
 */
void StartCounterTask(void const* argument) {
  while (1)  {
	  vTaskDelay(500);
	  if (counter >= 0xFFFFFFFF) counter = 0;
	  counter++;
	  if ((GPIOB->IDR & BTN_Pin) == GPIO_PIN_RESET) {
		  counter += 100;
		  wait_for_sem();
		  blink_led(100, 65);
		  xSemaphoreGive(xSemaphore);
	  }
	  vTaskDelay(500);
  }
}

/* Helper function to blink LED */
static void blink_led(uint8_t num_blinks, uint16_t delay) {
	  for (uint8_t i = 0; i < num_blinks; i++) {
		  // same as gpio_write(0)
		  GPIOC->BSRR = (uint32_t)LED_Pin << 16u; // setting hi bits
		  vTaskDelay(delay);
		  // same as gpio_write(1)
		  GPIOC->BSRR = (uint32_t)LED_Pin;	// set low bits
		  vTaskDelay(delay);
	  }
}

/* Get the semaphore */
static void wait_for_sem(void) {
  while (1) {
	  if(xSemaphoreTake(xSemaphore, 0) == pdTRUE)
		  return;
  }
}

/* LCD Screen function */
static void move(void) {
	while (1) {
		// go through all the rows(y)
		for (int i = 0, start_y = 1; i < 7; i++, start_y+=25) {
			// move the bikeman across the columns(x)
			for (int j = 0, start_x = 1; j < 12; ++j, start_x+=5) {
				LCD_DrawBitmap(start_x, start_y, 30, 25, bike_man);
				ST7528i_Flush();
				HAL_Delay(10);
				ST7528i_Clear();
			}
		}
		ST7528i_Clear();
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
