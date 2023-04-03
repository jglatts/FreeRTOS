#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

// FreeRTOS task functions
void StartDefaultTask(void const* argument);
void StartCounterTask(void const* argument);

#define LED_Pin 		GPIO_PIN_13
#define LED_GPIO_Port 	GPIOC
#define BTN_Pin 		GPIO_PIN_5
#define BTN_GPIO_Port	GPIOB
#define RESET_Pin 	GPIO_PIN_14
#define RESET_GPIO_Port GPIOC

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
