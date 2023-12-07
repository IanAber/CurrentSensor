/*
 * InterruptHandlers.c
 *
 *  Created on: Oct 12, 2022
 *      Author: ianaber
 */

//#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "main.h"
#include "InterruptHandlers.h"

extern osThreadId_t ADCTaskHandle;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ADC_INT_Pin) {
		if (ADCTaskHandle != 0) {
			vTaskNotifyGiveFromISR(ADCTaskHandle, NULL);
		}
	}
}



