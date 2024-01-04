/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include  "usart.h"
#include "Modbus.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct host_frame {
	uint32_t echo_id;
	uint32_t can_id;
	uint8_t can_dlc;
	uint8_t channel;
	uint8_t flags;
	uint8_t reserved;
	uint8_t data[8];
	uint32_t timestamp_us;
} hostFrame_t __attribute__ ((aligned (4)));

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_INT_Pin GPIO_PIN_3
#define ADC_INT_GPIO_Port GPIOA
#define ADC_INT_EXTI_IRQn EXTI2_3_IRQn
#define ADC_CS_Pin GPIO_PIN_4
#define ADC_CS_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_0
#define RED_LED_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_1
#define GREEN_LED_GPIO_Port GPIOB
#define ADDR0_Pin GPIO_PIN_2
#define ADDR0_GPIO_Port GPIOB
#define RS485_TXEN_Pin GPIO_PIN_8
#define RS485_TXEN_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_9
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_10
#define RS485_RX_GPIO_Port GPIOA
#define ADDR1_Pin GPIO_PIN_3
#define ADDR1_GPIO_Port GPIOB
#define ADDR2_Pin GPIO_PIN_4
#define ADDR2_GPIO_Port GPIOB
#define LK3_Pin GPIO_PIN_5
#define LK3_GPIO_Port GPIOB
#define CAN250_Pin GPIO_PIN_6
#define CAN250_GPIO_Port GPIOB
#define MODBUS9600_Pin GPIO_PIN_7
#define MODBUS9600_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern modbusHandler_t ModbusH;
#define ModbusTimeoutMs	100
#define ModbusSlaveID	10
#define NumCoils		8
#define NumDiscreteInputs 4
#define NumInputs 10
#define NumHoldingRegisters 8
#define RegisterBytes ((NumCoils + 7) / 8) + ((NumDiscreteInputs + 7) / 8) + (NumInputs * 2) + (NumHoldingRegisters * 2)
#define DiscreteStart (NumCoils + 7) / 8
#define InputRegistersStart DiscreteStart + ((NumDiscreteInputs + 7) / 8)
#define HoldingRegistersStart InputRegistersStart + (NumInputs * 2)
extern uint16_t ModbusDATA[RegisterBytes];
typedef uint16_t ModbusDATA_t[RegisterBytes];
extern ModbusDATA_t ModbusDATA;

// Register layout
#define VoltageRegister	0
#define CurrentLow			1
#define CurrentHigh		2
#define PowerLow			3
#define PowerHigh			4
#define EnergyLow			5
#define EnergyHigh			6
#define Frequency			7
#define PowerFactor		8
#define Alarm					9

#define AlarmThreshold	1
#define SlaveAddress		1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
