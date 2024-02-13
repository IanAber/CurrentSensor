/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "InterruptHandlers.h"
#include "semphr.h"
#include "MCP3462.h"
#include "can.h"
#include  "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern IWDG_HandleTypeDef hiwdg;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint16_t CANBaseAddress = 0x50;		// Base address for the CAN bus is offset by the links set on the board
int32_t lastCurrent = 0;
int32_t lastVoltage = 0;
bool newVoltage = false;
bool newCurrent = false;

/* USER CODE END Variables */
/* Definitions for ADCTask */
osThreadId_t ADCTaskHandle;
const osThreadAttr_t ADCTask_attributes = {
  .name = "ADCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GreenLedOffTimer */
osTimerId_t GreenLedOffTimerHandle;
const osTimerAttr_t GreenLedOffTimer_attributes = {
  .name = "GreenLedOffTimer"
};
/* Definitions for RedLedOffTimer */
osTimerId_t RedLedOffTimerHandle;
const osTimerAttr_t RedLedOffTimer_attributes = {
  .name = "RedLedOffTimer"
};
/* Definitions for sendCANDataTimer */
osTimerId_t sendCANDataTimerHandle;
const osTimerAttr_t sendCANDataTimer_attributes = {
  .name = "sendCANDataTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartADCTask(void *argument);
void TurnOffGreenLED(void *argument);
void TurnOffRedLED(void *argument);
void sendCANData(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	ModbusH.uModbusType = MB_SLAVE;
	ModbusH.port =  &huart1;

	// Links 0..2 set the low byte of the Modbus address so it ranges from 0x10 to 0x17
	int ModbusAddressOffset = 0;
	if (HAL_GPIO_ReadPin(ADDR0_GPIO_Port, ADDR0_Pin)) {
		ModbusAddressOffset |= 1;
		CANBaseAddress |= 0x08;
	}
	if (HAL_GPIO_ReadPin(ADDR1_GPIO_Port, ADDR1_Pin)) {
		ModbusAddressOffset |= 2;
		CANBaseAddress |= 0x10;
	}
	if (HAL_GPIO_ReadPin(ADDR2_GPIO_Port, ADDR2_Pin)) {
		ModbusAddressOffset |= 4;
		CANBaseAddress |= 0x20;
	}
	ModbusH.u8id = ModbusSlaveID + ModbusAddressOffset;
	ModbusH.u16timeOut = ModbusTimeoutMs;			// Modbus timeout in Milliseconds

	//  ModbusH.EN_Port = NULL;
	// Port for the TXEN line to enable the RS485 driver
	ModbusH.EN_Port = RS485_TXEN_GPIO_Port;
	ModbusH.EN_Pin = RS485_TXEN_Pin;

	ModbusH.u16regs = ModbusDATA;
	ModbusH.u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
	ModbusH.xTypeHW = USART_HW;
	//Initialize Modbus library
	ModbusInit(&ModbusH);

	//Start capturing traffic on serial Port
	ModbusStart(&ModbusH);

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of GreenLedOffTimer */
  GreenLedOffTimerHandle = osTimerNew(TurnOffGreenLED, osTimerOnce, NULL, &GreenLedOffTimer_attributes);

  /* creation of RedLedOffTimer */
  RedLedOffTimerHandle = osTimerNew(TurnOffRedLED, osTimerOnce, NULL, &RedLedOffTimer_attributes);

  /* creation of sendCANDataTimer */
  sendCANDataTimerHandle = osTimerNew(sendCANData, osTimerPeriodic, NULL, &sendCANDataTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */

	osTimerStart(sendCANDataTimerHandle, 5000);	// Send the calibration data every 5 seconds

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ADCTask */
  ADCTaskHandle = osThreadNew(StartADCTask, NULL, &ADCTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	StartCANReception(hcan);

  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartADCTask */
#define BufferSize 100

typedef struct {
	int32_t data[BufferSize];
	int dataPtr;
} ringBuffer;

struct {
	ringBuffer voltage;
	ringBuffer current;
} adcBuffer;

/**
 * Clears the ADC buffer
 */
void ResetBuffer() {
	for (int i = 0; i < BufferSize; i++) {
		adcBuffer.voltage.data[i] = 0;
		adcBuffer.voltage.dataPtr = 0;
		adcBuffer.current.data[i] = 0;
		adcBuffer.current.dataPtr = 0;
	}
}

/**
 * Calculate average voltage
 */
int32_t CalculateAverageVoltage() {
	int32_t voltage = 0;
	for (int i = 0; i < BufferSize; i++) {
		voltage += adcBuffer.voltage.data[i];
	}
	return voltage / BufferSize;
}

/**
 * Applies the calibration values to calculate the actual voltage
 */
uint16_t  CalculateActualVoltage(double rawVoltage) {
	double m;
	double c;
	double v;
	GetVoltageCalibration(&m, &c);
	v = (m * rawVoltage) + c;
	return (uint16_t)v;
}

/**
 * Calculate average current
 */
int32_t CalculateAverageCurrent() {
	int32_t current = 0;
	for (int i = 0; i < BufferSize; i++) {
		current += adcBuffer.current.data[i] * 100;
	}
	return current  / BufferSize;
}

/**
 * Applies the calibration values to calculate the actual current
 */
int32_t CalculateActualCurrent(double rawCurrent) {
	double m;
	double c;
	double i;
	GetCurrentCalibration(&m, &c);
	i = (m * rawCurrent) + c;
	return (int32_t)i;
}

uint8_t CANBuffer[8];	// Define an 8 byte buffer for information sent on the CAN bus
CAN_TxHeaderTypeDef msg;

void SendBufferOverCAN(int16_t messageID, const uint8_t *buffer) {
	uint32_t mb;	// TxMailbox returned by the HAL driver

	msg.DLC = 8;	// 8 bytes of data
	msg.ExtId = 0;
	msg.StdId = CANBaseAddress + messageID;
	msg.IDE = CAN_ID_STD;
	msg.RTR = CAN_RTR_DATA;
	msg.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {

		HAL_StatusTypeDef st = HAL_CAN_AddTxMessage(&hcan, &msg, buffer, &mb);
		if (st != HAL_OK) {
			// Turn on Red LED to indicate there is a problem
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
			xTimerStart(RedLedOffTimerHandle, 75);

//			char* err;
//			switch(hcan.ErrorCode) {
//			case HAL_CAN_ERROR_NONE : err = NULL;
//			break;
//			case HAL_CAN_ERROR_EWG : err =   "Protocol Error Warning";
//			break;
//			case HAL_CAN_ERROR_EPV : err =   "Error Passive";
//			break;
//			case HAL_CAN_ERROR_BOF : err =   "Bus-off error";
//			break;
//			case HAL_CAN_ERROR_STF : err =   "Stuff error";
//			break;
//			case HAL_CAN_ERROR_FOR : err =   "Form error";
//			break;
//			case HAL_CAN_ERROR_ACK : err =   "Acknowledgment error";
//			break;
//			case HAL_CAN_ERROR_BR : err =   "Bit recessive error";
//			break;
//			case HAL_CAN_ERROR_BD : err =   "Bit dominant error";
//			break;
//			case HAL_CAN_ERROR_CRC : err =   "CRC error";
//			break;
//			case HAL_CAN_ERROR_RX_FOV0 : err =   "Rx FIFO0 overrun error";
//			break;
//			case HAL_CAN_ERROR_RX_FOV1  : err =   "Rx FIFO1 overrun error";
//			break;
//			case HAL_CAN_ERROR_TX_ALST0 : err =   "TxMailbox 0 transmit failure due to arbitration lost";
//			break;
//			case HAL_CAN_ERROR_TX_TERR0 : err =   "TxMailbox 0 transmit failure due to transmit error";
//			break;
//			case HAL_CAN_ERROR_TX_ALST1 : err =   "TxMailbox 1 transmit failure due to arbitration lost";
//			break;
//			case HAL_CAN_ERROR_TX_TERR1 : err =   "TxMailbox 1 transmit failure due to transmit error";
//			break;
//			case HAL_CAN_ERROR_TX_ALST2 : err =   "TxMailbox 2 transmit failure due to arbitration lost";
//			break;
//			case HAL_CAN_ERROR_TX_TERR2 : err =   "TxMailbox 2 transmit failure due to transmit error ";
//			break;
//			case HAL_CAN_ERROR_TIMEOUT : err =   "Timeout error";
//			break;
//			case HAL_CAN_ERROR_NOT_INITIALIZED : err =   "Peripheral not initialized";
//			break;
//			case HAL_CAN_ERROR_NOT_READY : err =   "Peripheral not ready";
//			break;
//			case HAL_CAN_ERROR_NOT_STARTED : err = "Peripheral not started";
//			break;
//			case HAL_CAN_ERROR_PARAM : err =   "Parameter error ";
//			break;
//			}
		}
	} else {
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
		xTimerStart(RedLedOffTimerHandle, 75);
	}
}

void SendVoltageOverCAN(uint32_t raw, uint16_t calculated) {
	uint8_t Buffer[8];
	memset(Buffer, 0, sizeof(Buffer));
	memcpy(Buffer, &calculated, 2);			// Calculated value goes in the first two bytes
	memcpy(Buffer + 4, &raw, 4);				// Raw value goes in byte 3 to 6

	SendBufferOverCAN(0, Buffer);
	if (HAL_GPIO_ReadPin(Debug_GPIO_Port, Debug_Pin)) {
		HAL_GPIO_WritePin(Debug_GPIO_Port, Debug_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(Debug_GPIO_Port, Debug_Pin, GPIO_PIN_SET);
	}
}

void SendCurrentOverCAN(uint32_t raw, int32_t calculated) {
	uint8_t Buffer[8];
	memset(Buffer, 0, sizeof(Buffer));
	memcpy(Buffer, &calculated, 4);			// Calculated value goes in the first four bytes
	memcpy(Buffer + 4, &raw, 4);				// Raw value goes in byte 3 to 6

	SendBufferOverCAN(1, Buffer);
}


/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADCTask */
void StartADCTask(void *argument)
{
  /* USER CODE BEGIN StartADCTask */
	uint8_t status;
	int16_t	value16;
	int32_t	value32;
	uint8_t channel;
	ResetBuffer();

  MCP3462_ADCInit();
 // StartCANReception(hcan);

  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake(0, osWaitForever);

	  HAL_IWDG_Refresh(&hiwdg);

	  // Read the A-D converter
	  status = MCP3462_ADCRead(&value32, &channel);
	  if (((status & 0x18) == 0) || ((status & 0x18) == 0x18)) {
		  // No device is answering us
		  status = 0b01000000;
	  }

	  // If we have new data
	  if ((status & 0b01000000) == 0) {
		  if (channel == 0) {
//			  lastVoltage = value32;

			  SendVoltageOverCAN(value32, CalculateActualVoltage((double)value32));

//			  newVoltage = true;
			  adcBuffer.voltage.data[adcBuffer.voltage.dataPtr++] = value32;
			  if (adcBuffer.voltage.dataPtr >= BufferSize) {
				  adcBuffer.voltage.dataPtr = 0;
			  }
			  value16 = CalculateActualVoltage((double)CalculateAverageVoltage());
			  // Put the average data in the modbus registers
			  xSemaphoreTake(ModbusH.ModBusSphrHandle , 100);
			  ModbusH.u16regs[InputRegistersStart + VoltageRegister] = (uint16_t)value16;
			  xSemaphoreGive(ModbusH.ModBusSphrHandle);
		  } else {
			  lastCurrent = value32;

			  SendCurrentOverCAN(value32, CalculateActualCurrent((double)value32));

			  newCurrent = true;
			  adcBuffer.current.data[adcBuffer.current.dataPtr++] = value32;
			  if (adcBuffer.current.dataPtr >= BufferSize) {
				  adcBuffer.current.dataPtr = 0;
			  }
			  value32 = CalculateActualCurrent((double)CalculateAverageCurrent());
			  // Put the average data in the modbus registers
			  xSemaphoreTake(ModbusH.ModBusSphrHandle , 100);
			  ModbusH.u16regs[InputRegistersStart + CurrentLow] = (uint16_t)(value32 & 0xffff);
			  ModbusH.u16regs[InputRegistersStart + CurrentHigh] = (uint16_t)((value32 >> 16) & 0xffff);
			  xSemaphoreGive(ModbusH.ModBusSphrHandle);
		  }
	  }
  }
  /* USER CODE END StartADCTask */
}

/* TurnOffGreenLED function */
void TurnOffGreenLED(void *argument)
{
  /* USER CODE BEGIN TurnOffGreenLED */
	  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /* USER CODE END TurnOffGreenLED */
}

/* TurnOffRedLED function */
void TurnOffRedLED(void *argument)
{
  /* USER CODE BEGIN TurnOffRedLED */
	  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /* USER CODE END TurnOffRedLED */
}

/* sendCANData function */
void sendCANData(void *argument)
{
  /* USER CODE BEGIN sendCANData */
	// Send the voltage calibration as four 16 bit numbers
	SendBufferOverCAN(2, GetCalibrationDataPtr());

	vTaskDelay( 100 );

	// Send the low current calibration points as two 32 bit numbers
	SendBufferOverCAN(3, GetCalibrationDataPtr() + 8);

	vTaskDelay( 100 );

	// Send the high current calibration points as two 32 bit numbers
	SendBufferOverCAN(4, GetCalibrationDataPtr() + 16);
  /* USER CODE END sendCANData */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

