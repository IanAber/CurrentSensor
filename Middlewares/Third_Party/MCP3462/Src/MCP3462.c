#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "spi.h"
#include "MCP3462.h"

void MCP3462_ADCInit() {

	uint8_t TX_Data[19];

	// Reset the MCP3462
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
//	osDelay(50);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);
//	osDelay(50);

	TX_Data[0] = 0b01000110;	// Device = 1 - write starting at register 1
	TX_Data[1] = MCP3462_CLK_INT_AMCLK | MCP3462_CurrentNone | MCP3462_CONVERSION;
	TX_Data[2] = MCP3462_AMCLK_PRESCALE2 | MCP3462_OSR_512;
//	TX_Data[3] = MCP3462_BIAS_1 | MCP3462_GAIN_1_3 | MCP3462_AZ_MUX_DISABLED;
	TX_Data[3] = MCP3462_BIAS_1 | MCP3462_GAIN_1 | MCP3462_AZ_MUX_DISABLED;
	TX_Data[4] = MCP3462_MODE_CONTINUOUS | MCP3462_FORMAT_32_17_D | MCP3462_CRC16_16 | MCP3462_CRC_DISABLE | MCP3462_OFFCAL_DISABLE | MCP3462_GAINCAL_DISABLE;
	TX_Data[5] = MCP3462_IRQ_STATE | MCP3462_FAST_COMMAND;
	TX_Data[6] = MCP3462_VINPOS_CH0 | MCP3462_VINNEG_AGND;
//	TX_Data[7] = 0b00000000; 	// Scan 0		|
	TX_Data[7] = MCP3462_DMCLK64;
	TX_Data[8] = 0b00000000; 	// Scan 1		|
	TX_Data[9] = 0b00000011; 	// Scan 2		| Scanning channels 0 and 1
	TX_Data[10] = 0x00;
	TX_Data[11] = 0x00;
	TX_Data[12] = 0x01;			// No delay
	TX_Data[13] = 0x00;
	TX_Data[14] = 0x00;
	TX_Data[15] = 0x00;			// Offset Calibration
	TX_Data[16] = 0x00;
	TX_Data[17] = 0x00;
	TX_Data[18] = 0x00;			// Gain Calibration

	HAL_SPI_Transmit(&hspi1, TX_Data, sizeof(TX_Data), 5000);

	// Deselect the MCP3462 to end the command
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

}

uint8_t MCP3462_ADCRead(int32_t *value, uint8_t *channel) {
	uint32_t val = 0;
	uint8_t TX_Data[5];
	uint8_t RX_Data[5];

	TX_Data[0] = 0b01000011;		// Read register 0

	RX_Data[0] = 0xa0;	//Status
	RX_Data[1] = 0x0a;	//Data 24:31
	RX_Data[2] = 0xa0;	//Data 16:23
	RX_Data[3] = 0x0a;	//Data 08:15
	RX_Data[4] = 0xa0;	//Data 00:07

	// Select the MCP3462
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(&hspi1, TX_Data, RX_Data, sizeof(TX_Data), 5000);

	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);


	if (RX_Data[2] & 1) {
		val = 0xFFFF0000;
	}
	val |=  (RX_Data[3] << 8);
	val |= RX_Data[4];
	*value = val;
	*channel = RX_Data[1] >> 4;

	return RX_Data[0];
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	uint16_t myPin;
//
//	myPin = GPIO_Pin;
//}
