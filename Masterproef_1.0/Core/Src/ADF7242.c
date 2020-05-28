/*
 * ADF7242.c
 *
 *  Created on: Feb 28, 2020
 *      Author: jelle
 */
#include "ADF7242.h"

/* Variables -------------------------------------------------------------------*/

/* Functions -------------------------------------------------------------------*/
void ADF_Init(uint32_t frequency)
{
	ADF_reset();								// RESET
	HAL_Delay(10);

	ADF_SPI_MEM_WR(0x13e,0x00); 				// rc_mode = IEEE802.15.4 packet

	ADF_SPI_MEM_WR(0x3c7,0x00); 				// other interrupt 1 sources off
	ADF_SPI_MEM_WR(0x3c8,0x10); 				// generate interrupt 1: Packet transmission complete
	ADF_SPI_MEM_WR(0x3c9,0x00); 				// other interrupt 2 sources off
	ADF_SPI_MEM_WR(0x3ca,0x08); 				// generate interrupt 2: Packet received in Rx Buffer

	ADF_SPI_MEM_WR(0x3cb,0xff); 				// clear all interrupt flags
	ADF_SPI_MEM_WR(0x3cc,0xff); 				// clear all interrupt flags

	ADF_set_IDLE_mode();							// Idle mode
	HAL_Delay(10);

	ADF_SET_FREQ_kHz(frequency);				// Set frequency

	TX_BUFFER_BASE = ADF_SPI_MEM_RD(0x314);		// Read register txpb
	RX_BUFFER_BASE = ADF_SPI_MEM_RD(0x315);		// Read register rxpb

	ADF_set_IDLE_mode();			// Idle mode
	HAL_Delay(10);

	ADF_SPI_MEM_WR(0x3aa, 0xf1);				// PA pwr[7:4] min=3, max=15 & [3]=0 & [2:0]=1

	ADF_set_PHY_RDY_mode();
}

void ADF_reset(void)
{
	uint8_t byte[] = {0xc8};

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi2, byte, 1);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
}

void ADF_sleep(void)
{
	ADF_SPI_MEM_WR(0x317,0x08);					// Sleep BBRAM

	uint8_t bytes[] = {0xb1};

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi2, bytes, 1);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
}

void ADF_SET_FREQ_kHz(uint32_t frequency)
{
	uint8_t LSB = frequency&0xff;
	uint8_t MSB = (frequency>>8)&0xff;
	uint8_t HSB = (frequency>>16)&0xff;
	ADF_SPI_MEM_WR(0x302,HSB);
	ADF_SPI_MEM_WR(0x301,MSB);
	ADF_SPI_MEM_WR(0x300,LSB);
}

uint32_t ADF_RD_Frequency_MHz(void)
{
	uint32_t frequency = ADF_SPI_MEM_RD(0x302);
	frequency = (frequency<<8) | ADF_SPI_MEM_RD(0x301);
	frequency = (frequency<<8) | ADF_SPI_MEM_RD(0x300);

	frequency /= 100;

	return frequency;
}

void ADF_SPI_MEM_WR(uint16_t reg, uint8_t data)
{
	uint8_t SPI_MEM_WR_MODES[3] = {0x08, 0x09, 0x0b};
	uint8_t mode;

	if (reg < 0x100)
		mode = 0;
	else if (reg > 0x13f)
		mode = 2;
	else
		mode = 1;

	uint8_t bytes[3];
	bytes[0] = SPI_MEM_WR_MODES[mode] | ((reg>>8)&0x07);
	bytes[1] = reg&0xff;
	bytes[2] = data;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, bytes, 3, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
}

uint8_t ADF_SPI_MEM_RD(uint16_t reg)
{
	uint8_t SPI_MEM_RD_MODES[3] = {0x28, 0x29, 0x2b};
	uint8_t mode;
	uint8_t value;

	if (reg < 0x100)
		mode = 0;
	else if (reg > 0x13f)
		mode = 2;
	else
		mode = 1;

	uint8_t bytes[3];
	bytes[0] = SPI_MEM_RD_MODES[mode] | ((reg>>8)&0x07);
	bytes[1] = reg&0xff;
	bytes[2] = 0xff;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, bytes, 3, 50);
	HAL_SPI_Receive(&hspi2, &value, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	return value;
}

void ADF_set_turnaround_Tx_Rx(void)
{
	ADF_SPI_MEM_WR(0x107,0x08);						// Set auto turnaround tx-rx
}

void ADF_set_turnaround_Rx_Tx(void)
{
	ADF_SPI_MEM_WR(0x107,0x04);						// Set auto turnaround rx-tx
}

void ADF_set_IDLE_mode(void)
{
	uint8_t bytes[] = {0xb2};
	uint8_t status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi2, bytes, 1);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
}

void ADF_set_PHY_RDY_mode(void)
{
	uint8_t bytes[] = {0xb3};
	uint8_t status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi2, bytes, 1);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

}

void ADF_set_Tx_mode(void)
{
	uint8_t bytes[] = {0xb5};
	uint8_t status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi2, bytes, 1);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
}

void ADF_set_Rx_mode(void)
{
	uint8_t bytes[] = {0xb4};
	uint8_t status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi2, bytes, 1);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
}

uint8_t ADF_SPI_READY(void)
{
	uint8_t status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi2, &status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	if ((status&0x80) == 0x80)
		return 1;
	else
		return 0;
}

uint8_t ADF_Rx_READY(void)
{
	uint8_t status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi2, &status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	if ((status&0xA4) == 0xA4)
		return 1;
	else
		return 0;
}

uint8_t ADF_PHY_RDY_READY(void)
{
	uint8_t status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi2, &status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	if ((status&0xA3) == 0xA3)
		return 1;
	else
		return 0;
}

uint8_t ADF_IDLE_READY(void)
{
	uint8_t status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi2, &status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	if ((status&0xA1) == 0xA1)
		return 1;
	else
		return 0;
}

void ADF_clear_Rx_flag(void)
{
	ADF_SPI_MEM_WR(0x3cc,0x08);
}

void ADF_clear_Tx_flag(void)
{
	ADF_SPI_MEM_WR(0x3cc,0x10);
}
