/*
 * ADF7242.c
 *
 *  Created on: Feb 28, 2020
 *      Author: jelle
 */
#include "ADF7242.h"

extern SPI_HandleTypeDef hspi2;
extern uint8_t TX_BUFFER_BASE;

// Rx variables
extern uint32_t Rx_teller;
extern cbuf_handle_t Rx_buffer_handle_t;
extern uint8_t RX_BUFFER_BASE;
extern uint8_t Rx_resolution;
extern uint32_t Rx_Pkt_counter;
extern uint8_t Rx_Pkt_length;
extern uint8_t Rx_byteCounter;
extern uint8_t Rx_byte1;
extern uint8_t Rx_byte2;
extern uint8_t Rx_byte3;
extern uint16_t Rx_sample1;
extern uint16_t Rx_sample2;
extern uint8_t Rx_RSSI;
extern uint8_t Rx_SQI;


/* Variables -------------------------------------------------------------------*/
uint8_t result;

/* Functions -------------------------------------------------------------------*/
void ADF_Init(void)
{
	result = ADF_SPI_SEND_BYTE(0xc8);	//RESET
	HAL_Delay(10);

	ADF_SPI_MEM_WR(0x13e,0x00); //rc_mode = IEEE802.15.4 packet

	ADF_SPI_MEM_WR(0x3c7,0x00); //other interrupt 1 sources off
	ADF_SPI_MEM_WR(0x3c8,0x10); //generate interrupt 1: Packet transmission complete
	ADF_SPI_MEM_WR(0x3c9,0x00); //other interrupt 2 sources off
	ADF_SPI_MEM_WR(0x3ca,0x08); //generate interrupt 2: Packet received in Rx Buffer

	ADF_SPI_MEM_WR(0x3cb,0xff); //clear interrupt flag on chip ready
	ADF_SPI_MEM_WR(0x3cc,0xff); //clear interrupt flag on rx_pkt_rcv

	result = ADF_SPI_SEND_BYTE(0xb2);	//Idle mode
	HAL_Delay(10);

	ADF_SET_FREQ_kHz();	//Set frequency to 2.45 GHz

	//Tx buffer base
	TX_BUFFER_BASE = ADF_SPI_MEM_RD(0x314);		// Read register txpb
	RX_BUFFER_BASE = ADF_SPI_MEM_RD(0x315);		// Read register rxpb

	result = ADF_SPI_SEND_BYTE(0xb2);	//Idle mode
	HAL_Delay(10);

	//POWER
//	ADF_SPI_MEM_WR(0x36e,0x7f);			//PA bias[6:1] default=55, max=63
//	ADF_SPI_MEM_WR(0x3a8,0x15);			//PA cfg[4:0] default=13, max=21
	ADF_SPI_MEM_WR(0x3aa, 0xf0);			//PA pwr[7:4] min=3, max=15 & [3]=0 & [2:0]=1

	result = ADF_SPI_SEND_BYTE(0xb3);	//PHY_RDY mode
	HAL_Delay(10);
}

uint8_t ADF_SPI_STATUS(void)
{
	uint8_t bytes[1];
	bytes[0] = 0xff;
	uint8_t status[1];

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, bytes, 1, 50);
	HAL_SPI_Receive(&hspi2, &status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	return status[0];
}

uint8_t ADF_SET_RC_STATE(uint8_t state)
{
	uint8_t status;

	if (state == 0xc8)
	{
		status = ADF_SPI_SEND_BYTE(state);
	}
	else
	{
		while (wait_RC_RDY() == 0);
		status = ADF_SPI_SEND_BYTE(state);
	}
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

	while (SPI_READY() == 0);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);

	uint8_t bytes[3];
	bytes[0] = SPI_MEM_WR_MODES[mode] | ((reg>>8)&0x07);
	bytes[1] = reg&0xff;
	bytes[2] = data;

	HAL_SPI_Transmit(&hspi2, bytes, 3, 10);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (SPI_READY() == 0);
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

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);

	uint8_t bytes[3];
	bytes[0] = SPI_MEM_RD_MODES[mode] | ((reg>>8)&0x07);
	bytes[1] = reg&0xff;
	bytes[2] = 0xff;

	HAL_SPI_Transmit(&hspi2, bytes, 3, 10);
	HAL_SPI_TransmitReceive(&hspi2, 0xff, &value, 2, 10);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (SPI_READY() == 0);

	return value;
}

void ADF_SPI_RD_Rx_Buffer(void)
{
	uint8_t data;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);

	uint8_t bytes[2];
	bytes[0] = 0x30;
	bytes[1] = 0xff;

	HAL_SPI_Transmit(&hspi2, bytes, 2, 50);

	HAL_SPI_Receive(&hspi2, &Rx_Pkt_length, 1, 50);
	HAL_SPI_Receive(&hspi2, &Rx_resolution, 1, 50);

	Rx_byteCounter = 0;

	for (int i=2; i < Rx_Pkt_length-2; i++)
	{
		if (Rx_resolution == 8)
		{
			HAL_SPI_Receive(&hspi2, &Rx_byte1, 1, 50);
			circular_buf_put_overwrite(Rx_buffer_handle_t, Rx_byte1);
			Rx_teller++;
		}
		else if (Rx_resolution == 12)
		{
			if (Rx_byteCounter == 0)
			{
				HAL_SPI_Receive(&hspi2, &Rx_byte1, 1, 50);
				Rx_sample1 = Rx_byte1&0x0ff;
			}
			else if (Rx_byteCounter == 1)
			{
				HAL_SPI_Receive(&hspi2, &Rx_byte2, 1, 50);
				Rx_sample2 = Rx_byte2&0x00f;
				Rx_sample1 = ((Rx_byte2<<4)&0xf00)|Rx_sample1;
				circular_buf_put_overwrite(Rx_buffer_handle_t, Rx_sample1);
				Rx_teller++;
			}
			else if (Rx_byteCounter == 2)
			{
				HAL_SPI_Receive(&hspi2, &Rx_byte3, 1, 50);
				Rx_sample2 = ((Rx_byte3<<4)&0xff0)|Rx_sample2;
				circular_buf_put_overwrite(Rx_buffer_handle_t, Rx_sample2);
				Rx_teller++;
				Rx_byteCounter = -1;
			}
			Rx_byteCounter++;
		}
	}

	HAL_SPI_Receive(&hspi2, &Rx_RSSI, 1, 50);
	HAL_SPI_Receive(&hspi2, &Rx_SQI, 1, 50);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (SPI_READY() == 0);
}

uint8_t ADF_SPI_SEND_BYTE(uint8_t byte)
{
	uint8_t bytes[1];
	bytes[0] = byte;
	uint8_t status;

	while (SPI_READY() == 0);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, bytes, 1, 50);
	HAL_SPI_Receive(&hspi2, &status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (SPI_READY() == 0);

	return status;
}

void ADF_SPI_SEND_BYTE_NO_RECEIVE(uint8_t byte)
{
	uint8_t bytes[1];
	bytes[0] = byte;
	uint8_t status;

	while (SPI_READY() == 0);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, bytes, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (SPI_READY() == 0);
}


void ADF_SET_FREQ_kHz()
{
	//2.45 GHz
//	ADF_SPI_MEM_WR(0x300,0x08);
//	ADF_SPI_MEM_WR(0x301,0xBD);
//	ADF_SPI_MEM_WR(0x302,0x03);

	//2.47 GHz
	ADF_SPI_MEM_WR(0x302,0x03);
	ADF_SPI_MEM_WR(0x301,0xC4);
	ADF_SPI_MEM_WR(0x300,0xD8);
}

uint8_t SPI_READY(void)
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

uint8_t Rx_READY(void)
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

uint8_t PHY_RDY_READY(void)
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

uint8_t Idle_READY(void)
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

void set_IDLE(void)
{
	uint8_t status;
	result = ADF_SPI_SEND_BYTE(0xb2);	//Idle mode
	HAL_Delay(5);
}

void set_PHY_RDY(void)
{
	uint8_t status;
	status = ADF_SPI_SEND_BYTE(0xb3);	//PHY_RDY mode
	HAL_Delay(5);
}

uint8_t wait_RC_RDY(void)
{
	uint8_t status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi2, &status, 1, 50);
//	HAL_SPI_TransmitReceive(&hspi2, 0xff, &status, 2, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (SPI_READY() == 0);

	if ((status&0x20) == 0x20)
		return 1;
	else
		return 0;
}

void ADF_Tx_mode(void)
{
	ADF_SPI_SEND_BYTE_NO_RECEIVE(0xb5);											//Tx mode
}

void ADF_Rx_mode(void)
{
	result = ADF_SPI_SEND_BYTE(0xb4);											//Rx mode
	while (Rx_READY() == 0);
}

uint8_t ADF_WR_Tx_Buffer(uint8_t offset, uint8_t data)
{
	ADF_SPI_MEM_WR(TX_BUFFER_BASE + offset, data);
	return ADF_SPI_MEM_RD(TX_BUFFER_BASE + offset);
}

uint8_t ADF_RD_Tx_Buffer(uint8_t offset)
{
	return ADF_SPI_MEM_RD(TX_BUFFER_BASE + offset);
}

uint8_t ADF_Rx_flag_set(void)
{
	uint8_t status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi2, &status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	if((status&0x40) == 0x40)
		return 1;
	else
		return 0;
}

void ADF_clear_Rx_flag(void)
{
	ADF_SPI_MEM_WR(0x3cc,0xff);
	while (ADF_Rx_flag_set() == 1);
}

uint32_t ADF7242_RD_Frequency_MHz(void)
{
	uint32_t frequency = ADF_SPI_MEM_RD(0x302);
	frequency = (frequency<<8) | ADF_SPI_MEM_RD(0x301);
	frequency = (frequency<<8) | ADF_SPI_MEM_RD(0x300);

	frequency /= 100;

	return frequency;
}