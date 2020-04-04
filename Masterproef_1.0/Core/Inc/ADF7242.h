/*
 * ADF7242.h
 *
 *  Created on: Feb 28, 2020
 *      Author: jelle
 */

#ifndef INC_ADF7242_H_
#define INC_ADF7242_H_

#ifdef __cplusplus
extern C {
#endif

/**
 * The ADF7242 uses SPI for communication
 *
 * Default pinout
 *
ADF7242   |STM32F4    |DESCRIPTION

CS        |PB12       |Chip select (active low)
MOSI      |PB15       |Master output, slave input
SCLK      |PB10       |Serial clock line
MISO      |PB14       |Master input, slave output
IRQ1      |PA8        |Interrupt request 1
IRQ2      |PA9        |Interrupt request 2
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "CBUF.h"

/* Defines -------------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
void ADF_Init(void);
void ADF_SPI_MEM_WR(uint16_t, uint8_t);
uint8_t ADF_SPI_MEM_RD(uint16_t);
void ADF_SPI_RD_Rx_Buffer(void);
uint8_t ADF_SPI_SEND_BYTE(uint8_t);
void ADF_SET_FREQ_kHz(uint32_t);

uint8_t ADF_SET_RC_STATE(uint8_t);
void ADF_Tx_mode(void);
void ADF_Rx_mode(void);
uint8_t SPI_READY(void);
uint8_t Rx_READY(void);
uint8_t PHY_RDY_READY(void);
uint8_t Idle_READY(void);
void set_IDLE(void);
void set_PHY_RDY(void);
uint8_t ADF_WR_Tx_Buffer(uint8_t, uint8_t);
uint8_t ADF_RD_Tx_Buffer(uint8_t);
uint8_t ADF_SPI_STATUS(void);
uint8_t ADF_Rx_flag_set(void);
void ADF_clear_Rx_flag(void);
uint32_t ADF7242_RD_Frequency_MHz(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADF7242_H_ */
