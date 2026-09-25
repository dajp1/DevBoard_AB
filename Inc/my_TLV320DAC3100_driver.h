/*
 * myAudioDAC_stuff.h
 *
 *  Created on: 4 Aug 2026
 *      Author: Dave Pearce
 */

#ifndef MY_TLV30DAC3100_DRIVER_H_
#define MY_TLV30DAC3100_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

// I2C address for TLV320DAC3100:
#define TLV320DAC3100_I2C_ADDR 0x18

typedef struct {
	I2C_HandleTypeDef *hi2c;
    I2S_HandleTypeDef *hi2s;
    uint16_t dev_addr;
} DAC3100_HandleTypeDef;

// Function declarations:
void my_tlv320dac3100_selectPage(DAC3100_HandleTypeDef *dev_handle, uint8_t page);
uint8_t my_tlv320dac3100_read_reg(DAC3100_HandleTypeDef *dev_handle, uint8_t reg);
void my_tlv320dac3100_write_reg(DAC3100_HandleTypeDef *dev_handle, uint8_t reg, uint8_t value);
HAL_StatusTypeDef my_init_tlv320DAC3100(DAC3100_HandleTypeDef *dev_handle, I2C_HandleTypeDef *hi2c_bus, I2S_HandleTypeDef *hi2s_bus);
void DAC3100_StartAudio(I2S_HandleTypeDef *hi2s_bus, uint16_t *buffer, uint16_t size);

// Callback functions (required to be placed inside main.c):
void DAC3100_HalfTransfer_Callback(void);
void DAC3100_TransferComplete_Callback(void);

#endif /* MY_TLV30DAC3100_DRIVER_H_ */
