/*
 * myAudioDAC_stuff.c
 *
 *  Created on: 4 Aug 2026
 *      Author: Dave Pearce
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32g4xx_hal.h"
#include "my_TLV320DAC3100_driver.h"

// For the printf() function while debugging on the Nucleo board:
#include "stm32g4xx_nucleo.h"

#define TAG "TLV320DAC3100"

// I2C Configuration (gets set on initialisation):
extern I2C_HandleTypeDef *hI2C_for_DAC;

void my_i2c_write_byte_to_register(DAC3100_HandleTypeDef *dev_handle, uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
    	dev_handle->hi2c,     // STM32 I2C handle
        (dev_handle->dev_addr << 1),  // 7-bit device address shifted left by 1 bit
        reg,                        // Register address
        I2C_MEMADD_SIZE_8BIT,       // Register address size (1 byte)
        &value,                     // Pointer to data to write
        1,                          // Number of data bytes to write
        50                          // Timeout in milliseconds
    );

    if (status != HAL_OK) {
        printf("I2C Transmit failed: %d\r\n", status);
    }

    HAL_Delay(10);
}

uint8_t my_i2c_read_byte_from_register(DAC3100_HandleTypeDef *dev_handle, uint8_t reg)
{
    uint8_t val = 0;

    // HAL_I2C_Mem_Read sends the register address, issues a repeated start, and reads back data
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
    	dev_handle->hi2c,     // STM32 I2C handle
        (dev_handle->dev_addr << 1),  // 7-bit device address shifted left by 1 bit
        reg,                        // Register address
        I2C_MEMADD_SIZE_8BIT,       // Register address size (1 byte)
        &val,                       // Pointer to buffer for received data
        1,                          // Number of bytes to read
        50                          // Timeout in milliseconds
    );

    if (status != HAL_OK) {
        printf("I2C Read failed: %d\r\n", status);
    }

    HAL_Delay(10);
    return val;
}

void my_tlv320dac3100_selectPage(DAC3100_HandleTypeDef *dev, uint8_t page) {
    my_i2c_write_byte_to_register(dev, 0x00, page);
}
uint8_t my_tlv320dac3100_read_reg(DAC3100_HandleTypeDef *dev, uint8_t reg) {
    uint8_t thing = my_i2c_read_byte_from_register(dev, reg);
    return thing;
}
void my_tlv320dac3100_write_reg(DAC3100_HandleTypeDef *dev, uint8_t reg, uint8_t value) {
    my_i2c_write_byte_to_register(dev, reg, value);
}

void DAC3100_StartAudio(I2S_HandleTypeDef *hi2s_bus, uint16_t *buffer, uint16_t size)
{
    // Starts streaming via DMA in the background loop:
    HAL_I2S_Transmit_DMA(hi2s_bus, buffer, size);
}

HAL_StatusTypeDef my_init_tlv320DAC3100(DAC3100_HandleTypeDef *dev_handle,
		I2C_HandleTypeDef *hi2c_bus, I2S_HandleTypeDef *hi2s_bus) {

	if (dev_handle == NULL || hi2c_bus == NULL || hi2s_bus == NULL) return HAL_ERROR;

	// Set up the device handle for future use:
	dev_handle->hi2c = hi2c_bus;
	dev_handle->hi2s = hi2s_bus;
	dev_handle->dev_addr = TLV320DAC3100_I2C_ADDR;

    //////////////////////////////////////////////////////////
    // Setting up the audio DAC: Direct from the TI datasheet:
    //
    // "The following list gives an example sequence of items that must be executed in the time
    // between powering the # device up and reading data from the device. Note that there are
    // other valid sequences depending on which features are used."
    //
    // 1. Define starting point:
    //    (a) Power up applicable external hardware power supplies
    //    (b) Set register page to 0
    my_tlv320dac3100_write_reg(dev_handle, 0x00, 0x00);
    //
    //    (c) Initiate SW reset (PLL is powered off as part of reset)
    my_tlv320dac3100_write_reg(dev_handle, 0x01, 0x01);
    //
    // 2. Program clock settings
    //    (a) Program PLL clock dividers P, J, D, R (if PLL is used)
    //
    //  This was odd - the first version of this I tried had P = 1, R = 1, J = 8, D = 0, which would
    //  suggest a PLL_CLOCK of 11.2896 MHz (assuming MCLK is 11.2896 MHz).  However, that is out of
    //  spec, as the PLL_CLOCK needs to be between 80 MHz and 110 MHz.  Furthermore, dividing by
    //  NDAC = 8, MDAC = 2 then DOSR (128) would give a sample rate of 5.586 kHz, which is very wrong.
    //  And yet it seemed to work.  Maybe the PLL locked up to a harmonic of the input clock that was
    //  in the right range?
    //
    //  I will try setting R = 8.  That should give a PLL_CLOCK of 90.3168 MHz, which is in spec.
    //  Then dividing by NDAC = 8, MDAC = 2, DOSR = 128 should give the correct sample rate of 44.1 kHz.
    //
    // PLL_clkin = MCLK, codec_clkin = PLL_CLK
    my_tlv320dac3100_write_reg(dev_handle, 0x04, 0x07);  // Was 0x03 in the TI code; setting to 0x07 here to try and select BLCK as PLL input
    // J = 8
    my_tlv320dac3100_write_reg(dev_handle, 0x06, 0x08);
    // D = 0000, D(13:8) = 0, D(7:0) = 0
    my_tlv320dac3100_write_reg(dev_handle, 0x07, 0x00);
    my_tlv320dac3100_write_reg(dev_handle, 0x08, 0x00);
    //
    //    (b) Power up PLL (if PLL is used)
    // PLL Power up, P = 1, R = 8, giving a 1.4112 * 8 * 8 = 90.3168 MHz PLL clock
    //
    my_tlv320dac3100_write_reg(dev_handle, 0x05, 0x98);  // 0x98 for P = 1, R = 8
    HAL_Delay(20);    // Wait for PLL to lock
    //
    //    (c) Program and power up NDAC
    //
    // NDAC is powered up and set to 8
    my_tlv320dac3100_write_reg(dev_handle, 0x0B, 0x88);
    //
    //    (d) Program and power up MDAC
    //
    // MDAC is powered up and set to 2
    my_tlv320dac3100_write_reg(dev_handle, 0x0C, 0x82);
    //
    //    (e) Program OSR value
    //
    // DOSR = 128, DOSR(9:8) = 0, DOSR(7:0) = 128
    my_tlv320dac3100_write_reg(dev_handle, 0x0D, 0x00);
    my_tlv320dac3100_write_reg(dev_handle, 0x0E, 0x80);
    //
    //    (f) Program I2S word length if required (16, 20, 24, 32 bits)
    //        and master mode (BCLK and WCLK are outputs)
    //
    // mode is i2s, wordlength is 16, slave mode
    my_tlv320dac3100_write_reg(dev_handle, 0x1B, 0x00);
    //
    //    (g) Program the processing block to be used
    //
    // Select Processing Block PRB_P11
    my_tlv320dac3100_write_reg(dev_handle, 0x3C, 0x0B);
    my_tlv320dac3100_write_reg(dev_handle, 0x00, 0x08);
    my_tlv320dac3100_write_reg(dev_handle, 0x01, 0x04);
    my_tlv320dac3100_write_reg(dev_handle, 0x00, 0x00);
    //
    //    (h) Miscellaneous page 0 controls
    // DAC => volume control thru pin disable
    my_tlv320dac3100_write_reg(dev_handle, 0x74, 0x00);
    //
    // 3. Program analog blocks
    //
    //    (a) Set register page to 1
    // Set page to 1 for analog block configuration
    my_tlv320dac3100_write_reg(dev_handle, 0x00, 0x01);
    //
    //    (b) Program common-mode voltage (default = 1.35 V)
    //
    my_tlv320dac3100_write_reg(dev_handle, 0x1F, 0x04);
    //
    //    (c) Program headphone-specific depop settings (in case headphone driver is used)
    //
    // De-pop, Power on = 800 ms, Step time = 4 ms
    my_tlv320dac3100_write_reg(dev_handle, 0x21, 0x4E);
    //
    //    (d) Program routing of DAC output to the output amplifier (headphone/lineout or speaker)
    // LDAC routed to HPL out, RDAC routed to HPR out
    my_tlv320dac3100_write_reg(dev_handle, 0x23, 0x44);
    //
    //    (e) Unmute and set gain of output driver
    //
    // Unmute HPL, set gain = 0 dB
    my_tlv320dac3100_write_reg(dev_handle, 0x28, 0x06);
    // Unmute HPR, set gain = 0 dB
    my_tlv320dac3100_write_reg(dev_handle, 0x29, 0x06);
    // Unmute Class-D, set gain = 24 dB (this seems to reduce the high-frequency noise on
    // the headphone output at extreme values; I'm not entirely sure why this happens)
    my_tlv320dac3100_write_reg(dev_handle, 0x2A, 0x1C);
    //
    //    (f) Power up output drivers
    //
    // HPL and HPR powered up
    my_tlv320dac3100_write_reg(dev_handle, 0x1F, 0xC2);
    //Power-up Class-D driver
    my_tlv320dac3100_write_reg(dev_handle, 0x20, 0x86);
    // Enable HPL output analog volume, set = -9 dB
    my_tlv320dac3100_write_reg(dev_handle, 0x24, 0x92);
    // Enable HPR output analog volume, set = -9 dB
    my_tlv320dac3100_write_reg(dev_handle, 0x25, 0x92);
    // Enable Class-D output analog volume, set = -9 dB
    my_tlv320dac3100_write_reg(dev_handle, 0x26, 0x92);
    //
    // 4. Apply waiting time determined by the de-pop settings and the soft-stepping settings
    //    of the driver gain or poll page 1 / register 63
    HAL_Delay(800);
    //
    // 5. Power up DAC
    //    (a) Set register page to 0
    my_tlv320dac3100_write_reg(dev_handle, 0x00, 0x00);
    //
    //    (b) Power up DAC channels and set digital gain
    // Powerup DAC left and right channels (soft step enabled)
    my_tlv320dac3100_write_reg(dev_handle, 0x3F, 0xD4);
    //
    // DAC Left gain = -22 dB
    my_tlv320dac3100_write_reg(dev_handle, 0x41, 0x00);  // was 0x24 for -22 dB; setting to 0x00 here to try and set to 0 dB
    // DAC Right gain = -22 dB
    my_tlv320dac3100_write_reg(dev_handle, 0x42, 0x00);  // was 0x24 for -22 dB; setting to 0x00 here to try and set to 0 dB
    //
    //    (c) Unmute digital volume control
    //
    // Unmute DAC left and right channels
    my_tlv320dac3100_write_reg(dev_handle, 0x40, 0x00);

    // Tests to see if anything is working:
    HAL_Delay(100);
    my_tlv320dac3100_write_reg(dev_handle, 0x00, 0x00);
    uint8_t dacFlagRegister = my_tlv320dac3100_read_reg(dev_handle, 0x25);
    printf("DAC Flag register contains: 0x%02X\n", dacFlagRegister);
    uint8_t audioStatusRegister = my_tlv320dac3100_read_reg(dev_handle, 0x26);
    printf("Audio status register contains: 0x%02X\n", audioStatusRegister);
    uint8_t pll = my_tlv320dac3100_read_reg(dev_handle, 0x04);
    printf("PLL control = 0x%02X\n", pll);

    return HAL_OK;
}
