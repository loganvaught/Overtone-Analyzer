/*
 * ssd1306.h
 *
 *  Created on: Nov 5, 2026
 *      Author: Logan Vaught
 */

#ifndef CUSTOM_INC_SSD1306_H_
#define CUSTOM_INC_SSD1306_H_

// Includes
#include "main.h"
#include <math.h>
#include <stdlib.h>
#include "string.h"
#include "arm_math.h"

// Addressing (For slave address) Note: Accounts for read/write bit (bit 0 in the 8-byte address)
#define SSD1306_ADDRESS (0b01111000 | (0 << 1)) // Note: bit 1 is SA0 Bit. Put 0 if SA0 bit is tied low, 1 if it is tied high. Mine is tied low. 0th bit is for read / write.

// Control (To be transmitted right after addressing, signifying a data or command request. Assumes stand-alone control bits (continuation bit = 0))
#define SSD1306_CONTROL_DATA (0x00 | (1 << 6)) // Note: 7th bit is continuation bit, 0 = no extra continuation. 6th bit: 0 = command, 1 = data only
#define SSD1306_CONTROL_COMMAND (0x00 | (0 << 6))

// Commands (You should transmit SSD1306_CONTROL_DATA before sending command)
#define SSD1306_CONTRAST_SET 0x81 // Double byte command: For next byte, bits 7:0 signifies contrast from 1-256, higher = more contrast
#define SSD1306_DISPLAY_ALL_ON (0xA4 | (1 << 0)) // Entire display on. Send DISPLAY_RAM_BASED to switch back
#define SSD1306_DISPLAY_RAM_BASED (0xA4 | (0 << 0)) // Display is RAM based (Normal mode)
#define SSD1306_DISPLAY_INVERSION_NORMAL (0xA6 | (0 << 0)) // Display is not inverted: 0 in ram means pixel off
#define SSD1306_DISPLAY_INVERSION_INVERTED (0xA6 | (1 << 0)) // Display is not inverted: 0 in ram means pixel on
#define SSD1306_DISPLAY_ON (0xAE | (1 << 0)) // Display is on. Not in sleep mode
#define SSD1306_DISPLAY_OFF (0xAE | (0 << 0)) // Display is in sleep mode.

// Address setting command
#define SSD1306_ADDRESSING_MODE_SET 0x20
#define SSD1306_ADDRESSING_MODE_HORIZONTAL 0x00
#define SSD1306_ADDRESSING_MODE_VERTICAL 0x01
#define SSD1306_ADDRESSING_RANGE_SET_COLUMNS 0x21 // X axis
#define SSD1306_ADDRESSING_RANGE_MIN_COLUMNS 0x00
#define SSD1306_ADDRESSING_RANGE_MAX_COLUMNS 0x7F
#define SSD1306_ADDRESSING_RANGE_SET_PAGES 0x22 // Y axis
#define SSD1306_ADDRESSING_RANGE_MIN_PAGES 0x00
#define SSD1306_ADDRESSING_RANGE_MAX_PAGES 0x07
// Note: There are display scrolling commands I have not implemented yet. See datasheet scrolling commands

// Charge Pump commands (Internal step up from VDD to power the display. Must use if not supplying Vcc with own higher voltage)
#define SSD1306_CHARGE_PUMP_SET 0x8D // Double byte command. See defines below for possible settings
#define SSD1306_CHARGE_PUMP_ON 0x14 // Must first send CHARGE_PUMP_SET command


// Constants
#define SSD1306_TOTAL_PIXELS_BYTES 1024
#define SSD1306_VERTICAL_PIXELS 64
#define SSD1306_HORIZONTAL_PIXELS 128


// Enums

typedef enum {
	SSD1306_OK,
	SSD1306_Error,
	SSD1306_FailedEnableCP,
	SSD1306_FailedTurnOn,
	SSD1306_FailedAllOn,
	SSD1306_FailedRamBased,
	SSD1306_FailedSetAddressingMode,
	SSD1306_FailedSetAddressingRange,
	SSD1306_FailedClearScreen,
	SSD1306_FailedUpdateScreen,
	SSD1306_FailedGenerateBuffer,
	SSD1306_FailedGenerateFrequency,
	SSD1306_NotSupported,
	SSD1306_TooSmallScale,
} ssd1306_result_t;

typedef enum {
	SSD1306_BucketsWithLineRange,
	SSD1306_BucketsAverage,
} ssd1306_buffer_plot_mode_t;

typedef enum {
	SSD1306_Center,
	SSD1306_Left,
	SSD1306_Right,

} ssd1306_frequency_plot_mode_t;

// Handle

typedef struct {
	I2C_HandleTypeDef *i2c;
	GPIO_TypeDef *SDA_Port;
	uint16_t SDA_Pin;
	uint8_t screen_buffer[1024];
	float temp_frequency_buff[128];
} ssd1306_handle_t;

// Functions

ssd1306_result_t ssd1306_init(ssd1306_handle_t *device, I2C_HandleTypeDef *i2c, GPIO_TypeDef *SDA_Port, uint16_t SDA_Pin);
ssd1306_result_t ssd1306_all_on_mode(ssd1306_handle_t *device);
ssd1306_result_t ssd1306_ram_based_mode(ssd1306_handle_t *device);
ssd1306_result_t ssd1306_clear_screen(ssd1306_handle_t *device);

// Specific for waveform/buffer plotting
ssd1306_result_t ssd1306_generate_buffer(ssd1306_handle_t *device, float *buffer, uint16_t start_element, uint16_t number_elements, ssd1306_buffer_plot_mode_t mode, uint16_t scale);
//ssd1306_result_t ssd1306_generate_frequency(ssd1306_handle_t *device, uint16_t *frequency_buffer, uint8_t number_frequencies, uint16_t amplitude, ssd1306_frequency_plot_mode_t mode);

#endif /* CUSTOM_INC_SSD1306_H_ */
