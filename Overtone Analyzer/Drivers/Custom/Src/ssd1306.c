/*
 * ssd1306.c
 *
 *  Created on: Nov 5, 2026
 *      Author: Logan Vaught
 */


#include "ssd1306.h"

static ssd1306_result_t ssd1306_send_command(ssd1306_handle_t *device, uint8_t *command_buffer, uint16_t length_bytes) {
	ssd1306_result_t status = SSD1306_OK;
	if (HAL_I2C_Master_Transmit(device->i2c, SSD1306_ADDRESS, command_buffer, length_bytes, 100) != HAL_OK) {
		status = SSD1306_Error;
	}
	return status;
}

static ssd1306_result_t ssd1306_write(ssd1306_handle_t *device, uint8_t *data, uint16_t length_bytes) {
	ssd1306_result_t status = SSD1306_OK;
	// If I am not mistaken, the register it starts and ends with in GDDSRAM depends on what is configured with the column and page start and end addresses. For me, both are set to min (0) and max(127, and 7 respectively)
	// Note: The third parameter is the control register, because the SSD1306 does not require a specific mem address, it requires the control byte, which will be sent automatically here.
	if (HAL_I2C_Mem_Write(device->i2c, SSD1306_ADDRESS, SSD1306_CONTROL_DATA, I2C_MEMADD_SIZE_8BIT, data, length_bytes, 1000) != HAL_OK) {
		status = SSD1306_Error;
	}
	return status;
}

static ssd1306_result_t ssd1306_enable_charge_pump(ssd1306_handle_t *device) {
	ssd1306_result_t status = SSD1306_OK;

	uint8_t command[3] = {SSD1306_CONTROL_COMMAND, SSD1306_CHARGE_PUMP_SET, SSD1306_CHARGE_PUMP_ON};
	status = ssd1306_send_command(device, command, 3);
	if (status != SSD1306_OK) return SSD1306_FailedEnableCP;

	return status;
}

static ssd1306_result_t ssd1306_turn_on(ssd1306_handle_t *device) {
	ssd1306_result_t status = SSD1306_OK;

	uint8_t command[2] = {SSD1306_CONTROL_COMMAND, SSD1306_DISPLAY_ON};
	status = ssd1306_send_command(device, command, 2);
	if (status != SSD1306_OK) return SSD1306_FailedTurnOn;

	return status;
}

static ssd1306_result_t ssd1306_update_screen(ssd1306_handle_t *device) {
	ssd1306_result_t status = SSD1306_OK;

	status = ssd1306_write(device, device->screen_buffer, 1024);
	if (status != SSD1306_OK) return SSD1306_FailedUpdateScreen;

	return status;
}

ssd1306_result_t ssd1306_all_on_mode(ssd1306_handle_t *device) {
	ssd1306_result_t status = SSD1306_OK;

	uint8_t command[2] = {SSD1306_CONTROL_COMMAND, SSD1306_DISPLAY_ALL_ON};
	status = ssd1306_send_command(device, command, 2);
	if (status != SSD1306_OK) return SSD1306_FailedAllOn;


	return status;
}

ssd1306_result_t ssd1306_ram_based_mode(ssd1306_handle_t *device) {
	ssd1306_result_t status = SSD1306_OK;

	uint8_t command[2] = {SSD1306_CONTROL_COMMAND, SSD1306_DISPLAY_RAM_BASED};
	status = ssd1306_send_command(device, command, 2);
	if (status != SSD1306_OK) return SSD1306_FailedRamBased;


	return status;
}

ssd1306_result_t ssd1306_clear_screen(ssd1306_handle_t *device) {
	uint16_t i = 0;
	for (; i < SSD1306_TOTAL_PIXELS_BYTES; i ++) {
		device->screen_buffer[i] = 0x00;
	}

	ssd1306_result_t status = SSD1306_OK;
	status = ssd1306_update_screen(device);
	if (status != SSD1306_OK) return SSD1306_FailedClearScreen;

	return status;
}

ssd1306_result_t ssd1306_generate_buffer(ssd1306_handle_t *device, float *buffer, uint16_t start_element, uint16_t number_elements, ssd1306_buffer_plot_mode_t mode, uint16_t scale) {
	if (device == NULL || number_elements == 0 || buffer == NULL) return SSD1306_Error;

	ssd1306_result_t status = SSD1306_OK;

	switch (mode) {
	case SSD1306_BucketsAverage:
		memset(device->screen_buffer, 0, 1024);

		// How many bytes of the buffer will be represented in one horizontal pixel
		uint16_t bucket_size = number_elements / (SSD1306_HORIZONTAL_PIXELS);

		if (bucket_size == 0) return SSD1306_Error;

		// Mean for centering
		float mean = 0;
		arm_mean_f32(buffer + start_element, number_elements, &mean);

		for (uint8_t i = 0; i < SSD1306_HORIZONTAL_PIXELS; i ++) {
			// For each vertical column
			// find the value to represent here
			uint16_t start_bucket_index = i * bucket_size;
			float total = 0;

			for (uint16_t j = 0; j < bucket_size; j ++) {
				float value_here = buffer[start_element + start_bucket_index + j];
				total += value_here;
			}
			float average_bucket = total / (float) bucket_size;

			if (fabsf(average_bucket - mean) > scale) continue; // This one is too large to be seen in frame

			// Where vertical position is, lowest pixel = 0, highest pixel = 64. Note: This is a 65 range. It will have to be clamped
			float normalized = (average_bucket - mean) / (float) scale;
			int8_t calculated_y = (int8_t) ((normalized * ((float) SSD1306_VERTICAL_PIXELS) * 0.5f) + 32.0f);

			// Do a clamp for safety on calculated_y (
			if (calculated_y < 0) calculated_y = 0;
			if (calculated_y > SSD1306_VERTICAL_PIXELS - 1) calculated_y = SSD1306_VERTICAL_PIXELS - 1;

			// Toggle the correct bit in the GDDSRAM

			// Need to invert calculated_y because vertical addressing mode goes from top to bottom
			uint8_t screen_y = (SSD1306_VERTICAL_PIXELS - 1) - (uint8_t) calculated_y;
			// Pages range from 0 (top) to 7 (bottom)
			uint8_t page_on = screen_y / 8;
			// On each page, the highest pixel is the lsb.
			uint8_t bit_on_page = screen_y % 8;

			// Finally, update screen buffer
			device->screen_buffer[i * 8 + page_on] = 0x00 | (1 << bit_on_page);
		}
		// Update screen

		status = ssd1306_update_screen(device);
		if (status != SSD1306_OK) return status;
		break;
	default:
		return SSD1306_NotSupported;
	}

	return status;
}

ssd1306_result_t ssd1306_generate_frequencies(ssd1306_handle_t *device, uint16_t *frequency_buffer, uint8_t number_frequencies, uint16_t amplitude, ssd1306_frequency_plot_mode_t mode) {
	if (device == NULL) return SSD1306_Error;

	ssd1306_result_t status = SSD1306_OK;

	switch (mode) {
	case SSD1306_Center:

		// Assumes an empty screen. Will or all bits onto the screen

		for (int i = 0; i < nmber_frequencies; i ++) {

		}

		break;
	default:
		return SSD1306_NotSupported;
	}

	return status;
}

ssd1306_result_t ssd1306_init(ssd1306_handle_t *device, I2C_HandleTypeDef *i2c, GPIO_TypeDef *SDA_Port, uint16_t SDA_Pin) {
	ssd1306_result_t status = SSD1306_OK;

	device->SDA_Pin = SDA_Pin;
	device->SDA_Port = SDA_Port;
	device->i2c = i2c;


	// Enable charge pump
	status = ssd1306_enable_charge_pump(device);
	if (status != SSD1306_OK) return status;

	// Turn on display
	status = ssd1306_turn_on(device);
	if (status != SSD1306_OK) return status;

	// Set to vertical addressing mode
	uint8_t command[4] = {SSD1306_CONTROL_COMMAND, SSD1306_ADDRESSING_MODE_SET, SSD1306_ADDRESSING_MODE_VERTICAL, 0};
	status = ssd1306_send_command(device, command, 3);
	if (status != SSD1306_OK) return SSD1306_FailedSetAddressingMode;

	// Set column addressing range for horizontal address mode
	command[0] = SSD1306_CONTROL_COMMAND;
	command[1] = SSD1306_ADDRESSING_RANGE_SET_COLUMNS;
	command[2] = SSD1306_ADDRESSING_RANGE_MIN_COLUMNS;
	command[3] = SSD1306_ADDRESSING_RANGE_MAX_COLUMNS;
	status = ssd1306_send_command(device, command, 4);
	if (status != SSD1306_OK) return SSD1306_FailedSetAddressingRange;

	// Set page addressing range for horizontal address mode
	command[0] = SSD1306_CONTROL_COMMAND;
	command[1] = SSD1306_ADDRESSING_RANGE_SET_PAGES;
	command[2] = SSD1306_ADDRESSING_RANGE_MIN_PAGES;
	command[3] = SSD1306_ADDRESSING_RANGE_MAX_PAGES;
	status = ssd1306_send_command(device, command, 4);
	if (status != SSD1306_OK) return SSD1306_FailedSetAddressingRange;

	// Clear screen
	status = ssd1306_clear_screen(device);
	if (status != SSD1306_OK) return SSD1306_FailedClearScreen;

	return status;
}
