#include "hx711.h"

void hx711_delay(uint32_t delay) {
	while (delay--)
		;
}

void HX711_Init(hx711_t *hx711, GPIO_TypeDef *clkPort, uint16_t clkPin,
		GPIO_TypeDef *datPort, uint16_t datPin) {
	hx711->debuggerPort = NULL;
	hx711->clk_gpio = clkPort;
	hx711->clk_pin = clkPin;
	hx711->dat_gpio = datPort;
	hx711->dat_pin = datPin;

	GPIO_InitTypeDef gpio = { 0 };

	// Clock (PD_SCK) pin is configured as an output
	gpio.Pin = hx711->clk_pin;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(hx711->clk_gpio, &gpio);

	// Data (DOUT) pin is configured as an input
	gpio.Pin = hx711->dat_pin;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(hx711->dat_gpio, &gpio);

	// Initial state
	HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
}

void HX711_Init_Deb(hx711_t *hx711, UART_HandleTypeDef *debugger,
		GPIO_TypeDef *clkPort, uint16_t clkPin, GPIO_TypeDef *datPort,
		uint16_t datPin) {
	hx711->debuggerPort = debugger;
	hx711->clk_gpio = clkPort;
	hx711->clk_pin = clkPin;
	hx711->dat_gpio = datPort;
	hx711->dat_pin = datPin;

	GPIO_InitTypeDef gpio = { 0 };

	// Clock (PD_SCK) pin is configured as an output
	gpio.Pin = hx711->clk_pin;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(hx711->clk_gpio, &gpio);

	// Data (DOUT) pin is configured as an input
	gpio.Pin = hx711->dat_pin;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(hx711->dat_gpio, &gpio);

	// Initial state
	HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
}

void HX711_Calibrate(hx711_t *hx711, float known_weight) {
	uint32_t raw_data_sum = 0;
	uint32_t average_raw_data = 0;
	char buffer[50];

	// Inform the user to place the weight
	sprintf(buffer, "Place %.2f grams on the scale...\r\n", known_weight);
	HAL_UART_Transmit(hx711->debuggerPort, (uint8_t*) buffer, strlen(buffer),
			HAL_MAX_DELAY);

	HAL_Delay(3000); // Give time for the user to place the weight

	// Read raw data multiple times and calculate average
	for (int i = 0; i < 10; i++) {
		raw_data_sum += HX711_Read(hx711);
		HAL_Delay(100); // Small delay between readings
	}

	average_raw_data = raw_data_sum / 10;

	// Calculate the calibration factor
	hx711->calibration_factor = (float) average_raw_data / known_weight;

	// Send the calculated calibration factor to the user
	sprintf(buffer, "Calibration Factor: %.6f\r\n", hx711->calibration_factor);
	HAL_UART_Transmit(hx711->debuggerPort, (uint8_t*) buffer, strlen(buffer),
			HAL_MAX_DELAY);
}

uint32_t HX711_Read(hx711_t *hx711) {
	uint32_t hx711_data = 0;

	while (HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin) == GPIO_PIN_SET) {
		// wait until data is ready
	}

	for (int i = 0; i < 24; i++) {
		HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET); // PD_SCK = HIGH
		hx711_delay(1000);
		hx711_data = (hx711_data << 1)
				| HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin); // data read from DOUT
		HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET); // PD_SCK = LOW
		hx711_delay(1000);
	}

	HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET); // PD_SCK = HIGH (again)
	hx711_delay(1000);
	HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
	hx711_delay(1000);

	hx711_data = hx711_data ^ 0x800000; //for ex. 0x900000 ^ 0x800000
	return hx711_data;
}

float HX711_GetWeight(hx711_t *hx711) {
	uint32_t raw_data = HX711_Read(hx711);
	return raw_data / hx711->calibration_factor;
}

// Process weight reading from HX711
void ProcessWeightReading(hx711_t *hx711_sensor) {
	float weight = HX711_GetWeight(hx711_sensor);
	char buffer[50];
	sprintf(buffer, "Weight: %.2f grams\r\n", weight);
	HAL_UART_Transmit(hx711_sensor->debuggerPort, (uint8_t*) buffer, strlen(buffer),
			HAL_MAX_DELAY);
	LogDataToSD(weight);
}
