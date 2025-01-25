#ifndef INC_HX711_H_
#define INC_HX711_H_

#include "main.h"

typedef struct {
    GPIO_TypeDef *clk_gpio;  // GPIO port for the clock signal
    GPIO_TypeDef *dat_gpio;  // GPIO port for the data signal
    uint16_t clk_pin;        // GPIO pin for the clock signal
    uint16_t dat_pin;        // GPIO pin for the data signal
    float calibration_factor;
    UART_HandleTypeDef* debuggerPort;
} hx711_t;


void HX711_Init(hx711_t *hx711, GPIO_TypeDef *clkPort, uint16_t clkPin,
		GPIO_TypeDef *datPort, uint16_t datPin);
void HX711_Init_Deb(hx711_t *hx711, UART_HandleTypeDef* debugger, GPIO_TypeDef *clkPort, uint16_t clkPin,
		GPIO_TypeDef *datPort, uint16_t datPin);
void HX711_Calibrate(hx711_t *hx711, float known_weight);
uint32_t HX711_Read(hx711_t *hx711);
float HX711_GetWeight(hx711_t *hx711);
void ProcessWeightReading(hx711_t *hx711_sensor);

#endif
