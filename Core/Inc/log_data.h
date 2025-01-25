#ifndef INC_LOG_DATA_H_
#define INC_LOG_DATA_H_

#include "stm32f4xx_hal.h"
#include "fatfs.h"

// Function prototype for logging data to SD card
void LogDataToSD(float weight);
void SaveCalibrationFactorToSD(float factor);
float LoadCalibrationFactorFromSD(void);

#endif /* INC_LOG_DATA_H_ */
