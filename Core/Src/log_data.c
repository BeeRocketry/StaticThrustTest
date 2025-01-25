#include "log_data.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h> // for atof and sscanf
#include <inttypes.h>

void LogDataToSD(float weight) {
	FATFS fs;           // FATFS file system object
	FIL file;           // File object
	UINT bytes_written; // Variable to hold number of bytes written
	char buffer[50];    // Temporary buffer to hold the data

	size_t time_ms;
	// Mount the SD card
	if (f_mount(&fs, "", 1) == FR_OK) {
		// Open or create the file in append mode
		if (f_open(&file, "thrust_log.txt", FA_OPEN_APPEND | FA_WRITE)
				== FR_OK) {
			time_ms = HAL_GetTick();
			sprintf(buffer, "%zu,%.2f\n", time_ms, weight);
			f_write(&file, buffer, strlen(buffer), &bytes_written);

			f_close(&file);
		} else {
			printf("Error: Could not open thrust_log.txt for writing!\r\n");
		}

		// Unmount the SD card
		f_mount(NULL, "", 1);
	} else {
		printf("Error: Could not mount SD card!\r\n");
	}
}

void SaveCalibrationFactorToSD(float factor) {
	FATFS fs;
	FIL file;
	UINT bytes_written;
	char buffer[50];

	// Mount the SD card
	if (f_mount(&fs, "", 1) == FR_OK) {
		// Open or create the calibration file
		if (f_open(&file, "calibration.txt", FA_CREATE_ALWAYS | FA_WRITE)
				== FR_OK) {
			sprintf(buffer, "%.6f\n", factor); // Convert factor to string
			f_write(&file, buffer, strlen(buffer), &bytes_written);
			f_close(&file); // Close the file
		} else {
			printf("Error: Could not open calibration.txt for writing!\r\n");
		}
		f_mount(NULL, "", 1); // Unmount the SD card
	} else {
		printf("Error: Could not mount SD card for saving calibration!\r\n");
	}
}

float LoadCalibrationFactorFromSD(void) {
	FATFS fs;
	FIL file;
	UINT bytes_read;
	char buffer[50];
	float factor = 1.0f; // Default value in case of failure

	// Mount the SD card
	if (f_mount(&fs, "", 1) == FR_OK) {
		// Open the calibration file
		if (f_open(&file, "calibration.txt", FA_READ) == FR_OK) {
			memset(buffer, 0, sizeof(buffer)); // Clear buffer
			f_read(&file, buffer, sizeof(buffer) - 1, &bytes_read); // Read file content
			f_close(&file); // Close the file

			if (strlen(buffer) > 0) { // Ensure buffer is not empty
				factor = atof(buffer);

				if (factor == 0.0f && buffer[0] != '0') {
					factor = 1.0f; // Default value if atof fails
					printf(
							"Warning: Invalid calibration data. Using default value.\r\n");
				}
			}
		} else {
			printf("Error: Could not open calibration.txt for reading.\r\n");
		}
		f_mount(NULL, "", 1); // Unmount the SD card
	} else {
		printf("Error: Could not mount SD card for loading calibration!\r\n");
	}
	return factor;
}

