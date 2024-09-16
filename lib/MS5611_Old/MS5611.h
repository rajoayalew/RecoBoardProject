#ifndef __MS5611
#define __MS5611

#include "main.h"

typedef enum {
	  LOWEST_D1 = 0x40,
	  LOW_D1 = 0x42,
	  MED_D1 = 0x44,
	  HIGH_D1 = 0x46,
	  HIGHEST_D1 = 0x48,
	  LOWEST_D2 = 0x50,
	  LOW_D2 = 0x52,
	  MED_D2 = 0x54,
	  HIGH_D2 = 0x56,
	  HIGHEST_D2 = 0x58,
} BARO_CONV;

typedef enum {
	READ_ADC = 0x00,
	BARO_RESET = 0x1E,
	PROM_READ = 0xA0,
} BARO_COMMANDS;

typedef struct {
	float temperature;
	float pressure;
	BARO_CONV tempConv; // Use only the D1 values
	BARO_CONV pressConv; // Use only the D2 values
	float C[7];
} BARO_HANDLE;

void resetBarometer(SPI_HandleTypeDef *baroSPI, uint16_t baro_ncs);
void initBarometer(SPI_HandleTypeDef *baroSPI, BARO_HANDLE *baroHandle, uint16_t baro_ncs);
void getTempPress(SPI_HandleTypeDef *baroSPI, BARO_HANDLE *baroHandle, uint16_t baro_ncs);
void tempConv(SPI_HandleTypeDef *baroSPI, BARO_HANDLE *baroHandle, uint16_t baro_ncs);
void pressConv(SPI_HandleTypeDef *baroSPI, BARO_HANDLE *baroHandle, uint16_t baro_ncs);

#endif
