#ifndef __MS5611
#define __MS5611

#include "stm32h7xx_hal.h"
#include "SPI_Device.h"

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
} baro_accuracy_t;

typedef enum {
    READ_ADC = 0x00,
    BARO_RESET = 0x1E,
    PROM_READ = 0xA0,
} baro_commands_t;

typedef enum {
    LOWEST_TIME = 1,
    LOW_TIME = 2,
    MED_TIME = 3,
    HIGH_TIME = 5,
    HIGHEST_TIME = 9,
} baro_conversion_time_t;

// Make sure you use the correct percision value for each

typedef struct {
    int32_t temperature;
    int32_t pressure;
    baro_accuracy_t tempAccuracy; // Use only the D1 values
    baro_accuracy_t pressureAccuracy; // Use only the D2 values
    baro_conversion_time_t convertTime;
    uint16_t coefficients[8];
} baro_handle_t;

void resetBarometer(SPI_HandleTypeDef* baroSPI);
void getPROMData(SPI_HandleTypeDef* baroSPI, baro_handle_t* baroHandle);
void getCurrTempPressure(SPI_HandleTypeDef* baroSPI, baro_handle_t* baroHandle);

#endif




