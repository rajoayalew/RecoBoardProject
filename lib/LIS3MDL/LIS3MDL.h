#ifndef _LIS3MDL
#define _LIS3MDL

#include "stm32h7xx_hal.h"
#include "stdbool.h"

typedef enum {
	WHO_AM_I_MAG = 0x0F,
	OFFSET_X_REG_L_M = 0x05,
	OFFSET_X_REG_H_M = 0x06,
	OFFSET_Y_REG_L_M = 0x07,
	OFFSET_Y_REG_H_M = 0x08,
	OFFSET_Z_REG_L_M = 0x09,
	OFFSET_Z_REG_H_M = 0x0A,
	CTRL_REG1 = 0x20,
	CTRL_REG2 = 0x21,
	CTRL_REG3 = 0x22,
	CTRL_REG4 = 0x23,
	CTRL_REG5 = 0x24,
	STATUS_REG = 0x27,
	OUT_X_L = 0x28,
	OUT_X_H = 0x29,
	OUT_Y_L = 0x2A,
	OUT_Y_H = 0x2B,
	OUT_Z_L = 0x2C,
	OUT_Z_H = 0x2D,
	TEMP_OUT_L = 0x2E,
	TEMP_OUT_H = 0x2F,
	INT_CFG = 0x30,
	INT_SRC = 0x31,
	INT_THS_L = 0x32,
	INT_THS_H = 0x33,
} mag_reg_t;

typedef struct mag_data_t {
	  uint16_t MAG_X_VAL[200];
	  uint16_t MAG_Y_VAL[200];
	  uint16_t MAG_Z_VAL[200];
} mag_data_t;

uint8_t MAG_RESERVED_REG[];

uint8_t generateMagAddress(mag_reg_t regAddress, bool readFlag, bool consecutiveFlag);
uint8_t* readMagMultipleRegisters(SPI_HandleTypeDef* magSPIHandler, mag_reg_t regAddress, uint8_t numRegisters);
uint8_t readMagSingleRegister(SPI_HandleTypeDef* magSPIHandler, mag_reg_t regAddress);
uint16_t readMagDoubleRegister(SPI_HandleTypeDef* magSPIHandler, mag_reg_t upperRegAddress, mag_reg_t lowerRegAddress);
uint8_t writeMagRegister(SPI_HandleTypeDef* magSPIHandler, mag_reg_t regAddress, uint8_t valueToWrite);
uint8_t getStatusRegister(uint8_t* returnArray, SPI_HandleTypeDef* hspi);


#endif

