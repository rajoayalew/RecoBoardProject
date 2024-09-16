#include "LIS3MDL.h"

uint8_t MAG_RESERVED_REG[] = {0x0B, 0x0C, 0x0D, 0x0E, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x25, 0x26};

uint8_t generateMagAddress(mag_reg_t regAddress, bool readFlag, bool consecutiveFlag) {

	uint8_t newAddress = regAddress & 0x3F;

	if (readFlag) {
		newAddress |= (1 << 7);
	}

	if (consecutiveFlag) {
		newAddress |= (1 << 6);
	}

	return (uint8_t) newAddress;
}

uint8_t readMagSingleRegister(SPI_HandleTypeDef* magSPIHandler, mag_reg_t regAddress) {

	uint8_t actualAddress = generateMagAddress(regAddress, true, false);

	short SPIResult;
	uint8_t regValue;

	HAL_GPIO_WritePin(MAG_NCS_GPIO_Port, MAG_NCS_Pin, GPIO_PIN_RESET);
	SPIResult = HAL_SPI_Transmit(magSPIHandler, &actualAddress, 1, HAL_MAX_DELAY);
	SPIResult = HAL_SPI_Receive(magSPIHandler, &regValue, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MAG_NCS_GPIO_Port, MAG_NCS_Pin, GPIO_PIN_SET);


	return regValue;
}

uint16_t readMagDoubleRegister(SPI_HandleTypeDef* magSPIHandler, mag_reg_t upperRegAddress, mag_reg_t lowerRegAddress) {

	uint8_t upperReg = generateMagAddress(upperRegAddress, true, false);
	uint8_t lowerReg = generateMagAddress(lowerRegAddress, true, false);

	short result;
	uint16_t finalResult;
	uint8_t upper8;
	uint8_t lower8;

	HAL_GPIO_WritePin(MAG_NCS_GPIO_Port, MAG_NCS_Pin, GPIO_PIN_RESET);

	result = HAL_SPI_Transmit(magSPIHandler, &lowerReg, 1, HAL_MAX_DELAY);
	result = HAL_SPI_Receive(magSPIHandler, &lower8, 1, HAL_MAX_DELAY);

	result = HAL_SPI_Transmit(magSPIHandler, &upperReg, 1, HAL_MAX_DELAY);
	result = HAL_SPI_Receive(magSPIHandler, &upper8, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(MAG_NCS_GPIO_Port, MAG_NCS_Pin, GPIO_PIN_SET);

	finalResult = (uint16_t) upper8 << 8 | (uint16_t) lower8;
	return finalResult;
}

uint8_t* readMagMultipleRegisters(SPI_HandleTypeDef* magSPIHandler, mag_reg_t regAddress, uint8_t numRegisters) {

	regAddress = generateMagAddress(regAddress, true, true);

	short SPIResult;
	uint8_t regValue[numRegisters];

	SPIResult = HAL_SPI_Transmit(magSPIHandler, &regAddress, 1, HAL_MAX_DELAY);
	SPIResult = HAL_SPI_Receive(magSPIHandler, regValue, numRegisters, HAL_MAX_DELAY);

	return regValue;
}

int writeMagRegister(SPI_HandleTypeDef* magSPIHandler, mag_reg_t regAddress, uint8_t valueToWrite) {


	uint8_t actualAddress = generateMagAddress(regAddress, false, false);
	uint8_t command[] = {actualAddress, valueToWrite};

	HAL_GPIO_WritePin(MAG_NCS_GPIO_Port, MAG_NCS_Pin, GPIO_PIN_RESET);
	short result = HAL_SPI_Transmit(magSPIHandler, command, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MAG_NCS_GPIO_Port, MAG_NCS_Pin, GPIO_PIN_SET);

	return result;
}

uint8_t getStatusRegister(uint8_t* returnArray, SPI_HandleTypeDef* hspi) {

	uint8_t statusRegAddress[] = {0x80 | 0x27, 0};
	uint8_t output[] = {0, 0};

  HAL_GPIO_WritePin(MAG_NCS_GPIO_Port, MAG_NCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, statusRegAddress, output, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(MAG_NCS_GPIO_Port, MAG_NCS_Pin, GPIO_PIN_SET);
	uint8_t actualOut = output[1];

	/*
	returnArray[0] = isNthBitSet(actualOut, 0);
	returnArray[1] = isNthBitSet(actualOut, 1);
	returnArray[2] = isNthBitSet(actualOut, 2);
	returnArray[3] = isNthBitSet(actualOut, 3);
	returnArray[4] = isNthBitSet(actualOut, 4);
	returnArray[5] = isNthBitSet(actualOut, 5);
	returnArray[6] = isNthBitSet(actualOut, 6);
	returnArray[7] = isNthBitSet(actualOut, 7);
	*/

	return actualOut;
}

