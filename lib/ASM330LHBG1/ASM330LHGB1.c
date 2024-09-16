#include "ASM330LHGB1.h"

uint8_t generateIMUAddress(imu_reg_t imuRegNum, bool readFlag) {

  uint8_t newAddress = imuRegNum & 0x7F;

  if (readFlag) {
    newAddress |= (1 << 7);
  }

  return (uint8_t) newAddress;
}

uint8_t ensureIMUNotReserved(imu_reg_t regToCheck) {

  if ((regToCheck == 0x0C) || (regToCheck == 0x1F) || (0x2E <= regToCheck && regToCheck <= 0x34) ||
      (0x3C <= regToCheck && regToCheck <= 0x3F) || (0x44 <= regToCheck && regToCheck <= 0x55) ||
      (regToCheck == 0x57) || (regToCheck == 0x5A) || (regToCheck == 0x60) || (regToCheck == 0x61) ||
      (0x64 <= regToCheck && regToCheck <= 0x72) || (regToCheck == 0x76) || (regToCheck == 0x77)) {

        printf("ERROR: YOU ARE WRITING TO A RESERVED REGISTER: %d", regToCheck);
        return 1;

      }

  return 0;
}

uint8_t writeIMURegister(SPI_HandleTypeDef* imuSPI, imu_reg_t imuRegNum, uint8_t valueToWrite) {

	uint8_t actualRegNumber = generateIMUAddress(imuRegNum, false);
	uint8_t command[] = {actualRegNumber, valueToWrite};

	HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET);
	uint8_t SPIResult = HAL_SPI_Transmit(imuSPI, command, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET);

	return SPIResult;
}

uint8_t readIMUSingleRegister(SPI_HandleTypeDef* imuSPI, imu_reg_t imuRegNum) {

	uint8_t actualRegNumber = generateIMUAddress(imuRegNum, true);
	uint8_t regValue;

	HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET);
	uint8_t SPIResult = HAL_SPI_Transmit(imuSPI, &actualRegNumber, 1, HAL_MAX_DELAY);
	SPIResult = HAL_SPI_Receive(imuSPI, &regValue, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET);

	return regValue;
}

uint16_t readIMUDoubleRegister(SPI_HandleTypeDef* imuSPI, imu_reg_t upperRegAddress, imu_reg_t lowerRegAddress) {

	uint8_t upper8 = readIMUSingleRegister(imuSPI, upperRegAddress);
	uint8_t lower8 = readIMUSingleRegister(imuSPI, lowerRegAddress);

	uint16_t finalResult = (uint16_t) upper8 << 8 | (uint16_t) lower8;
	return finalResult;
}

uint8_t initializeIMU(SPI_HandleTypeDef* imuSPI) {

	HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET);
	uint8_t status = writeIMURegister(imuSPI, PIN_CTRL, 0b01111111);
	HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET);

	return status;
}