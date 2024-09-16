#include "MS5611.h"

void resetBarometer(spi_device_t* baroSPI) {
  	uint8_t baroReset = BARO_RESET;
    SPI_Device_Transmit(baroSPI, &baroReset, 1, HAL_MAX_DELAY);
}

void getPROMData(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

    uint8_t PROM_COMMAND = PROM_READ;
    uint8_t rxBuffer[2];

    for (int i = 0; i < 6; i++) {
        SPI_Device_Transmit(baroSPI->hspi, &PROM_COMMAND, 1, HAL_MAX_DELAY);
        SPI_Device_Receive(baroSPI->hspi, rxBuffer, 2, HAL_MAX_DELAY);
        baroHandle->coefficients[i] = (uint16_t) rxBuffer[1] << 8 | rxBuffer[0];
        PROM_COMMAND += 2;
    }

    return;
}

void initBarometer(spi_device_t* baroSPI, baro_handle_t* baroHandle) {
    resetBarometer(baroSPI);
    getPROMData(baroSPI, baroHandle);

    /*

    baroHandle->coefficients[0] = baroHandle->coefficients[0] << 8;
    baroHandle->coefficients[1] = baroHandle->coefficients[1];
    baroHandle->coefficients[2] = baroHandle->coefficients[2];
    baroHandle->coefficients[3] = baroHandle->coefficients[3];
    baroHandle->coefficients[4] = baroHandle->coefficients[4];
    baroHandle->coefficients[5] = baroHandle->coefficients[5];

    */


    return;
}

void getTemp() {

}

void getPressure() {

}

void getCurrTempPressure(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

	HAL_GPIO_WritePin(GPIOC, BAR_NCS_Pin, GPIO_PIN_RESET);
    uint16_t coefficients[] = baroHandle->coefficients;
    uint8_t readADCCommand[4] = {READ_ADC, 0, 0, 0};

    HAL_SPI_Transmit(baroSPI, &(baroHandle->tempAccuracy), 1, HAL_MAX_DELAY);
    HAL_Delay(baroHandle->convertTime);

    uint8_t digitalTempBuff[4];
    HAL_SPI_TransmitReceive(baroSPI, readADCCommand, digitalTempBuff, 4, HAL_MAX_DELAY);

    uint32_t digitalTemp = (digitalTempBuff[1] << 16) | (digitalTempBuff[2] << 8) | (digitalTempBuff[3] << 0);

    HAL_SPI_Transmit(baroSPI, &(baroHandle->pressureAccuracy), 1, HAL_MAX_DELAY);
    HAL_Delay(baroHandle->convertTime);

    uint8_t digitalPressBuff[4];
    HAL_SPI_TransmitReceive(baroSPI, readADCCommand, digitalPressBuff, 4, HAL_MAX_DELAY);

    uint32_t digitalPress = (digitalPressBuff[1] << 16) | (digitalPressBuff[2] << 8) | (digitalPressBuff[3] << 0);

    int32_t dT = digitalTemp - (coefficients.C5 << 8);
    int32_t firstTemp = 2000 + dT * (coefficients.C6 >> 23);

    int64_t offset = (coefficients.C1 << 16) + ((coefficients.C4 * dT) >> 7);
    int64_t sensitivity = (coefficients.C1 << 15) + ((coefficients.C3 * dT) >> 8);

    if (firstTemp >= 20) {
        int32_t firstPress = (digitalPress * (sensitivity >> 21) - offset) >> 15;
        baroHandle->temperature = firstTemp;
        baroHandle->pressure = firstPress;
        HAL_GPIO_WritePin(GPIOC, BAR_NCS_Pin, GPIO_PIN_SET);
        return;
    }

    int32_t T2 = ((dT * dT) >> 31);
    int64_t OFF2 = 5 * ((firstTemp - 2000) * (firstTemp - 2000)) / 2;
    int64_t SENS2 = 5 * ((firstTemp - 2000) * (firstTemp - 2000)) / 4;

    if (firstTemp < -15) {
        OFF2 = OFF2 + 7 * ((firstTemp + 1500) * (firstTemp + 1500));
        SENS2 = SENS2 + 11 * ((firstTemp + 1500) * (firstTemp + 1500)) / 2;
    }

    offset = offset - OFF2;
    sensitivity = sensitivity - SENS2;

    int32_t secondPress = (digitalPress * (sensitivity >> 21) - offset) >> 15;
    int32_t secondTemp = firstTemp - T2;

    baroHandle->temperature = secondTemp;
    baroHandle->pressure = secondPress;
    HAL_GPIO_WritePin(GPIOC, BAR_NCS_Pin, GPIO_PIN_SET);
    return;
}