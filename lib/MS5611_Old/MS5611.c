#include "MS5611.h"

void resetBarometer(SPI_HandleTypeDef* baroSPI, uint16_t baro_ncs) {
    uint8_t data = BARO_RESET;
	HAL_GPIO_WritePin(GPIOC, baro_ncs, GPIO_PIN_RESET);
    HAL_SPI_Transmit(baroSPI, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOC, baro_ncs, GPIO_PIN_SET);
}

void initBarometer(SPI_HandleTypeDef *baroSPI, BARO_HANDLE *baroHandle, uint16_t baro_ncs) {
	resetBarometer(baroSPI, baro_ncs);

	HAL_Delay(10);
    uint8_t rxBuffer[2] = {0, 0};
    volatile uint8_t PROM_READ_Actual = PROM_READ;

    for (int i = 1; i < 7; i++) {
        PROM_READ_Actual += 2;
    	HAL_GPIO_WritePin(GPIOC, baro_ncs, GPIO_PIN_RESET);
        HAL_SPI_Transmit(baroSPI, &PROM_READ_Actual, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(baroSPI, rxBuffer, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOC, baro_ncs, GPIO_PIN_SET);

        baroHandle->C[i] = (float) (rxBuffer[1] << 8 | rxBuffer[0]);

        HAL_Delay(10);
    }

    baroHandle->C[1] *= (float) (1 << 16);
    baroHandle->C[2] *= (float) (1 << 17);
    baroHandle->C[3] /= (float) (1 << 7);
    baroHandle->C[4] /= (float) (1 << 6);
    baroHandle->C[5] *= (float) (1 << 8);
    baroHandle->C[6] /= (float) (1 << 23);

    return;
}

void getTempPress(SPI_HandleTypeDef *baroSPI, BARO_HANDLE *baroHandle, uint16_t baro_ncs) {
	uint8_t rxBuffer[3] = {0, 0, 0};
	float tempMeas = 0;
	float pressMeas = 0;
	float D1 = 0;
	float D2 = 0;

	tempConv(baroSPI, baroHandle, baro_ncs);

	HAL_GPIO_WritePin(GPIOC, baro_ncs, GPIO_PIN_RESET);
    HAL_SPI_Transmit(baroSPI, READ_ADC, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(baroSPI, rxBuffer, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOC, baro_ncs, GPIO_PIN_SET);

    D2 = (float) (rxBuffer[2] << 16 | rxBuffer[1] << 8 | rxBuffer[0]);
    tempMeas = 2000 + (D2 - baroHandle->C[5]) * baroHandle->C[6];

    baroHandle->temperature = tempMeas;
    return;
}

void tempConv(SPI_HandleTypeDef *baroSPI, BARO_HANDLE *baroHandle, uint16_t baro_ncs) {
    uint8_t data = HIGHEST_D2;
	HAL_GPIO_WritePin(GPIOC, baro_ncs, GPIO_PIN_RESET);
    HAL_SPI_Transmit(baroSPI, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOC, baro_ncs, GPIO_PIN_SET);

    HAL_Delay(10);
    return;
}

void pressConv(SPI_HandleTypeDef *baroSPI, BARO_HANDLE *baroHandle, uint16_t baro_ncs) {
    uint8_t data = HIGHEST_D1;
	HAL_GPIO_WritePin(GPIOC, baro_ncs, GPIO_PIN_RESET);
    HAL_SPI_Transmit(baroSPI, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOC, baro_ncs, GPIO_PIN_SET);

    HAL_Delay(10);
    return;
}
