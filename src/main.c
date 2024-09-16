/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

typedef enum {
    LOWEST_TIME = 1,
    LOW_TIME = 2,
    MED_TIME = 3,
    HIGH_TIME = 5,
    HIGHEST_TIME = 9,
} baro_conversion_time_t;

typedef struct PROM_DATA {
      uint16_t C1; // Pressure sensitivity
      uint16_t C2; // Pressure offset
      uint16_t C3; // Temperature coefficient of pressure sensitivity
      uint16_t C4; // Temperature coefficient of pressure offset
      uint16_t C5; // Reference temperature
      uint16_t C6; // Temperature coefficient of temperature
} prom_data_t;

// Make sure you use the correct percision value for each

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

uint8_t* readMagMultipleRegisters(SPI_HandleTypeDef* magSPIHandler, mag_reg_t regAddress, uint8_t numRegisters) {

	regAddress = generateMagAddress(regAddress, true, true);

	short SPIResult;
	uint8_t regValue[numRegisters];

	SPIResult = HAL_SPI_Transmit(magSPIHandler, &regAddress, 1, HAL_MAX_DELAY);
	SPIResult = HAL_SPI_Receive(magSPIHandler, regValue, numRegisters, HAL_MAX_DELAY);

	return regValue;
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

int isNthBitSet(int num, int n) {

	if (num & (1 << n)) {
		return 1;
	} else {
		return 0;
	}
}


uint8_t ensureMagNotReserved(mag_reg_t regToCheck) {

    if ((0x0B <= regToCheck && regToCheck <= 0x0E) ||
        (0x10 <= regToCheck && regToCheck <= 0x1F) ||
        (regToCheck == 0x25) ||
        (regToCheck == 0x26)) {

      printf("ERROR: YOU ARE WRITING TO A RESERVED REGISTER: %d", regToCheck);
      return 1;
    }

    return 0;
}


typedef enum {
    FUNC_CFG_ACCESS        = 0x01,
	PIN_CTRL 			   = 0x02,
    SENSOR_SYNC_TIME_FRAME = 0x04,
    SENSOR_SYNC_RES_RATIO  = 0x05,
    FIFO_CTRL1             = 0x06,
    FIFO_CTRL2             = 0x07,
    FIFO_CTRL3             = 0x08,
    FIFO_CTRL4             = 0x09,
    FIFO_CTRL5             = 0x0A,
    ORIENT_CFG_G           = 0x0B,
    INT1_CTRL              = 0x0D,
    INT2_CTRL              = 0x0E,
    WHO_AM_I_IMU              = 0x0F,
    CTRL1_XL               = 0x10,
    CTRL2_G                = 0x11,
    CTRL3_C                = 0x12,
    CTRL4_C                = 0x13,
    CTRL5_C                = 0x14,
    CTRL6_C                = 0x15,
    CTRL7_G                = 0x16,
    CTRL8_XL               = 0x17,
    CTRL9_XL               = 0x18,
    CTRL10_C               = 0x19,
    ALL_INT_SRC            = 0x1A,
    WAKE_UP_SRC            = 0x1B,
    TAP_SRC                = 0x1C,
    D6D_SRC                = 0x1D,
    STATUS_REG_IMU            = 0x1E,
    OUT_TEMP_L             = 0x20,
    OUT_TEMP_H             = 0x21,
    OUTX_L_G               = 0x22,
    OUTX_H_G               = 0x23,
    OUTY_L_G               = 0x24,
    OUTY_H_G               = 0x25,
    OUTZ_L_G               = 0x26,
    OUTZ_H_G               = 0x27,
    OUTX_L_A               = 0x28,
    OUTX_H_A               = 0x29,
    OUTY_L_A               = 0x2A,
    OUTY_H_A               = 0x2B,
    OUTZ_L_A               = 0x2C,
    OUTZ_H_A               = 0x2D,
    EMB_FUNC_STATUS_MAINPAGE = 0x35,
    FSM_STATUS_A_MAINPAGE  = 0x36,
    FSM_STATUS_B_MAINPAGE  = 0x37,
    STATUS_MASTER_MAINPAGE = 0x39,
    FIFO_STATUS1           = 0x3A,
    FIFO_STATUS2           = 0x3B,
    TIMESTAMP0_REG         = 0x40,
    TIMESTAMP1_REG         = 0x41,
    TIMESTAMP2_REG         = 0x42,
    STEP_TIMESTAMP_L       = 0x49,
    STEP_TIMESTAMP_H       = 0x4A,
    STEP_COUNTER_L         = 0x4B,
    STEP_COUNTER_H         = 0x4C,
    EMB_FUNC_FIFO_STATUS   = 0x4D,
    FSM_ENABLE_A           = 0x4E,
    FSM_ENABLE_B           = 0x4F,
    EMB_FUNC_INIT_A        = 0x50,
    EMB_FUNC_INIT_B        = 0x51,
    FSM_LONG_COUNTER_L     = 0x52,
    FSM_LONG_COUNTER_H     = 0x53,
    EMB_FUNC_SRC           = 0x56,
    FSM_OUTS1              = 0x58,
    FSM_OUTS2              = 0x59,
    FSM_OUTS3              = 0x5A,
    FSM_OUTS4              = 0x5B,
    FSM_OUTS5              = 0x5C,
    FSM_OUTS6              = 0x5D,
    FSM_OUTS7              = 0x5E,
    FSM_OUTS8              = 0x5F,
    FSM_OUTS9              = 0x60,
    FSM_OUTS10             = 0x61,
    FSM_OUTS11             = 0x62,
    FSM_OUTS12             = 0x63,
    FSM_OUTS13             = 0x64,
    FSM_OUTS14             = 0x65,
    FSM_OUTS15             = 0x66,
    FSM_OUTS16             = 0x67,
    EMB_FUNC_ODR_CFG_B     = 0x7F,
    I3C_BUS_AVB            = 0x62,
    INTERNAL_FREQ_FINE     = 0x63
} imu_reg_t;

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

uint8_t writeIMURegister(SPI_HandleTypeDef* imuSPI, mag_reg_t imuRegNum, uint8_t valueToWrite) {

	uint8_t actualRegNumber = generateIMUAddress(imuRegNum, false);
	uint8_t command[] = {actualRegNumber, valueToWrite};

	HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET);
	uint8_t SPIResult = HAL_SPI_Transmit(imuSPI, command, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET);

	return SPIResult;
}

uint8_t readIMUSingleRegister(SPI_HandleTypeDef* imuSPI, mag_reg_t imuRegNum) {

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

uint8_t initializeIMU(SPI_HandleTypeDef* hspi) {
	uint8_t status = writeIMURegister(hspi, PIN_CTRL, 0b01111111);
	return status;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  /*
  uint8_t WHO_AM_I_CMD = generateAddress(WHO_AM_I, true, false);
  uint8_t tx[2];
  uint8_t rx[2];
  tx[0] = WHO_AM_I_CMD;

  HAL_GPIO_WritePin(MAG_NCS_GPIO_Port, MAG_NCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(MAG_NCS_GPIO_Port, MAG_NCS_Pin, GPIO_PIN_SET);
  */

  /*
  BARO_HANDLE* barometer = {0};
  barometer->tempAccuracy = MED_D1;
  barometer->pressureAccuracy = MED_D2;
  barometer->convertTime = MED_TIME;
  resetBarometer(&hspi1);
  getPROMData(&hspi1, barometer);
  getCurrTempPressure(&hspi1, barometer);
  */

  uint8_t WHO_AM_I_VALUE = readMagSingleRegister(&hspi1, WHO_AM_I_MAG);
  uint8_t CTRL_REG = readMagSingleRegister(&hspi1, CTRL_REG3);
  int result = writeMagRegister(&hspi1, CTRL_REG3, 0b00000000);
  CTRL_REG = readMagSingleRegister(&hspi1, CTRL_REG3);
  uint8_t precision = readMagSingleRegister(&hspi1, CTRL_REG4);
  result = writeMagRegister(&hspi1, CTRL_REG4, 0b00001100);
  precision = readMagSingleRegister(&hspi1, CTRL_REG4);
  precision = writeMagRegister(&hspi1, CTRL_REG1, 0b11111100);
  precision = readMagSingleRegister(&hspi1, CTRL_REG1);

 /*
 volatile BARO_HANDLE* baroHandle;
  baroHandle->tempConv = HIGHEST_D1;
  baroHandle->pressConv = HIGHEST_D2;

  resetBarometer(&hspi1, BAR_NCS_Pin);
  initBarometer(&hspi1, baroHandle, BAR_NCS_Pin);
  */

while (1) {
  uint8_t pdata = 5;
  HAL_SPI_Transmit(&hspi1, &pdata, 1, HAL_MAX_DELAY);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 2570-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 100-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 2750-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 100-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BAR_NCS_Pin|IMU_NCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAG_NCS_GPIO_Port, MAG_NCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PER_NCS_GPIO_Port, PER_NCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D2_OUT_GPIO_Port, D2_OUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : BAR_NCS_Pin IMU_NCS_Pin */
  GPIO_InitStruct.Pin = BAR_NCS_Pin|IMU_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MAG_NCS_Pin */
  GPIO_InitStruct.Pin = MAG_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAG_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MAG_DRDY_Pin MAG_INT_Pin */
  GPIO_InitStruct.Pin = MAG_DRDY_Pin|MAG_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PER_NCS_Pin */
  GPIO_InitStruct.Pin = PER_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PER_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D2_OUT_Pin */
  GPIO_InitStruct.Pin = D2_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D2_OUT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
