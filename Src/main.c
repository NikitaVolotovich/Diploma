/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include  <string.h>
#include  <stdlib.h>
#include  <stdint.h>
#include  <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CCS811_ADDR 0xB4 //7-bit unshifted default I2C Address
#define CSS811_STATUS 0x00
#define CSS811_MEAS_MODE 0x01
#define CSS811_ALG_RESULT_DATA 0x02
#define CSS811_RAW_DATA 0x03
#define CSS811_ENV_DATA 0x05
#define CSS811_NTC 0x06
#define CSS811_THRESHOLDS 0x10
#define CSS811_BASELINE 0x11
#define CSS811_HW_ID 0x20
#define CSS811_HW_VERSION 0x21
#define CSS811_FW_BOOT_VERSION 0x23
#define CSS811_FW_APP_VERSION 0x24
#define CSS811_ERROR_ID 0xE0
#define CSS811_APP_START 0xF4
#define CSS811_SW_RESET 0xFF

#define BME280_ADDRESS  0xEC//BME280 I2C ADDRES (0x76<<1)0xEC
#define BME280_REG_ID 0xD0 //BME280 ID REGISTER
#define BME280_ID 0x60 //BME280 I2C ID
#define BME280_REG_SOFTRESET 0xE0 //BME280 SOFT RESET REGISTER
#define BME280_SOFTRESET_VALUE 0xB6 //BME280 SOFT RESET VALUE
#define BME280_REGISTER_STATUS 0XF3 //BME280 STATUS REGISTER
#define BME280_STATUS_MEASURING 0X08 //Running conversion
#define BME280_STATUS_IM_UPDATE 0X01 //NVM data copying
#define BME280_REG_CONFIG 0xF5 // Configuration register
#define BME280_REG_CTRL_HUM 0xF2 // Humidity measure control register
#define BME280_REG_CTRL_MEAS 0xF4 // Control register pressure and temperature measure
#define BME280_REGISTER_DIG_T1 0x88
#define BME280_REGISTER_DIG_T2 0x8A
#define BME280_REGISTER_DIG_T3 0x8C
#define BME280_REGISTER_DIG_P1 0x8E
#define BME280_REGISTER_DIG_P2 0x90
#define BME280_REGISTER_DIG_P3 0x92
#define BME280_REGISTER_DIG_P4 0x94
#define BME280_REGISTER_DIG_P5 0x96
#define BME280_REGISTER_DIG_P6 0x98
#define BME280_REGISTER_DIG_P7 0x9A
#define BME280_REGISTER_DIG_P8 0x9C
#define BME280_REGISTER_DIG_P9 0x9E
#define BME280_REGISTER_DIG_H1 0xA1
#define BME280_REGISTER_DIG_H2 0xE1
#define BME280_REGISTER_DIG_H3 0xE3
#define BME280_REGISTER_DIG_H4 0xE4
#define BME280_REGISTER_DIG_H5 0xE5
#define BME280_REGISTER_DIG_H6 0xE7
#define BME280_STBY_MSK 0xE0
#define BME280_STBY_0_5 0x00
#define BME280_STBY_62_5 0x20
#define BME280_STBY_125 0x40
#define BME280_STBY_250 0x60
#define BME280_STBY_500 0x80
#define BME280_STBY_1000 0xA0
#define BME280_STBY_10 0xC0
#define BME280_STBY_20 0xE0
#define BME280_FILTER_MSK 0x1C
#define BME280_FILTER_OFF 0x00
#define BME280_FILTER_2 0x04
#define BME280_FILTER_4 0x08
#define BME280_FILTER_8 0x0C
#define BME280_FILTER_16 0x10
#define BME280_OSRS_T_MSK 0xE0
#define BME280_OSRS_T_SKIP 0x00
#define BME280_OSRS_T_x1 0x20
#define BME280_OSRS_T_x2 0x40
#define BME280_OSRS_T_x4 0x60
#define BME280_OSRS_T_x8 0x80
#define BME280_OSRS_T_x16 0xA0
#define BME280_OSRS_P_MSK 0x1C
#define BME280_OSRS_P_SKIP 0x00
#define BME280_OSRS_P_x1 0x04
#define BME280_OSRS_P_x2 0x08
#define BME280_OSRS_P_x4 0x0C
#define BME280_OSRS_P_x8 0x10
#define BME280_OSRS_P_x16 0x14
#define BME280_OSRS_H_MSK 0x07
#define BME280_OSRS_H_SKIP 0x00
#define BME280_OSRS_H_x1 0x01
#define BME280_OSRS_H_x2 0x02
#define BME280_OSRS_H_x4 0x03
#define BME280_OSRS_H_x8 0x04
#define BME280_OSRS_H_x16 0x05
#define BME280_MODE_MSK 0x03
#define BME280_MODE_SLEEP 0x00
#define BME280_MODE_FORCED 0x01
#define BME280_MODE_NORMAL 0x03
#define BME280_REGISTER_PRESSUREDATA 0xF7
#define BME280_REGISTER_TEMPDATA 0xFA
#define BME280_REGISTER_HUMIDDATA 0xFD
#define be16toword(a) ((((a)>>8)&0xff)|(((a)<<8)&0xff00))
#define be24toword(a) ((((a)>>16)&0x000000ff)|((a)&0x0000ff00)|(((a)<<16)&0x00ff0000))
#define SEALEVELPRESSURE_HPA (1013.25)
#define SEALEVELPRESSURE_PA (1013250)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t dt;
char buffer[16];
uint32_t restore_Baselines;

uint8_t nr[] = "\n\r";
uint8_t str_sensor1[] = "\nSensor 1: "; //11
uint8_t str_sensor2[] = "\nSensor 2: "; //11
uint8_t str_counter[] = "\nCounter: "; //10
uint8_t str_sum[] = "\nSum: "; //6
uint8_t str_resistance[] = "\nResistance: "; //13
uint8_t str_baseline[] = "\nBaseline: "; //11
uint8_t str_CO2[] = "\nCO2: "; //6
uint8_t str_tVOC[] = "\nVOC: "; //6
uint8_t str_new[] = "\n=== new_While ==="; //18
uint8_t str_sensor_status[] = "\n\rCCS811 status is: "; //20
uint8_t str_hw_id[] = "\n\rCCS811 hw_id is: "; //19
uint8_t str_hw_version[] = "\n\rCCS811 version is: "; //21
uint8_t str_boot_version[] = "\n\rCCS811 boot version is: "; //26
uint8_t str_app_version[] = "\n\rCCS811 app version is: "; //25
uint8_t str_sensor_mode[] = "\n\rCCS811 mode is: "; //18
uint8_t str_excel_tabulation[] = "\t";

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} BME280_CalibData;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void ConfigureCCS811(void);

void ReadAlgorithmResults(void);

void SetDriveMode(uint8_t mode);

uint8_t ReadRegister(uint8_t addr);

void WriteRegister(uint8_t addr, uint8_t val);

void CheckSensorInfo(void);

void SensorReset(void);

uint32_t GetSensorResistance(void);

unsigned int GetBaseline(void);

void RestoreBaseline(void);

//BME280
void Error(void);

void BME280_Init(void);

static uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg);

static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value);

static void I2Cx_ReadData16(uint16_t Addr, uint8_t Reg, uint16_t *Value);

static void I2Cx_ReadData24(uint16_t Addr, uint8_t Reg, uint32_t *Value);

void BME280_WriteReg(uint8_t Reg, uint8_t Value);

uint8_t BME280_ReadReg(uint8_t Reg);

void BME280_ReadReg_U16(uint8_t Reg, uint16_t *Value);

void BME280_ReadReg_S16(uint8_t Reg, int16_t *Value);

void BME280_ReadCoefficients(void);

void BME280_SetStandby(uint8_t tsb);

void BME280_SetFilter(uint8_t filter);

void BME280_SetOversamplingTemper(uint8_t osrs);

void BME280_SetOversamplingPressure(uint8_t osrs);

void BME280_SetOversamplingHum(uint8_t osrs);

void BME280_SetMode(uint8_t mode);

float BME280_ReadTemperature(void);

float BME280_ReadPressure(void);

float BME280_ReadHumidity(void);

float BME280_ReadAltitude(float seaLevel);

void BME280_ReadReg_U24(uint8_t Reg, uint32_t *Value);

void BME280_ReadReg_BE_U24(uint8_t Reg, uint32_t *Value);

void BME280_ReadReg_BE_S16(uint8_t Reg, int16_t *Value);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_I2C1_Init(void);

static void MX_I2C2_Init(void);

static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned int tVOC = 0;
unsigned int CO2 = 0;

unsigned int tVOC_2 = 0;
unsigned int CO2_2 = 0;

unsigned int CO2_ResultValue = 0;
unsigned int tVOC_ResultValue = 0;

uint16_t adc_raw = 0;
uint8_t current_value = 0;
uint8_t dummyread = 0;
uint8_t appvalue = 0;
uint8_t errvalue = 0;
uint8_t mosetting = 0;
uint8_t dtvalue = 0;
uint8_t appStart = 0;
uint32_t ELBaseline_period = 0;
uint32_t ALBaseline_period = 0;
uint8_t Mode_CCS811 = 1;

//bmp280
char str1[100];
uint8_t value = 0;
uint32_t value32 = 0;
float tf = 0.0f, pf = 0.0f, af = 0.0f, hf = 0.0f;
BME280_CalibData CalibData;
int32_t temper_int;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    ConfigureCCS811();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    RestoreBaseline();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    CheckSensorInfo();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

    BME280_Init();
    //value = BME280_ReadReg(BME280_REG_ID);

    int counter = 0;
    while (1) {
        //HAL_UART_Transmit(&huart1, str_new, 18, 100);
        HAL_UART_Transmit(&huart2, nr, 1, 100);

        ReadAlgorithmResults();

        HAL_UART_Transmit(&huart2, str_counter, 10, 100);
        HAL_UART_Transmit(&huart2, (uint8_t *) buffer, sprintf(buffer, "%d", counter), 100);
        HAL_UART_Transmit(&huart2, str_CO2, 6, 100);
        HAL_UART_Transmit(&huart2, (uint8_t *) buffer, sprintf(buffer, "%d", CO2), 100);
        HAL_UART_Transmit(&huart2, str_tVOC, 7, 100);
        HAL_UART_Transmit(&huart2, (uint8_t *) buffer, sprintf(buffer, "%d", tVOC), 100);

        tf = BME280_ReadTemperature();
        sprintf(str1, "\nTemperature: %.3f *C", tf);
        HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);

        pf = BME280_ReadPressure();
        sprintf(str1, "\nPressure: %.3f Pa; %.3f hPa; %.3f mmHg", pf, pf / 1000.0f, pf * 0.000750061683f);
        HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);

        af = BME280_ReadAltitude(SEALEVELPRESSURE_PA);
        sprintf(str1, "\nAltitude: %.3f m", af);
        HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);

        hf = BME280_ReadHumidity();
        sprintf(str1, "\nHumidity: %.3f %%", hf);
        HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);

        //HAL_UART_Transmit(&huart2, str_excel_tabulation, 1, 100);
        //HAL_UART_Transmit(&huart2, (uint8_t*) buffer, sprintf(buffer, "%d", CO2_2), 100);
        //HAL_UART_Transmit(&huart2, str_excel_tabulation, 1, 100);
        //HAL_UART_Transmit(&huart2, (uint8_t*) buffer, sprintf(buffer, "%d", tVOC_2), 100);


        HAL_Delay(1000);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

        counter++;

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void) {

    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void ConfigureCCS811() {
    HAL_Delay(69);
    //Verify the hardware ID is what we expect
    uint8_t hwID = ReadRegister(0x20); //Hardware ID should be 0x81
    if (hwID != 0x81) { //Serial.println("CCS811 not found. Please check wiring.");
        while (1); //Freeze
    }
    uint8_t lodata[1];
    lodata[0] = CSS811_APP_START;
    HAL_I2C_Master_Transmit(&hi2c1, CCS811_ADDR, lodata, 1, 100);
    HAL_Delay(20);
    SetDriveMode(Mode_CCS811); //Read every second
    HAL_Delay(10);
}

void ReadAlgorithmResults() {
    uint8_t data_rq[4];
    uint8_t status = HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDR, (uint8_t) CSS811_ALG_RESULT_DATA, I2C_MEMADD_SIZE_8BIT,
                                      data_rq, 4, 100);
    uint8_t co2MSB = data_rq[0];
    uint8_t co2LSB = data_rq[1];
    uint8_t tvocMSB = data_rq[2];
    uint8_t tvocLSB = data_rq[3];
/*  TVOC value, in parts per billion (ppb)
		eC02 value, in parts per million (ppm)*/
    CO2 = ((unsigned int) co2MSB << 8) | co2LSB;
    tVOC = ((unsigned int) tvocMSB << 8) | tvocLSB;
}

void SetDriveMode(uint8_t mode) {
    if (mode > 4) mode = 4; //Error correction
    mosetting = ReadRegister(CSS811_MEAS_MODE); //Read what's currently there
    mosetting &= ~(7 << 4); //Clear DRIVE_MODE bits
    mosetting |= (mode << 4); //Mask in mode
    WriteRegister(CSS811_MEAS_MODE, mosetting);
    mosetting = ReadRegister(CSS811_MEAS_MODE); //Read what's currently there
}

void WriteRegister(uint8_t addr, uint8_t val) {
    HAL_I2C_Mem_Write(&hi2c1, CCS811_ADDR, (uint8_t) addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 300);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
    while (HAL_I2C_IsDeviceReady(&hi2c1, CCS811_ADDR, 10, 300) == HAL_TIMEOUT);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
}

uint8_t ReadRegister(uint8_t addr) {
    uint8_t dt;
    HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDR, (uint8_t) addr, 1, &dt, 1, 300);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
    return dt;
}

void CheckSensorInfo(void) {
    uint16_t temp = 0;
    temp = ReadRegister(0x00);
    HAL_UART_Transmit(&huart2, str_sensor_status, 20, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *) buffer, sprintf(buffer, "%d", temp), 100);

    temp = ReadRegister(0x20);
    HAL_UART_Transmit(&huart2, str_hw_id, 19, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *) buffer, sprintf(buffer, "%d", temp), 100);

    temp = ReadRegister(0x21);
    HAL_UART_Transmit(&huart2, str_hw_version, 21, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *) buffer, sprintf(buffer, "%d", temp), 100);

    temp = ReadRegister(0x23);
    HAL_UART_Transmit(&huart2, str_boot_version, 26, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *) buffer, sprintf(buffer, "%d", temp), 100);

    temp = ReadRegister(0x24);
    HAL_UART_Transmit(&huart2, str_app_version, 25, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *) buffer, sprintf(buffer, "%d", temp), 100);

    temp = ReadRegister(0x01);
    HAL_UART_Transmit(&huart2, str_sensor_mode, 18, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *) buffer, sprintf(buffer, "%d", temp), 100);
    HAL_UART_Transmit(&huart2, nr, 1, 100);
}

void SensorReset(void) {
    uint8_t rstCMD[5] = {CSS811_SW_RESET, 0x11, 0xE5, 0x72, 0x8A};
    HAL_I2C_Mem_Write(&hi2c1, CCS811_ADDR, CSS811_SW_RESET, I2C_MEMADD_SIZE_8BIT, rstCMD, 5, 300);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
}

/*
  * @brief  	 //restore the baseline value
	//Used for telling sensor what 'clean' air is
	//You must put the sensor in clean air and record this value
  * @param  NONE.
  * @retval NONE.
*/
void RestoreBaseline() {
    uint32_t restore_Baseline = 30000;
    //restore_Baseline= * ((  uint32_t *)DATA_EEPROM_BASE);
    restore_Baselines = restore_Baseline;
    uint8_t res_bs[2];
    res_bs[0] = restore_Baseline >> 8;
    res_bs[1] = restore_Baseline & 0x000000FF;
    HAL_I2C_Mem_Write(&hi2c1, CCS811_ADDR, CSS811_BASELINE, I2C_MEMADD_SIZE_8BIT, res_bs, 2, 300);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
}

/*
  * @brief  	//Returns the baseline value and saves into EEPROM
	//Used for telling sensor what 'clean' air is
	//You must put the sensor in clean air and record this value
  * @param  NONE.
  * @retval BASELINE VALUE.
 */
unsigned int GetBaseline() {
    uint8_t ada[2];
    HAL_StatusTypeDef status = HAL_OK;
    if (status) {}
    status = HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDR, (uint8_t) CSS811_BASELINE, 1, ada, 2, 100);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
    while (HAL_I2C_IsDeviceReady(&hi2c1, CCS811_ADDR, 10, 300) == HAL_TIMEOUT);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

    uint8_t baselineMSB = ada[0];
    uint8_t baselineLSB = ada[1];

    unsigned int baseline = ((unsigned int) baselineMSB << 8) | baselineLSB;
    //HAL_FLASHEx_DATAEEPROM_Unlock();
    //HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, DATA_EEPROM_BASE, baseline);
    return (baseline);
}

uint32_t GetSensorResistance() {
    uint8_t sensor_Resistance_raw[2];
    uint32_t sensor_Resistance;
    HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDR, CSS811_RAW_DATA, I2C_MEMADD_SIZE_8BIT, sensor_Resistance_raw, 2, 100);
    current_value = sensor_Resistance_raw[0] >> 2;
    sensor_Resistance_raw[0] = sensor_Resistance_raw[0] & 0x03;
    adc_raw = (sensor_Resistance_raw[0] << 8) | sensor_Resistance_raw[1];
    sensor_Resistance = ((165 * adc_raw) * 10000) / (current_value * 1023);
    return sensor_Resistance;
}


/* BME280 */
void Error(void) {
    while (1);
}

uint8_t BME280_ReadStatus(void) {
    //clear unuset bits
    uint8_t res = BME280_ReadReg(BME280_REGISTER_STATUS) & 0x09;
    return res;
}

void BME280_Init(void) {
    value = BME280_ReadReg(BME280_REG_ID);
    HAL_UART_Transmit(&huart2, nr, 1, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *) buffer, sprintf(buffer, "%d", value), 100);
    HAL_UART_Transmit(&huart2, nr, 1, 100);
    if (value != BME280_ID) {
        Error();
    }
    BME280_WriteReg(BME280_REG_SOFTRESET, BME280_SOFTRESET_VALUE);
    while (BME280_ReadStatus() & BME280_STATUS_IM_UPDATE);
    BME280_ReadCoefficients();
    BME280_SetStandby(BME280_STBY_1000);
    BME280_SetFilter(BME280_FILTER_4);
    BME280_SetOversamplingTemper(BME280_OSRS_T_x4);
    BME280_SetOversamplingPressure(BME280_OSRS_P_x1);
    BME280_SetOversamplingHum(BME280_OSRS_H_x1);
    value32 = BME280_ReadReg(BME280_REG_CTRL_MEAS);
    value32 |= BME280_ReadReg(BME280_REG_CTRL_HUM) << 8;
    sprintf(str1, "\nMeasurements status: %04X", value32);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    sprintf(str1, "\nTemperature: %s\nPressure: %s\nHumidity: %s",
            (value32 & BME280_OSRS_T_MSK) ? "ON" : "OFF",
            (value32 & BME280_OSRS_P_MSK) ? "ON" : "OFF",
            ((value32 >> 8) & BME280_OSRS_H_MSK) ? "ON" : "OFF");
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_SetMode(BME280_MODE_NORMAL);
}

static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value) {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Write(&hi2c2, Addr, (uint16_t) Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);
    if (status != HAL_OK) Error();
}

//------------------------------------------------

static uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t value = 0;
    status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
    if (status != HAL_OK) Error();
    return value;
}

static void I2Cx_ReadData16(uint16_t Addr, uint8_t Reg, uint16_t *Value) {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *) Value, 2, 0x10000);
    if (status != HAL_OK) Error();
}

//------------------------------------------------

static void I2Cx_ReadData24(uint16_t Addr, uint8_t Reg, uint32_t *Value) {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *) Value, 3, 0x10000);
    if (status != HAL_OK) Error();
}

void BME280_WriteReg(uint8_t Reg, uint8_t Value) {
    I2Cx_WriteData(BME280_ADDRESS, Reg, Value);
}

uint8_t BME280_ReadReg(uint8_t Reg) {
    uint8_t res = I2Cx_ReadData(BME280_ADDRESS, Reg);
    return res;
}

void BME280_ReadReg_U16(uint8_t Reg, uint16_t *Value) {
    I2Cx_ReadData16(BME280_ADDRESS, Reg, Value);
}

void BME280_ReadReg_S16(uint8_t Reg, int16_t *Value) {
    I2Cx_ReadData16(BME280_ADDRESS, Reg, (uint16_t *) Value);
}

void BME280_ReadCoefficients(void) {
    BME280_ReadReg_U16(BME280_REGISTER_DIG_T1, &CalibData.dig_T1);
    sprintf(str1, "DIG_T1: %urn", CalibData.dig_T1);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_S16(BME280_REGISTER_DIG_T2, &CalibData.dig_T2);
    sprintf(str1, "DIG_T2: %drn", CalibData.dig_T2);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_S16(BME280_REGISTER_DIG_T3, &CalibData.dig_T3);
    sprintf(str1, "DIG_T3: %drn", CalibData.dig_T3);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_U16(BME280_REGISTER_DIG_P1, &CalibData.dig_P1);
    sprintf(str1, "DIG_P1: %urn", CalibData.dig_P1);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_S16(BME280_REGISTER_DIG_P2, &CalibData.dig_P2);
    sprintf(str1, "DIG_P2: %drn", CalibData.dig_P2);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_S16(BME280_REGISTER_DIG_P3, &CalibData.dig_P3);
    sprintf(str1, "DIG_P3: %drn", CalibData.dig_P3);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_S16(BME280_REGISTER_DIG_P4, &CalibData.dig_P4);
    sprintf(str1, "DIG_P4: %drn", CalibData.dig_P4);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_S16(BME280_REGISTER_DIG_P5, &CalibData.dig_P5);
    sprintf(str1, "DIG_P5: %drn", CalibData.dig_P5);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_S16(BME280_REGISTER_DIG_P6, &CalibData.dig_P6);
    sprintf(str1, "DIG_P6: %drn", CalibData.dig_P6);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_S16(BME280_REGISTER_DIG_P7, &CalibData.dig_P7);
    sprintf(str1, "DIG_P7: %drn", CalibData.dig_P7);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_S16(BME280_REGISTER_DIG_P8, &CalibData.dig_P8);
    sprintf(str1, "DIG_P8: %drn", CalibData.dig_P8);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_S16(BME280_REGISTER_DIG_P9, &CalibData.dig_P9);
    sprintf(str1, "DIG_P9: %drn", CalibData.dig_P9);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    CalibData.dig_H1 = BME280_ReadReg(BME280_REGISTER_DIG_H1);
    sprintf(str1, "DIG_H1: %drn", CalibData.dig_H1);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    BME280_ReadReg_S16(BME280_REGISTER_DIG_H2, &CalibData.dig_H2);
    sprintf(str1, "DIG_H2: %drn", CalibData.dig_H2);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    CalibData.dig_H3 = BME280_ReadReg(BME280_REGISTER_DIG_H3);
    sprintf(str1, "DIG_H3: %drn", CalibData.dig_H3);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    CalibData.dig_H4 =
            (BME280_ReadReg(BME280_REGISTER_DIG_H4) << 4) | (BME280_ReadReg(BME280_REGISTER_DIG_H4 + 1) & 0xF);
    sprintf(str1, "DIG_H4: %drn", CalibData.dig_H4);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    CalibData.dig_H5 =
            (BME280_ReadReg(BME280_REGISTER_DIG_H5 + 1) << 4) | (BME280_ReadReg(BME280_REGISTER_DIG_H5) >> 4);
    sprintf(str1, "DIG_H5: %drn", CalibData.dig_H5);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
    CalibData.dig_H6 = (int8_t) BME280_ReadReg(BME280_REGISTER_DIG_H6);
    sprintf(str1, "DIG_H6: %drn", CalibData.dig_H3);
    HAL_UART_Transmit(&huart2, (uint8_t *) str1, strlen(str1), 0x1000);
}

void BME280_SetStandby(uint8_t tsb) {
    uint8_t reg;
    reg = BME280_ReadReg(BME280_REG_CONFIG) & ~BME280_STBY_MSK;
    reg |= tsb & BME280_STBY_MSK;
    BME280_WriteReg(BME280_REG_CONFIG, reg);
}

void BME280_SetFilter(uint8_t filter) {
    uint8_t reg;
    reg = BME280_ReadReg(BME280_REG_CONFIG) & ~BME280_FILTER_MSK;
    reg |= filter & BME280_FILTER_MSK;
    BME280_WriteReg(BME280_REG_CONFIG, reg);
}

void BME280_SetOversamplingTemper(uint8_t osrs) {
    uint8_t reg;
    reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_T_MSK;
    reg |= osrs & BME280_OSRS_T_MSK;
    BME280_WriteReg(BME280_REG_CTRL_MEAS, reg);
}

//------------------------------------------------
void BME280_SetOversamplingPressure(uint8_t osrs) {
    uint8_t reg;
    reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_P_MSK;
    reg |= osrs & BME280_OSRS_P_MSK;
    BME280_WriteReg(BME280_REG_CTRL_MEAS, reg);
}

//------------------------------------------------
void BME280_SetOversamplingHum(uint8_t osrs) {
    uint8_t reg;
    reg = BME280_ReadReg(BME280_REG_CTRL_HUM) & ~BME280_OSRS_H_MSK;
    reg |= osrs & BME280_OSRS_H_MSK;
    BME280_WriteReg(BME280_REG_CTRL_HUM, reg);
    //The 'ctrl_hum' register needs to be written
    //after changing 'ctrl_hum' for the changes to become effwctive.
    reg = BME280_ReadReg(BME280_REG_CTRL_MEAS);
    BME280_WriteReg(BME280_REG_CTRL_MEAS, reg);
}

void BME280_SetMode(uint8_t mode) {
    uint8_t reg;
    reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;
    reg |= mode & BME280_MODE_MSK;
    BME280_WriteReg(BME280_REG_CTRL_MEAS, reg);
}

//------------------------------------------------
float BME280_ReadTemperature(void) {
    float temper_float = 0.0f;
    int temper_raw;
    int32_t val1, val2;
    BME280_ReadReg_BE_U24(BME280_REGISTER_TEMPDATA, &temper_raw);
    temper_raw >>= 4;
    //sprintf(str1, "Temperature RAW: 0x%08Xrn", temper_raw);
    //HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
    val1 = ((((temper_raw >> 3) - ((int32_t) CalibData.dig_T1 << 1))) *
            ((int32_t) CalibData.dig_T2)) >> 11;
    /*val2 = (((((temper_raw>>4) - ((int32_t)CalibData.dig_T1)) *
    ((temper_raw>>4) - ((int32_t)CalibData.dig_T1))) >> 12) *
    ((int32_t)CalibData.dig_T3)) >> 14;*/
    val2 = (int32_t)(((((temper_raw >> 4) - CalibData.dig_T1) *
                       ((temper_raw >> 4) - CalibData.dig_T1)) >> 12) * CalibData.dig_T3) >> 14;
    temper_int = val1 + val2;
    temper_float = ((temper_int * 5 + 128) >> 8);
    temper_float /= 100.0f;
    return temper_float;

}

//------------------------------------------------
float BME280_ReadPressure(void) {
    float press_float = 0.0f;
    uint32_t press_raw, pres_int;
    int64_t val1, val2, p;
    BME280_ReadTemperature(); // must be done first to get t_fine
    BME280_ReadReg_BE_U24(BME280_REGISTER_PRESSUREDATA, &press_raw);
    press_raw >>= 4;
    val1 = ((int64_t) temper_int) - 128000;
    val2 = val1 * val1 * (int64_t) CalibData.dig_P6;
    val2 = val2 + ((val1 * (int64_t) CalibData.dig_P5) << 17);
    val2 = val2 + ((int64_t) CalibData.dig_P4 << 35);
    val1 = ((val1 * val1 * (int64_t) CalibData.dig_P3) >> 8) + ((val1 * (int64_t) CalibData.dig_P2) << 12);
    val1 = (((((int64_t) 1) << 47) + val1)) * ((int64_t) CalibData.dig_P1) >> 33;
    if (val1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - press_raw;
    p = (((p << 31) - val2) * 3125) / val1;
    val1 = (((int64_t) CalibData.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    val2 = (((int64_t) CalibData.dig_P8) * p) >> 19;
    p = ((p + val1 + val2) >> 8) + ((int64_t) CalibData.dig_P7 << 4);
    pres_int = ((p >> 8) * 1000) + (((p & 0xff) * 390625) / 100000);
    press_float = pres_int / 100.0f;

    return press_float;
}

//------------------------------------------------
float BME280_ReadHumidity(void) {
    float hum_float = 0.0f;
    int16_t hum_raw;
    int32_t hum_raw_sign, v_x1_u32r;
    BME280_ReadTemperature(); // must be done first to get t_fine
    BME280_ReadReg_BE_S16(BME280_REGISTER_HUMIDDATA, &hum_raw);
    hum_raw_sign = ((int32_t) hum_raw) & 0x0000FFFF;
    //sprintf(str1, "Humidity RAW: 0x%08X\r\n", hum_raw_sign);
    //HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
    v_x1_u32r = (temper_int - ((int32_t) 76800));
    v_x1_u32r = (((((hum_raw_sign << 14) - (((int32_t) CalibData.dig_H4) << 20) -
                    (((int32_t) CalibData.dig_H5) * v_x1_u32r)) + ((int32_t) 16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t) CalibData.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t) CalibData.dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) +
                    ((int32_t) 2097152)) * ((int32_t) CalibData.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t) CalibData.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    hum_float = (v_x1_u32r >> 12);
    hum_float /= 1024.0f;
    return hum_float;
}

//------------------------------------------------
float BME280_ReadAltitude(float seaLevel) {
    float att = 0.0f;
    float atm = BME280_ReadPressure();
    att = 44330.0 * (1.0 - pow(atm / seaLevel, 0.1903));
    return att;
}

void BME280_ReadReg_U24(uint8_t Reg, uint32_t *Value) {
    I2Cx_ReadData24(BME280_ADDRESS, Reg, Value);
    *(uint32_t *) Value &= 0x00FFFFFF;
}

void BME280_ReadReg_BE_U24(uint8_t Reg, uint32_t *Value) {
    I2Cx_ReadData24(BME280_ADDRESS, Reg, Value);
    *(uint32_t *) Value = be24toword(*(uint32_t *) Value) & 0x00FFFFFF;
}

void BME280_ReadReg_BE_S16(uint8_t Reg, int16_t *Value) {
    I2Cx_ReadData16(BME280_ADDRESS, Reg, (uint16_t *) Value);
    *(uint16_t *) Value = be16toword(*(uint16_t *) Value);
}
//------------------------------------------------
//------------------------------------------------


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
