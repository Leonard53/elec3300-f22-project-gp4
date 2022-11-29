/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
struct YPin {
    uint16_t voltageAtZeroWeight;
    uint16_t voltageAtSampledWeight;
    uint16_t weightAtSampledWeight;
    double weightCoef;
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
void Check_touchkey();

void clearAcceleRecord();

void clearAngleRecord();

void shiftAcceleRecord();

void shiftAngleRecord();

void insertAcceleRecord(uint16_t rawX, uint16_t rawY, uint16_t rawZ);

void insertAngleRecord(uint8_t newAngle);

void updateDistance(int16_t newX, int16_t newY, int16_t newZ);

int16_t combineUint_8ts(uint8_t a, uint8_t b);

short isRotating();

void getY(uint8_t index, uint8_t delay);

void initializeHMC5883L();

void drawBackToHome();

void initWeightSensor(struct YPin pin, uint16_t vAt0, uint16_t vAtS, uint16_t wAtS);

void initWeightSensors(struct YPin *pins);

uint16_t calculateWeight(double weightCoef, uint16_t weight);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
