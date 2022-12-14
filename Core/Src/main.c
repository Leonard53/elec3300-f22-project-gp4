/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
#include "lcdtp.h"
#include "xpt2046.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct accelerometerRecord {
    int16_t rawX, rawY, rawZ;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACCELE_FACTOR 0.0039
#define ACCELE_ACCEPTABLE_ERROR 12
#define RECORD_MAX_SIZE 100
#define ADXL_ADDR 0x1D
#define HMC5883L_ADDR 0x1E
#define N_SAMPLE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c2;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
enum Page {
    home, weight, accelerometer
};
enum Page currentPage = home;
int changingPage = 1;
int16_t initX_Acc_Reading = 0, initY_Acc_Reading = 0, initZ_Acc_Reading = 0;

struct accelerometerRecord accleRecord[RECORD_MAX_SIZE]; //storing the past data of the accelerometer for calculating distance
uint16_t angleRecord[RECORD_MAX_SIZE]; //storing the past data of the angle the user is facing for determining if the user is rotating while standing
short acceleRecordSize, angleRecordSize = 0; //storing the current index of accleRecord/angleRecord. Elemets should shift left if full
uint32_t distanceTraveled = 0; //storing the distance travelled in total. Reset when users return to home.
double actualDistance = 0.0;
struct YPin weightSensors[8];
struct YPinData weightSensorsData[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Check_touchkey() {
    strType_XPT2046_Coordinate strDisplayCoordinate;
    if (XPT2046_Get_TouchedPoint(&strDisplayCoordinate, &strXPT2046_TouchPara)) {
        if (currentPage == home) {
            if (strDisplayCoordinate.y > 210 && strDisplayCoordinate.y < 250) {
                if (strDisplayCoordinate.x > 20 && strDisplayCoordinate.x < 100) {
                    currentPage = weight;
                    changingPage = 1;
                } else if (strDisplayCoordinate.x > 130 && strDisplayCoordinate.x < 210) {
                    currentPage = accelerometer;
                    changingPage = 1;
                }
            }
        } else {
            if (strDisplayCoordinate.y > 267 && strDisplayCoordinate.y < 293) {
                if (strDisplayCoordinate.x > 20 && strDisplayCoordinate.x < 150) {
                    currentPage = home;
                    changingPage = 1;
                }
            }
        }
    }
}

void clearAcceleRecord() {
    for (short i = 0; i < RECORD_MAX_SIZE; ++i) {
        accleRecord[i].rawX = 0;
        accleRecord[i].rawY = 0;
        accleRecord[i].rawZ = 0;
    }
    acceleRecordSize = 0;
    distanceTraveled = 0;
    actualDistance = 0.0;
}

void clearAngleRecord() {
    for (short i = 0; i < RECORD_MAX_SIZE; ++i) {
        angleRecord[i] = 0;
    }
    angleRecordSize = 0;
}

void shiftAcceleRecord() {
    if (acceleRecordSize < RECORD_MAX_SIZE) {
        return;
    }
    for (short i = 1; i < RECORD_MAX_SIZE; ++i) {
        accleRecord[i - 1].rawX = accleRecord[i].rawX;
        accleRecord[i - 1].rawY = accleRecord[i].rawY;
        accleRecord[i - 1].rawZ = accleRecord[i].rawZ;
    }
    accleRecord[RECORD_MAX_SIZE - 1].rawX = 0;
    accleRecord[RECORD_MAX_SIZE - 1].rawY = 0;
    accleRecord[RECORD_MAX_SIZE - 1].rawZ = 0;
    --acceleRecordSize;
}

void shiftAngleRecord() {
    if (angleRecordSize < RECORD_MAX_SIZE) {
        return;
    }
    for (short i = 1; i < RECORD_MAX_SIZE; ++i) {
        angleRecord[i - 1] = angleRecord[i];
    }
    --angleRecordSize;
}

short isWithin(int16_t A, int16_t B, uint16_t range){
    return abs(A - B) <= range ? 1 : 0;
}

int16_t offset(const int16_t value, const int16_t initialReading){
    return isWithin(value, initialReading, ACCELE_ACCEPTABLE_ERROR) == 1 ? value : 0;
}

void insertAcceleRecord(int16_t rawX, int16_t rawY, int16_t rawZ) {
    while (acceleRecordSize >= RECORD_MAX_SIZE) {
        shiftAcceleRecord();
    }
//    char debug[10] = "";
//    sprintf(debug, "%03d", acceleRecordSize);
//    LCD_DrawString_Color(45, 10, debug, BLACK, WHITE);
//    if (N_SAMPLE > acceleRecordSize) {
//        // NOT ENOUGH SAMPLE, DISREGRAD THIS DATA
//        accleRecord[acceleRecordSize].rawX = 0;
//        accleRecord[acceleRecordSize].rawY = 0;
//        accleRecord[acceleRecordSize].rawZ = 0;
//        ++acceleRecordSize;
//        return;
//    }
    rawX = offset(rawX, initX_Acc_Reading);
    rawY = offset(rawY, initY_Acc_Reading);
    rawZ = offset(rawZ, initZ_Acc_Reading);
    int32_t accum_x = 0, accum_y = 0, accum_z = 0;
    for(uint16_t i = RECORD_MAX_SIZE - acceleRecordSize - 1; i < RECORD_MAX_SIZE; ++i){
        accum_x += accleRecord[i].rawX;
        accum_y += accleRecord[i].rawY;
        accum_z += accleRecord[i].rawZ;
    }
    const int16_t avg_x = (int16_t) floor(accum_x / N_SAMPLE);
    const int16_t avg_y = (int16_t) floor(accum_y / N_SAMPLE);
    const int16_t avg_z = (int16_t) floor(accum_z / N_SAMPLE);
    //constant unchanging acceleration,
    if (rawX != 0) rawX = isWithin(rawX, avg_x, ACCELE_ACCEPTABLE_ERROR) == 1 ? rawX : 0;
    if (rawY != 0) rawY = isWithin(rawY, avg_y, ACCELE_ACCEPTABLE_ERROR) == 1 ? rawY : 0;
    if (rawZ != 0) rawZ = isWithin(rawZ, avg_z, ACCELE_ACCEPTABLE_ERROR) == 1 ? rawZ : 0;
    accleRecord[acceleRecordSize].rawX = rawX;
    accleRecord[acceleRecordSize].rawY = rawY;
    accleRecord[acceleRecordSize].rawZ = rawZ;
    ++acceleRecordSize;
}

void insertAngleRecord(uint8_t newAngle) {
    while (angleRecordSize > RECORD_MAX_SIZE) {
        shiftAngleRecord();
    }
    angleRecord[angleRecordSize++] = newAngle;
}

/* This function takes in a newRecord and add to the distance traveled below */
void updateDistance(int16_t newX, int16_t newY, int16_t newZ) {
    int32_t accum_x = 0, accum_y = 0, accum_z = 0;
    for (short i = 0; i < acceleRecordSize; ++i) {
        double ratioDelay; // focus on the most recent samples, old data decaying overtime
        if ((double) i / acceleRecordSize < 0.25) ratioDelay = 0.05;
        else if ((double) i / acceleRecordSize < 0.50) ratioDelay = 0.15;
        else if ((double) i / acceleRecordSize < 0.75) ratioDelay = 0.3;
        else ratioDelay = 0.5;
        accum_x += (int16_t) floor(accleRecord[i].rawX * ratioDelay);
        accum_y += (int16_t) floor(accleRecord[i].rawY * ratioDelay);
        accum_z += (int16_t) floor(accleRecord[i].rawZ * ratioDelay);
    }
    const int16_t avg_x = accum_x / RECORD_MAX_SIZE, avg_y = accum_y / RECORD_MAX_SIZE, avg_z =
            accum_z / RECORD_MAX_SIZE;
    /* Ignore the axis if the most recent accelerometer is less than the average of recent samples
     * Otherwise, only calculate the difference between the new values and the averaged values*/
    (newX <= avg_x) ? (accum_x = 0) : (accum_x -= avg_x);
    (newY <= avg_y) ? (accum_y = 0) : (accum_y -= avg_y);
    (newZ <= avg_z) ? (accum_z = 0) : (accum_z -= avg_z);
    const double calDis = sqrt(pow(accum_x, 2) + pow(accum_y, 2) + pow(accum_z, 2));
    distanceTraveled += (uint16_t) floor(calDis);
    actualDistance += (calDis / 1000);
}

int16_t combineUint_8ts(uint8_t a, uint8_t b) {
    return ((0xFFFF & a) << 8) | (0xFFFF & b);
}

short checkIfInOrder(short isAscending, uint16_t A, uint16_t B) {
    if (isAscending == 1) {
        return A < B ? 1 : 0;
    } else {
        return A > B ? 1 : 0;
    }
}

/* The following function will determine if the accelerometer is rotating with the user
 * returns 1 if it is rotating, 0 otherwise*/
short isRotating() {
    if (angleRecordSize < 30) return 0; //too few sample, cannot determine
    uint16_t previousAngle = angleRecord[angleRecordSize - 30];
    short isAscending = angleRecord[angleRecordSize - 29] <= previousAngle ? 1 : 0;
    for (int i = angleRecordSize - 29; i < angleRecordSize; ++i) {
        if (checkIfInOrder(isAscending, previousAngle, angleRecord[i]) == 1
            || abs(angleRecord[i] - previousAngle) <= 5
            || (previousAngle >= 350 && previousAngle <= 360) && (angleRecord[i] >= 0 && angleRecord[i] <= 10)
                ) {
            // if angle continue to increase/decrease in trend,
            // the difference is within the range of error, or angle reset from ~360 to ~0
            previousAngle = angleRecord[i];
        } else return 0;
    }
    return 1;
}

void getY(uint8_t index, uint8_t delay) {
    if ((index & 0x01) == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    }

    if ((index & 0x02) == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    }

    if ((index & 0x04) == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
    }

    HAL_Delay(delay);
}

void initWeightSensor(struct YPin pin, uint16_t vAt0, uint16_t vAtS, uint16_t wAtS) {
    pin.voltageAtZeroWeight = vAt0;
    pin.voltageAtSampledWeight = vAtS;
    pin.weightAtSampledWeight = wAtS;
    pin.weightCoef = wAtS * 1.0 / (vAtS - vAt0);
}

void initAcceleData(){
    for(uint16_t i = 0; i < RECORD_MAX_SIZE; ++i){
        accleRecord[i].rawX = 0;
        accleRecord[i].rawY = 0;
        accleRecord[i].rawZ = 0;
    }
}

void initWeightSensors(struct YPin *pins) {
    pins[0].voltageAtZeroWeight = 1200;
    pins[0].voltageAtSampledWeight = 1500;
    pins[0].weightAtSampledWeight = 270;

    pins[1].voltageAtZeroWeight = 1300;
    pins[1].voltageAtSampledWeight = 1500;
    pins[1].weightAtSampledWeight = 270;

    pins[2].voltageAtZeroWeight = 1800;
    pins[2].voltageAtSampledWeight = 2600;
    pins[2].weightAtSampledWeight = 300;

    pins[3].voltageAtZeroWeight = 2800;
    pins[3].voltageAtSampledWeight = 3200;
    pins[3].weightAtSampledWeight = 300;

    pins[4].voltageAtZeroWeight = 1000;
    pins[4].voltageAtSampledWeight = 1800;
    pins[4].weightAtSampledWeight = 300;

    pins[5].voltageAtZeroWeight = 1800;
    pins[5].voltageAtSampledWeight = 2000;
    pins[5].weightAtSampledWeight = 300;

    pins[6].voltageAtZeroWeight = 1800;
    pins[6].voltageAtSampledWeight = 2100;
    pins[6].weightAtSampledWeight = 300;

    pins[7].voltageAtZeroWeight = 1000;
    pins[7].voltageAtSampledWeight = 21500;
    pins[7].weightAtSampledWeight = 300;

    for (uint8_t i = 0; i < 8; i++)
        pins[i].weightCoef = pins[i].weightAtSampledWeight * 1.0 / (pins[i].voltageAtSampledWeight - pins[i].voltageAtZeroWeight);
}

void initializeHMC5883L() {
    // 0x1E for developer access
    uint8_t initValue = 0x70;
    HAL_I2C_Mem_Write(&hi2c2, HMC5883L_ADDR << 1, 0x00, 1, &initValue, 1, 100);
    initValue = 0xA0;
    HAL_I2C_Mem_Write(&hi2c2, HMC5883L_ADDR << 1, 0x01, 1, &initValue, 1, 100);
}

void mainPage(void) {
    if (changingPage) {
        changingPage = 0;
        currentPage = home;
        LCD_Clear(0, 0, 240, 320, BLACK);
        char *output_text = "Welcome to the smart suitcase system";
        LCD_DrawString_Color_With_Delay(0, 40, output_text, BLUE, WHITE, 10);
        HAL_Delay(100);
        output_text = "Please choose an option below.";
        LCD_DrawString_Color(0, 100, output_text, BLACK, WHITE);
        HAL_Delay(200);
        LCD_Clear(20, 230, 80, 40, CYAN); //WEIGHT BOX: 20 ~ 100 / 210 ~ 250
        output_text = "WEIGHT";
        LCD_DrawString_Color_With_Delay(35, 242, output_text, CYAN, BLACK, 15);
        HAL_Delay(200);
        output_text = "ACCEL.";
        LCD_Clear(130, 230, 80, 40, YELLOW); //ACCELE BOX: 130 ~ 210 / 210 ~ 250
        LCD_DrawString_Color_With_Delay(150, 242, output_text, YELLOW, BLACK, 15);
    }
}

void drawBackToHome() {
    const char *output_text = "Return to Home";
    LCD_Clear(20, 280, 130, 25, CYAN); // RETURN HOME BOX: 20 ~ 150 / 267 ~ 293
    LCD_DrawString_Color_With_Delay(30, 285, output_text, CYAN, BLACK, 10);
    HAL_Delay(20);
}

void weightPage(void) {
    char *output_text;
    if (changingPage) {
        changingPage = 0;
        currentPage = weight;
        output_text = "W E I G H T D E T E C T I O N";
        LCD_Clear(0, 0, 240, 320, BLACK);
        LCD_DrawString_Color_With_Delay(10, 40, output_text, BLUE, WHITE, 10);
        drawBackToHome();

        for (uint8_t i = 0; i < 8; i++) {
            char temp[10] = "";
            sprintf(temp, "Y%d: ", i);
            LCD_DrawString_Color(110, 100 + 18 * i, temp, BACKGROUND, WHITE);
        }
    }

    uint16_t sum = 0;
    for (uint8_t counter = 0; counter < 8; counter++) {
        getY(counter, 20);

        unsigned int val = HAL_ADC_GetValue(&hadc2);
        int16_t weightPin = weightSensors[counter].weightCoef * (val - weightSensors[counter].voltageAtZeroWeight);
        if (weightPin < 0) weightPin = 0;
        sum += weightPin;

        if (counter < 4)
        	LCD_DrawHeatCircle(30, 240 - counter * 40, 30, 85 - val * 85 / 4096, 85);
        else
        	LCD_DrawHeatCircle(70, 120 + (counter - 4) * 40, 30, 85 - val * 85 / 4096, 85);

        //LCD_SetPinColor(&weightSensorsData[counter], 85 - val * 85 / 4096);

        char dec[20] = "";
        sprintf(dec, "%4d %5d", val, weightPin);
        LCD_DrawString_Color(140, 100 + 18 * counter, dec, BACKGROUND, WHITE);
    }
	//LCD_PrintHeatMap(weightSensorsData);

    char output_gram[20] = "";
    sprintf(output_gram, "%6d Gram", sum);
    LCD_DrawString_Color(120, 80, output_gram, BLUE, WHITE);

    HAL_Delay(20);
}

void accelerometerPage() {
    char *output_text;
    if (changingPage) {
        /* BEGIN CALIBRATING ACCELEROMETER */
        clearAcceleRecord();
        clearAngleRecord();
        initAcceleData();
        uint8_t setPWLMode = 0x00; //reset accelerometer
        HAL_I2C_Mem_Write(&hi2c2, ADXL_ADDR << 1, 0x2D, 1, &setPWLMode, 1, 100);
        HAL_Delay(100);
        setPWLMode = 0x08; //disable auto sleep, always measure
        HAL_I2C_Mem_Write(&hi2c2, ADXL_ADDR << 1, 0x2D, 1, &setPWLMode, 1, 100);
        uint8_t formatting = 0x05; // full res +-4g
        HAL_I2C_Mem_Write(&hi2c2, ADXL_ADDR << 1, 0x31, 1, &formatting, 1, 100);
        uint8_t debug = 0x00;
        HAL_I2C_Mem_Read(&hi2c2, ADXL_ADDR << 1, 0x00, 1, &debug, 1, 100);
        LCD_Clear(0, 0, 240, 320, BLACK);
        HAL_Delay(100);
        if (debug != 0xE5) {
            output_text = "SENSORS NOT PRESENT";
            LCD_DrawString_Color_With_Delay(50, 100, output_text, RED, WHITE, 5);
            output_text = "PLEASE RETRY AFTER CONNECTING THE ACCELEROMETER";
            LCD_DrawString_Color_With_Delay(0, 150, output_text, BLACK, WHITE, 3);
            HAL_Delay(3000);
            currentPage = home;
            changingPage = 1;
            return; //BACK TO HOME
        }
        output_text = "CALIBRATING IN 2s. DO NOT MOVE YOUR LUGGAGE.";
        LCD_DrawString_Color_With_Delay(0, 100, output_text, GREEN, BLACK, 10);
        HAL_Delay(2000);
        int32_t accumX = 0, accumY = 0, accumZ = 0;
        LCD_Clear(0, 0, 240, 320, BLACK);
        output_text = "CALIBRATING...";
        LCD_DrawString_Color(80, 100, output_text, RED, WHITE);
        for (short i = 0; i < 50; ++i) {
            uint8_t *arrayOfData = malloc(6 * sizeof(uint8_t));
            HAL_I2C_Mem_Read(&hi2c2, ADXL_ADDR << 1, 0x32, 1, arrayOfData, 6, 100);
            /* Take maximum reading only */
            accumX += combineUint_8ts(arrayOfData[1], arrayOfData[0]) >> 5;
            accumY += combineUint_8ts(arrayOfData[3], arrayOfData[2]) >> 5;
            accumZ += combineUint_8ts(arrayOfData[5], arrayOfData[4]) >> 5;
            free(arrayOfData);
            HAL_Delay(10);
        }
        initX_Acc_Reading = (int16_t) floor((double) accumX / 50);
        initY_Acc_Reading = (int16_t) floor((double) accumY / 50);
        initZ_Acc_Reading = (int16_t) floor((double) accumZ / 50);
        LCD_Clear(0, 0, 240, 320, BLACK);
        initializeHMC5883L();
        output_text = "CALIBRATING FINISHED. CONTINUING";
        LCD_DrawString_Color_With_Delay(0, 100, output_text, GREEN, BLACK, 10);
        char x_debug[20] = "", y_debug[20] = "", z_debug[20] = "";
        sprintf(x_debug, "%04d", initX_Acc_Reading);
        sprintf(y_debug, "%04d", initY_Acc_Reading);
        sprintf(z_debug, "%04d", initZ_Acc_Reading);
        strcat(x_debug, "  x");
        strcat(y_debug, "  y");
        strcat(z_debug, "  z");
        LCD_DrawString_Color(60, 140, x_debug, BLACK, WHITE);
        LCD_DrawString_Color(60, 160, y_debug, BLACK, WHITE);
        LCD_DrawString_Color(60, 180, z_debug, BLACK, WHITE);
        HAL_Delay(1500);
        /* COMPLETED CALIBRATING ACCELEROMETER */
        changingPage = 0;
        currentPage = accelerometer;
        output_text = "A C C E L E R O M E T E R.";
        LCD_Clear(0, 0, 240, 320, BLACK);
        LCD_DrawString_Color_With_Delay(10, 40, output_text, BLUE, WHITE, 10);
        drawBackToHome();
        output_text = "X: ";
        LCD_DrawString_Color(40, 100, output_text, CYAN, BLACK);
        output_text = "Y: ";
        LCD_DrawString_Color(40, 130, output_text, YELLOW, BLACK);
        output_text = "Z: ";
        LCD_DrawString_Color(40, 160, output_text, BLUE, BLACK);
        output_text = "DIS: ";
        LCD_DrawString_Color(25, 190, output_text, WHITE, BLACK);
        /* DEBUG MESSAGE BELOW */
        if (debug != 0xE5) {
            output_text = "SENSOR NP";
            LCD_DrawString_Color(140, 280, output_text, RED, WHITE);
        }
        /* DEBUG MESSAGE ABOVE */
    }
    uint8_t *acceleromterData = malloc(6 * sizeof(uint8_t));
    //arrayofData[1] = x1, arrayofData[0] = x2, arrayofData[3] = y1 .....
    HAL_I2C_Mem_Read(&hi2c2, ADXL_ADDR << 1, 0x32, 1, acceleromterData, 6, 90);
    char x_print[20] = "", y_print[20] = "", z_print[20] = "";
    const int16_t combined_aX = combineUint_8ts(acceleromterData[1], acceleromterData[0]) >> 5;
    const int16_t combined_aY = combineUint_8ts(acceleromterData[3], acceleromterData[2]) >> 5;
    const int16_t combined_aZ = combineUint_8ts(acceleromterData[5], acceleromterData[4]) >> 5;
    uint8_t *compassData = malloc(6 * sizeof(uint8_t));
    uint8_t writeMode = 0x01;
    HAL_I2C_Mem_Write(&hi2c2, HMC5883L_ADDR << 1, 0x02, 1, &writeMode, 1, 100);
    HAL_I2C_Mem_Read(&hi2c2, HMC5883L_ADDR << 1, 0x03, 1, compassData, 6, 100);
    const int16_t thetaX = combineUint_8ts(compassData[0], compassData[1]);
    const int16_t thetaY = combineUint_8ts(compassData[2], compassData[3]);
    const int16_t thetaZ = combineUint_8ts(compassData[4], compassData[5]);
    /* DEBUG MESSAGE BELOW */
    sprintf(x_print, "%+04d", combined_aX);
    sprintf(y_print, "%+04d", combined_aY);
    sprintf(z_print, "%+04d", combined_aZ);
    LCD_DrawString_Color(20, 210, x_print, BLACK, WHITE);
    LCD_DrawString_Color(80, 210, y_print, BLACK, WHITE);
    LCD_DrawString_Color(140, 210, z_print, BLACK, WHITE);
    sprintf(x_print, "%+04d", initX_Acc_Reading);
    sprintf(y_print, "%+04d", initY_Acc_Reading);
    sprintf(z_print, "%+04d", initZ_Acc_Reading);
    LCD_DrawString_Color(20, 230, x_print, BLACK, WHITE);
    LCD_DrawString_Color(80, 230, y_print, BLACK, WHITE);
    LCD_DrawString_Color(140, 230, z_print, BLACK, WHITE);
    sprintf(x_print, "%+04d", accleRecord[RECORD_MAX_SIZE - 1].rawX);
    sprintf(y_print, "%+04d", accleRecord[RECORD_MAX_SIZE - 1].rawY);
    sprintf(z_print, "%+04d", accleRecord[RECORD_MAX_SIZE - 1].rawZ);
    LCD_DrawString_Color(20, 250, x_print, BLACK, WHITE);
    LCD_DrawString_Color(80, 250, y_print, BLACK, WHITE);
    LCD_DrawString_Color(140, 250, z_print, BLACK, WHITE);
    sprintf(x_print, "%03d", acceleRecordSize);
    LCD_DrawString_Color(10, 10, x_print, BLACK, WHITE);
    /* DEBUG MESSAGE ABOVE */

    // All axes have a scale factor of 7.8
    const double finalizedX = (isWithin(combined_aX, initX_Acc_Reading, ACCELE_ACCEPTABLE_ERROR) == 0) ? (combined_aX *
                                                                                                        ACCELE_FACTOR)
                                                                                                     : 0.0;
    const double finalizedY = (isWithin(combined_aY, initY_Acc_Reading, ACCELE_ACCEPTABLE_ERROR) == 0) ? (combined_aY *
                                                                                                          ACCELE_FACTOR)
                                                                                                       : 0.0;
    const double finalizedZ = (isWithin(combined_aZ, initZ_Acc_Reading, ACCELE_ACCEPTABLE_ERROR) == 0) ? (combined_aZ *
                                                                                                          ACCELE_FACTOR)
                                                                                                       : 0.0;
    sprintf(x_print, "%+0.2f", finalizedX);
    sprintf(y_print, "%+0.2f", finalizedY);
    sprintf(z_print, "%+0.2f", finalizedZ);
    LCD_DrawString_Color(100, 100, x_print, CYAN, BLACK);
    LCD_DrawString_Color(100, 130, y_print, YELLOW, BLACK);
    LCD_DrawString_Color(100, 160, z_print, BLUE, BLACK);
    sprintf(x_print, "%+05d", thetaX);
    sprintf(y_print, "%+05d", thetaY);
    sprintf(z_print, "%+05d", thetaZ);
    LCD_DrawString_Color(160, 100, x_print, CYAN, BLACK);
    LCD_DrawString_Color(160, 130, y_print, YELLOW, BLACK);
    LCD_DrawString_Color(160, 160, z_print, BLUE, BLACK);
    const double angle = atan2(thetaY, thetaX) * 180 / M_PI;
    const uint16_t polishedAngle = (int) angle < 0 ? 360 - (int) fabs(angle) : (int) angle;
    char angle_print[8];
    sprintf(angle_print, "%03d deg", polishedAngle);
    LCD_DrawString_Color(170, 20, angle_print, BLACK, WHITE);
    insertAngleRecord(polishedAngle);
    insertAcceleRecord(combined_aX, combined_aY, combined_aZ);
    //if(isRotating() == 1)
    //take in the most recent n entries for average calculation
    updateDistance(combined_aX, combined_aY, combined_aZ);
    char dis_print[10] = "";
    sprintf(dis_print, "%05lu", distanceTraveled);
    LCD_DrawString_Color(100, 190, dis_print, WHITE, BLACK);
    sprintf(dis_print, "%0.3f", actualDistance);
    LCD_DrawString_Color(160, 190, dis_print, WHITE, BLACK);
    free(acceleromterData);
    free(compassData);
}

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
  MX_FSMC_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc2);
  HAL_ADC_PollForConversion(&hadc2, 1000);
  HAL_ADC_Start(&hadc2);
  macXPT2046_CS_DISABLE();
  LCD_INIT();
  currentPage = home;
  HAL_Delay(50);
  while (!XPT2046_Touch_Calibrate());
  LCD_GramScan(1);
  LCD_Clear(0, 0, 240, 320, BLACK);
  mainPage();

  initWeightSensors(weightSensors);
  LCD_InitHeatCoords(weightSensorsData);

  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if (currentPage == home) mainPage();
    else if (currentPage == weight) weightPage();
    else if (currentPage == accelerometer) accelerometerPage();
    else mainPage();
    if (ucXPT2046_TouchFlag == 1) {
        Check_touchkey();
        ucXPT2046_TouchFlag = 0;
    }
    HAL_Delay(150);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
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
