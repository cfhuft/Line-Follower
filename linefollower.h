/*
 * linefollower.h
 *
 *  Created on: May 12, 2021
 *      Author: Samo Novak
 */

#ifndef INC_LINEFOLLOWER_H_
#define INC_LINEFOLLOWER_H_

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "fonts.h"
#include "test.h"
#include "ssd1306.h"


extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;

void follow(void);
int32_t map(int32_t value, int32_t fromLow, int32_t fromHigh, int32_t toLow, int32_t toHigh);

#endif /* INC_LINEFOLLOWER_H_ */
