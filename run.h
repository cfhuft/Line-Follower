/*
 * run.h
 *
 *  Created on: May 12, 2021
 *      Author: Samo Novak
 */

#ifndef INC_RUN_H_
#define INC_RUN_H_

#include "linefollower.h"
#define ADC_BUF_LEN 14

void Run(void);

extern uint8_t cell1;
extern uint8_t cell2;
extern uint16_t adc_buf[ADC_BUF_LEN]; //DMA buff

#endif /* INC_RUN_H_ */
