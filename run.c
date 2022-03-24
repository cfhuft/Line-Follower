/*
 * init.c
 *
 *  Created on: May 11, 2021
 *      Author: Samo Novak
 */

#include <run.h>

void Run(void){

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN);
	HAL_Delay(10);

	cell1=map(adc_buf[5], 1390, 1666, 0, 99);
	cell2=map(adc_buf[6], 2820, 3388, 0, 99);

	SSD1306_Init (); // initialize the display

	follow(); 		//follow() does not return
}




