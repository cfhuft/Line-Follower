/*
 * oled_UI.c
 *
 *  Created on: May 11, 2021
 *      Author: Samo Novak
 */

#include "linefollower.h"

//----------------------------------------------------------------------------------DMA
#define ADC_BUF_LEN 14
uint16_t adc_buf[ADC_BUF_LEN]; 			//DMA ADC buffer
volatile uint16_t dma_clbck=0;			//DMA callback

uint16_t sensor0_cal=1000;			//sensor default calibrations black/white
uint16_t sensor1_cal=1000;
uint16_t sensor2_cal=3400;
uint16_t sensor3_cal=2200;
uint16_t sensor4_cal=2200;

//----------------------------------------------------------------------------------Encoders
volatile uint16_t m=0;				//encoder ticks m
volatile uint16_t n=0;				//encoder ticks n
float encoder_angle_piece[7000];		//position values for dead reckoning

//----------------------------------------------------------------------------------interrupt buttons
volatile uint8_t one_int=0;			//interrupt button presses
volatile uint8_t two_int=0;
volatile uint8_t three_int=0;
uint8_t one=0;					//previous button presses
uint8_t two=0;
uint8_t three=0;

//-----------------------------------------------------------------------------------Battery
char battery_buf[8];				//battery % buffer for OLED display
uint8_t cell1;					//cell 1 ADC number
uint8_t cell2;					//cell 2 ADC number

//-----------------------------------------------------------------------------------FSM (The state transition table for OLED user interface)
enum states {
   mod, calibration, start, m1, m2, m3, v1, v2, v3, v4, ok, ok2, x
};

enum columns {
    CURRENT, LEFT, MID, RIGHT, NEXT
};

#define ROWS 32
#define COLS 5

int state_table[ROWS][COLS] = {
// current    			 LEFT   MID   RIGHT  next
    { mod,     			true, false, false, calibration },
    { mod,     			false, false, true, start },
    { mod,    			false, true, false, m1 },
    { calibration,    	true, false, false, start },
    { calibration,   	false, false, true, mod },
    { calibration,      false, true, false, ok },
    { start,   			true, false, false, mod },
    { start,     		false, false, true, calibration },
    { start,     		false, true, false, x },
    { m1,     			true, false, false, m2 },
    { m1,     			false, false, true, m3 },
    { m1,     			false, true, false, mod },
    { m2,     			true, false, false, m3 },
    { m2,     			false, false, true, m1 },
    { m2,     			false, true, false, v1 },
    { m3,     			true, false, false, m1 },
    { m3,     			false, false, true, m2 },
    { m3,     			false, true, false, v3 },
    { v1,     			true, false, false, v2 },
    { v1,     			false, false, true, v2 },
    { v1,     			false, true, false, mod },
    { v2,     			true, false, false, v1 },
    { v2,     			false, false, true, v1 },
    { v2,     			false, true, false, mod },
    { v3,     			true, false, false, v4 },
    { v3,     			false, false, true, v4 },
    { v3,     			false, true, false, mod },
    { v4,     			true, false, false, v3 },
    { v4,     			false, false, true, v3 },
    { v4,     			false, true, false, mod },
	{ ok,     			false, true, false, ok2 },
	{ ok2,     			false, true, false, calibration },
};

//-------------------------------------------------------------------------------FSM interface
uint8_t currentState = mod;
uint8_t previousState = mod;
uint8_t event = -1;

//-------------------------------------------------------------------------------PID IR sensors

float error=0;
float integral=0;
float differential=0;
float PID=0;
float prev_error=0;
float Kp=0;
float Ki=0;
float Kd=0;
int32_t PWM=0;
int16_t i=-1;
int16_t average=0;				//average PID line value
int16_t sum=1;
int16_t position=0;
int16_t setpoint=200;
int16_t PID_buf[5];
uint16_t noLine = 0;
float slow_by_angle = 1;


//dead reckoning scanning
float cos_vector=0;
float sin_vector=0;
uint16_t vector_counter=0;

//-------------------------------------------------------------------------------PID Encoders

float error_enc=0;
float prev_error_enc=0;
float integral_enc=0;
float differential_enc=0;
float PID_enc=0;
float Kp2=0;
float Ki2=0;
float Kd2=0;
int32_t PWM_enc;

uint16_t enc_counter=0;
uint8_t n_2=0;
uint8_t m_2=0;

float angle_n=0;
float angle_m=0;
float angle;
float angle_2=0;
float setpoint_enc=0;
float position_enc=0;

//-----------------------------------------------------------------------------On run_parameters
uint16_t mode = 1;
float PWM_multiplier=1.0;


//-----------------------------------------------------------------------------Additional Functions
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin==GPIO_PIN_11){
			m++;				//encoder interrupt
		}
	else if (GPIO_Pin==GPIO_PIN_12){
			n++;				//encoder interrupt
		}
	else if (GPIO_Pin==GPIO_PIN_4){
			two_int++;			//button interrupt
	}
	else if (GPIO_Pin==GPIO_PIN_3){
			three_int++;			//button interrupt
		}
	else if (GPIO_Pin==GPIO_PIN_8){
			one_int++;			//button interrupt
		}

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	dma_clbck=0;						//dma half-full callback
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	dma_clbck=1;						//dma full callback
}

void calibration_function(void){				//calibrate IR sensors with ADC data
	if (sensor0_cal != 0){
		sensor0_cal = (sensor0_cal+adc_buf[0])/2;	//calibration OFF-LINE
		sensor1_cal = (sensor1_cal+adc_buf[1])/2;
		sensor2_cal = (sensor2_cal+adc_buf[2])/2;
		sensor3_cal = (sensor3_cal+adc_buf[3])/2;
		sensor4_cal = (sensor4_cal+adc_buf[4])/2;
	}
	else{
		sensor0_cal = adc_buf[0];			//calibration ON-LINE
		sensor1_cal = adc_buf[1];
		sensor2_cal = adc_buf[2];
		sensor3_cal = adc_buf[3];
		sensor4_cal = adc_buf[4];
	}
}

void getADC(uint8_t buffer_half, int16_t *PID_buf){		//update IR sensor PID controller feedback
	if (adc_buf[buffer_half-2]<sensor4_cal){PID_buf[4]=1;}
	else {PID_buf[4]=0;}
	if (adc_buf[buffer_half-3]<sensor3_cal){PID_buf[3]=1;}
	else {PID_buf[3]=0;}
	if (adc_buf[buffer_half-4]<sensor2_cal){PID_buf[2]=1;}
	else {PID_buf[2]=0;}
	if (adc_buf[buffer_half-5]<sensor1_cal){PID_buf[1]=1;}
	else {PID_buf[1]=0;}
	if (adc_buf[buffer_half-6]<sensor0_cal){PID_buf[0]=1;}
	else {PID_buf[0]=0;}
}

int32_t map(int32_t value, int32_t fromLow, int32_t fromHigh, int32_t toLow, int32_t toHigh)	//map function
{
	if (value > fromHigh){value = fromHigh;}
	return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

int32_t constrain(int32_t x, int32_t high, int32_t low){					//constrain function
	if (x>high){return high;}
	else if (x<low){return low;}
	else return x;
}

//---------------------------------------------------------------------------Start LineFollower

void follow(void){

while(1){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);

	currentState = mod;					//initial states of the FSM
	previousState = mod;
	event = -1;

	sprintf(battery_buf, "C1:%d", cell1);			//refresh initial OLED drawing
	SSD1306_GotoXY (0,1);
	SSD1306_Puts (battery_buf, &Font_11x18, 0);
	sprintf(battery_buf, "C2:%d", cell2);
	SSD1306_GotoXY (70, 1);
	SSD1306_Puts (battery_buf, &Font_11x18, 0);

	SSD1306_GotoXY (0, 24);
	SSD1306_Puts ("MODE: ", &Font_11x18, 1);
	SSD1306_GotoXY (0, 45);
	SSD1306_Puts (" 1 ", &Font_11x18, 1);
	SSD1306_Puts (" ", &Font_11x18, 1);
	SSD1306_Puts (" 2 ", &Font_11x18, 1);
	SSD1306_Puts (" ", &Font_11x18, 1);
	SSD1306_Puts (" 3 ", &Font_11x18, 1);
	SSD1306_UpdateScreen();

	while (1) {

		int row, col;
		bool checkRows = true, checkCols;

		if(one_int>one){		//was button 1 pressed?
			event = 1;
			one=one_int;
		}
		if(two_int>two){		//was button 2 pressed?
			event = 2;
			two=two_int;
			        	}
		if(three_int>three){		//was button 3 pressed?
			event = 3;
			three=three_int;
		}




		for (row = 0; checkRows && (row < ROWS); row++) {				//Find a row with the active state
			checkCols = true;
			if (currentState == state_table[row][CURRENT]) {			//if found
				for (col = CURRENT + 1; checkCols && (col < NEXT); col++) {	//Find a column with the active state
					if (state_table[row][col]) {				//if found
						if (event == col) {				//is the button that triggered the event equal to the column?
							previousState = currentState;
			                currentState = state_table[row][NEXT];			//do the transition

			                if (currentState == 0){					//INITIAL STATE
			                	if (previousState == 3){			//if chosen mode was scanning - display the choice
			                		PWM_multiplier=1.0; mode=1;
				                    SSD1306_Clear();
				                    SSD1306_GotoXY (40,1);
				                    SSD1306_Puts ("SCAN", &Font_11x18, 1);
				                    SSD1306_GotoXY (40, 24);
				                    SSD1306_Puts ("MODE", &Font_11x18, 1);
			                        SSD1306_UpdateScreen();
			                        HAL_Delay(1200);				//return to state 0 without change
			                    }
			                    else if (previousState == 6){PWM_multiplier=1.0; mode=2;}	//if chosen mode was dead reckoning - speed low
			                    else if (previousState == 7){PWM_multiplier=1.5; mode=2;}	//if chosen mode was dead reckoning - speed high
			                    else if (previousState == 8){PWM_multiplier=1.5; mode=3;}	//if chosen mode was smart following - speed low
			                    else if (previousState == 9){PWM_multiplier=2; mode=3;}	//if chosen mode was smart following - speed low
			                    SSD1306_Clear();
			                    sprintf(battery_buf, "C1:%d", cell1);
			                    SSD1306_GotoXY (0,1);
			                    SSD1306_Puts (battery_buf, &Font_11x18, 0);
			                    sprintf(battery_buf, "C2:%d", cell2);
			                    SSD1306_GotoXY (70, 1);
			                    SSD1306_Puts (battery_buf, &Font_11x18, 0);
			                    SSD1306_GotoXY (0, 24);
			                    SSD1306_Puts ("MODE: ", &Font_11x18, 1);
			                    SSD1306_GotoXY (0, 45);
			                    SSD1306_Puts (" 1 ", &Font_11x18, 1);
			                    SSD1306_Puts (" ", &Font_11x18, 1);
			                    SSD1306_Puts (" 2 ", &Font_11x18, 1);
			                    SSD1306_Puts (" ", &Font_11x18, 1);
			                    SSD1306_Puts (" 3 ", &Font_11x18, 1);
			                    SSD1306_UpdateScreen();
			                }
			                else if (currentState == 1){					//INITIAL CALIBRATING STATE
			            	    if (previousState == 11){
			            	    	SSD1306_Clear();
			                        SSD1306_GotoXY (0, 25);
			                        SSD1306_Puts ("CALIBRATING", &Font_11x18, 1);
			                        SSD1306_UpdateScreen();
			                        HAL_Delay(1500);
			                        calibration_function();
			                        }
			                    SSD1306_Clear();
			                    sprintf(battery_buf, "C1:%d", cell1);
			                    SSD1306_GotoXY (0,1);
			                    SSD1306_Puts (battery_buf, &Font_11x18, 0);
			                    sprintf(battery_buf, "C2:%d", cell2);
			                    SSD1306_GotoXY (70, 1);
			                    SSD1306_Puts (battery_buf, &Font_11x18, 0);
			                    SSD1306_GotoXY (0, 24);
			                    SSD1306_Puts ("CALIBRATION", &Font_11x18, 1);
		                        SSD1306_GotoXY (40, 45);
		                        SSD1306_Puts (" OK ", &Font_11x18, 0);
			                    SSD1306_UpdateScreen();
			                }
			                else if (currentState == 2){					//START? STATE
				                SSD1306_Clear();
				                sprintf(battery_buf, "C1:%d", cell1);
				                SSD1306_GotoXY (0,1);
				                SSD1306_Puts (battery_buf, &Font_11x18, 0);
				                sprintf(battery_buf, "C2:%d", cell2);
				                SSD1306_GotoXY (70, 1);
				                SSD1306_Puts (battery_buf, &Font_11x18, 0);
				                SSD1306_GotoXY (0, 24);
				                SSD1306_Puts ("START?", &Font_11x18, 1);
			                    SSD1306_GotoXY (40, 45);
			                    SSD1306_Puts (" OK ", &Font_11x18, 0);
				                SSD1306_UpdateScreen();
			                }
			                else if (currentState == 3){					//MODE 1 STATE
			                	SSD1306_GotoXY (0, 45);
			                    SSD1306_Puts (" 1 ", &Font_11x18, 0);
			                    SSD1306_Puts (" ", &Font_11x18, 1);
			                    SSD1306_Puts (" 2 ", &Font_11x18, 1);
			                    SSD1306_Puts (" ", &Font_11x18, 1);
			                    SSD1306_Puts (" 3 ", &Font_11x18, 1);
			                    SSD1306_UpdateScreen();
			                }
			                else if (currentState == 4){					//MODE 1 STATE
				                SSD1306_GotoXY (0, 45);
				                SSD1306_Puts (" 1 ", &Font_11x18, 1);
				                SSD1306_Puts (" ", &Font_11x18, 1);
				                SSD1306_Puts (" 2 ", &Font_11x18, 0);
				                SSD1306_Puts (" ", &Font_11x18, 1);
				                SSD1306_Puts (" 3 ", &Font_11x18, 1);
				                SSD1306_UpdateScreen();
			                }
			                else if (currentState == 5){					//MODE 1 STATE
				                SSD1306_GotoXY (0, 45);
				                SSD1306_Puts (" 1 ", &Font_11x18, 1);
				                SSD1306_Puts (" ", &Font_11x18, 1);
				                SSD1306_Puts (" 2 ", &Font_11x18, 1);
				                SSD1306_Puts (" ", &Font_11x18, 1);
				                SSD1306_Puts (" 3 ", &Font_11x18, 0);
				                SSD1306_UpdateScreen();
			                }

			                else if (currentState == 6){					//MODE 2 STATE
				                SSD1306_Clear();
				                SSD1306_GotoXY (0,1);
				                SSD1306_Puts ("RECKONING", &Font_11x18, 1);
				                SSD1306_GotoXY (0, 24);
				                SSD1306_Puts ("SPEED? ", &Font_11x18, 1);
			                    SSD1306_GotoXY (0, 45);
			                    SSD1306_Puts ("LOW", &Font_11x18, 0);
			                    SSD1306_Puts ("   HIGH", &Font_11x18, 1);
				                SSD1306_UpdateScreen();
			                }

			                else if (currentState == 7){					//MODE 2 STATE
			                    SSD1306_GotoXY (0, 45);
			                    SSD1306_Puts ("LOW   ", &Font_11x18, 1);
			                    SSD1306_Puts ("HIGH", &Font_11x18, 0);
				                SSD1306_UpdateScreen();
			                }
			                else if (currentState == 8){					//MODE 3 STATE
				                SSD1306_Clear();
				                SSD1306_GotoXY (0,1);
				                SSD1306_Puts ("SMART", &Font_11x18, 1);
				                SSD1306_GotoXY (0, 24);
				                SSD1306_Puts ("SPEED? ", &Font_11x18, 1);
			                    SSD1306_GotoXY (0, 45);
			                    SSD1306_Puts ("LOW", &Font_11x18, 0);
			                    SSD1306_Puts ("   HIGH", &Font_11x18, 1);
				                SSD1306_UpdateScreen();
			                }
			                else if (currentState == 9){					//MODE 3 STATE
			                    SSD1306_GotoXY (0, 45);
			                    SSD1306_Puts ("LOW   ", &Font_11x18, 1);
			                    SSD1306_Puts ("HIGH", &Font_11x18, 0);
				                SSD1306_UpdateScreen();
			                }
			                else if (currentState == 10){					//CALIBRATING ON-LINE STATE
				                SSD1306_Clear();
				                SSD1306_GotoXY (0, 24);
				                SSD1306_Puts ("PUT ON LINE", &Font_11x18, 1);
			                    SSD1306_GotoXY (40, 45);
			                    SSD1306_Puts (" OK ", &Font_11x18, 0);
				                SSD1306_UpdateScreen();
			                }
			                else if (currentState == 11){					//CALIBRATING OFF-LINE STATE
			                    SSD1306_Clear();
			                    SSD1306_GotoXY (0, 25);
			                    SSD1306_Puts ("CALIBRATING", &Font_11x18, 1);
			                    SSD1306_UpdateScreen();
			                    HAL_Delay(1500);
			                    sensor0_cal=0;
			                    sensor1_cal=0;
			                    sensor2_cal=0;
			                    sensor3_cal=0;
			                    sensor4_cal=0;
			                    calibration_function();

				                SSD1306_GotoXY (0, 24);
				                SSD1306_Puts ("PUT OFFLINE", &Font_11x18, 1);
			                    SSD1306_GotoXY (40, 45);
			                    SSD1306_Puts (" OK ", &Font_11x18, 0);
				                SSD1306_UpdateScreen();
			                }
			                else if (currentState == 12){				//STARTING STATE
			                    SSD1306_Clear();
			                    SSD1306_GotoXY (20, 25);
			                    SSD1306_Puts ("STARTING", &Font_11x18, 1);
			                    SSD1306_UpdateScreen();
			                    HAL_Delay(2000);
			                    SSD1306_Clear();
			                }

			                checkRows = checkCols = false;				//Stop checking any further rows and columns
						}else {
							checkCols = false;			//The button does not match the column, so this transitions won't be triggered
			        	}
					}
				}
			}
		}
		HAL_Delay(300);
		one_int=one;
		two_int=two;
		three_int=three;
		while((one==one_int)&&(two==two_int)&&(three==three_int)&&(currentState!=12)){	//If the state is not 12 (starting) the robot stays in here until one of the buttons get pressed again

			cell1=map(adc_buf[5], 1390, 1666, 0, 99); 				// 1666 max ADC at 4,2V   1390 min ADC at 3,5V
			cell2=map(adc_buf[6], 2820, 3388, 0, 99); 				//3388 max ADC at 8,4V    2820 min ADC at 7V

			if (cell1<6||cell2<6){							//low battery warning
				while(1){
					SSD1306_Clear();
				    SSD1306_GotoXY (0,1);
				    SSD1306_Puts ("BATTERY LOW", &Font_11x18, 0);
				    SSD1306_GotoXY (0, 24);
				    SSD1306_Puts ("UNPLUG AND", &Font_11x18, 1);
				    SSD1306_GotoXY (0, 45);
				    SSD1306_Puts ("RECHARGE!!", &Font_11x18, 0);
				    SSD1306_UpdateScreen();
			    	HAL_Delay(5000);
			    }
			}
		}
		if (currentState == 12)break;			//if the state is 12 the robot exits the while loop to start following the line
	}

	n=0;										//initialisations
	m=0;
	setpoint_enc=0;
	position_enc=0;
	error_enc=0;
	angle=0;
	angle_2=0;
	position_enc=0;
	cos_vector=0;
	sin_vector=0;
	enc_counter=0;
	vector_counter = 0;
	noLine=0;
	n_2=0;
	m_2=0;
	integral=0;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

/*----------------------------------------------mode 1 (scanning)----------------------------------------------------*/

	while(currentState == 12 && mode == 1 && noLine < 650){

		if(n+m>50){							//every 50 encoder ticks on both encoders
			angle_n=n*0.00352699;					//calculate angle [radians] based on the encoder ticks (number 0.00352699 comes from wheel and robot dimensions)
			angle_m=m*0.00352699;

			if (sum==0){noLine+=(n+m);}				//robot off the line summation
			else {noLine=0;}

			n_2+=n;
			m_2+=m;
			n=0;
			m=0;
			angle = angle + (angle_n-angle_m)*180/3.14;		//calculate the overall angle change [degrees]
			encoder_angle_piece[enc_counter] = angle;		//save the new angle in an array
			enc_counter++;
			if (enc_counter == 6999)break;				//break if the buffer is full
		}

		if(n_2>50 || m_2>50){						//every 50 encoder ticks on either encoder
			n_2=0;
			m_2=0;
			if(angle<0){						//negative or positive angle?
				angle_2=360+angle;
			}else{angle_2=angle;}
			cos_vector = cos_vector + (cos(angle_2/180*3.14)); 	//[radians]
			sin_vector = sin_vector + (sin(angle_2/180*3.14));
			vector_counter++;
			if (cos_vector<2 && cos_vector>-2 && sin_vector<2 && sin_vector>-2 && vector_counter > 10){
				encoder_angle_piece[enc_counter+1]=0;
				break;						//when the robot is in the same position as at the start, break the while loop
			}
		}

		if (dma_clbck==0){						//DMA half full (new data ready)
			getADC(6, &PID_buf[0]);
		}
		else if (dma_clbck==1){						//DMA full (new data ready)
			getADC(13, &PID_buf[0]);
		}

		dma_clbck=2;
		position = 0;
		sum = 0;

		for(i=0; i<=4; i++){
			position += PID_buf[i] * i * 100;			//average signal position of the IR sensors
			sum += PID_buf[i];					//sum signal position of the IR sensors
		}
		if(sum!=0){
			average = position/sum;					//position of the robot
			error = average - setpoint;				//error
			Kp=1;
			PID=Kp*error;
			PWM=PID;
			PWM=constrain(PWM, Kp*200, -Kp*200);
			PWM=map(PWM, (-Kp*200), (Kp*200), -250, 250);
			if (error<0){
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (250+PWM)*PWM_multiplier);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250*PWM_multiplier);
			}
			else if (error == 0){
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250*PWM_multiplier);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 250*PWM_multiplier);
			}
			else{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (250-PWM)*PWM_multiplier);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 250*PWM_multiplier);
			}
		}
	}

/*----------------------------------------------mode 2 (dead reckoning)-------------------------------------------------*/

	while(currentState == 12 && mode == 2){

		if(n_2+m_2>50){							//take a new set point every 50 ticks (same rate as it was scanned at)
			n_2=0;
			m_2=0;
			setpoint_enc=encoder_angle_piece[enc_counter];		//set point = saved position
			enc_counter++;
		}
		if(n+m>12){							//correct the position every 12 ticks
			if (enc_counter>5 && setpoint_enc==0)break;		//break if set point is 0 (we made once circle)
			angle_n=n*0.00353699;					//the constant 0.00353699 is slightly adjusted compared to the previous mode - to correct a small error
			angle_m=m*0.00353699;
			n_2+=n;
			m_2+=m;
			n=0;
			m=0;
			position_enc = position_enc + (angle_n-angle_m)*180/3.14;			//calculate the actual position
			error_enc = position_enc - setpoint_enc;					//error
			if (abs(prev_error_enc/1.7) < abs(error_enc)){integral_enc+=error_enc;}		//integral
			else {integral_enc=error_enc;}
			differential_enc = error_enc - prev_error_enc;					//differential
			prev_error_enc = error_enc;

			Kp2=1;
			Ki2=1;
			Kd2=0.5;

			PID_enc= Kp2*error_enc + Ki2*integral_enc + Kd2*differential_enc;
			PID_enc=PID_enc*100;
			PWM_enc=PID_enc;
			PWM_enc=constrain(PWM_enc, 700, -700);
			PWM_enc=map(PWM_enc, (-700), (700), -250, 250);

			if (PWM_enc <= 0){
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (250+PWM_enc)*PWM_multiplier);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250*PWM_multiplier);
			}
			else if (PWM_enc > 0){
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (250-PWM_enc)*PWM_multiplier);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 250*PWM_multiplier);
			}
		}
	}

/*----------------------------------------------mode 3 (smart following)-------------------------------------------------*/

	while(currentState == 12 && mode == 3 && noLine < 650){
		if(n+m>50){
			if (sum==0){noLine+=(n+m);}
			else {noLine=0;}
			n=0;
			m=0;
			setpoint_enc=encoder_angle_piece[enc_counter];						//setpoint = saved position
			enc_counter++;
			if (enc_counter>5 && setpoint_enc==0)break;						//break if setpoint is 0

			if(fabs(encoder_angle_piece[enc_counter+1]-encoder_angle_piece[enc_counter+7])>=20){	//if there is a turn incoming larger than 20°
				slow_by_angle = 95/PWM_multiplier/100;						//slow down to base speed (~250PWM) - depending on the size of the PWM_multiplier value
			}
			else{											//else if the turn is smaller
				slow_by_angle = (map((int32_t)(fabs(encoder_angle_piece[enc_counter+1]-encoder_angle_piece[enc_counter+7])), 20, 0, (int32_t)95/PWM_multiplier, 100))/100; //map the speed reduction to the turn size
			}
			if(slow_by_angle > 0.8 && fabs(encoder_angle_piece[enc_counter+8]-encoder_angle_piece[enc_counter+28])>=20){	//if there is a turn further down the path and the speed reduction is
				slow_by_angle=slow_by_angle/(PWM_multiplier/1.1);}							//smaller than *0.8, slow down additionaly (1,1 is experimentally determined)
			}

			if (dma_clbck==0){					//DMA half full (new data ready)
				getADC(6, &PID_buf[0]);
			}
			else if (dma_clbck==1){					//DMA full (new data ready)
				getADC(13, &PID_buf[0]);
			}

			dma_clbck=2;
			position = 0;
			sum = 0;

			for(i=0; i<=4; i++){
				position += PID_buf[i] * i * 100;			//average signal position of the IR sensors
				sum += PID_buf[i];					//sum signal position of the IR sensors
			}

			if(sum!=0){
				average = position/sum;					//position of the robot
				error = average - setpoint;				//error
				if (abs(prev_error) <= abs(error)){integral+=error;}	//integral
				else {integral=error;}
				differential =error-prev_error;				//differential
				prev_error=error;
				Kp=1;
				Ki=0;
				Kd=0;

				PID=Kp*error+Ki*integral+Kd*differential;
				PWM=PID;
				PWM=constrain(PWM, (Kp*200+Ki*600+Kd*200), (-Kp*200-Ki*600-Kd*200));
				PWM=map(PWM, (-Kp*200-Ki*600-Kd*200), (Kp*200+Ki*600+Kd*200), -250, 250);

				if (error<0){
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (250+PWM)*PWM_multiplier*slow_by_angle);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250*PWM_multiplier*slow_by_angle);
				}
				else if (error == 0){
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250*PWM_multiplier*slow_by_angle);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 250*PWM_multiplier*slow_by_angle);
				}
				else {
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (250-PWM)*PWM_multiplier*slow_by_angle);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 250*PWM_multiplier*slow_by_angle);
				}
			}
		}
	}
}

