#include "ecSTM32F4v2.h"

//Stepping mode
int mode=HALF;

//Pulse per rotation
int pulse_per_rev=FULL_PULSE_PER_ROTATION;

//Number of revolutions
uint8_t revolution=10;

//Input rpm speed
int rpm_speed=14.9999;

//Speed scaler. Full:1, Half:2
int speed_multiplier=0;


void setup(void);
void mode_changing(void);
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	Stepper_step(revolution*pulse_per_rev, 1, mode);  // (Step : 2048, Direction : 0 or 1, Mode : FULL or HALF)
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){;}
}

// Initialiization 
void setup(void){
	
	RCC_PLL_init();                                 // System Clock = 84MHz
	SysTick_init(1);                                 // Systick init

	Stepper_init(PB_10,PB_4,PB_5,PB_3); 			// Stepper GPIO pin initialization
	
	mode_changing(); //choose full stepping or half stepping
	Stepper_setSpeed(speed_multiplier*rpm_speed);                          	//  set stepper motor speed
}


void mode_changing(void)
{
	//Change pulse per rotation and speed by stepping mode
	if(mode==FULL) 
	{
		pulse_per_rev=FULL_PULSE_PER_ROTATION;
		speed_multiplier=1;
	}
	else if(mode==HALF) 
	{
		pulse_per_rev=HALF_PULSE_PER_ROTATION;
		speed_multiplier=2;
	}
	
	
}



