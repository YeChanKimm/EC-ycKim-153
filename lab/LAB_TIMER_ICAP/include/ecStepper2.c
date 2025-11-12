#include "stm32f4xx.h"
#include "ecStepper2.h"

//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


// Stepper Motor function
uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64*32;
	 

// Stepper Motor variable
volatile Stepper_t myStepper; 


//FULL stepping sequence  - FSM
typedef struct {
  	uint32_t next[2];
	uint8_t out[4];

} State_full_t;

State_full_t FSM_full[4] = {  	// 1010 , 0110 , 0101 , 1001
 	{{S3,S1},{1,1,0,0}},		// ABA'B'
 	{{S0,S2},{0,1,1,0}},
	{{S1,S3},{0,0,1,1}},
	{{S2,S0},{1,0,0,1}},
};

//HALF stepping sequence
typedef struct {
	uint32_t next[2];
	uint8_t out[4];
} State_half_t;

State_half_t FSM_half[8] = {	// 1000 , 1010 , 0010 , 0110 , 0100 , 0101, 0001, 1001
 	{{S7,S1},{1,0,0,0}},	
	{{S0,S2},{1,1,0,0}},
	{{S1,S3},{0,1,0,0}},
	{{S2,S4},{0,1,1,0}},
	{{S3,S5},{0,0,1,0}},
	{{S4,S6},{0,0,1,1}},
	{{S5,S7},{0,0,0,1}},
	{{S6,S0},{1,0,0,1}}
};



void Stepper_init(PinName_t pinName1, PinName_t pinName2, PinName_t pinName3, PinName_t pinName4){
	 
	//  GPIO Digital Out Initiation
	myStepper.pin1 = pinName1;
	// Repeat for port2,pin3,pin4 
	myStepper.pin2 = pinName2;
	myStepper.pin3 = pinName3;
	myStepper.pin4 = pinName4;

	//  GPIO Digital Out Initiation
	// No pull-up Pull-down , Push-Pull, Fast	
	// Pin1 ~ Pin4
	GPIO_init(myStepper.pin1,OUTPUT);
	GPIO_init(myStepper.pin2,OUTPUT);
	GPIO_init(myStepper.pin3,OUTPUT);
	GPIO_init(myStepper.pin4,OUTPUT);

	//PUPD
	GPIO_pupd(myStepper.pin1,NO_PUPD);
	GPIO_pupd(myStepper.pin2,NO_PUPD);
	GPIO_pupd(myStepper.pin3,NO_PUPD);
	GPIO_pupd(myStepper.pin4,NO_PUPD);

	//output type
	GPIO_otype(myStepper.pin1, PUSH_PULL);
	GPIO_otype(myStepper.pin2, PUSH_PULL);
	GPIO_otype(myStepper.pin3, PUSH_PULL);
	GPIO_otype(myStepper.pin4, PUSH_PULL);
	
	//speed
	GPIO_ospeed(myStepper.pin1, FAST_SPEED);
	GPIO_ospeed(myStepper.pin2, FAST_SPEED);
	GPIO_ospeed(myStepper.pin3, FAST_SPEED);
	GPIO_ospeed(myStepper.pin4, FAST_SPEED);

}


void Stepper_pinOut (uint32_t state, uint32_t mode){	
   	if (mode == FULL){         // FULL mode
		GPIO_write(myStepper.pin1, (FSM_full[state].out[0]));
		GPIO_write(myStepper.pin2, (FSM_full[state].out[1]));
		GPIO_write(myStepper.pin3, (FSM_full[state].out[2]));
		GPIO_write(myStepper.pin4, (FSM_full[state].out[3]));
  		
	}	 
 	else if (mode == HALF){    // HALF mode
		GPIO_write(myStepper.pin1, (FSM_half[state].out[0]));
		GPIO_write(myStepper.pin2, (FSM_half[state].out[1]));
		GPIO_write(myStepper.pin3, (FSM_half[state].out[2]));
		GPIO_write(myStepper.pin4, (FSM_half[state].out[3]));
	}
}


void Stepper_setSpeed (long whatSpeed){      // rpm [rev/min]
		 step_delay = 60000/(step_per_rev*whatSpeed);  // Convert rpm to  [msec/step] delay
		//step_delay = 100*whatSpeed;
}


void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode){
	 uint32_t state = 0;
	 myStepper._step_num = steps;

	 for(; myStepper._step_num > 0; myStepper._step_num--){ // run for step size
		    delay_ms(step_delay); 				 
	    	if (mode == FULL) 		 												
			state =FSM_full[state].next[direction];// YOUR CODE       // state = next state
		else if (mode == HALF) 
			state = FSM_half[state].next[direction];// YOUR CODE       // state = next state		
		Stepper_pinOut(state, mode);

		if(mode==FULL)
		{
			if(myStepper._step_num%2048==0) delay_ms(500);
		}
		
		else if(mode==HALF)
		{
			if(myStepper._step_num%4096==0) delay_ms(500);
		}
		
   	}
}


void Stepper_stop (void){ 
    	myStepper._step_num = 0;    
	// All pins(A,AN,B,BN) set as DigitalOut '0'
	GPIO_write(myStepper.pin1, 0);
	GPIO_write(myStepper.pin2, 0);
	GPIO_write(myStepper.pin3, 0);
	GPIO_write(myStepper.pin4, 0);
}

