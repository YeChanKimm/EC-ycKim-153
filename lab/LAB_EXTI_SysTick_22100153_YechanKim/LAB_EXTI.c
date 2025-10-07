#include "ecSTM32F4v2.h"

#define BUTTON_PIN PA_4

// Initialiization 
void setup(void)
{
	//Initialize PLL and Systick
	RCC_PLL_init();
	SysTick_init(1);
	
	//Initialize Button pin
	GPIO_init(BUTTON_PIN, INPUT);
	GPIO_pupd(BUTTON_PIN, PULL_UP);

	//Initialize 7segment
    seven_seg_FND_init(OUTPUT, PUSH_PULL, NO_PUPD, MEDIUM_SPEED);

	// Set External Interrupt as highst priority 
	EXTI_init(BUTTON_PIN, FALL, 0);
}

//Initializing current number, display, buttonstate
int crtNum=0;
int selectedFND=1;
int ButtonState=LOW;


int main(void) {
	setup();
	
	while (1) {
		//Display the 10-digit number on the 7 segment
		seven_seg_FND_display_Final(crtNum,selectedFND);
		
	}
}


//EXTI for Pin 13
void EXTI4_IRQHandler(void) {
	
	//Check if the interupt is on pending
	if (is_pending_EXTI(BUTTON_PIN)) {
		
		//Delay
		for(int i=0; i<500000; i++);

		//If the interupt is triggered(pressing button), update the displayed number
		if ( GPIO_read(BUTTON_PIN)== LOW) { 
            crtNum = (crtNum + 1) % 20; 
            ButtonState=HIGH;
        }

		//Clear the pending
		clear_pending_EXTI(BUTTON_PIN); 
		
	}

}