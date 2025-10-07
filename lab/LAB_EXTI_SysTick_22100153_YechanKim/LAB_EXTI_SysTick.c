#include "ecSTM32F4v2.h"
#define BUTTON_PIN PA_4
ButtonState=HIGH;

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
	
	//Initialize EXTI
	EXTI_init(BUTTON_PIN, FALL, 0);
}

uint8_t crtNum=0;

//PA_10 Display
int selectedFND=3;



int main(void) {
	setup();
	//SysTick_counter();
	while (1) {

		//If 1sec is passed, update the current number. 
        delay_ms(1000); 
		crtNum = (crtNum + 1) % 10; 
        seven_seg_FND_display(crtNum,selectedFND);
	
	}
}


void EXTI4_IRQHandler(void) {
	
	//Check if the interupt is on pending
	if (is_pending_EXTI(BUTTON_PIN)) {
		

		//If the interupt is triggered(pressing button), initialize the number to 0. 
		if ( GPIO_read(BUTTON_PIN)== LOW) { 
            crtNum = 9;
            ButtonState=HIGH;
			//delay_ms(1000);
        }

		//Clear the pending
		clear_pending_EXTI(BUTTON_PIN); 
		
	}

}



