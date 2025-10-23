/*----------------------------------------------------------------\
@ Embedded Controller by Yechan Kim - Handong Global University
Author           : Yechan Kim
Student Number	 : 22100153
Created          : 10-18-2025
/----------------------------------------------------------------*/
#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"
#include "ecSysTick2.h"
#include "ecEXTI2.h"

// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN PC_13
#define PWM_PIN PA_1


uint32_t pulse_width=500;
void setup(void);


int main(void) {
	// Initialization --------------------------------------------------
	setup();	
	
	// Infinite Loop ---------------------------------------------------
	while(1){ }
}


// Initialiization 
void setup(void) {	
	
    //PLL initialize
    RCC_PLL_init();
	
    // Button Pin initialize
	GPIO_init(BUTTON_PIN, INPUT);
    GPIO_pupd(BUTTON_PIN, PULL_UP);
    EXTI_init(BUTTON_PIN, FALL, 0);


    //PWM Pin initialize
    PWM_init(PWM_PIN);
    PWM_period_us(PWM_PIN,20000);
    GPIO_otype(PWM_PIN, PUSH_PULL);
    GPIO_pupd(PWM_PIN, PULL_UP);
    GPIO_ospeed(PWM_PIN,FAST_SPEED);

	//Timer Interupt initialize
    TIM_UI_init(TIM3,500);
    
}

uint32_t unit_move=110;
int currentState=0;

void TIM3_IRQHandler(void)
{
    
    if(is_UIF(TIM3)){
        
        //Update current state
        currentState++;

        //If the angle of the motor is lower than 180°, move 20°(pulse width 110 us)
        if((currentState/19)%2==0)                                 
        {
            unit_move=110;
        }

        //If the angle of the motor is higher than 180°, move -20°(pulse width -110 us) and go to the first place
        else if((currentState/19)%2==1)
        {
            unit_move=-110;
            
        }

        //Update the pulse width
        pulse_width+=unit_move;
        PWM_pulsewidth_us(PWM_PIN, pulse_width);
       
       
        clear_UIF(TIM3);
    }
}

void EXTI15_10_IRQHandler(void)
{
    if(is_pending_EXTI(BUTTON_PIN))
    {
        //Initial state of pulse width and current state
        pulse_width=500;
        currentState=0;
        
        //Pending clear
        clear_pending_EXTI(BUTTON_PIN);
    }
}