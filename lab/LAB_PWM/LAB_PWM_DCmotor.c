/*----------------------------------------------------------------\
@ Embedded Controller by Yechan Kim - Handong Global University
Author           : Yechan Kim
Student Number	 : 22100153
Created          : 10-18-2025
/----------------------------------------------------------------*/
#include "ecSTM32F4v2.h"
#include <stdlib.h>

// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN PC_13
#define DIRECTION_PIN PC_2
#define PWM_PIN PA_0
#define LED PA_5


int timerState=0;



float targetPWM=0.25;
int motorState=0;

void setup(void);
void motor_on_off(void);

int main(void) {
	// Initialization --------------------------------------------------
	setup();	
	
	// Infinite Loop ---------------------------------------------------
	while(1){ 

        motor_on_off();
     
    }
}

void motor_on_off(void)
{
    //If the button is pressed and the motor was paused, release it. 
    if(motorState==0) PWM_duty(PWM_PIN,targetPWM);

    //If the button is pressed and the motor was released, pause it.  
    else if(motorState==1) PWM_duty(PWM_PIN,0.0);

}

// Initialiization 
void setup(void) {	
	
    //PLL initialize
    RCC_PLL_init();
	
    // Button Pin initialize
	GPIO_init(BUTTON_PIN, INPUT);
    GPIO_pupd(BUTTON_PIN, PULL_UP);
    EXTI_init(BUTTON_PIN, FALL, 0);


    //Direction Pin initialize
    GPIO_init(DIRECTION_PIN, OUTPUT);
    GPIO_otype(DIRECTION_PIN, PUSH_PULL);
  

    //PWM PIN initialize
	PWM_init(PWM_PIN);
    PWM_period_ms(PWM_PIN,1);
    GPIO_otype(PWM_PIN, PUSH_PULL);
    GPIO_pupd(PWM_PIN, PULL_UP);
    GPIO_ospeed(PWM_PIN,FAST_SPEED);
    

	//Timer3 interupt  0.5s
    TIM_UI_init(TIM3,500);
    
}

uint32_t buttonState=0;
void EXTI15_10_IRQHandler(void)
{
    if(is_pending_EXTI(BUTTON_PIN))
    {   
        //Toggle the motor state to change the speed. 
        motorState=!motorState;
    }

    clear_pending_EXTI(BUTTON_PIN);
}

void TIM3_IRQHandler(void)
{
    if((is_UIF(TIM3)) == HIGH){
       
       //Update the time state
        timerState++;

        //If 2seconds passed, change the duty ratio
       if(timerState%4==0) 
       {
            //Slow->Fast
            if(targetPWM==0.25) targetPWM = 0.75;

            //Fast->Slow
            else if(targetPWM==0.75) targetPWM = 0.25;
       }
       clear_UIF(TIM3);
    }
}
