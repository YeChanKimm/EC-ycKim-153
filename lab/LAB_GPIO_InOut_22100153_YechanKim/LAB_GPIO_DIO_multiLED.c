/*----------------------------------------------------------------\
@ Embedded Controller by Yechan Kim - Handong Global University
Author           : Yechan Kim
Student Number	 : 22100153
Created          : 09-17-2025
/----------------------------------------------------------------*/

//Header files used in the project
#include "ecRCC2.h"
#include "ecGPIO2.h"
#include "ecPinNames.h"

//Addresses of LEDs Used in the code
PinName_t LED[4] = {
    PB_12,PB_13,PB_14,PB_15
};

//Address of the Button Used in the code
#define BUTTON_PIN PA_4 


//Parameter numbers
#define PULL_DOWN 1
#define PULL_UP 2
#define PUSH_PULL 0
#define OPEN_DRAIN 1
#define LOW_SPEED 0
#define MEDIUM_SPEED 1
#define FAST_SPEED 2
#define HIGH_SPEED 3



void setup(void)
{   
    
    // Initialize system clock to 16 MHz using the internal HSI oscillator
    RCC_HSI_init();

    // initialize the pushbutton pin as an input
    GPIO_init(BUTTON_PIN, INPUT);

    // Configure th button with an internal pull-up resistor
	GPIO_pupd(BUTTON_PIN, PULL_UP);

    // initialize LED pins as an output:
    for (int i=0; i<4; ++i) GPIO_init(LED[i], OUTPUT);

    //Set output type as push pull(open drain)
    for (int i=0; i<4; ++i) GPIO_otype(LED[i], PUSH_PULL);

    // Configure LED pins with an internal pull-up resistor
    for (int i=0; i<4; ++i) GPIO_pupd(LED[i], PULL_UP);

    //Set the output speed as medium
    for (int i=0; i<4; ++i) GPIO_ospeed(LED[i], MEDIUM_SPEED);

}
	

int main(void)
{
    //Set the register
    setup();

    //Declare the current and previous state of the button
    int buttonState=LOW;
    int Last_buttonState=LOW;

    //Initialize the current LED
    int crtLED=0;
 
    while(1)
    {
        //check if the pushbutton is pressed. Turn LED on/off accordingly:
        buttonState=GPIO_read(BUTTON_PIN);

        //Set it so that the state changes only when the button is pressed.
        if((buttonState==LOW && Last_buttonState==HIGH) )
        {
            //Turn off all of LED
            for(int i=0; i<4; ++i) GPIO_write(LED[i],LOW);
           
            //Turn on current LED
            GPIO_write(LED[crtLED],HIGH);

            //Set the next LED as the current one
            crtLED=(crtLED+1)%4;
            
        }

        //Store the current state of the button in the previous state variable.
        Last_buttonState=buttonState; 
    } 
}