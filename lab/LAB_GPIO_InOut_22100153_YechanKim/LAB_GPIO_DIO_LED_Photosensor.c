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

//Address of the LED Used in the code
#define LED_PIN PB_12   

//Address of the Button Used in the code
#define BUTTON_PIN PA_0 

//Parameter numbers
#define PULL_DOWN 1
#define PULL_UP 2
#define PUSH_PULL 0
#define OPEN_DRAIN 1
#define LOW_SPEED 0
#define MEDIUM_SPEED 1
#define FAST_SPEED 2
#define HIGH_SPEED 3


// Initialiization 
void setup(void) {
	
	// Initialize system clock to 16 MHz using the internal HSI oscillator
	RCC_HSI_init();
	
	// initialize the pushbutton pin as an input
	GPIO_init(BUTTON_PIN, INPUT);  

	// Configure th button with an internal pull-up resistor
	GPIO_pupd(BUTTON_PIN, PULL_UP);
	
	// initialize the LED pin as an output:
	GPIO_init(LED_PIN, OUTPUT);
	
	//Set output type as push pull(open drain)
	GPIO_otype(LED_PIN, PUSH_PULL);

	// Configure the LED pin with an internal pull-up resistor
	GPIO_pupd(LED_PIN, PULL_DOWN);

	//Set the output speed as medium
	GPIO_ospeed(LED_PIN, MEDIUM_SPEED);

}
	
int main(void) { 
 	setup();

	//Declare the current and previous state of the button
	int buttonState=0;
	int prev_buttonState=0;
	
	
	while(1){
		//check if the pushbutton is pressed. Turn LED on/off accordingly:
		buttonState = GPIO_read(BUTTON_PIN);

		//If the button is not pressed, turn off the LED.
		if(buttonState)	GPIO_write(LED_PIN, LOW);
		
		//Set it so that the state changes only when the button is pressed.
		if(buttonState==LOW && prev_buttonState==HIGH)
		{
			//If the LED is on, turn it off; if it is off, turn it on.
			buttonState=~(buttonState);
			GPIO_write(LED_PIN, buttonState);
		}

		//Store the current state of the button in the previous state variable.
		prev_buttonState=buttonState;
	}
}