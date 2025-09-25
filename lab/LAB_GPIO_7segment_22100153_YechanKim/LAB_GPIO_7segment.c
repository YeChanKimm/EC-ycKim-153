#include "ecSTM32F4v2.h"

//The address of the button
#define BUTTON_PIN PA_4


void setup(void){
    // Intialize System Clock
    RCC_HSI_init();

    //Initialize registers: Output mode, Push-Pull, No Pull up/Pull down, Medium Speed
    seven_seg_FND_init(OUTPUT, PUSH_PULL, NO_PUPD, MEDIUM_SPEED); 
};

int main(void) {
    setup();
    
    //Board to display increasing numbers
    uint8_t selectFND=0;

    //Initialize current and previous button state
    int buttonState=LOW;
	int prev_buttonState=HIGH;

    //Set initial number as 0
    int crtNum=0;
    
    while (1) {

        //Read the button register to check if the button is pressed
        buttonState=GPIO_read(BUTTON_PIN);
        
        //When curren state is 'pressed' previous state is 'unpressed'(Edge), increase the number and display it
        if(buttonState==LOW && prev_buttonState==HIGH)
        {
            
            seven_seg_FND_display(crtNum,selectFND);
            crtNum=(crtNum+1)%10;
        }    
        
        //Make curren button state as previous one
        prev_buttonState=buttonState;

        //Delay the system for stabilization 
        for(int i=0; i<10000; ++i);
    }
}

