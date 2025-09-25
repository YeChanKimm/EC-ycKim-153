#include "ecSTM32F4v2.h"

//Store the address of initialized pins as a global variable
PinName_t SEVEN_SEG_FND_PIN[8]={0};

//Store the address of initialized display as a global variable
PinName_t SEVEN_SEG_FND_SELECT[4]={0};



void setup(void){
    // Intialize System Clock
    RCC_HSI_init();

    //Initialize registers
    seven_seg_FND_init(OUTPUT, PUSH_PULL, NO_PUPD, MEDIUM_SPEED); 
};

int main(void) {
    
    //Initialize the system
    setup();

    //Selected number and display
    uint8_t numDisplay=9;
    uint8_t selectFND=3;

    //Display the number on the selected display
    while (1) {
        seven_seg_FND_display(numDisplay,selectFND);
    }
}





