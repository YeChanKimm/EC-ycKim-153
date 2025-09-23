#include "stm32f4xx.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"

//Parameter numbers
#define PULL_DOWN 1
#define PULL_UP 2
#define PUSH_PULL 0
#define OPEN_DRAIN 1
#define LOW_SPEED 0
#define MEDIUM_SPEED 1
#define FAST_SPEED 2
#define HIGH_SPEED 3

// Initialize 7 DOUT pins for 7 segment leds
void seven_seg_FND_init(void); 

// Select display: 0 to 3
// Display a number 0 - 9 only
void seven_seg_FND_display(uint8_t  num, uint8_t select);
void sevensegment_decoder(uint8_t num);




PinName_t SEVEN_SEG_FND_PIN[8]={0};
PinName_t SEVEN_SEG_FND_SELECT[4]={0};



void setup(void){
    // Intialize System Clock
    RCC_HSI_init();
    seven_seg_FND_init(); 
};

int main(void) {
    setup();
    uint8_t numDisplay=4;
    uint8_t selectFND=3;

    while (1) {
        
        //for(int i=0; i<4; ++i) GPIO_write(SEVEN_SEG_FND_SELECT[i], LOW);
       
        seven_seg_FND_display(numDisplay,selectFND);
      ;
    }
}


// Initialize DOUT pins for 7 segment leds
void seven_seg_FND_init(void){	
    //pin name array
    PinName_t pinsFND[12]={PB_7, PB_6, PB_5, PB_4, PB_3, PB_2, PB_1, PB_0, PC_3, PC_4, PA_11, PA_10};
    for (int i=0; i<8; ++i) 
    {
        GPIO_init(pinsFND[i], OUTPUT);
        GPIO_otype(pinsFND[i], PUSH_PULL);
        SEVEN_SEG_FND_PIN[i]=pinsFND[i];
    }    
   
    for(int i=8; i<12; ++i) 
    {
        GPIO_init(pinsFND[i], OUTPUT);
        GPIO_otype(pinsFND[i], PUSH_PULL);
        SEVEN_SEG_FND_SELECT[i-8]=pinsFND[i];
    }
    
    
}

// Select display: 0 to 3
// Display a number 0 - 9 only

void seven_seg_FND_display(uint8_t  num, uint8_t select)
{

    GPIO_write(SEVEN_SEG_FND_SELECT[select], HIGH);
    

    uint8_t decoder_number[10]={
        0b00111111,//0
        0b00000110,//1 
        0b01011011,//2 
        0b01001111,//3
        0b01100110,//4
        0b01101101,//5
        0b01111101,//6
        0b00000111,//7
        0b01111111,//8
        0b01101111//9
    };

    for(int i=0; i<8; ++i)
    {
        GPIO_write(SEVEN_SEG_FND_PIN[7-i], (decoder_number[num]>>i)&0x01);

    }




}
















// void seven_seg_FND_display(uint8_t  num, uint8_t select)
// {   
//     for(int i=0; i<4; ++i)
//     {
//         GPIO_write(SEVEN_SEG_FND_SELECT[i], LOW);
//     }
    
    
//     GPIO_write(SEVEN_SEG_FND_SELECT[select], HIGH);


//     switch (num)
//     {
//         case 0: 
//             for(int i=0; i<12; ++i)
//             {
//                 if(i==0 || i==1) 
//                 {
//                     continue;
//                 }
//                 GPIO_write(SEVEN_SEG_FND_PIN[i], HIGH);
//              }
//             break;

//         case 1: 
            
//                 GPIO_write(SEVEN_SEG_FND_PIN[5], HIGH);
//                 GPIO_write(SEVEN_SEG_FND_PIN[6], HIGH);
//                 break;
//         case 2: 
//             for(int i=0; i<12; ++i)
//                 {
//                     if(i==0 || i==2 || i==5) 
//                     {
//                         continue;
//                     }
//                     GPIO_write(SEVEN_SEG_FND_PIN[i], HIGH);
//                 }
//             break;

//         case 3: 
//             for(int i=0; i<12; ++i)
//                 {
//                     if(i==0 || i==2 || i==3) 
//                     {
//                         continue;
//                     }
//                     GPIO_write(SEVEN_SEG_FND_PIN[i], HIGH);
//                 }
//             break;

//         case 4: 
//             for(int i=0; i<12; ++i)
//                 {
//                     if(i==0 || i==3 || i==4 || i==7) 
//                     {
//                         continue;
//                     }
//                     GPIO_write(SEVEN_SEG_FND_PIN[i], HIGH);
//                 }
//             break;
                
//         case 5:  
//             for(int i=0; i<12; ++i)
//                 {
//                     if(i==0 || i==3 || i==6) 
//                     {
//                         continue;
//                     }
//                     GPIO_write(SEVEN_SEG_FND_PIN[i], HIGH);
//                 }
//             break;
                
                
//         case 6:
//             for(int i=0; i<12; ++i)
//                 {
//                     if(i==0 || i==6) 
//                     {
//                         continue;
//                     }
//                     GPIO_write(SEVEN_SEG_FND_PIN[i], HIGH);
//                 }
//             break;

//         case 7: 
//             for(int i=0; i<12; ++i)
//                 {
//                     if(i==0 || i==2 || i==3 || i==4) 
//                     {
//                         continue;
//                     }
//                     GPIO_write(SEVEN_SEG_FND_PIN[i], HIGH);
//                 }
//             break;

//         case 8:
//             for(int i=0; i<12; ++i)
//                 {
//                     if(i==0) 
//                     {
//                         continue;
//                     }
//                     GPIO_write(SEVEN_SEG_FND_PIN[i], HIGH);
//                 }   
//             break; 
        
//         case 9: 
//             for(int i=0; i<12; ++i)
//                 {
//                     if(i==0 || i==3 || i==4) 
//                     {
//                         continue;
//                     }
//                     GPIO_write(SEVEN_SEG_FND_PIN[i], HIGH);
//                 }
//             break;    
                
//     }
// }