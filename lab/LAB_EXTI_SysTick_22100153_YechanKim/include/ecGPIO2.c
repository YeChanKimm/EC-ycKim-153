/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : YechanKim
Student Number   : 22100153
Created          : 09-16-2025

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecPinNames.h"

void GPIO_init(PinName_t pinName, uint32_t mode){     
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName, &Port, &pin);
	
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	
	if (Port == GPIOD)
		RCC_GPIOD_enable();
	
	if (Port == GPIOE)
		RCC_GPIOE_enable();
	
	
	if (Port == GPIOH)
		RCC_GPIOH_enable();
	
	GPIO_mode(pinName, mode);
}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(PinName_t pinName, uint32_t mode){
   GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode<<(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(PinName_t pinName, int speed){

	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName,&Port,&pin);

	//Clear
	Port->OSPEEDR &=~(3UL<<(2*pin));
	
	//Write
	Port->OSPEEDR |=  (speed<<(pin*2));
	
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(PinName_t pinName, int type){
   	
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName,&Port,&pin);

	//Clear
	Port->OTYPER &=~(1UL<<pin);

	//Write
	Port->OTYPER |= (type<<pin);

}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(PinName_t pinName, int pupd){
   
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName,&Port,&pin);

	//Clear
	Port->PUPDR &=~(3UL<<(2*pin));

	//Write
	Port->PUPDR |=  (pupd<<(pin*2));
	
}

int GPIO_read(PinName_t pinName){

	GPIO_TypeDef *Port;
    unsigned int pin;
    ecPinmap(pinName, &Port, &pin);

	//read the bit
	return (Port->IDR >> pin) & (1);	

}

void GPIO_write(PinName_t pinName, int Output){
	GPIO_TypeDef *Port;
    unsigned int pin;
    ecPinmap(pinName, &Port, &pin);

	//Clear
	Port->ODR &=~(1<<pin);

	//Write
	Port->ODR|=((Output&1)<<pin);


}


// Initialize DOUT pins for 7 segment leds
void seven_seg_FND_init(int mode, int otype, int pupd, int ospeed){	
     //pin name array; 0~7: pin address, 8~11: display address. 
    PinName_t pinsFND[12]={PB_7, PB_6, PB_5, PB_4, PB_3, PB_2, PB_1, PB_0, PC_3, PC_4, PA_11, PA_10};

    
    for (int i=0; i<8; ++i) 
    {
        //Set 7 segment pins
        GPIO_init(pinsFND[i], mode);
        GPIO_otype(pinsFND[i], otype);
		GPIO_pupd(pinsFND[i], pupd);
		GPIO_ospeed(pinsFND[i], ospeed);

       
       
    }    
   
    for(int i=8; i<12; ++i) 
    {
        //Set 7 segment displays
        GPIO_init(pinsFND[i], mode);
        GPIO_otype(pinsFND[i], otype);
		GPIO_pupd(pinsFND[i], pupd);
		GPIO_ospeed(pinsFND[i], ospeed);

    
      }
    
    
}

void seven_seg_FND_display(uint8_t  num, uint8_t select)
{

	//pin name array; 0~7: pin address, 8~11: display address.
    PinName_t pinsFND[12]={PB_7, PB_6, PB_5, PB_4, PB_3, PB_2, PB_1, PB_0, PC_3, PC_4, PA_11, PA_10};
    
	//Write HIGH on selected display board. 
    GPIO_write(pinsFND[8+select], HIGH);
    
    //Map the numbers 0 to 9 to the corresponding pins on the 7-segment display.
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

    //Using bitwise operations, write 0 or 1 to the display to show the selected number.
    for(int i=0; i<8; ++i)
    {
        GPIO_write(pinsFND[7-i], (decoder_number[num]>>i)&0x01);
    }

}

void seven_seg_FND_display_OneDigit(int  num, int select)
{

	//pin name array; 0~7: pin address, 8~11: display address.
    PinName_t pinsFND[12]={PB_7, PB_6, PB_5, PB_4, PB_3, PB_2, PB_1, PB_0, PC_3, PC_4, PA_11, PA_10};
    
	//Write HIGH on selected display board. 
    
	
	GPIO_write(pinsFND[8+select], LOW);
    
    //Map the numbers 0 to 9 to the corresponding pins on the 7-segment display.
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
	
	int one_digit=num%10;
    //Using bitwise operations, write 0 or 1 to the display to show the selected number.
    for(int i=0; i<8; ++i)
    {
		GPIO_write(pinsFND[7-i], (decoder_number[one_digit]>>i)&0x01);
    }
	GPIO_write(pinsFND[9+select], HIGH);

    
}

void seven_seg_FND_display_TenDigit(int  num, int select)
{
	
	//pin name array; 0~7: pin address, 8~11: display address.
    PinName_t pinsFND[12]={PB_7, PB_6, PB_5, PB_4, PB_3, PB_2, PB_1, PB_0, PC_3, PC_4, PA_11, PA_10};
    
	//Write HIGH on selected display board. 
    
	
	GPIO_write(pinsFND[9+select], LOW);
    
    //Map the numbers 0 to 9 to the corresponding pins on the 7-segment display.
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
	
	int ten_digit=num/10;
    //Using bitwise operations, write 0 or 1 to the display to show the selected number.
    for(int i=0; i<8; ++i)
    {
		GPIO_write(pinsFND[7-i], (decoder_number[ten_digit]>>i)&0x01);
    }

	GPIO_write(pinsFND[8+select], HIGH);
    
	

}



void seven_seg_FND_display_Final(uint8_t  num, uint8_t select)
{
    seven_seg_FND_display_TenDigit(num,select);
    delay_ms(5);
    seven_seg_FND_display_OneDigit(num,select);
    delay_ms(5);

}