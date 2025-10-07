/*----------------------------------------------------------------\
@ Embedded Controller by Yechan Kim - Handong Global University
Author           : Yechan Kim
Created          : 09-24-2025
Language/ver     : C in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/


#ifndef __ECGPIO2_H
#define __ECGPIO2_H


#include "stm32f411xe.h"
#include "ecPinNames.h"
#include "ecRCC2.h"


#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define HIGH 1
#define LOW  0

//Parameter numbers
#define PULL_DOWN 1
#define PULL_UP 2
#define NO_PUPD 0
#define PUSH_PULL 0
#define OPEN_DRAIN 1
#define LOW_SPEED 0
#define MEDIUM_SPEED 1
#define FAST_SPEED 2
#define HIGH_SPEED 3

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


void GPIO_init(PinName_t pinName, uint32_t mode);  //   
void GPIO_write(PinName_t pinName, int Output);//
int  GPIO_read(PinName_t pinName);//
void GPIO_mode(PinName_t pinName, uint32_t mode);//
void GPIO_ospeed(PinName_t pinName, int speed);//
void GPIO_otype(PinName_t pinName, int type);//
void GPIO_pupd(PinName_t pinName, int pupd);//
void seven_seg_FND_init(int mode, int otype, int pupd, int ospeed);
void seven_seg_FND_display(uint8_t  num, uint8_t select);

void HAL_SYSTICK(void);


 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __ECGPIO2_H
