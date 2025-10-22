# EC-ycKim-153

## Embedded Controller - STM32F411 Driver Library

Written by: Yechan Kim

Program: C

IDE/Compiler: Keil uVision 5

OS: WIn11

MCU: STM32F411RE, Nucleo-64

**Table of contents**

- GPIO Digital In/out
  - ecGPIO2.h
  - ecPinNames.h
  - ecRCC2.h

#### 

## #include "ecGPIO2.h"

It combines the port name and pin number into a single variable and maps it to the actual register address.

```c
/*----------------------------------------------------------------\
@ Embedded Controller by Yechan Kim - Handong Global University
Author           : Yechan Kim
Created          : 09-24-2025
Modified         : 10-06-2025
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
void seven_seg_FND_display_OneDigit(uint8_t  num, uint8_t select);
void seven_seg_FND_display_TenDigit(uint8_t  num, uint8_t select);
void seven_seg_FND_display_Final(uint8_t  num, uint8_t select);
void HAL_SYSTICK(void);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __ECGPIO2_H
```

#### GPIO_init()

Initializes GPIO pins with default setting and Enables GPIO Clock. Mode: In/Out/AF/Analog

```c
void GPIO_init(PinName_t pinName, uint32_t mode);
```

**Parameters** 

- **pinName**: Port and Pin name(i.g PA_4)
- **mode**: INPUT (0), OUTPUT (1), AF(02), ANALOG (03)

**Example code**

```c
void GPIO_mode(PA_4, OUTPUT);
```

#### GPIO_mode()

Configures GPIO pin modes: In/Out/AF/Analog

```c
void GPIO_init(PinName_t pinName, int mode);
```

**Parameters**

- **pinName:** Port and Pin name(i.g PA_4)
- **mode**: INPUT (0), OUTPUT (1), AF(02), ANALOG (03)

**Example code**

```c
void GPIO_mode(PA_4, OUTPUT);
```

#### GPIO_write()

Write the data to GPIO pin: High, Low

```c
void GPIO_write(PinName_t pinName, int Output);
```

**Parameters**

- **pinName:** Port and Pin name(i.g PA_4)
- **Output**: LOW(0), HIGH(1)

**Example code**

```c
void GPIO_mode(PA_4, 1);// 1: High
```

#### GPIO_read()

Read the data from GPIO pin

```c
int GPIO_read(PinName_t pinName);
```

**Parameters**

- **pinName:** Port and Pin name(i.g PA_4)

**Example code**

```c
GPIO_read(GPIOC, 13);
```

#### GPIO_ospeed()

Configures output speed of GPIO pin : Low, Mid, Fast, High

```c
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
```

**Parameters**

- **pinName:** Port and Pin name(i.g PA_4)
- **speed**: LOW_SPEED(0), MID_SPEED(1), FAST_SPEED(2) , HIGH_SPEED(3)

**Example code**

```
GPIO_ospeed(PA_5, 2);  // 2: FAST_SPEED
```

#### GPIO_otype()

Configures output type of GPIO pin: Push-Pull / Open-Drain

```c
void GPIO_otype(PinName_t pinName, int type);
```

**Parameters**

- **pinName:** Port and Pin name(i.g PA_4)
- **type**: PUSH_PULL(0), OPEN_DRAIN(1)

**Example code**

```c
GPIO_otype(PA_5, 0);  //// 0: Push-Pull
```

#### GPIO_pupdr()

Configures Pull-up/Pull-down mode of GPIO pin: No Pull-up, Pull-down/ Pull-up/ Pull-down/ Reserved

```c
void GPIO_pupdr(PinName_t pinName, int pupd);
```

**Parameters**

- **pinName:** Port and Pin name(i.g PA_4)
- **pupd**: NO_PUPD(0), PULL_UP(1), PULL_DOWN(2), RESERVED(3)

**Example code**

```c
GPIO_pupdr(PA_5, 0);// 0: No Pull-up, Pull-down
```

#### seven_seg_FND_init()

Bellow is `seven_seg_FND_init()` which initialize each 7 segment displays and pins.

The address of the display and the pins are declared in array form. 

```c
void seven_seg_FND_init(int mode, int otype, int pupd, int ospeed);
```

**Parameters**

- **mode**: INPUT (0), OUTPUT (1), AF(02), ANALOG (03)
- **type**: PUSH_PULL(0), OPEN_DRAIN(1)
- **pupd**: NO_PUPD(0), PULL_UP(1), PULL_DOWN(2), RESERVED(3)
- **speed**: LOW_SPEED(0), MID_SPEED(1), FAST_SPEED(2) , HIGH_SPEED(3)

**Example code**

```c
seven_seg_FND_init(OUTPUT, PUSH_PULL, NO_PUPD, MEDIUM_SPEED); 
```

#### seven_seg_FND_display()

Using bitwise operations, a number from 0 to 9 is shown on the selected display. The address of the display and the pins are declared in array form. 

```c
void seven_seg_FND_display(uint8_t  num, uint8_t select);
```

**Parameters**

- **num**: Number to display(0~9)
- **select**: The selected display among the four displays

**Example code**

```c
//Selected number and display
uint8_t numDisplay=9;
uint8_t selectFND=3;

//Display the number on the selected display
while (1) {
        seven_seg_FND_display(numDisplay,selectFND);
}
```

#### seven_seg_FND_display_OneDigit();

This function is used to display the ones digit while showing the tens digit on the display.

```c
void seven_seg_FND_display_OneDigit(uint8_t  num, uint8_t  select)
```

**Parameters**

- **num**: Number to display(0~9)
- **select**: The selected display among the four displays

**example code**

```c
//Selected number and display
uint8_t numDisplay=9;
uint8_t selectFND=3;

//Display the number on the selected display
while (1) {
        seven_seg_FND_display_OneDigit(numDisplay,selectFND);
}
```

#### seven_seg_FND_display_TenDigit();

This function is used to display the tens digit when showing the tens place on the display.

```c
void seven_seg_FND_display_TenDigit(uint8_t  num, uint8_t  select)
```

**Parameters**

- **num**: Number to display(0~9)
- **select**: The selected display among the four displays

**example code**

```c
//Selected number and display
uint8_t numDisplay=9;
uint8_t selectFND=3;

//Display the number on the selected display
while (1) {
        seven_seg_FND_display_OneDigit(numDisplay,selectFND);
}
```

#### seven_seg_FND_display_Final();

This function combines the ones and tens digits into a single display when showing the tens place.

```c
void seven_seg_FND_display_Final(uint8_t  num, uint8_t select);
```

**Parameters**

- **num**: Number to display(0~9)
- **select**: The selected display among the four displays

**example code**

```c
//Selected number and display
uint8_t numDisplay=9;
uint8_t selectFND=3;

//Display the number on the selected display
while (1) {
        seven_seg_FND_display_Final(numDisplay,selectFND);
}
```

## #include "ecPinNames.h"

It combines the port name and pin number into a single variable and maps it to the actual register address.

```c
#ifndef EC_PINNAMES_H
#define EC_PINNAMES_H

#include "stm32f411xe.h"

#ifdef __cplusplus
extern "C" {
#endif


// Bitwise Macro Definition
#define BIT_SET(REG, BIT)          ((REG) |= 1<< (BIT))
#define BIT_CLEAR(REG, BIT)     ((REG) &= ~1<<(BIT))
#define BIT_READ(REG, BIT)      ((REG)>>BIT & (1))
#define BITS_SET(REG, BIT,NUM)     ((REG) |= NUM<< (BIT))
#define BITS_CLEAR(REG, BIT,NUM)   ((REG) &= ~(NUM<< (BIT)))
//#define BITS_CLEAR(REG, BIT,NUM)   ((REG) &= ~((0x1<< NUM)-1)<<(BIT))


// Pinname Config
typedef enum {
    PortA = 0,
    PortB = 1,
    PortC = 2,
    PortD = 3,
    PortE = 4,
    PortF = 5,
    PortG = 6,
    PortH = 7,
    PortI = 8,
    PortJ = 9,
    PortK = 10
} PortName_t;



typedef enum {
    PA_0  = 0x00,
    PA_1  = 0x01,    
    PA_2  = 0x02,
    PA_3  = 0x03,
    PA_4  = 0x04,    
    PA_5  = 0x05,
    PA_6  = 0x06,
    PA_7  = 0x07,
    PA_8  = 0x08,
    PA_9  = 0x09,
    PA_10 = 0x0A,
    PA_11 = 0x0B,
    PA_12 = 0x0C,
    PA_13 = 0x0D,
    PA_14 = 0x0E,
    PA_15 = 0x0F,

    PB_0  = 0x10,
    PB_1  = 0x11,
    PB_2  = 0x12,
    PB_3  = 0x13,
    PB_4  = 0x14,
    PB_5  = 0x15,
    PB_6  = 0x16,
    PB_7  = 0x17,
    PB_8  = 0x18,
    PB_9  = 0x19,
    PB_10 = 0x1A,
    PB_12 = 0x1C,
    PB_13 = 0x1D,
    PB_14 = 0x1E,
    PB_15 = 0x1F,

    PC_0  = 0x20,
    PC_1  = 0x21,
    PC_2  = 0x22,
    PC_3  = 0x23,
    PC_4  = 0x24,
    PC_5  = 0x25,
    PC_6  = 0x26,
    PC_7  = 0x27,
    PC_8  = 0x28,
    PC_9  = 0x29,
    PC_10 = 0x2A,
    PC_11 = 0x2B,
    PC_12 = 0x2C,
    PC_13 = 0x2D,
    PC_14 = 0x2E,
    PC_15 = 0x2F,

    PD_2  = 0x32,

    PH_0  = 0x70,
    PH_1  = 0x71,


    // Arduino connector namings
    A0          = PA_0,
    A1          = PA_1,
    A2          = PA_4,
    A3          = PB_0,
    A4          = PC_1,
    A5          = PC_0,
    D0          = PA_3,
    D1          = PA_2,
    D2          = PA_10,
    D3          = PB_3,
    D4          = PB_5,
    D5          = PB_4,
    D6          = PB_10,
    D7          = PA_8,
    D8          = PA_9,
    D9          = PC_7,
    D10         = PB_6,
    D11         = PA_7,
    D12         = PA_6,
    D13         = PA_5,
    D14         = PB_9,
    D15         = PB_8,


    // Not connected
    NC = (int)0xFFFFFFFF
} PinName_t;


void ecPinmap(PinName_t pinName, GPIO_TypeDef **GPIOx, unsigned int *pin);




#ifdef __cplusplus
}
#endif

#endif
```

#### ecPinmap()

Maps a combined port/pin identifier to its GPIO port address and pin number.

```c
void ecPinmap(PinName_t pinName, GPIO_TypeDef **GPIOx, unsigned int *pin);
```

**Parameters**

- **pinName:** Port and Pin name(i.g PA_4)
- **GPIOx**: Pointer to the base address of a specific GPIO port
- **pin**:  Variable that stores the specific pin number within the selected GPIO port

**Example code**

```c
ecPinmap(pinName,&Port,&pin);
```

## #include "ecRCC2.h"

It enables clock of the each port. 

```c
#ifndef __EC_RCC2_H
#define __EC_RCC2_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//#include "stm32f411xe.h"

void RCC_HSI_init(void);
void RCC_PLL_init(void);
void RCC_GPIOA_enable(void);
void RCC_GPIOB_enable(void);
void RCC_GPIOC_enable(void);
void RCC_GPIOD_enable(void);
void RCC_GPIOE_enable(void);
void RCC_GPIOH_enable(void);

extern int EC_SYSCL;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __EC_RCC2_H
```

#### RCC_HSI_init()

It initializes the High-Speed Internal (HSI) 16 MHz clock and configures it as the system clock source for the STM32F4.

```c
void RCC_HSI_init();
```

**Example code**

```c
RCC_HSI_init();
```

#### 

#### RCC_PLL_init()

configures and enables the PLL using HSI to set the STM32F4 system clock to 84 MHz, with proper FLASH latency and bus prescalers.

```c
void RCC_PLL_init()
```

**Example code**

```
RCC_PLL_init();
```

#### 

#### RCC_GPIOA_enable~ RCC_GPIOH_enable

These functions enable the peripheral clock for GPIO port A~H(except F, G) by setting the corresponding bit in the RCC AHB1ENR register.

```c
void RCC_GPIOA_enable(void);
void RCC_GPIOB_enable(void);
void RCC_GPIOC_enable(void);
void RCC_GPIOD_enable(void);
void RCC_GPIOE_enable(void);
void RCC_GPIOH_enable(void);
```

**Example code**

```c
RCC_GPIOA_enable();
```

## #include "ecEXTI2.h"

It controls EXTI registers. 

```c
#ifndef __EC_EXTI2_H
#define __EC_EXTI2_H

#include "stm32f411xe.h"
#include "ecPinNames.h"

#define FALL 0
#define RISE 1
#define BOTH 2



#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void EXTI_init(PinName_t pinName, int trig_type,int priority);
void EXTI_enable(PinName_t pinName);
void EXTI_disable(PinName_t pinName);
uint32_t is_pending_EXTI(PinName_t pinName);
void clear_pending_EXTI(PinName_t pinName);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __EC_EXTI2_H
```

#### EXTI_init()

This function initializes an external interrupt (EXTI) for a specific GPIO pin. It connects the selected pin to the EXTI line through the system configuration controller (SYSCFG), sets the trigger condition (rising, falling, or both edges), and enables the interrupt mask to allow the signal. Finally, it configures the corresponding interrupt line in the NVIC by setting its priority and enabling the interrupt request.

```c
void EXTI_init(PinName_t pinName, int trig_type,int priority);
```

**Parameters**

- **pinName**: Port and Pin name(i.g PA_4)
- **trig_type**: FALL, RISING, BOTH
- **priority**: Interupt Priority

**Example Code**

```c
EXTI_init(PA_4, FALL, 0);
```

#### EXTI_enable()

This function **enables** the external interrupt for the specified GPIO pin.  
It sets the corresponding bit in the **Interrupt Mask Register (IMR)**, allowing the EXTI line to generate interrupt requests.

```c
void EXTI_enable(PinName_t pinName);
```

**Parameters**

- **pinName**: Port and Pin name(i.g PA_4)

**Example Code**

```c
EXTI_enable(PA_4);
```

#### EXTI_disable()

This function disables the external interrupt for the specified GPIO pin.  It clears the corresponding bit in the IMR, masking the EXTI line so it no longer triggers interrupts.

```c
void EXTI_disable(PinName_t pinName);
```

**Parameters**

- **pinName**: Port and Pin name(i.g PA_4)

**Example Code**

```c
EXTI_disable(PA_4);
```

#### is_pending_EXTI()

This function checks whether an interrupt pending flag is set for the given pin.  It reads the Pending Register (PR) to detect if an interrupt has occurred on that EXTI line.

```c
uint32_t is_pending_EXTI(PinName_t pinName)
```

**Parameters**

- **pinName**: Port and Pin name(i.g PA_4)

**Example Code**

```c
is_pending_EXTI(PA_4)
```

#### clear_pending_EXTI()

This function clears the pending interrupt flag for the specified pin.  Writing a ‘1’ to the corresponding bit in the Pending Register (PR) resets the pending state, indicating that the interrupt has been handled.

```c
void clear_pending_EXTI(PinName_t pinName)
```

**Parameters**

- **pinName**: Port and Pin name(i.g PA_4)

**Example Code**

```c
clear_pending_EXTI(PA_4)
```

## #include "ecSysTick2.h"

This file is for controlling SysTick registers. 

```c
#ifndef __EC_SYSTICK2_H
#define __EC_SYSTICK2_H

#include "stm32f4xx.h"
#include "ecRCC2.h"
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void SysTick_init(uint32_t msec);
void SysTick_Handler(void);
void SysTick_counter();
void delay_ms(uint32_t msec);
void SysTick_reset(void);
uint32_t SysTick_val(void);
void SysTick_enable(void);
void SysTick_disable (void);
uint32_t crtNum_ms(void);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __EC_SYSTICK2_H
```

#### SysTick_init()

Initializes the SysTick timer to generate periodic interrupts at the specified time interval (`msec`).  It selects the processor clock source, sets the reload value, clears the counter, enables SysTick interrupts, and activates the timer.

```c
void SysTick_init(uint32_t msec);
```

**Parameters**

- **msec**: Interupt period(msec)

**Example Code**

```c
SysTick_init(1)
```

#### delay_ms()

Implements a blocking delay function in milliseconds.  It waits until the specified time (`mesc`) has passed by continuously checking the difference in `msTicks`.

```c
void delay_ms(uint32_t msec);
```

**Parameters**

- **msec**: Delay period(msec)

**Example Code**

```c
delay_ms(1000);
```

#### SysTick_val()

Returns the current value of the SysTick counter (`VAL` register).  Useful for reading the timer’s live count state.

```c
uint32_t SysTick_val(void);
```

**Example Code**

```c
int value=SysTick_val();
```

#### SysTick_reset

Resets the current SysTick counter value by clearing the VAL register.  This restarts the counting cycle.

```c
void SysTick_reset(void);
```

**Example Code**

```c
 SysTick_reset();
```

#### SysTick_enable(), SysTick_disable()

They enble/disable the system timer in CTRL register. 

```c
void SysTick_enable(void);
void SysTick_disable (void);
```

**Example Code**

```c
SysTick_enable();
SysTick_disable();
```

#### SysTick_Handler, SysTick_counter()

For    `Systick_Handler()`, it is the interrupt service routine (ISR) for SysTick.  It is automatically called each time the SysTick timer counts down to zero and calls `SysTick_counter()`.

For `SysTick_counter()` , Increments the global millisecond counter `msTicks` every time a SysTick interrupt occurs.  Used for timing and delay functions.

```c
void SysTick_Handler(void);
void SysTick_counter(void);
```

**Example Code**

```c
SysTick_Handler();
SysTick_counter();
```

## #include "ecTIME2.h"

This header file controls timers. 

```c
/**
******************************************************************************
* @author  Yechan Kim
* @brief   Embedded Controller:  EC_HAL_for_student_exercise 
******************************************************************************
*/
#ifndef __EC_TIM2_H 
#define __EC_TIM2_H
#include "stm32f411xe.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Timer Configuration */
// Initialize  TIMERx
void TIM_init(TIM_TypeDef* TIMx);

// Default Setting:  TIM_period_ms(TIMx, 1 msec) with Counter_Clk 100kHz / PSC=840-1, ARR=100-1

//Choose Timer Update Period  (a) msec or  (b) usec
void TIM_period(TIM_TypeDef* TIMx, uint32_t msec);    // msec of TimerUEV with Counter_Clk 100kHz / PSC=840, ARR=100*msec
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);  // usec of TimerUEV with Counter_Clk 1MHz / PSC=84, ARR=100*msec


//Initialize TIM_UI with TIMERx
void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec); 

// Start by Enabling TIM_UI 
void TIM_UI_enable(TIM_TypeDef* TIMx);
void TIM_UI_disable(TIM_TypeDef* TIMx);

uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __EC_TIM2_H 
```

#### TIM_init()

This function initializes a timer (TIMx) on the STM32 by enabling its clock, setting it as an upcounter, and defining its counting period in microseconds. The `TIM_period_us()` function configures the prescaler (PSC) and auto-reload register (ARR) so the timer runs at 1 MHz, allowing precise timing from 1 µs up to about 65 ms.

```c
void TIM_init(TIM_TypeDef* TIMx);
```

**Parameters**

- **TIMx**: Timer number

**Example Code**

```c
TIM_init(TIM2);
```

#### TIM_period()

This function sets the timer period by configuring the prescaler and auto-reload registers so the timer runs at 100 kHz, allowing timing intervals from about 1 ms up to 6.5 seconds.

There are 2 way of setting timer period. 

- **TIM_period_ms()** : setting period in milliseconds. 

- **TIM_period_us()** :  setting period in microseconds

default function `TIM_period()` is in milliseconds. 

```c
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);
void TIM_period(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters**

- **TIMx** : Timer number

- **msec/usec**: timer period

**Example Code**

```c
TIM_period_ms(TIM2, 1);
TIM_period_us(TIM3, 1000);
TIM_period(TIM4, 1);
```

#### TIM_UI_init()

This function initializes a timer with a specified millisecond period, enables its update interrupt, and configures the NVIC to handle the corresponding timer interrupt with priority level 2.

```c
void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters**

- **TIMx** : Timer number

- **msec**: timer period

**Example Code**

```c
TIM_UI_init(TIM2, 1);
```

#### TIM_UI_enable()/TIM_UI_disable()

These two functions enable or disable the timer’s update interrupt by setting or clearing bit 0 of the DIER register, which controls whether the timer generates an interrupt on update events.

```c
void TIM_UI_enable(TIM_TypeDef* TIMx);
void TIM_UI_disable(TIM_TypeDef* TIMx);
```

**Parameters**

- **TIMx** : Timer number

**Example Code**

```c
TIM_UI_enable(TIM2);
TIM_UI_disable(TIM2);
```

#### is_UIF()/clear_UIF()

These two functions handle the timer’s update interrupt flag (UIF): `is_UIF()` checks if the flag is set, while `clear_UIF()` resets it by clearing bit 0 of the status register (SR).

```c
uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);
```

**Parameters**

- **TIMx** : Timer number

**Example Code**

```c
if(is_UIF(TIM2) clear_UIF(TIM2);
```





## #include "ecPWM2.h"

This header controls PWM using timers. 

```c
/**
******************************************************************************
* @author  Yechan Kim
* @brief   Embedded Controller:  EC_HAL_for_student_exercise 
******************************************************************************
*/

#ifndef __EC_PWM2_H
#define __EC_PWM2_H

#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecTIM2.h"
#include "ecPinNames.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* PWM Configuration using PinName_t Structure */

/* PWM initialization */
// Default: 84MHz PLL, 1MHz CK_CNT, 50% duty ratio, 1msec period
void PWM_init(PinName_t pinName);
void PWM_init_AF3(PinName_t pinName);
void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);


/* PWM PERIOD SETUP */
// allowable range for msec:  1~2,000
void PWM_period(PinName_t pinName,  uint32_t msec);	
void PWM_period_ms(PinName_t pinName,  uint32_t msec);	// same as PWM_period()
// allowable range for usec:  1~1,000
void PWM_period_us(PinName_t pinName, uint32_t usec);


/* DUTY RATIO SETUP */
// High Pulse width in msec
void PWM_pulsewidth(PinName_t pinName, double pulse_width_ms);
void PWM_pulsewidth_ms(PinName_t pinName, double pulse_width_ms);  // same as void PWM_pulsewidth
void PWM_pulsewidth_us(PinName_t pinName, double pulse_width_us);
// Duty ratio 0~1.0;
void PWM_duty(PinName_t pinName, float duty);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __EC_PWM2_H
```

#### PWM_init()

This function initializes PWM output for a specified pin by mapping it to its corresponding timer and channel, setting the GPIO to alternate function mode, configuring the timer for PWM mode (default 50% duty cycle), and enabling the timer counter to generate PWM signals.

```c
void PWM_init(PinName_t pinName);
```

**Parameters**

- **pinName**: Port and Pin name(i.g PA_4)

**Example code**

```c
PWM_init(PA_0);
```



#### PWM_pinmap()

This function maps a given pin to its corresponding timer (`TIMx`) and channel (`chN`) based on its GPIO port and pin number, allowing the correct timer configuration for PWM signal generation.

```c
void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);
```

**Parameters**

- **pinName**: Port and Pin name(i.g PA_4)
- **TIMx** : Timer number
- **chN** : channel number 

**Example code**

```c
PWM_pinmap(pinName, &TIMx, &chN);
```





#### PWM_period()

This function sets the PWM signal period by identifying the corresponding timer from the given pin and configuring its timer period

There are 2 way of setting PWM period:

- **PWM_period_ms()** : setting period in milliseconds.

- **PWM_period_us()** : setting period in microseconds

default function `PWM_period()` is in milliseconds.

```c
void PWM_period(PinName_t pinName,  uint32_t msec);	
void PWM_period_ms(PinName_t pinName,  uint32_t msec);	// same as PWM_period()
void PWM_period_us(PinName_t pinName, uint32_t usec);
```

**Parameters**

- **pinName**: Port and Pin name

- **msec**: timer period

**Example code**

```c
PWM_period(PA_0, 1);
PWM_period_ms(PA_0, 1);
PWM_period_us(PA_0, 1000);
```



#### PWM_pulsewidth()

This function sets the PWM pulse width (high time) by calculating the corresponding timer compare value based on the system clock and prescaler, then updating the appropriate capture/compare register (CCR) for the selected channel.

There are 2 way of setting PWM period:

- **PWM_pulsewidth_ms()** : setting period in milliseconds.

- **PWM_pulsewidth_us()** : setting period in microseconds

```c
void PWM_pulsewidth(PinName_t pinName, double pulse_width_ms);
void PWM_pulsewidth_ms(PinName_t pinName, double pulse_width_ms);  // same as void PWM_pulsewidth
void PWM_pulsewidth_us(PinName_t pinName, double pulse_width_us);
```

**Parameters**

- **pinName**: Port and Pin name

- **pulse_width_ms/pulse_width_us**: Pulse period

**Example code**

```c
PWM_pulsewidth(PA_0, 0.2);
PWM_pulsewidth_ms(PA_0, 0.2);  // same as void PWM_pulsewidth
PWM_pulsewidth_us(PA_0, 200)
```



#### PWM_duty()

This function sets the PWM duty cycle by calculating the compare value as a fraction of the timer’s auto-reload value (ARR) and updating the corresponding capture/compare register (CCR) for the selected channel.

```c
void PWM_duty(PinName_t pinName, float duty);
```

**Parameters**

- **pinName**: Port and Pin name
- **duty** : Duty ratio of the PWM

**Example code**

```c
PWM_duty(PA_), 0.5);
```
