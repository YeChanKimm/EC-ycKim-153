# EC-ycKim-153



## Embedded Controller - STM32F411 Driver Library

Written by: Yechan Kim

Program: C

IDE/Compiler: Keil uVision 5

OS: WIn11

MCU: STM32F411RE, Nucleo-64



**Table of contents**

- GPIO Digital In/out
  - Header files
  - GPIO_init()
  - GPIO_mode()
  - GPIO_write()
  - GPIO_read()
  - GPIO_ospeed()
  - GPIO_otype()
  - GPIO_pupdr()

### GPIO Digital InOut

#### `#include "ecGPIO2.h"`

It combines the port name and pin number into a single variable and maps it to the actual register address.

```c
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
void GPIO_pupdr(PinName_t pinName, int type);
```

**Parameters**

- **pinName:** Port and Pin name(i.g PA_4)
- **speed**: NO_PUPD(0), PULL_UP(1), PULL_DOWN(2), RESERVED(3)

**Example code**

```c
GPIO_pupdr(PA_5, 0);// 0: No Pull-up, Pull-down
```



#### `#include "ecPinNames.h"`

It combines the port name and pin number into a single variable and maps it to the actual register address.

```c
#ifndef EC_PINNAMES_H
#define EC_PINNAMES_H

#include "stm32f411xe.h"

#ifdef __cplusplus
extern "C" {
#endif


// Bitwise Macro Definition
#define BIT_SET(REG, BIT)      	((REG) |= 1<< (BIT))
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





### `#include "ecRCC2.h"`

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



#### RCC_PLL_init()

configures and enables the PLL using HSI to set the STM32F4 system clock to 84 MHz, with proper FLASH latency and bus prescalers.

```c
void RCC_PLL_init()
```

**Example code**

```
RCC_PLL_init();
```



####  RCC_GPIOA_enable~ RCC_GPIOH_enable

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



