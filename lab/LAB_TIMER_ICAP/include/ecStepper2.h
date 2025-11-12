/**
******************************************************************************
* @author  Yechan Kim
* @brief   Embedded Controller:  EC_HAL_for_student_exercise 
******************************************************************************
*/


#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecSysTick2.h"
			
#ifndef __EC_STEPPER2_H
#define __EC_STEPPER2_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//State mode
#define HALF 0
#define FULL 1	 

#define FULL_PULSE_PER_ROTATION 2048
#define HALF_PULSE_PER_ROTATION 4096
	 
/* Stepper Motor */
//stepper motor function

typedef struct{
	PinName_t pin1;
	PinName_t pin2;
	PinName_t pin3;
	PinName_t pin4;
	uint32_t _step_num;
} Stepper_t;



	 
void Stepper_init(PinName_t pinName1, PinName_t pinName2, PinName_t pinName3, PinName_t pinName4);
void Stepper_setSpeed(long whatSpeed);
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode); 
void Stepper_stop(void);
void Stepper_pinOut (uint32_t state, uint32_t mode);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __EC_STEPPER2_H
