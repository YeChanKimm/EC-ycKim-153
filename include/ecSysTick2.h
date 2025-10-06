#ifndef __EC_SYSTICK2_H
#define __EC_SYSTICK2_H

#include "stm32f4xx.h"
#include "ecRCC2.h"
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void SysTick_init(uint32_t msec);
void delay_ms(uint32_t msec);
uint32_t SysTick_val(void);
void SysTick_reset(void);
void SysTick_enable(void);
void SysTick_disable (void);

void SysTick_Handler(void);
void SysTick_counter(void);




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __EC_SYSTICK2_H
