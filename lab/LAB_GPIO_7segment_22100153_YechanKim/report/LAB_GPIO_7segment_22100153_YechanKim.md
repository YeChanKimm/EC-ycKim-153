# LAB: GPIO Digital InOut 7-segment

**Date:** 2025-09-23

**Author:** Yechan Kim

**Github:** https://github.com/YeChanKimm/EC-ycKim-153

**Demo Video:** 

**PDF version:** 1.1



## Introduction

In this lab, the goal is to create a simple program to control a 7-segment display to show a decimal number (0~9) that increases by pressing a push-button.

The flowchart for the overall lab is as follows:

![Flowchart](.\img\OverallFlowchart.png)

### Requirement

**Hardware**

- MCU
  - NUCLEO-F411RE
- Eval Board

**Software**

- PlatformIO, CMSIS, EC_HAL library



## Exercise

| **Port/Pin**   | **Description**              | **Register setting**          |
| -------------- | ---------------------------- | ----------------------------- |
| Port A Pin 5   | Clear Pin5 mode              | GPIOA->MODER &=~(3<<(5*2))    |
| Port A Pin 5   | Set Pin5 mode = Output       | GPIOA->MODER \|=(1<<(5*2))    |
| Port A Pin 6   | Clear Pin6 mode              | GPIOA->MODER &=~(3<<(6*2))    |
| Port A Pin 6   | Set Pin6 mode = Output       | GPIOA->MODER \|=(1<<(5*2))    |
| Port A Pin Y   | Clear PinY mode              | GPIOA->MODER &=~(1<<(Y*2))    |
| Port A Pin 5~9 | Clear Pin5~9 mode            | GPIOA->MODER &=~(1023<<(5*2)) |
|                | Set Pin5~9 mode = Output     | GPIOA->MODER \|=(1023<<(5*2)) |
| Port X Pin Y   | Clear Pin Y mode             | GPIOX->MODER &=~(1<<(Y*2))    |
|                | Set Pin Y mode = Output      | GPIOX->MODER \|=(1<<(Y*2))    |
| Port A Pin5    | Set Pin5 otype=push-pull     | GPIOA->OTYPER \|=(1<<5)       |
| Port A PinY    | Set PinY otype=push-pull     | GPIOA->OTYPER \|=(1<<Y)       |
| Port A Pin5    | Set Pin5 ospeed=Fast         | GPIOA->OSPEEDR \|=(2<<5)      |
| Port A PinY    | Set PinY ospeed=Fast         | GPIOA->OSPEEDR \|=(2<<Y)      |
| Port A Pin 5   | Set Pin5 PUPD=no pullup/down | GPIOA->PUPDR\|=(0<<(5*2))     |
| Port A Pin Y   | Set PinY PUPD=no pullup/down | GPIOA->PUPDR\|=(0<<(Y*2))     |



## Problem 0: Connection of 7-Segment Display and Decoder

### Procedure

For problem 0, the goal is to display one number on 7 segment without button. 