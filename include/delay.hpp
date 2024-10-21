#pragma once //исходный файл при компиляции подключался строго один раз
#include <stm32f1xx.h>

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);