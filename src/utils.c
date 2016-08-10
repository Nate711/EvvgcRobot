/*
 *  utils.c
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis caat
 */

#include <math.h>
#include "utils.h"
#include "stm32f10x_rcc.h"
#include "comio.h"
#include "pins.h"

void LEDon(void)
{
    GPIO_SetBits(LED1_PORT, LED1_PIN); //LED on
}

void LEDoff(void)
{
    GPIO_ResetBits(LED1_PORT, LED1_PIN); //LED off
}

void LEDtoggle(void)
{
    __disable_irq();
    GPIO_ToggleBits(LED1_PORT, LED1_PIN);
    __enable_irq();
}

void DEBUG_LEDon(void)
{
    GPIO_SetBits(LED2_PORT, LED2_PIN); //LED on
}

void DEBUG_LEDoff(void)
{
    GPIO_ResetBits(LED2_PORT, LED2_PIN); //LED off
}

void DEBUG_LEDtoggle(void)
{
    __disable_irq();
    GPIO_ToggleBits(LED2_PORT, LED2_PIN);
    __enable_irq();
}

void Blink(void)
{
    DEBUG_PutChar('B');

    LEDon();            //blinking led
    Delay_ms(200);
    LEDoff();
    Delay_ms(200);
}

#define STM32_DELAY_US_MULT         12

void Delay_us(unsigned int us)
{
    us *= STM32_DELAY_US_MULT;

    /* fudge for function call overhead  */
    //us--;
    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r"(us)
                 : "r0");
}

void Delay_ms(unsigned int ms)
{
    Delay_us(1000 * ms);
}

float Rad2Deg(float x)
{
    return x * (180.0F / M_PI);
}

float Deg2Rad(float x)
{
    return x * (M_PI / 180.0F);
}

float Round(float x)
{
    if (x >= 0)
    {
        return x + 0.5F;
    }
    else
    {
        return x - 0.5F;
    }
}
