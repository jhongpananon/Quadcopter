/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @ingroup Drivers
 *
 * 20140610: Initial
 */
#ifndef LPC_TIMERS_H__
#define LPC_TIMERS_H__
#ifdef __cplusplus
extern "C" {
#endif
#include <LPC17xx.h>
#include <stdint.h>



/**
 * The type of timers supported by LPC17xx
 */
typedef enum {
    lpc_timer0,
    lpc_timer1,
    lpc_timer2,
    lpc_timer3,
} lpc_timer_t;



/**
 * Enables and starts the timer with the given tick rate
 * @param [in] timer        The timer type
 * @param [in] us_per_tick  The resolution of each tick
 */
static inline void lpc_timer_enable( lpc_timer_t timer, uint32_t us_per_tick)
{
    uint32_t timerMemBases[] = { LPC_TIM0_BASE, LPC_TIM1_BASE, LPC_TIM2_BASE, LPC_TIM3_BASE };
    LPC_TIM_TypeDef *pTimerStruct = (LPC_TIM_TypeDef*) timerMemBases[timer];

    /* Power on the timer, and set the pclk = cpu clock (divide by 1) */
    switch (timer)
    {
        case lpc_timer0: lpc_pconp(pconp_timer0, true); lpc_pclk(pclk_timer0, clkdiv_1); break;
        case lpc_timer1: lpc_pconp(pconp_timer1, true); lpc_pclk(pclk_timer1, clkdiv_1); break;
        case lpc_timer2: lpc_pconp(pconp_timer2, true); lpc_pclk(pclk_timer2, clkdiv_1); break;
        case lpc_timer3: lpc_pconp(pconp_timer3, true); lpc_pclk(pclk_timer3, clkdiv_1); break;
    }

    /* Enable the timer, and increment on PCLK */
    pTimerStruct->TCR = 1;
    pTimerStruct->CTCR = 0;

    /* Set the resolution */
    pTimerStruct->PR = (sys_get_cpu_clock() * us_per_tick) / (1000*1000);
}

/**
 * Get the value of the timer
 * @param [in] timer    The timer type
 */
static inline uint32_t lpc_timer_value_get( lpc_timer_t timer)
{
    uint32_t timerMemBases[] = { LPC_TIM0_BASE, LPC_TIM1_BASE, LPC_TIM2_BASE, LPC_TIM3_BASE };
    LPC_TIM_TypeDef *pTimerStruct = (LPC_TIM_TypeDef*) timerMemBases[timer];
    return (pTimerStruct->TC);
}

/**
 * Set the value of the timer
 * @param [in] timer    The timer type
 * @param [in] value    The value to set.
 */
static inline void lpc_timer_value_set(lpc_timer_t timer, uint32_t value)
{
    uint32_t timerMemBases[] = { LPC_TIM0_BASE, LPC_TIM1_BASE, LPC_TIM2_BASE, LPC_TIM3_BASE };
    LPC_TIM_TypeDef *pTimerStruct = (LPC_TIM_TypeDef*) timerMemBases[timer];
    pTimerStruct->TC = 0;
}



#ifdef __cplusplus
}
#endif
#endif /* LPC_TIMERS_H__ */
