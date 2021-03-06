/**
 *
 */
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "quadcopter.hpp"
#include "quad_tasks.hpp"
#include "lpc_timers.h"
#include "eint.h"

#include "flight_stabilizer.hpp"
#include "shared_handles.h"
#include "file_logger.h"



/// Enumeration of channels
typedef enum {
    rc_chan1_pitch,     ///< Channel used for pitch control
    rc_chan2_roll,      ///< Channel used for roll control
    rc_chan3_yaw,       ///< Channel used for yaw control
    rc_chan4_throttle,  ///< Channel used for throttle control

    rc_chan5,           ///< Available aux channel (PID turning maybe?)
    rc_chan6,           ///< Available aux channel (arm/disarm?)

    rc_total_channels,
} rc_channel_t;

/**
 * Queue data sent by ISR
 * Upon falling edge ISR, we send this struct to the queue, and the task
 * then processes the data to turn it to flight controller parameters.
 */
typedef struct {
    rc_channel_t channel;   ///< Channel number
    uint32_t pulse_time_us; ///< Pulse width time in micro-seconds
} rc_receiver_pulse_t;

/// The rising edge time-stamp of each channel
static uint32_t rc_channel_start_times[rc_total_channels] = { 0 };

/**
 * The timer to use for the RC receiver.
 * timer1 is preferred because in case we go with the hardware capture unit (CAP1.0)
 * then the timer can remain the same.
 *
 * To use the capture, all we should need is just store the captured value to a buffer
 * and the channel_n_rising_edge() and channel_n_falling_edge() functions should
 * get the timer value from captured register, rather than timer1 value.
 */
static const lpc_timer_t g_rc_receiver_timer = lpc_timer1;

/**
 * Macro to setup interrupts
 * This should match the pin-outs defined at main.cpp
 */
#define SETUP_CHANNEL_EINTS()                                           \
        do {                                                            \
            eint3_enable_port0(0,  eint_rising_edge,  ch1_rising_isr);  \
            eint3_enable_port0(0,  eint_falling_edge, ch1_falling_isr); \
            eint3_enable_port0(1,  eint_rising_edge,  ch2_rising_isr);  \
            eint3_enable_port0(1,  eint_falling_edge, ch2_falling_isr); \
            eint3_enable_port0(29, eint_rising_edge,  ch3_rising_isr);  \
            eint3_enable_port0(29, eint_falling_edge, ch3_falling_isr); \
            eint3_enable_port0(30, eint_rising_edge,  ch4_rising_isr);  \
            eint3_enable_port0(30, eint_falling_edge, ch4_falling_isr); \
        }                                                               \
        while (0)

/*
 * Just designing for four channels so not setting up EINTs for other pins
 *
            eint3_enable_port2(6,  eint_rising_edge,  ch5_rising_isr);  \
            eint3_enable_port2(6,  eint_falling_edge, ch5_falling_isr); \
            eint3_enable_port2(7,  eint_rising_edge,  ch6_rising_isr);  \
            eint3_enable_port2(7,  eint_falling_edge, ch6_falling_isr); \
 */

/**
 * @ Generic callbacks for rising and falling edges
 */
static void channel_n_rising_edge(const rc_channel_t ch)
{
    /* Restart the timer when we are about to capture the start of new pulses from
     * the RC receiver.  This avoids the case of timer overflow.
     */
    if (rc_chan1_pitch == ch) {
        lpc_timer_set_value(g_rc_receiver_timer, 0);
    }

    rc_channel_start_times[ch] = lpc_timer_get_value(g_rc_receiver_timer);
}

static void channel_n_falling_edge(const rc_channel_t ch)
{
    const uint32_t stop = lpc_timer_get_value(g_rc_receiver_timer);
    const QueueHandle_t qh = scheduler_task::getSharedObject(shared_RcReceiverQueue);

    rc_receiver_pulse_t rc;
    rc.channel = ch;
    rc.pulse_time_us = stop - rc_channel_start_times[ch];

    xQueueSendFromISR(qh, &rc, NULL);
}
/** @} */



/**
 * @{ Actual ISR callbacks of each channel's rising and falling edges
 */
static void ch1_rising_isr (void) { channel_n_rising_edge (rc_chan1_pitch);    }
static void ch1_falling_isr(void) { channel_n_falling_edge(rc_chan1_pitch);    }
static void ch2_rising_isr (void) { channel_n_rising_edge (rc_chan2_roll);     }
static void ch2_falling_isr(void) { channel_n_falling_edge(rc_chan2_roll);     }
static void ch3_rising_isr (void) { channel_n_rising_edge (rc_chan3_yaw);      }
static void ch3_falling_isr(void) { channel_n_falling_edge(rc_chan3_yaw);      }
static void ch4_rising_isr (void) { channel_n_rising_edge (rc_chan4_throttle); }
static void ch4_falling_isr(void) { channel_n_falling_edge(rc_chan4_throttle); }
#if 0
static void ch5_rising_isr (void) { channel_n_rising_edge (rc_chan5);          }
static void ch5_falling_isr(void) { channel_n_falling_edge(rc_chan5);          }
static void ch6_rising_isr (void) { channel_n_rising_edge (rc_chan6);          }
static void ch6_falling_isr(void) { channel_n_falling_edge(rc_chan6);          }
#endif
/** @} */


/// Define the stack size this task is estimated to use
#define RC_RX_TASK_STACK_BYTES        (3 * 512)



rc_remote_task::rc_remote_task(const uint8_t priority) :
    scheduler_task("rcrx", RC_RX_TASK_STACK_BYTES, priority),
    mQuadcopter(Quadcopter::getInstance())
{
    /* Use init() for memory allocation */
    mRcRxFlightCmd = 0;
}

bool rc_remote_task::init(void)
{
    bool success = true;

    /* Initialize the queue that the ISR will input the data to */
    if (success) {
        /* RC receiver updates at 50Hz, so if we use queue depth of (2 * rc_total_channels), then
         * we have about 40ms for our task to dequeue the data, and we don't expect the CPU to
         * not be able to service this task by then.
         */
        QueueHandle_t queueHandle = xQueueCreate(2 * rc_total_channels, sizeof(rc_receiver_pulse_t));
        addSharedObject(shared_RcReceiverQueue, queueHandle);
        success = (NULL != queueHandle);
    }

    /* Initialize the timer, and the channel pins' rising and falling edge ISRs */
    if (success)
    {
        SETUP_CHANNEL_EINTS();
        lpc_timer_enable(g_rc_receiver_timer, 1);

        /* This code is compiled-out because we are not using timer1 capture method */
#if 0
        LPC_TIM1->CCR &= ~(7 << 0);            // Clear Bits 2:1:0
        LPC_TIM1->CCR |=  (1 << 2) | (1 << 1); // Enable Falling Edge capture0 with interrupt

        // Select P1.18 as CAP1.0 by setting bits 5:4 to 0b11
        LPC_PINCON->PINSEL3 |= (3 << 4);

        // Finally, enable interrupt of Timer1 to interrupt upon falling edge capture
        NVIC_EnableIRQ(TIMER1_IRQn);
#endif
    }

    // Do not update task statistics (stack usage)
    setStatUpdateRate(0);

    return success;
}

int8_t rc_remote_task::getNormalizedValue(const uint32_t &pulseWidthUs)
{
    int16_t normalizedValue = 0;

    /* Normalize to 0-200 range */
    normalizedValue = (200 * pulseWidthUs) / mscMaxPulseWidthUs;

    /* Convert to -100 -> +100 */
    normalizedValue -= 100;

    return (int8_t) normalizedValue;
}

bool rc_remote_task::run(void *p)
{
    const TickType_t timeout = OS_MS(1000);
    const uint8_t noiseMicroSeconds = 25;
    rc_receiver_pulse_t channelData;

    /* Wait for queue data to be sent */
    if (!xQueueReceive(getSharedObject(shared_RcReceiverQueue), &channelData, timeout)) {
        LOG_ERROR("RC receiver failed!");

        /* Tell the Quadcopter class that the RC receiver has failed */
        const bool healthy = false;
        mQuadcopter.setRcReceiverStatus(healthy);

        /* XXX Returning false to suspend the task until RC receiver is attached */
        return false;
    }

    /* Cap the maximum value */
    if (channelData.pulse_time_us > mscMaxPulseWidthUs) {
        channelData.pulse_time_us = mscMaxPulseWidthUs;
    }

    /* Filter out the "noise", or the computation needed for us to measure the pulse width */
    if (channelData.pulse_time_us < noiseMicroSeconds) {
        channelData.pulse_time_us = 0;
    }

    /* Decode all the inputs */
    switch (channelData.channel)
    {
        case rc_chan1_pitch:
            mRcRxFlightCmd.angle.pitch = getNormalizedValue(channelData.pulse_time_us);
            break;

        case rc_chan2_roll:
            mRcRxFlightCmd.angle.roll = getNormalizedValue(channelData.pulse_time_us);
            break;

        case rc_chan3_yaw:
            mRcRxFlightCmd.angle.yaw = getNormalizedValue(channelData.pulse_time_us);
            break;

        case rc_chan4_throttle:
        {
            /* Convert normalized data from -100->+100 to 0->100 */
            mRcRxFlightCmd.throttle = (uint8_t) ((int16_t) getNormalizedValue(channelData.pulse_time_us) + 100) / 2;

            /* Since we have all the inputs, set them all to the flight controller */
            const bool healthy = true;
            mQuadcopter.setRcReceiverStatus(healthy);
            mQuadcopter.setFlightControl(mRcRxFlightCmd);

            /* Reset the parameters so we don't use stale values next time */
            mRcRxFlightCmd = 0;

            break;
        }

        case rc_chan5:
        case rc_chan6:
        default:
            LOG_ERROR("Invalid case");
            break;
    }

    return true;
}
