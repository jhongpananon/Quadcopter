#include "FreeRTOS.h"
#include "semphr.h"

#include "quad_tasks.hpp"
#include "lpc_timers.h"
#include "eint.h"

#include "flight_controller.hpp"
#include "shared_handles.h"
#include "file_logger.h"



/// Enumeration of channels
typedef enum {
    rc_chan1_pitch,
    rc_chan2_roll,
    rc_chan3_yaw,
    rc_chan4_throttle,
    rc_chan5,
    rc_chan6,
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
} rc_receiver_t;

/// The rising edge time-stamp of each channel
static uint32_t rc_channel_start_times[rc_total_channels] = { 0 };

/**
 * The timer to use for RC receiver.
 * timer1 is preferred because in case we go with the hardware capture unit (CAP1.0)
 * then the timer can remain the same.
 *
 * To use the capture, all we should need is just store the captured value to a buffer
 * and the channel_n_rising_edge() and channel_n_falling_edge() functions should
 * get the timer value from captured register, rather than timer1 value.
 */
static lpc_timer_t g_rc_receiver_timer = lpc_timer1;

/// Define the stack size this task is estimated to use
#define RC_RX_TASK_STACK_BYTES        (3 * 512)

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
            eint3_enable_port2(6,  eint_rising_edge,  ch5_rising_isr);  \
            eint3_enable_port2(6,  eint_falling_edge, ch5_falling_isr); \
            eint3_enable_port2(7,  eint_rising_edge,  ch6_rising_isr);  \
            eint3_enable_port2(7,  eint_falling_edge, ch6_falling_isr); \
        }                                                               \
        while (0)



/**
 * @ Generic callbacks for rising and falling edges
 */
void channel_n_rising_edge(const rc_channel_t ch)
{
    /* Restart the timer when we are about to capture the start of new pulses from the RC receiver */
    if (rc_chan1_pitch == ch) {
        lpc_timer_value_set(g_rc_receiver_timer, 0);
    }

    rc_channel_start_times[ch] = lpc_timer_value_get(g_rc_receiver_timer);
}

void channel_n_falling_edge(const rc_channel_t ch)
{
    const uint32_t stop = lpc_timer_value_get(g_rc_receiver_timer);
    static const QueueHandle_t qh = scheduler_task::getSharedObject(shared_RcReceiverQueue);

    rc_receiver_t rc;
    rc.channel = ch;
    rc.pulse_time_us = stop - rc_channel_start_times[ch];

    xQueueSendFromISR(qh, &rc, NULL);
}
/** @} */



/**
 * @{ Actual ISR callbacks of each channel's rising and falling edges
 */
void ch1_rising_isr(void)  { channel_n_rising_edge (rc_chan1_pitch);    }
void ch1_falling_isr(void) { channel_n_falling_edge(rc_chan1_pitch);    }
void ch2_rising_isr(void)  { channel_n_rising_edge (rc_chan2_roll);     }
void ch2_falling_isr(void) { channel_n_falling_edge(rc_chan2_roll);     }
void ch3_rising_isr(void)  { channel_n_rising_edge (rc_chan3_yaw);      }
void ch3_falling_isr(void) { channel_n_falling_edge(rc_chan3_yaw);      }
void ch4_rising_isr(void)  { channel_n_rising_edge (rc_chan4_throttle); }
void ch4_falling_isr(void) { channel_n_falling_edge(rc_chan4_throttle); }
void ch5_rising_isr(void)  { channel_n_rising_edge (rc_chan5);          }
void ch5_falling_isr(void) { channel_n_falling_edge(rc_chan5);          }
void ch6_rising_isr(void)  { channel_n_rising_edge (rc_chan6);          }
void ch6_falling_isr(void) { channel_n_falling_edge(rc_chan6);          }
/** @} */



rc_remote_task::rc_remote_task(const uint8_t priority) :
    scheduler_task("rcrx", RC_RX_TASK_STACK_BYTES, priority),
    mPitch(0), mRoll(0), mYaw(0), mThrottle(0)
{
    /* Use init() for memory allocation */
}

bool rc_remote_task::init(void)
{
    bool success = true;

    if (success) {
        /* Init RC remote receiver input pin interrupts */
        QueueHandle_t queueHandle = xQueueCreate(rc_total_channels, sizeof(rc_channel_t));
        addSharedObject(shared_RcReceiverQueue, queueHandle);
        success = (NULL != queueHandle);
    }

    /* Initialize the timer, and the channel pins' rising and falling edge ISRs */
    if (success) {
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

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(5 * 60 * 1000);

    return success;
}

int8_t rc_remote_task::getNormalizedValue(const uint32_t &pulseWidthUs)
{
    int16_t normalizedValue = 0;

    /* Normalize to 0-200 range, then convert to -100 -> +100 range */
    normalizedValue = (200 * pulseWidthUs) / mscMaxPulseWidthUs;
    normalizedValue -= 100;

    return (int8_t) normalizedValue;
}

bool rc_remote_task::run(void *p)
{
    /* XXX This data should be set on the Singleton class */
    FlightController f;
    const TickType_t timeout = OS_MS(1000);

    rc_receiver_t channelData;

    /* Wait for queue data to be sent */
    if (!xQueueReceive(getSharedObject(shared_RcReceiverQueue), &channelData, timeout)) {
        LOG_ERROR("RC receiver failed!");
        return false;
    }

    /* Cap the maximum value */
    if (channelData.pulse_time_us > mscMaxPulseWidthUs) {
        channelData.pulse_time_us = mscMaxPulseWidthUs;
    }

    /* Decode all the inputs */
    switch (channelData.channel)
    {
        case rc_chan1_pitch:
            mPitch = getNormalizedValue(channelData.pulse_time_us);
            break;

        case rc_chan2_roll:
            mRoll = getNormalizedValue(channelData.pulse_time_us);
            break;

        case rc_chan3_yaw:
            mYaw = getNormalizedValue(channelData.pulse_time_us);
            break;

        case rc_chan4_throttle:
            /* Convert normalized data from -100->+100 to 0->100 */
            mThrottle = (uint8_t) ((int16_t) getNormalizedValue(channelData.pulse_time_us) + 100) / 2;

            /* Since we have all the inputs, set them all to the flight controller */
            f.setFlightParameters(mPitch, mRoll, mYaw, mThrottle);
            break;

        case rc_chan5:
        case rc_chan6:
        default:
            break;
    }

    return true;
}
