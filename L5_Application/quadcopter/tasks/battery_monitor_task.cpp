/**
 * @file
 */
#include "quad_tasks.hpp"
#include "quadcopter.hpp"

#include "adc0.h"
#include "tlm/c_tlm_var.h"
#include "file_logger.h"



/// Define the stack size this task is estimated to use
#define BATTERY_TASK_STACK_BYTES        (3 * 512)

/// Define the ADC channel to grab external voltage
#define EXTERNAL_VOLTAGE_ADC_CH_NUM     3

/// Define to initialize ADC pin
#define EXTERNAL_VOLTAGE_ADC_INIT()     do { LPC_PINCON->PINSEL1 |= (1 << 20); } while(0)

/// Define the voltage divider used externally
#define EXTERNAL_VOLTAGE_DIVIDER        10



battery_monitor_task::battery_monitor_task(const uint8_t priority) :
    scheduler_task("battery", BATTERY_TASK_STACK_BYTES, priority),
    mLowestVoltage(1000), /* A really high voltage, it will be reset upon actual voltage sensed */
    mHighestVoltage(-1)   /* A really low voltage, it will be reset upon actual voltage sensed */
{
    /* Use init() for memory allocation */
}

bool battery_monitor_task::init(void)
{
    bool success = true;

    if (success) {
        EXTERNAL_VOLTAGE_ADC_INIT();
    }

    /* Register disk variables */
    if (success) {
        tlm_component *disk = tlm_component_get_by_name(DISK_TLM_NAME);
        TLM_REG_VAR(disk, mLowestVoltage, tlm_float);
        TLM_REG_VAR(disk, mHighestVoltage, tlm_float);
    }

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(5 * 60 * 1000);

    // Monitor the battery at a very slow rate
    setRunDuration(1000);

    return success;
}

bool battery_monitor_task::run(void *p)
{
    /* Read the ADC here */
    uint16_t rawAdc = adc0_get_reading(EXTERNAL_VOLTAGE_ADC_CH_NUM);

    /* Convert ADC to voltage value */
    const uint32_t adcConverterBitResolution = 12;
    const float voltsPerAdcStep = 3.3f / (1 << adcConverterBitResolution);
    float voltage = (rawAdc * voltsPerAdcStep) * EXTERNAL_VOLTAGE_DIVIDER;

    /* Update the high/low voltages */
    if (voltage > mHighestVoltage) {
        mHighestVoltage = voltage;
    }
    if (voltage < mLowestVoltage) {
        mLowestVoltage = voltage;
    }

    /* Convert voltage value to percentage value.
     * We add 0.01f to avoid divide by zero.
     */
    float percent = voltage / (0.01f + mHighestVoltage - mLowestVoltage) * 100.0f;

    /* Set the value to the quadcopter class */
    uint8_t percentUint = (uint8_t) percent;

    /* XXX This is just done for the example, we should set it to the real quadcopter class */
    Quadcopter q;
    q.setBatteryPercentage(percentUint);

    static uint8_t logPeriodically = 0;
    const uint8_t period = 15;
    if (0 == (++logPeriodically % period)) {
        LOG_INFO_SIMPLE("Battery: %.1f volts, estimated charge: %u%% (%.2f/%.2f)",
                        voltage, percentUint, mLowestVoltage, mHighestVoltage);
    }

    return true;
}
