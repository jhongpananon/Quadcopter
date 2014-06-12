#include "quad_tasks.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include "shared_handles.h"
#include "flight_controller.hpp"
#include "io.hpp"


/**
 * XXX Not happy with this design, several questions stand:
 *      - Who owns the sensors?  FlightController?  Sensor task?
 *      - Do we really need "ThreeAxisSensor" ?
 *      - Do we really need sensor task?  Can't it be combined with quadcopter class?
 *
 */
/// Define the stack size this task is estimated to use
#define SENSOR_TASK_STACK_BYTES     (3 * 512)



sensor_task::sensor_task(const uint8_t priority) :
    scheduler_task("sensor", SENSOR_TASK_STACK_BYTES, priority)
{
    /* Use init() for memory allocation */
}

bool sensor_task::init(void)
{
    bool success = true;
    FlightController &f = Quadcopter::getInstance().mFlightController;
    SemaphoreHandle_t sensorDataReadySem = xSemaphoreCreateBinary();

    if (success) {
        success = (NULL != sensorDataReadySem);
    }

    if (success) {
        success = addSharedObject(shared_SensorDataReadySemaphore, sensorDataReadySem);
    }

    /* TODO: Set min/max according to the particular sensor */
    if (success) {
        f.mAccelerationSensor.setMinimumMaximumForAllAxis((4 * -1024), (4 * +1024));
                f.mGyroSensor.setMinimumMaximumForAllAxis((4 * -1024), (4 * +1024));
               f.mMagnoSensor.setMinimumMaximumForAllAxis((4 * -1024), (4 * +1024));
    }

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(5 * 60 * 1000);

    // 100Hz sensor loop
    setRunDuration(10);

    return success;
}

bool sensor_task::run(void *p)
{
    bool success = true;
    int32_t x = 0, y = 0, z = 0;

    /* Get the sensor data
     * TODO Get values from the attached MPU-9150
     */
    x = AS.getX();
    y = AS.getY();
    z = AS.getZ();

    /* Send the data to the flight controller class */
    Quadcopter &q = Quadcopter::getInstance();
    q.mFlightController.mAccelerationSensor.setAll(x, y, z);
    q.mFlightController.mGyroSensor.setAll(0, 0, 0);
    q.mFlightController.mMagnoSensor.setAll(0, 0, 0);

    /* Now let the processing task process the values and run its algorithms */
    success = xSemaphoreGive(getSharedObject(shared_SensorDataReadySemaphore));

    return success;
}
