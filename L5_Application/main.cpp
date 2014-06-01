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
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 * @note  printf of %f may be turned off to save memory, this can be configured at sys_config.h
 */
#include <stdio.h>
#include "tasks.hpp"
#include "quad_tasks.hpp"
#include "uart2.hpp"
#include "uart3.hpp"
#include "file_logger.h"



/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at cpp_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
int main(void)
{
    /* Very important to use & for reference - I learned it the hard way :( */
    Uart2 &bluetoothUart = Uart2::getInstance();
    Uart3 &gpsUart = Uart3::getInstance();

    /* Initialize the baud rates here, so when tasks run, their UART is ready
     * We cannot init() Uarts at the tasks since they only have pointer to UartDev,
     * but it is the Uart2 or Uart3 pointers whose init() initializes their pins.
     * UartDev only initializes uart registers, not the PINSEL or interrupt registers
     */
    bluetoothUart.init(38400, 128, 1024);
    gpsUart.init      (38400, 128, 32);

    /* Log a message to initialize the logger task and log the time of startup */
    LOG_INFO_SIMPLE("System Startup");

    /* Terminal task needs high priority to access the system in case a task gets stuck */
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    /* Quadcopter tasks should be the highest priority to process the flight controller algorithms */
    scheduler_add_task(new sensor_task    (PRIORITY_HIGH));
    scheduler_add_task(new quadcopter_task(PRIORITY_HIGH));

    /* GPS and RC receiver tasks can execute and miss their deadline without a big issue */
    scheduler_add_task(new gps_task       (PRIORITY_MEDIUM, &gpsUart));
    scheduler_add_task(new rc_remote_task (PRIORITY_MEDIUM));

    /* Low priority tasks are designed to only execute if there is any CPU left */
    scheduler_add_task(new battery_monitor_task (PRIORITY_LOW));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    /* No need for IR remote control task */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you
     * want the terminal task to always be responsive so you can poke around in
     * case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(false, true); ///< This shouldn't return
    return -1;
}
