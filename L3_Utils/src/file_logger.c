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

#include <stdlib.h>
#include <stdio.h>    // sprintf()
#include <string.h>   // strlen()
#include <stdarg.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "file_logger.h"
#include "lpc_sys.h"
#include "rtc.h"
#include "ff.h"



static FIL *gp_file_ptr = NULL;                     ///< The pointer to the file object
static int g_blocked_calls = 0;                     ///< Number of logging calls that blocked
static int g_highest_file_write_time = 0;           ///< Highest time spend while trying to write file buffer
static int g_buffer_watermark = 0;                  ///< The watermark of the number of buffers consumed
static char * gp_file_buffer = NULL;                ///< Pointer to local buffer space before it is written to file
static QueueHandle_t g_write_buffer_queue = NULL;   ///< Log message pointers are written to this queue
static QueueHandle_t g_empty_buffer_queue = NULL;   ///< Log message pointers are available from this queue
char * gp_buffer_ptrs[FILE_LOGGER_MSG_BUFFERS] = { 0 };

/// XXX Remove this
#undef FILE_LOGGER_FILENAME
#define FILE_LOGGER_FILENAME "1:log.csv"

/**
 * Test 1: With 20ms quadcopter task logging data:
 * With vsnprintf() commented out and buffering:
 *  - Seems to work
 *  - Some parts of file still has bad data
 *
 * Test 2: with 20ms quadcopter task logging data
 * With vsnprintf() commmented out and no buffering:
 *  - Still fails
 *
 * Test 3: No FreeRTOS usage
 * With 10000 logging calls in main() without starting FreeRTOS, it works!
 * This is when we directly call Storage::append() API
 *
 * Test 4: With just terminal task and logging task:
 * I have a suspicion that the quadcopter task might be the culprit.
 * nops... using terminal command "qlog test 10000 hello" fails.
 *
 * Test 5:
 * Only logger task and special task just to log data 10,000 times.
 * It worked, but file had some corrupted areas... so could this be a
 * problem with our logger since terminal task was copying data just
 * fine when it was doing file I/O using "cp" command. That means that
 *
 * File I/O is fully working...
 * This leads me to believe that the problem is here in the logger!
 *
 * Test 6: Force os_running to false and log at a high frequency!
 * This forces the log message to be fully formed, but it writes the message
 * immediately to the file bypassing the logger task.  This basically verifies
 * that the logic of logger_log() is fully working without dealing with FreeRTOS
 * queues.  Spending about 5 minutes to do the logging....
 * File turned out 840K and it fully worked!!!
 *
 * The problem is definitely logging task.
 *
 * Test 6: Run minimal logic at logging task that only dequeues pointer and
 * immediately writes the data to file without buffering. "#if 0" changed to "if 1"
 * THIS FAILS!  WHY???
 *
 * Test 7: Increase logger size to 3K
 * This fails too!
 *
 * Test 8: Just logger task and terminal task logging 10K messages --> ALSO FAILS!
 *
 * So what do we know?
 * logger_write_to_file() is okay
 * logger_log() is okay
 *  Something is wrong with the logging task!!!
 *
 * Test 9: Change num logging buffers to 1, and test that logger always dequeues correct pointer
 * Still fails, but we seem to be dequeueing correct pointer.
 */

#if 0
    for (int i = 0; i < 10000; i++) {
        if (0 == (i%100)) {
            printf("%i\n", i);
        }
        char buffer[128];
        sprintf(buffer, "%u: 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ\n", (unsigned int) sys_get_uptime_ms());
        Storage::append("1:test.txt", buffer, strlen(buffer), 0);
    }
#endif


static bool logger_write_to_file(const void * buffer, const uint32_t bytes_to_write)
{
    bool success = false;
    FRESULT err = 0;
    UINT bytes_written = 0;
    const UINT bytes_to_write_uint = bytes_to_write;
    const int start_time = sys_get_uptime_ms();

    if (0 == bytes_to_write_uint) {
        success = true;
    }
    /* File already open, so just write the data */
    #if (FILE_LOGGER_KEEP_FILE_OPEN)
    else if (FR_OK == (err = f_write(gp_file_ptr, buffer, bytes_to_write_uint, &bytes_written)))
    {
        f_sync(gp_file_ptr);
    }
    #else
    /* File not opened, open it, seek it, and then write it */
    else if(FR_OK == (err = f_open(gp_file_ptr, FILE_LOGGER_FILENAME, FA_OPEN_ALWAYS | FA_WRITE)))
    {
        if (FR_OK == (err = f_lseek(gp_file_ptr, f_size(gp_file_ptr))))
        {
            err = f_write(gp_file_ptr, buffer, bytes_to_write_uint, &bytes_written);
        }
        f_close(gp_file_ptr);
    }
    #endif
    else {
        printf("Failed file write: ");
    }

    /* Capture the time */
    const int end_time = sys_get_uptime_ms();
    const int diff_time = end_time - start_time;
    if (diff_time > g_highest_file_write_time) {
        g_highest_file_write_time = diff_time;
    }

    /* To be successful, bytes written should be the same count as the bytes intended to be written */
    success = (bytes_to_write_uint == bytes_written);

    if (!success) {
        printf("Error %u writing logfile. %u/%u written. Fptr: %u\n",
                (unsigned)err, (unsigned)bytes_written, (unsigned)bytes_to_write,
                (unsigned) (gp_file_ptr->fsize));
    }

    return success;
}

/**
 * This is the actual logger FreeRTOS task responsible for:
 *      - Retrieve a log message written to the write queue
 *      - Copy it to our local buffer
 *      - If local buffer is full, write it to the file
 *      - Put-back the log message buffer to available queue
 */
static void logger_task(void *p)
{
    /* Use "char * const" to disallow this pointer to be moved.
     * We don't want to use "const char *" because we do want to be able to
     * write to this pointer instead of using excessive casts later.
     */
    char * const start_ptr = gp_file_buffer;
    char * const end_ptr = start_ptr + FILE_LOGGER_BUFFER_SIZE;
    const uint32_t buffer_size = (end_ptr - start_ptr);

    char * log_msg = NULL;
    char * write_ptr = start_ptr;
    uint32_t len = 0;
    uint32_t buffer_overflow_cnt = 0;

    while (1)
    {
        /* Receive the log message we wish to write to our buffer */
        log_msg = NULL;
        xQueueReceive(g_write_buffer_queue, &log_msg, OS_MS(1000 * FILE_LOGGER_FLUSH_TIMEOUT));

        if ((len = uxQueueMessagesWaiting(g_write_buffer_queue)) > g_buffer_watermark) {
            g_buffer_watermark = len;
        }

        /* If we receive NULL pointer, we assume it is to flush the data.
         * log_msg will also be NULL after the FILE_LOGGER_FLUSH_TIMEOUT which
         * is another way the logger will flush the data to the file.
         */
        if (NULL == log_msg) {
            /* We don't see the NULL pointer to the queue */
            logger_write_to_file(start_ptr, (write_ptr - start_ptr));
            write_ptr = start_ptr;
            continue;
        }

        /* This is test code to immediately write data to the file.  It is highly in-efficient and is
         * included here just for reference or test purposes.
         */
        #if 1
        else {
            if (gp_buffer_ptrs[0] != log_msg) {
                printf("Wanted %p, got %p\n", gp_buffer_ptrs[0], log_msg);
            }
            len = strlen(log_msg);

            log_msg[len] = '\n';
            log_msg[++len] = '\0';
            logger_write_to_file(log_msg, len);

            xQueueSend(g_empty_buffer_queue, &log_msg, portMAX_DELAY);
            continue;
        }
        #endif

        /* Get the length and append the newline character */
        len = strlen(log_msg);
        log_msg[len] = '\n';
        log_msg[++len] = '\0';

        /* If we will overflow our buffer we need to write the full buffer and do partial copy */
        if (len + write_ptr >= end_ptr)
        {
            buffer_overflow_cnt = (len + write_ptr - end_ptr);

            /* Copy partial message */
            memcpy(write_ptr, log_msg, (end_ptr - write_ptr));

            /* Copy the entire buffer to the file */
            logger_write_to_file(start_ptr, buffer_size);

            /* Optional: Zero out the buffer space */
            // memset(start_ptr, '\0', buffer_size);

            /* Copy the left-over message to the start of "fresh" buffer space (after writing to file) */
            if (buffer_overflow_cnt > 0) {
                memcpy(start_ptr, (log_msg + len - buffer_overflow_cnt), buffer_overflow_cnt);
            }
            write_ptr = start_ptr + buffer_overflow_cnt;
        }
        /* Buffer has enough space, write the entire message to the buffer */
        else {
            memcpy(write_ptr, log_msg, len);
            write_ptr += len;
        }

        /* Put the data pointer back to available buffer */
        xQueueSend(g_empty_buffer_queue, &log_msg, portMAX_DELAY);
    }
}

static bool logger_internal_init(void)
{
    uint32_t i = 0;
    char * ptr = NULL;
    const bool failure = false;
    const bool success = true;

    /* Create the buffer space we write the logged messages to (before we flush it to the file) */
    gp_file_buffer = (char*) malloc(FILE_LOGGER_BUFFER_SIZE);
    if (NULL == gp_file_buffer) {
        return failure;
    }
    else {
        memset(gp_file_buffer, 0, FILE_LOGGER_BUFFER_SIZE);
    }

    /* Create the queues that keep track of the written buffers, and available buffers */
    g_write_buffer_queue = xQueueCreate(FILE_LOGGER_MSG_BUFFERS, sizeof(char*));
    g_empty_buffer_queue = xQueueCreate(FILE_LOGGER_MSG_BUFFERS, sizeof(char*));
    if (NULL == g_write_buffer_queue || NULL == g_empty_buffer_queue) {
        return failure;
    }

    /* Create the actual buffers for log messages */
    for (i = 0; i < FILE_LOGGER_MSG_BUFFERS; i++)
    {
        ptr = (char*) malloc(FILE_LOGGER_MSG_MAX_LEN);
        memset(ptr, 0, FILE_LOGGER_MSG_MAX_LEN);
        gp_buffer_ptrs[i] = ptr;

        if (NULL != ptr) {
            xQueueSendFromISR(g_empty_buffer_queue, &ptr, NULL);
        }
        else {
            return failure;
        }
    }

    // xxx Change after testing successfully: gp_file_ptr = malloc (sizeof(*gp_file_ptr));
    gp_file_ptr = malloc (sizeof(FIL));
    if (NULL == gp_file_ptr)
    {
        return failure;
    }

#if (FILE_LOGGER_KEEP_FILE_OPEN)
    if(FR_OK != f_open(gp_file_ptr, FILE_LOGGER_FILENAME, FA_OPEN_ALWAYS | FA_WRITE))
    {
        return failure;
    }
#endif

    if (!xTaskCreate(logger_task, "logger", FILE_LOGGER_STACK_SIZE, NULL, FILE_LOGGER_OS_PRIORITY, NULL))
    {
        return failure;
    }

    return success;
}

void logger_send_flush_request(void)
{
    char * null_ptr_to_flush = NULL;
    xQueueSend(g_write_buffer_queue, &null_ptr_to_flush, portMAX_DELAY);
}

int logger_get_blocked_call_count(void)
{
    return g_blocked_calls;
}

int logger_get_highest_file_write_time_ms(void)
{
    return g_highest_file_write_time;
}

int logger_get_num_buffers_watermark(void)
{
    return g_buffer_watermark;
}

void logger_init(void)
{
    /* Prevent double init */
    if (NULL == gp_file_buffer)
    {
        if (!logger_internal_init()) {
            printf("logger initialization failure\n");
        }
    }
}

static char * logger_get_buffer_ptr(const bool os_running)
{
    char * buffer = NULL;

    /* Get an available buffer to write the data to, and if OS is not running, just use our first buffer */
    if (!os_running) {
        buffer = gp_buffer_ptrs[0];
    }
    else if (!xQueueReceive(g_empty_buffer_queue, &buffer, OS_MS(FILE_LOGGER_BLOCK_TIME_MS))) {
        ++g_blocked_calls;

        /* This time, just block forever until we get a buffer */
        xQueueReceive(g_empty_buffer_queue, &buffer, portMAX_DELAY);
    }

    return buffer;
}

static void logger_write_log_message(char * buffer, const bool os_running)
{
    size_t len = 0;

    /* Send the buffer to the queue for the logger task to write */
    if (os_running) {
        xQueueSend(g_write_buffer_queue, &buffer, portMAX_DELAY);
    }
    else {
        len = strlen(buffer);
        buffer[len] = '\n';
        buffer[++len] = '\0';
        logger_write_to_file(buffer, len);
    }
}

void logger_log(logger_msg_t type, const char * filename, const char * func_name, unsigned line_num,
                const char * msg, ...)
{
    char * buffer = NULL;
    char * temp_ptr = NULL;
    uint32_t len = 0;
    const rtc_t time = rtc_gettime();
    const unsigned int uptime = sys_get_uptime_ms();
    const bool os_running = (taskSCHEDULER_RUNNING == xTaskGetSchedulerState());

    /* This must match up with the logger_msg_t enumeration */
    const char * const type_str[] = { "invalid", "info", "warn", "error" };

    // Find the back-slash or forward-slash to get filename only, not absolute or relative path
    if(0 != filename) {
        temp_ptr = strrchr(filename, '/');
        // If forward-slash not found, find back-slash
        if(0 == temp_ptr) temp_ptr = strrchr(filename, '\\');
        if(0 != temp_ptr) filename = temp_ptr+1;
    }
    else {
        filename = "";
    }

    if (0 == func_name) {
        func_name = "";
    }

    /* Get an available buffer */
    buffer = logger_get_buffer_ptr(os_running);

    do {
        int mon = time.month;
        int day = time.day;
        int hr = time.hour;
        int min = time.min;
        int sec = time.sec;
        unsigned int up = uptime;
        const char *log_type_str = type_str[type];
        const char *func_parens  = func_name[0] ? "()" : "";

        /* Write the header including time, filename, function name etc */
        len = sprintf(buffer, "%u/%u,%02d:%02d:%02d,%u,%s,%s,%s%s,%u,",
                      mon, day, hr, min, sec, up, log_type_str, filename, func_name, func_parens, line_num);
    } while (0);

    /* Append actual user message, and leave one space for \n to be appended by the logger task.
     * There is no efficient way to append \n here since we will have to use strlen(),
     * but since the logger task will take strlen() anyway, it can append it there.
     *
     * Example: max length = 10, and say we printed 5 chars so far "hello"
     *          we will sprintf "world" to "hello" where n = 10-5-1 = 4
     *          So, this sprintf will append and make it: "hellowor\0" leaving space to add \n
     *
     * Note: You cannot use returned value from vsnprintf() because snprintf() returns:
     *       "number of chars that would've been printed if n was sufficiently large"
     *
     * Note: "size" of snprintf() includes the NULL character
     */
    do {
        va_list args;
        va_start(args, msg);
        vsnprintf(buffer + len, FILE_LOGGER_MSG_MAX_LEN-len-1, msg, args);
        va_end(args);
    } while (0);

    /* Send the buffer to the queue for the logger task to write */
    logger_write_log_message(buffer, os_running);
}

void logger_log_raw(const char * msg, ...)
{
    char * buffer = NULL;
    const bool os_running = (taskSCHEDULER_RUNNING == xTaskGetSchedulerState());

    /* Get an available buffer */
    buffer = logger_get_buffer_ptr(os_running);

    /* Print the actual user message to the buffer */
    do {
        va_list args;
        va_start(args, msg);
        vsnprintf(buffer, FILE_LOGGER_MSG_MAX_LEN-1, msg, args);
        va_end(args);
    } while (0);

    /* Send the buffer to the queue for the logger task to write */
    logger_write_log_message(buffer, os_running);
}
