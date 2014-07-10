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


static FIL *gp_file_ptr = NULL;
static int g_blocked_calls = 0;                     ///< Number of logging calls that blocked
static char * gp_file_buffer = NULL;                ///< Pointer to local buffer space before it is written to file
static QueueHandle_t g_write_buffer_queue = NULL;   ///< Log message pointers are written to this queue
static QueueHandle_t g_empty_buffer_queue = NULL;   ///< Log message pointers are available from this queue


#if 0
// xxx
#undef FILE_LOGGER_FILENAME
#define FILE_LOGGER_FILENAME "1:log.csv"
#endif

static bool logger_write_to_file(const void * buffer, const uint32_t bytes_to_write)
{
    bool success = false;
    UINT bytes_written = 0;
    UINT bytes_to_write_dword = bytes_to_write;
    FRESULT err = 0;

    if (0 == bytes_to_write_dword) {
        success = true;
    }
    /* File already open, so just write the data */
    #if (FILE_LOGGER_KEEP_FILE_OPEN)
    else if (FR_OK == (err = f_write(gp_file_ptr, buffer, bytes_to_write, &bytes_written)))
    {
        f_sync(gp_file_ptr);
    }
    #else
    /* File not opened, open it, seek it, and then write it */
    else if(FR_OK == (err = f_open(gp_file_ptr, FILE_LOGGER_FILENAME, FA_OPEN_ALWAYS | FA_WRITE)))
    {
        if (FR_OK == (err = f_lseek(gp_file_ptr, f_size(gp_file_ptr))))
        {
            err = f_write(gp_file_ptr, buffer, bytes_to_write, &bytes_written);
        }
        f_close(gp_file_ptr);
    }
    #endif
    else {
        printf("Failed file write: ");
    }

    success = (bytes_written == bytes_to_write);
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
    char * log_msg = NULL;
    char * write_ptr = gp_file_buffer;
    const char * start_ptr = gp_file_buffer;
    const char * end_ptr = start_ptr + FILE_LOGGER_BUFFER_SIZE;
    const uint32_t buffer_size = buffer_size;

    uint32_t len = 0;
    uint32_t buffer_overflow_cnt = 0;

    while (1)
    {
        /* Receive the log message we wish to write to our buffer */
        log_msg = NULL;
        xQueueReceive(g_write_buffer_queue, &log_msg, OS_MS(1000 * FILE_LOGGER_FLUSH_TIMEOUT));

        /* If we receive NULL pointer, we assume it is to flush the data.
         * log_msg will also be NULL after the FILE_LOGGER_FLUSH_TIMEOUT which
         * is another way the logger will flush the data to the file.
         */
        if (NULL == log_msg) {
            /* We don't see the NULL pointer to the queue */
            logger_write_to_file(start_ptr, (write_ptr - start_ptr));
            write_ptr = (char *) start_ptr;
            continue;
        }

        /* XXX Test code to write file immediately without buffering.
         * Need to investigate why logger write is writing weird data to the file.
         * This is happening both on SPI flash and SD card.
         * I have tried updating to latest FATFS, and manipulating various different
         * options at ffconf.h and also by manipulating SPI bus speed.
         *
         * I have also tried testing the disk io layer by writing all sectors with a unique
         * pattern and then by reading all the sectors and matched the unique pattern, so disk
         * layer is not a problem.
         *
         * I have also tried copying large data files from SD card to flash memory and vice versa
         * and the files are getting copied correctly, so that is also not a problem.  So why
         * is the logger task writing the data a problem?
         */
#if 1
        else {
            UINT bytes_to_write = strlen(log_msg);

            log_msg[bytes_to_write] = '\n';
            log_msg[++bytes_to_write] = '\0';
            logger_write_to_file(log_msg, bytes_to_write);

            xQueueSend(g_empty_buffer_queue, &log_msg, portMAX_DELAY);
            continue;
        }
#endif

        /* else */
        len = strlen(log_msg);

        /* Append newline character */
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

            /* Optional: Zero out buffer space */
            memset((char*) start_ptr, '*', buffer_size);

            /* Copy the left-over message back to our buffer */
            memcpy((char*) start_ptr, (log_msg + len - buffer_overflow_cnt), buffer_overflow_cnt);
            write_ptr = (char*) start_ptr + buffer_overflow_cnt;
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

static bool logger_init(void)
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

        if (NULL != ptr) {
            xQueueSendFromISR(g_empty_buffer_queue, &ptr, NULL);
        }
        else {
            return failure;
        }
    }

    gp_file_ptr = malloc (sizeof(*gp_file_ptr));
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

void logger_log(logger_msg_t type, const char * filename, const char * func_name, unsigned line_num,
                const char * msg, ...)
{
    char * buffer = NULL;
    char * temp_ptr = NULL;
    uint32_t len = 0;
    const rtc_t time = rtc_gettime();
    const unsigned int uptime = sys_get_uptime_ms();

    /* This must match up with the logger_msg_t enumeration */
    const char * const type_str[] = { "invalid", "info", "warn", "error" };

    taskENTER_CRITICAL();
    /* If file buffer is NULL, that means the logger is not initialized */
    if (NULL == gp_file_buffer)
    {
        if (!logger_init()) {
            printf("logger initialization failure\n");
        }
    }
    taskEXIT_CRITICAL();

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

    /* Get an available buffer to write the data to */
    if (!xQueueReceive(g_empty_buffer_queue, &buffer, OS_MS(FILE_LOGGER_BLOCK_TIME_MS))) {
        ++g_blocked_calls;

        /* This time, just block forever until we get a buffer */
        xQueueReceive(g_empty_buffer_queue, &buffer, portMAX_DELAY);
    }

    /* Write the header including time, filename, function name etc */
    len = sprintf(buffer, "%u/%u,%02d:%02d:%02d,%u,%s,%s,%s%s,%u,",
                    (unsigned)time.month, (unsigned)time.day, (unsigned)time.hour,
                    (unsigned)time.min, (unsigned)time.sec, uptime,
                    type_str[type], filename, func_name,
                    func_name[0] ? "()" : "", /* If function name exists, only then append () */
                    line_num
                    );

    /* Append actual user message, and leave one space for \n to be appended by the logger task.
     * Example: max length = 10, and say we printed 5 chars so far "hello"
     *          we will sprintf "world" to "hello" where n = 10-5-1 = 4
     *          So, this sprintf will append and make it: "hellowor\0"
     *
     * Note: snprintf returns "number of chars that would've been printed if n was sufficiently large"
     *       "size" of snprintf() includes the NULL character
     */
    va_list args;
    va_start(args, msg);
    vsnprintf(buffer + len, FILE_LOGGER_MSG_MAX_LEN-len-1, msg, args);
    va_end(args);

    // printf("%p: len: %i msg: |%s|\n", buffer, strlen(buffer), buffer);

    /* There is no efficient way to append \n here since we will have to use strlen(), but since the
     * logger task will take strlen() anyway, it can append it there.
     */

    /* Send the buffer to the queue for the logger task to write */
    xQueueSend(g_write_buffer_queue, &buffer, portMAX_DELAY);
}

void logger_log_raw(const char * msg, ...)
{
    char * buffer = NULL;

    /* Get an available buffer to write the data to */
    if (!xQueueReceive(g_empty_buffer_queue, &buffer, OS_MS(FILE_LOGGER_BLOCK_TIME_MS))) {
        ++g_blocked_calls;

        /* This time, just block forever until we get a buffer */
        xQueueReceive(g_empty_buffer_queue, &buffer, portMAX_DELAY);
    }

    va_list args;
    va_start(args, msg);
    vsnprintf(buffer, FILE_LOGGER_MSG_MAX_LEN-1, msg, args);
    va_end(args);

    /* Send the buffer to the queue for the logger task to write */
    xQueueSend(g_write_buffer_queue, &buffer, portMAX_DELAY);
}
