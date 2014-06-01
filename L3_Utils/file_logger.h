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
 * @file fileLogger.hpp
 * @brief This is a logger that logs data to a file on the system such as an SD Card.
 * @ingroup Utilities
 *
 * 20140529: Changed completely to C and FreeRTOS based logger
 * 20120923: modified flush() to use semaphores
 * 20120619: Initial
 */
#ifndef FILE_LOGGER_HPP__
#define FILE_LOGGER_HPP__
#ifdef __cplusplus
extern "C" {
#endif



/**
 * @{
 * The main parameters are the buffer size, and number of message buffers.
 * The buffer size controls how much data we can cache before we are forced
 * to write it to the output file.
 *
 * The message buffers are the maximum number of LOGGING calls (macros) that can
 * occur at once after which the caller to LOG macro will be blocked.  Note that
 * even a small number like 5 is good enough because as soon as there is time
 * for the logging task to run, it will transfer the LOGGED message into the
 * file buffer immediately.  So this number is kind of like a double buffer.
 *
 * The flush timeout is the timeout after which point we are forced to flush
 * the data buffer to the file.  So in an event when no logging calls occur and
 * there is data in the buffer, we will write it to the file after this time.
 */
#define FILE_LOGGER_BUFFER_SIZE      1024       ///< Recommend multiples of 512
#define FILE_LOGGER_MSG_BUFFERS      5          ///< Number of buffers
#define FILE_LOGGER_MSG_MAX_LEN      128        ///< Max length of a log message
#define FILE_LOGGER_FILENAME         "log.csv"  ///< Destination filename
#define FILE_LOGGER_STACK_SIZE       (1200 / 4) ///< Stack size in 32-bit (1 = 4 bytes for 32-bit CPU)
#define FILE_LOGGER_FLUSH_TIMEOUT    60         ///< Logs are flushed after this time
/** @} */



/**
 * @{ Macros to log a message using printf style API
 *
 * @warning As of now, the logging only works if FreeRTOS is running!
 *          If FreeRTOS is not running, you will likely crash!
 */
#define LOG_ERROR(msg, p...)  logger_log (log_info, __FILE__, __FUNCTION__, __LINE__, msg, ## p)
#define LOG_WARN(msg, p...)   logger_log (log_warn, __FILE__, __FUNCTION__, __LINE__, msg, ## p)
#define LOG_INFO(msg, p...)   logger_log (log_info, __FILE__, __FUNCTION__, __LINE__, msg, ## p)
/** @} */


/**
 * This macro will log INFO message without filename, function name, and line number.
 * This can save space to log simple messages when you don't want to know the
 * function name, line number and filename of where the logger function was called.
 */
#define LOG_INFO_SIMPLE(msg)   logger_log (log_info, NULL, NULL, 0, msg, ##p)

/**
 * Flushes the cached log data to the file
 */
void logger_flush(void);

/**
 * Macro to flush the logs
 */
#define LOG_FLUSH() logger_flush()

/**
 * Enumeration of the type of the log message.
 * You should not use this directly, the macros pass it to logger_log()
 */
typedef enum {
    log_invalid = 0,
    log_info,
    log_warn,
    log_error,
} logger_msg_t;

/**
 * Logs a message.
 * You should not use this directly, the macros pass the arguments to this function.
 */
void logger_log(logger_msg_t type, const char * filename, const char * func_name, unsigned line_num,
                const char * msg, ...);



#ifdef __cplusplus
}
#endif
#endif /* FILE_LOGGER_HPP__ */
