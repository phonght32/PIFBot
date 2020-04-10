// MIT License

// Copyright (c) 2020 thanhphong98 & thuanpham98

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _LOG_H_
#define _LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdarg.h>

/*
 * Log level output.
 */
typedef enum {
    STM_LOG_NONE = 0,   /*!< No log output */
    STM_LOG_ERROR,      /*!< Critical errors, software module can not recover on its own */
    STM_LOG_WARN,       /*!< Error conditions from which recovery measures have been taken */
    STM_LOG_INFO,       /*!< Information messages which describe normal flow of events */
    STM_LOG_DEBUG,      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    STM_LOG_VERBOSE     /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
} stm_log_level_t;

/*
* @brief   Initialize stm log.
* @param   huart Uart handle.
* @return  None.
*/
void stm_log_init(void);

/*
* @brief   Set log level for given tag.
* @param   tag Description tag.
* @param   level Log level output.
* @return  None.
*/
void stm_log_level_set(const char *tag, stm_log_level_t level);

/*
 * @brief   Get FreeRTOS tick count.
 * @param   None.
 * @return  timestamp in miliseconds.
 */
uint32_t stm_log_timestamp(void);

/*
 * @brief   Write message into the log. This function is not recommended to be
 *          use directly. Instead, use one of STM_LOGE, STM_LOGW, STM_LOGI,
 *          STM_LOGD, STM_LOGV macros.
 * @param   level Log level.
 * @param   tag Tag description.
 * @param   format Display format
 * @param   ... Arguments.
 * @return  None.
 */
void stm_log_write(stm_log_level_t level, const char *tag, const char *format, ...);

/** @cond */
#include "stm_log_internal.h"
/** @endcond */

/*
 * Log local level.
 */
#define LOG_LOCAL_LEVEL     5

/*
 * @brief   Log a buffer of hex bytes at specified level, separated into 16 bytes each line.
 *
 * @param   tag Description tag.
 * @param   buffer Pointer to the buffer array.
 * @param   buff_len Length of buffer in bytes.
 * @param   level Level of the log.
 *
 */
#define STM_LOG_BUFFER_HEX_LEVEL(tag, buffer, buff_len, level)              \
    do {                                                                    \
        if ( LOG_LOCAL_LEVEL >= level ) {                                   \
            stm_log_buffer_hex_internal(tag, buffer, buff_len, level);      \
        }                                                                   \
    } while(0)

/**
 * @brief   Log a buffer of characters at specified level, separated into 16 bytes each line.
 *          Buffer should contain only printable characters.
 * @param   tag Description tag.
 * @param   buffer Pointer to the buffer array.
 * @param   buff_len Length of buffer in bytes.
 * @param   level Level of the log.
 *
 */
#define STM_LOG_BUFFER_CHAR_LEVEL(tag, buffer, buff_len, level)             \
    do {                                                                    \
        if(LOG_LOCAL_LEVEL >= level) {                                      \
            stm_log_buffer_char_internal(tag, buffer, buff_len, level);     \
        }                                                                   \
    } while(0)

/*
 * @brief  Log a buffer of hex bytes at Info level.
 * @param  tag Description tag.
 * @param  buffer Pointer to the buffer array.
 * @param  buff_len Length of buffer in bytes.
 * @see    ``stm_log_buffer_hex_level``.
 */
#define STM_LOG_BUFFER_HEX(tag, buffer, buff_len)                           \
    do {                                                                    \
        if(LOG_LOCAL_LEVEL >= STM_LOG_INFO) {                               \
            STM_LOG_BUFFER_HEX_LEVEL(tag, buffer, buff_len, STM_LOG_INFO);  \
        }                                                                   \
    } while(0)

/*
 * @brief   Log a buffer of characters at Info level. Buffer should contain only printable characters.
 * @param   tag Description tag.
 * @param   buffer Pointer to the buffer array.
 * @param   buff_len Length of buffer in bytes.
 * @see     ``stm_log_buffer_char_level``.
 */
#define STM_LOG_BUFFER_CHAR(tag, buffer, buff_len)                          \
    do {                                                                    \
        if(LOG_LOCAL_LEVEL >= STM_LOG_INFO) {                               \
            STM_LOG_BUFFER_CHAR_LEVEL(tag, buffer, buff_len, STM_LOG_INFO); \
        }                                                                   \
    } while(0)

/*
 * Compatible macros in lowcase.
 */
#define stm_log_buffer_hex  STM_LOG_BUFFER_HEX
#define stm_log_buffer_char STM_LOG_BUFFER_CHAR

/*
 * This macro color base on ANSI Format. You can display your log in terminals
 * which support ANSI or VT100.
 */
#define LOG_COLOR_BLACK     "30"        /*!< Black color */
#define LOG_COLOR_RED       "31"        /*!< Red color */
#define LOG_COLOR_GREEN     "32"        /*!< Green color */
#define LOG_COLOR_YELLOW    "33"        /*!< Yellow color */
#define LOG_COLOR_BLUE      "34"        /*!< Blue color */
#define LOG_COLOR_PURPLE    "35"        /*!< Purple color */
#define LOG_COLOR_CYAN      "36"        /*!< Cyan color */

#define LOG_COLOR(COLOR)    "\033[0;" COLOR "m"             /*!< Log with color */
#define LOG_BOLD(COLOR)     "\033[1;" COLOR "m"             /*!< Log with color and bold */
#define LOG_RESET_COLOR     "\033[0m"                       /*!< Reset log output type */

/*
 * Log output color.
 */
#define LOG_COLOR_E         LOG_COLOR(LOG_COLOR_RED)        /*!< Log error color */
#define LOG_COLOR_W         LOG_COLOR(LOG_COLOR_YELLOW)     /*!< Log warning color */
#define LOG_COLOR_I         LOG_COLOR(LOG_COLOR_BLACK)      /*!< Log information color */
#define LOG_COLOR_D         LOG_COLOR(LOG_COLOR_GREEN)      /*!< Log debug color */
#define LOG_COLOR_V         LOG_COLOR(LOG_COLOR_BLUE)       /*!< Log verbose color */

/*
 * Macros to output logs.
 */
#define LOG_FORMAT(letter, format)  LOG_COLOR_ ## letter #letter " (%d) %s: " format LOG_RESET_COLOR "\r\n"

/* Macro to out put log at ERROR level */
#define STM_LOGE(tag, format, ...)  STM_LOG_LEVEL_LOCAL(STM_LOG_ERROR  , tag, format, ##__VA_ARGS__)
/* Macro to out put log at WARNNING level */
#define STM_LOGW(tag, format, ...)  STM_LOG_LEVEL_LOCAL(STM_LOG_WARN   , tag, format, ##__VA_ARGS__)
/* Macro to out put log at INFO  level */
#define STM_LOGI(tag, format, ...)  STM_LOG_LEVEL_LOCAL(STM_LOG_INFO   , tag, format, ##__VA_ARGS__)
/* Macro to out put log at DEBUG level */
#define STM_LOGD(tag, format, ...)  STM_LOG_LEVEL_LOCAL(STM_LOG_DEBUG  , tag, format, ##__VA_ARGS__)
/* Macro to out put log at VERBOSE level */
#define STM_LOGV(tag, format, ...)  STM_LOG_LEVEL_LOCAL(STM_LOG_VERBOSE, tag, format, ##__VA_ARGS__)

/*
 * Macros to output logs at specified level.
 */
#define STM_LOG_LEVEL(level, tag, format, ...) do {                                                                                                     \
        if      (level==STM_LOG_ERROR )     { stm_log_write(STM_LOG_ERROR,      tag, LOG_FORMAT(E, format), stm_log_timestamp(), tag, ##__VA_ARGS__); } \
        else if (level==STM_LOG_WARN )      { stm_log_write(STM_LOG_WARN,       tag, LOG_FORMAT(W, format), stm_log_timestamp(), tag, ##__VA_ARGS__); } \
        else if (level==STM_LOG_DEBUG )     { stm_log_write(STM_LOG_DEBUG,      tag, LOG_FORMAT(D, format), stm_log_timestamp(), tag, ##__VA_ARGS__); } \
        else if (level==STM_LOG_VERBOSE )   { stm_log_write(STM_LOG_VERBOSE,    tag, LOG_FORMAT(V, format), stm_log_timestamp(), tag, ##__VA_ARGS__); } \
        else                                { stm_log_write(STM_LOG_INFO,       tag, LOG_FORMAT(I, format), stm_log_timestamp(), tag, ##__VA_ARGS__); } \
    } while(0)

/*
 * Runtime macro to output logs at a specified level. Also check the
 * level with ``LOG_LOCAL_LEVEL``.
 */
#define STM_LOG_LEVEL_LOCAL(level, tag, format, ...) do {                                   \
        if ( LOG_LOCAL_LEVEL >= level ) STM_LOG_LEVEL(level, tag, format, ##__VA_ARGS__);   \
    } while(0)


#ifdef __cplusplus
}
#endif

#endif /* _LOG_H_ */
