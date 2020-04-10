#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "sys/queue.h"
#include "stm_log.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

/* Print number of bytes per line for stm_log_buffer_char and stm_log_buffer_hex */
#define BYTES_PER_LINE      16

/* Number of tags to be cached. Must be 2**n - 1, n >= 2. */
#define TAG_CACHE_SIZE 31

/* Buffer for UART */
#define LOG_BUF_SIZE        256


/* Maximum time to wait for the mutex in a logging statement */
#define MAX_MUTEX_WAIT_MS 10
#define MAX_MUTEX_WAIT_TICKS ((MAX_MUTEX_WAIT_MS + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)

/* FreeRTOS mutex macros */
#define mutex_lock(x)       while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS);
#define mutex_unlock(x)     xSemaphoreGive(x)
#define mutex_create()      xSemaphoreCreateMutex()
#define mutex_destroy(x)    vQueueDelete(x)

/* UART */
static UART_HandleTypeDef huart_log;
static char log_buf[LOG_BUF_SIZE];

/*
 * Cached tag entry.
 */
typedef struct {
    const char* tag;
    uint32_t level : 3;
    uint32_t generation : 29;
} cached_tag_entry_t;

/*
 * Uncached tag entry.
 */
typedef struct uncached_tag_entry_ {
    SLIST_ENTRY(uncached_tag_entry_) entries;
    uint8_t level;
    char tag[0];
} uncached_tag_entry_t;

/* Variable for set log level */
static stm_log_level_t set_log_default_level = STM_LOG_VERBOSE;
static SLIST_HEAD(log_tags_head, uncached_tag_entry_) set_log_tags = SLIST_HEAD_INITIALIZER(set_log_tags);
static uint32_t set_log_cache_max_generation = 0;
static uint32_t set_log_cache_entry_count = 0;
static cached_tag_entry_t set_log_cache[TAG_CACHE_SIZE];
static SemaphoreHandle_t set_log_lock = NULL;

/* Static functions */
static inline bool get_cached_log_level(const char* tag, stm_log_level_t* level);
static inline bool get_uncached_log_level(const char* tag, stm_log_level_t* level);
static inline void add_to_cache(const char* tag, stm_log_level_t level);
static void heap_bubble_down(int index);
static inline void heap_swap(int i, int j);
static inline bool should_output(stm_log_level_t level_for_message, stm_log_level_t level_for_tag);
static inline void clear_log_level_list();

void stm_log_init(void)
{
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    huart_log.Instance = UART4;
    huart_log.Init.BaudRate = 115200;
    huart_log.Init.WordLength = UART_WORDLENGTH_8B;
    huart_log.Init.StopBits = UART_STOPBITS_1;
    huart_log.Init.Parity = UART_PARITY_NONE;
    huart_log.Init.Mode = UART_MODE_TX_RX;
    huart_log.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart_log.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart_log);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void stm_log_level_set(const char *tag, stm_log_level_t level)
{
    if (!set_log_lock)
    {
        set_log_lock = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(set_log_lock, portMAX_DELAY);

    /* For wildcard tag, remove all linked list item and clear the cache */
    if (strcmp(tag, "*") == 0)
    {
        set_log_default_level = level;
        clear_log_level_list();
        xSemaphoreGive(set_log_lock);
        return;
    }

    /* Searching exist tag */
    uncached_tag_entry_t *it = NULL;
    SLIST_FOREACH(it, &set_log_tags, entries)
    {
        if (strcmp(it->tag, tag) == 0)
        {
            /* Update level for tag already been in linked list */
            it->level = level;
            break;
        }
    }

    /* Append new one if have no exist tag */
    if (it == NULL)
    {
        /* Allocate memory for linked list entry and append it to the head of the list */
        size_t entry_size = offsetof(uncached_tag_entry_t, tag) + strlen(tag) + 1;
        uncached_tag_entry_t *new_entry = (uncached_tag_entry_t*)malloc(entry_size);
        if (!new_entry)
        {
            xSemaphoreGive(set_log_lock);
            return;
        }
        new_entry->level = (uint8_t)level;
        strcpy(new_entry->tag, tag);
        SLIST_INSERT_HEAD(&set_log_tags, new_entry, entries);
    }

    /* Searching in the cache and update */
    for (int i = 0; i < set_log_cache_entry_count; ++i)
    {
        if (strcmp(set_log_cache[i].tag, tag) == 0)
        {
            set_log_cache[i].level = level;
            break;
        }
    }
    xSemaphoreGive(set_log_lock);
}

static void clear_log_level_list(void)
{
    uncached_tag_entry_t *it;
    while ((it = SLIST_FIRST(&set_log_tags)) != NULL) {
        SLIST_REMOVE_HEAD(&set_log_tags, entries);
        free(it);
    }

    /* Reset log cache counter */
    set_log_cache_entry_count = 0;
    set_log_cache_max_generation = 0;
}

void stm_log_write(stm_log_level_t level, const char* tag, const char* format, ...)
{
    if (!set_log_lock)
    {
        set_log_lock = xSemaphoreCreateMutex();
    }

    if (xSemaphoreTake(set_log_lock, MAX_MUTEX_WAIT_TICKS) == pdFALSE)
    {
        return;
    }

    /* Look or the tag in cache first, then in the linked va_list of all tags */
    stm_log_level_t level_for_tag;
    if (!get_cached_log_level(tag, &level_for_tag))
    {
        if (!get_uncached_log_level(tag, &level_for_tag))
        {
            level_for_tag = set_log_default_level;
        }
        add_to_cache(tag, level_for_tag);
    }

    xSemaphoreGive(set_log_lock);
    if (!should_output(level, level_for_tag))
    {
        return;
    }

    va_list list;
    va_start(list, format);
    uint16_t length = vsnprintf((char*)log_buf, 128, format, list);
    HAL_UART_Transmit(&huart_log, (uint8_t*)log_buf, length, 100);
    va_end(list);
}

static inline bool get_cached_log_level(const char *tag, stm_log_level_t *level)
{
    /* Look for tag in cache */
    int i;
    for (i = 0; i < set_log_cache_entry_count; ++i)
    {
        if (set_log_cache[i].tag == tag) {
            break;
        }
    }

    if (i == set_log_cache_entry_count)
    {
        return false;
    }

    /* Return level from cache */
    *level = (stm_log_level_t) set_log_cache[i].level;

    /* If cache has been filled, start taking ordering into account
    (other options are: dynamically resize cache, add "dummy" entries
     to the cache; this option was chosen because code is much simpler,
     and the unfair behavior of cache will show it self at most once, when
     it has just been filled) */
    if (set_log_cache_entry_count == TAG_CACHE_SIZE)
    {
        /* Update item generation */
        set_log_cache[i].generation = set_log_cache_max_generation++;

        /* Restore heap odering */
        heap_bubble_down(i);
    }
    return true;
}

static inline void add_to_cache(const char *tag, stm_log_level_t level)
{
    uint32_t generation = set_log_cache_max_generation++;

    /* First consider the case when cache is not filled yet.
    In this case, just add new entry at the end.
    This happens to satisfy binary min-heap ordering. */
    if (set_log_cache_entry_count < TAG_CACHE_SIZE)
    {
        set_log_cache[set_log_cache_entry_count] = (cached_tag_entry_t) {
            .generation = generation,
            .level = level,
            .tag = tag
        };
        ++set_log_cache_entry_count;
        return;
    }

    /* Cache is full, so we replace the oldest entry (which is at index 0
    because this is a min-heap) with the new one, and do bubble-down
    operation to restore min-heap ordering. */
    set_log_cache[0] = (cached_tag_entry_t) {
        .tag = tag,
        .level = level,
        .generation = generation
    };
    heap_bubble_down(0);
}

static inline bool get_uncached_log_level(const char *tag, stm_log_level_t *level)
{
    /* Walk the linked list of all tags and see if given tag is present in the list.
    This is slow because tags are compared as strings. */
    uncached_tag_entry_t *it;
    SLIST_FOREACH(it, &set_log_tags, entries)
    {
        if (strcmp(tag, it->tag) == 0)
        {
            *level = it->level;
            return true;
        }
    }
    return false;
}

static inline bool should_output(stm_log_level_t level_for_message, stm_log_level_t level_for_tag)
{
    return level_for_message <= level_for_tag;
}

static void heap_bubble_down(int index)
{
    while (index < TAG_CACHE_SIZE / 2)
    {
        int left_index = index * 2 + 1;
        int right_index = left_index + 1;
        int next = (set_log_cache[left_index].generation < set_log_cache[right_index].generation) ? left_index : right_index;
        heap_swap(index, next);
        index = next;
    }
}

static inline void heap_swap(int i, int j)
{
    cached_tag_entry_t tmp = set_log_cache[i];
    set_log_cache[i] = set_log_cache[j];
    set_log_cache[j] = tmp;
}

uint32_t stm_log_timestamp(void)
{
    return HAL_GetTick();

}

void stm_log_buffer_hex_internal(const char *tag,
                                 const char *buffer,
                                 uint16_t buff_len,
                                 stm_log_level_t log_level)
{
    if (buff_len == 0) return;
    char temp_buffer[BYTES_PER_LINE + 3];
    char hex_buffer[3 * BYTES_PER_LINE + 1];
    int bytes_current_line;

    do {
        if ( buff_len > BYTES_PER_LINE)
        {
            bytes_current_line = BYTES_PER_LINE;
        }
        else
        {
            bytes_current_line = buff_len;
        }

        memcpy(temp_buffer, buffer, (bytes_current_line + 3) / 4 * 4);

        for (int i = 0; i < buff_len; i++)
        {
            sprintf(&hex_buffer[i * 3], "%02x ", buffer[i]);
        }

        STM_LOG_LEVEL(log_level, tag, "%s", hex_buffer);
        buff_len -= bytes_current_line;
        buffer += bytes_current_line;
    } while (buff_len);
}

void stm_log_buffer_char_internal(const char *tag,
                                  const char *buffer,
                                  uint16_t buff_len,
                                  stm_log_level_t log_level)
{
    if (buff_len == 0) return;
    char temp_buffer[BYTES_PER_LINE + 3];
    char char_buffer[BYTES_PER_LINE + 1];
    int bytes_current_line;

    do {
        if (buff_len > BYTES_PER_LINE)
        {
            bytes_current_line = BYTES_PER_LINE;
        }
        else
        {
            bytes_current_line = buff_len;
        }

        memcpy(temp_buffer, buffer, (bytes_current_line + 3) / 4 * 4);

        for (int i = 0; i < buff_len; i++)
        {
            sprintf(&char_buffer[i], "%c ", buffer[i]);
        }

        STM_LOG_LEVEL(log_level, tag, "%s", char_buffer);
        buffer += bytes_current_line;
        buff_len -= bytes_current_line;
    } while (buff_len);
}


