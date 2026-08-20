/*
 * profiler.c
 *
 *  Created on: Aug 20, 2026
 *      Author: mason
 */


#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "global_board_config.h"

#ifdef PROFILER_ENABLE

#define DWT_LAR (*(volatile uint32_t *)0xE0001FB0UL)
#define PROFILER_MAX_TASKS  20

static TaskStatus_t s_status[PROFILER_MAX_TASKS];
static uint32_t     s_prevRun[PROFILER_MAX_TASKS];
static uint32_t     s_prevTotal;
static char         s_line[80];

void configureTimerForRunTimeStats(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT_LAR = 0xC5ACCE55UL;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

unsigned long getRunTimeCounterValue(void)
{
    return (unsigned long)(DWT->CYCCNT >> 8);
}

/* Bounded ITM write. Returns immediately if no debugger has enabled
 * the port, so this is safe to leave compiled in when running standalone. */
static void itm_puts(const char *s)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0u) return;
    if ((ITM->TER & 1u) == 0u)                 return;

    while (*s) {
        uint32_t guard = 10000u;
        while ((ITM->PORT[0].u32 == 0u) && (--guard != 0u)) { }
        if (guard == 0u) return;              /* FIFO stuck: give up */
        ITM->PORT[0].u8 = (uint8_t)*s++;
    }
}

void profiler_report(void)
{
    uint32_t total;
    UBaseType_t n = uxTaskGetSystemState(s_status, PROFILER_MAX_TASKS, &total);

    uint32_t dTotal = total - s_prevTotal;     /* wrap-safe */
    s_prevTotal = total;
    if (dTotal == 0u) return;

    itm_puts("\r\nTask              CPU%   Stack\r\n");

    for (UBaseType_t i = 0; i < n; i++) {
        UBaseType_t num = s_status[i].xTaskNumber;
        if (num >= PROFILER_MAX_TASKS) continue;

        uint32_t dRun = s_status[i].ulRunTimeCounter - s_prevRun[num];
        s_prevRun[num] = s_status[i].ulRunTimeCounter;

        uint32_t pct = (uint32_t)(((uint64_t)dRun * 1000u) / dTotal);

        snprintf(s_line, sizeof(s_line), "%-16s %3lu.%lu%%  %5u\r\n",
                 s_status[i].pcTaskName,
                 (unsigned long)(pct / 10u),
                 (unsigned long)(pct % 10u),
                 (unsigned int)s_status[i].usStackHighWaterMark);

        itm_puts(s_line);
    }
}

#endif
