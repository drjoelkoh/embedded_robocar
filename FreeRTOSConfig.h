#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "pico/stdlib.h"

#define configUSE_PREEMPTION            1
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0
#define configCPU_CLOCK_HZ              125000000 // RP2040 default clock
#define configTICK_RATE_HZ              1000
#define configMAX_PRIORITIES            5
#define configMINIMAL_STACK_SIZE        128
#define configTOTAL_HEAP_SIZE           (10 * 1024)
#define configMAX_TASK_NAME_LEN         16
#define configUSE_TRACE_FACILITY        1
#define configUSE_16_BIT_TICKS          0
#define configIDLE_SHOULD_YIELD         1
#define configUSE_MUTEXES               1
#define configQUEUE_REGISTRY_SIZE       10
#define configCHECK_FOR_STACK_OVERFLOW  2
#define configUSE_RECURSIVE_MUTEXES     1
#define configUSE_MALLOC_FAILED_HOOK    1
#define configUSE_COUNTING_SEMAPHORES   1
#define configUSE_TIMERS                1
#define configTIMER_TASK_PRIORITY       2
#define configTIMER_QUEUE_LENGTH        5
#define configTIMER_TASK_STACK_DEPTH    256

/* Interrupt nesting behavior configuration. */
#define configKERNEL_INTERRUPT_PRIORITY         (7 << 5)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (5 << 5)
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 7
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define vPortSVCHandler svcall_handler
#define xPortPendSVHandler pendsv_handler
#define xPortSysTickHandler systick_handler

#endif /* FREERTOS_CONFIG_H */
