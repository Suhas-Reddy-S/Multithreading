/*
 * @file seqgen.c
 * @Author Suhas Reddy and Rajesh
 *
 * References: https://www.freertos.org/a00106.html,
 *             TiVA TM4C123GXL reference manual/datasheet
 *             Tim Scherr, Sam Siewert:- https://github.com/siewertsmooc/RTES-ECEE-5623
 *
 * Date: 9th April 2024
 *
 * */

#define seqgen (1)
#if seqgen

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stdbool.h"

int fibCnt = 300;
unsigned int idx = 0, jdx = 1;
unsigned int seqIterations = 47;
volatile unsigned int fib = 0, fib0 = 0, fib1 = 1;
// Authored by Nisheeth Bhatt modified by Suhas and Rajesh
#define FIB_TEST(seqCnt, iterCnt)      \
   for(idx=0; idx < iterCnt; idx++)    \
   {                                   \
      fib = fib0 + fib1;               \
      while(jdx < seqCnt)              \
      {                                \
         fib0 = fib1;                  \
         fib1 = fib;                   \
         fib = fib0 + fib1;            \
         jdx++;                        \
      }                                \
      jdx = 0;                         \
   };                                  \

xSemaphoreHandle semSched, semS1, semS2, semS3, semS4, semS5, semS6, semS7;
bool abortS1 = false, abortS2 = false, abortS3 = false, abortS4 = false, abortS5 = false, abortS6 = false, abortS7 = false;
uint32_t wcet[7] = {0, 0, 0, 0, 0, 0, 0}, T[8] = {1000/30, 1000/3, 1000, 2000, 1000, 2000, 1000, 10000};

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    while(1)
    {
    }
}

void Timer0IntHandler(void)
{
    static volatile uint32_t g_tick = 0;
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    g_tick++;

    if (g_tick % 33 == 0) {
        xSemaphoreGive(semSched);  // release sequencer semaphore every 33 milliseconds
    }

}

void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

// Runs @ 30Hz
static void Sequencer_thread(void *params)
{
    uint32_t startTime;
    static volatile uint32_t schedCnt = 0;
    startTime = xTaskGetTickCount();
    UARTprintf("Sequencer Started at %u msec\n", startTime);
    while (schedCnt < 900) {
        xSemaphoreTake(semSched, portMAX_DELAY);
        if(schedCnt % 10 == 0) {
            xSemaphoreGive(semS1);
        }
        if(schedCnt % 30 == 0) {
            xSemaphoreGive(semS2);
        }
        if(schedCnt % 60 == 0) {
            xSemaphoreGive(semS3);
        }
        if(schedCnt % 30 == 0) {
            xSemaphoreGive(semS4);
        }
        if(schedCnt % 60 == 0) {
            xSemaphoreGive(semS5);
        }
        if(schedCnt % 30 == 0) {
            xSemaphoreGive(semS6);
        }
        if(schedCnt % 300 == 0) {
            xSemaphoreGive(semS7);
        }
        schedCnt++;
    }
    abortS1 = true, abortS2 = true, abortS3 = true, abortS4 = true, abortS5 = true, abortS6 = true, abortS7 = true;
    int i = 0;
    UARTprintf("Sequencer Thread: Time Period = %u msec\n", T[i]);
    for (i = 1; i < 8; i++) {
        UARTprintf("Service %d: Time Period = %u msec, Deadline = %u msec, WCET = %u msec\n", i, T[i], T[i], wcet[i-1]);
    }
    UARTprintf("\n");
}

// Runs @ 3Hz
static void service1(void *params)
{
    uint32_t startTime, execTime;

    while (!abortS1) {
        xSemaphoreTake(semS1, portMAX_DELAY);
        startTime = xTaskGetTickCount();
        FIB_TEST(seqIterations, fibCnt);
        execTime = xTaskGetTickCount() - startTime;
        if(execTime > wcet[0]) { wcet[0] = execTime;}
    }
}

// Runs @ 1Hz
static void service2(void *params)
{
    uint32_t startTime, execTime;

    while (!abortS2) {
        xSemaphoreTake(semS2, portMAX_DELAY);
        startTime = xTaskGetTickCount();
        FIB_TEST(seqIterations, fibCnt);
        execTime = xTaskGetTickCount() - startTime;
        if(execTime > wcet[1]) { wcet[1] = execTime;}
    }
}

// Runs @ 0.5Hz
static void service3(void *params)
{
    uint32_t startTime, execTime;

    while (!abortS3) {
        xSemaphoreTake(semS3, portMAX_DELAY);
        startTime = xTaskGetTickCount();
        FIB_TEST(seqIterations, fibCnt);
        execTime = xTaskGetTickCount() - startTime;
        if(execTime > wcet[2]) { wcet[2] = execTime;}
    }
}

// Runs @ 1Hz
static void service4(void *params)
{
    uint32_t startTime, execTime;

    while (!abortS4) {
        xSemaphoreTake(semS4, portMAX_DELAY);
        startTime = xTaskGetTickCount();
        FIB_TEST(seqIterations, fibCnt);
        execTime = xTaskGetTickCount() - startTime;
        if(execTime > wcet[3]) { wcet[3] = execTime;}
    }
}

// Runs @ 0.5Hz
static void service5(void *params)
{
    uint32_t startTime, execTime;

    while (!abortS5) {
        xSemaphoreTake(semS5, portMAX_DELAY);
        startTime = xTaskGetTickCount();
        FIB_TEST(seqIterations, fibCnt);
        execTime = xTaskGetTickCount() - startTime;
        if(execTime > wcet[4]) { wcet[4] = execTime;}
    }
}

// Runs @ 1Hz
static void service6(void *params)
{
    uint32_t startTime, execTime;

    while (!abortS6) {
        xSemaphoreTake(semS6, portMAX_DELAY);
        startTime = xTaskGetTickCount();
        FIB_TEST(seqIterations, fibCnt);
        execTime = xTaskGetTickCount() - startTime;
        if(execTime > wcet[5]) { wcet[5] = execTime;}
    }
}

// Runs @ 0.1Hz
static void service7(void *params)
{
    uint32_t startTime, execTime;

    while (!abortS7) {
        xSemaphoreTake(semS7, portMAX_DELAY);
        startTime = xTaskGetTickCount();
        FIB_TEST(seqIterations, fibCnt);
        execTime = xTaskGetTickCount() - startTime;
        if(execTime > wcet[6]) { wcet[6] = execTime;}
    }
}

int main(void)
{
    ROM_FPULazyStackingEnable();

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                           SYSCTL_XTAL_16MHZ);

    ConfigureUART();
    UARTprintf("\nRTES Exercise 5 Q2 - seqgen.c\n\n");

// Initialize semaphores
    semSched = xSemaphoreCreateMutex();
    semS1 = xSemaphoreCreateMutex();
    semS2 = xSemaphoreCreateMutex();
    semS3 = xSemaphoreCreateMutex();
    semS4 = xSemaphoreCreateMutex();
    semS5 = xSemaphoreCreateMutex();
    semS6 = xSemaphoreCreateMutex();
    semS7 = xSemaphoreCreateMutex();

// Restrict services to start before sequencer
    xSemaphoreTake(semS1, portMAX_DELAY);
    xSemaphoreTake(semS2, portMAX_DELAY);
    xSemaphoreTake(semS3, portMAX_DELAY);
    xSemaphoreTake(semS4, portMAX_DELAY);
    xSemaphoreTake(semS5, portMAX_DELAY);
    xSemaphoreTake(semS6, portMAX_DELAY);
    xSemaphoreTake(semS7, portMAX_DELAY);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()/1000);   // Configure timer0A to run at 1000HZ

    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    ROM_TimerEnable(TIMER0_BASE, TIMER_A);

    xTaskCreate(Sequencer_thread, "Sequencer Thread", 128, NULL, tskIDLE_PRIORITY + 5, NULL);
    xTaskCreate(service1, "Service 1", 128, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(service2, "Service 2", 128, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(service3, "Service 3", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(service4, "Service 4", 128, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(service5, "Service 5", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(service6, "Service 6", 128, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(service7, "Service 7", 128, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    while(1)
    {
    }
}

#endif
