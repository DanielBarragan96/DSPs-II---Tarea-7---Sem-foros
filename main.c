/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Tarea - Semaphore.c
 * @brief   Application entry point.
 */
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//for interruptions
#define PORTC_IRQ_MASK 6
#define PORTA_IRQ_MASK 4
//for interruptions
#define PORTC_IRQ_MASK 6
#define PORTA_IRQ_MASK 4
//PINS constants
#define BLUE_LED_PIN 21
#define RED_LED_PIN 22
#define GREEN_LED_PIN 26
#define SW2_PIN 6
#define SW3_PIN 4

SemaphoreHandle_t g_blue_led_semaphore;
SemaphoreHandle_t g_green_led_semaphore;

void PORTA_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    PORT_ClearPinsInterruptFlags(PORTA, 1<<PORTA_IRQ_MASK);
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( g_blue_led_semaphore, &xHigherPriorityTaskWoken );//liberar semáforo
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );//cambio de contexto si el parámetro es true
}

void PORTC_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    PORT_ClearPinsInterruptFlags(PORTC, 1<<PORTC_IRQ_MASK);
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( g_green_led_semaphore, &xHigherPriorityTaskWoken );//liberar semáforo
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );//cambio de contexto si el parámetro es true
}

void blue_led_task(void *arg)
{
    for(;;)
    {
        xSemaphoreTake(g_blue_led_semaphore,portMAX_DELAY);//espera a que liberen el semáforo
        GPIO_TogglePinsOutput(GPIOB,1<<BLUE_LED_PIN);
    }
}

void green_led_task(void *arg)
{
    for(;;)
    {
        xSemaphoreTake(g_green_led_semaphore,portMAX_DELAY);//espera a que liberen el semáforo
        GPIO_TogglePinsOutput(GPIOE,1<<GREEN_LED_PIN);
    }
}

void dummy(void *arg)
{
    xSemaphoreGive(g_blue_led_semaphore);
    for(;;)
    {

    }
}

int main(void)
{

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();

       CLOCK_EnableClock(kCLOCK_PortA);
       CLOCK_EnableClock(kCLOCK_PortB);
       CLOCK_EnableClock(kCLOCK_PortC);
       CLOCK_EnableClock(kCLOCK_PortE);
       CLOCK_EnableClock(kCLOCK_Pit0);

       // Input pin PORT configuration P-833 SDK
       const port_pin_config_t config =
               {
                   kPORT_PullDisable,
                   kPORT_FastSlewRate,
                   kPORT_PassiveFilterDisable,
                   kPORT_OpenDrainDisable,
                   kPORT_LowDriveStrength,
                   kPORT_MuxAsGpio,
                   kPORT_UnlockRegister,
               };
       const port_pin_config_t config_switch =
               {
                   kPORT_PullDisable,
                   kPORT_FastSlewRate,
                   kPORT_PassiveFilterDisable,
                   kPORT_OpenDrainDisable,
                   kPORT_LowDriveStrength,
                   kPORT_MuxAsGpio,
                   kPORT_UnlockRegister,
                 };

       // Sets the configuration
       //configure LEDs
       PORT_SetPinConfig(PORTB, BLUE_LED_PIN, &config);//BLUE
       PORT_SetPinConfig(PORTB, RED_LED_PIN, &config);//RED
       PORT_SetPinConfig(PORTE, GREEN_LED_PIN, &config);//GREEN
       //configure Switches
       PORT_SetPinConfig(PORTA, SW3_PIN, &config_switch);//SW3
       PORT_SetPinConfig(PORTC, SW2_PIN, &config_switch);//SW2

       //Output pin configuration
       const gpio_pin_config_t led_config =
           {
               kGPIO_DigitalOutput,//configurar como output
               1,//valor default
           };
       const gpio_pin_config_t switch_config =
             {
               kGPIO_DigitalInput,//configurar como output
               1,//valor default
             };

       // Sets the configuration
       GPIO_PinInit(GPIOB, BLUE_LED_PIN, &led_config);
       GPIO_PinInit(GPIOB, RED_LED_PIN, &led_config);
       GPIO_PinInit(GPIOE, GREEN_LED_PIN, &led_config);
       GPIO_PinInit(GPIOA, SW3_PIN, &switch_config);
       GPIO_PinInit(GPIOC, SW2_PIN, &switch_config);

       //enable switches interrupt
       PORT_SetPinInterruptConfig(PORTA, SW3_PIN, kPORT_InterruptFallingEdge);
       PORT_SetPinInterruptConfig(PORTC, SW2_PIN, kPORT_InterruptFallingEdge);

       //enable interruptions
       NVIC_EnableIRQ(PORTA_IRQn);
       NVIC_SetPriority(PORTA_IRQn,5);
       NVIC_EnableIRQ(PORTC_IRQn);
       NVIC_SetPriority(PORTC_IRQn,5);

    g_blue_led_semaphore = xSemaphoreCreateBinary();
    g_green_led_semaphore = xSemaphoreCreateBinary();


    xTaskCreate(blue_led_task, "blue task", 110, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(green_led_task, "green task", 110, NULL, configMAX_PRIORITIES-1, NULL);

    vTaskStartScheduler();

    while (1)
    {

    }
    return 0;
}
