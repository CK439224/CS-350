/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
/* Driver Header files */
#include <ti/drivers/GPIO.h>
/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/Timer.h>
/*setting TimerFlag to 0*/
volatile unsigned char TimerFlag = 0;
/*When called, it will increase the TimerFlag*/
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    /*Changed the period to 500000 500 ms = 500000 us*/
    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
         /* Failed to initialized timer */
         while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
         /* Failed to start timer */
         while (1) {}
    }
}
/* State Machine */
enum SM_STATES {
    SM_Start,
    SM_1,
    SM_O,
    SM_2,
    SM_K,
    SM_Pause,

} SM_STATE;
/* setting the ButtonClick variable */
unsigned char ButtonClick = 0;
/* setting the OkOrMorse value to default to Morse */
unsigned char OkOrMorse = 0;
void Morse_Tick(){
    static unsigned char i = 0;
    switch(SM_STATE) {
        case SM_Start:
            i = 0;
            SM_STATE = SM_1;
            break;
        /*Will change the state after 8 */
        case SM_1:
            if (!(i < 8)){
                i = 0;
                SM_STATE = SM_O;
            }
            break;
        /*Will change the state after 14 */
        case SM_O:
            if (!(i <14)){
                i = 0;
                if (OkOrMorse){
                    SM_STATE = SM_K;
                }
                else{
                    SM_STATE = SM_2;
                }
            }
            break;
        /*Will change the state after 5 */
        case SM_2:
            if (!(i <5)){
                i = 0;
                SM_STATE = SM_Pause;
            }
            break;
        /*Will change the state after 9 */
        case SM_K:
            if (!(i <9)){
                i = 0;
                        SM_STATE = SM_Pause;
            }
            break;
        /*Will change the state after 7 */

        case SM_Pause:
            if (!(i < 7)){
                i = 0;
                if (OkOrMorse){
                    SM_STATE = SM_O;
                }
                else {
                    SM_STATE = SM_1;
                }
            }
            break;
        default:
            SM_STATE = SM_Start;
            break;
       }

    switch(SM_STATE){
        case SM_Start:
            break;
        case SM_1:
            if (i == 0 || i == 2 || i ==4){
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            }
            else {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
            //i++;
            break;
        case SM_O:
            if ((i < 3) || (i > 3 && i < 7) || (i > 7 && i < 11)) {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            }
            //i++;
            break;
        case SM_2:
            if (i == 0 || i == 2 || i == 4) {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
            //i++;
            break;
        case SM_K:
            if (i < 3 || i > 5) {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            }

            if (i == 4) {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
            //i++;
            break;
        case SM_Pause:
            if (i == 0) {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            } else if (i == 6) {
                /* If a button was pressed, switch morse and reset
ButtonClick */
                if (ButtonClick) {
                    OkOrMorse = !OkOrMorse;
                    ButtonClick = 0;
                }
            }
            //i++;
            break;
        default:
            break;
    }
    i++;
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED */
    ButtonClick = 1;
}
/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle an LED */
    ButtonClick = 1;

}
/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU |
GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU |
GPIO_CFG_IN_INT_FALLING);
    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU |
GPIO_CFG_IN_INT_FALLING);
        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    SM_STATE = SM_Start;
    initTimer();
    while(1){
        Morse_Tick();
        while(!TimerFlag){}

        TimerFlag = 0;
    }
}
