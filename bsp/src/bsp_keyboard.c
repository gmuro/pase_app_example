/* Copyright 2017, Gustavo Muro
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief source para MCU
 **
 ** archivo de inicilización del microcontrolador
 **
 **/

/** \addtogroup PASE_APP_EXAMPLE
 ** @{ */
/** \addtogroup MCU
 ** @{ */

/*==================[inclusions]=============================================*/
#include "board.h"
#include "bsp_keyboard.h"

/*==================[macros and definitions]=================================*/

#define TIME_DEBOAUNCE_ACT       20
#define TIME_DEBOAUNCE_DEACT     20

#define convertTimeMsToInternalTime(time)    (time/KEYBOARD_TASK_TIME_MS)

typedef enum
{
   KEY_STATE_WAIT_PRESS = 0,
   KEY_STATE_DEBOUNCE_ACT,
   KEY_STATE_WAIT_NO_PRESS,
   KEY_STATE_DEBOUNCE_DEACT,
}stateKey_enum;

typedef struct
{
   stateKey_enum state;
   int16_t timerDebounce;
   int16_t timerUp;
}varsKey_type;

/*==================[internal data declaration]==============================*/
static varsKey_type varsKey[BOARD_TEC_ID_TOTAL];
static int32_t idKeyPressed;
static int16_t prescaler100ms;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
extern void bsp_keyboardInit(void)
{
   int32_t i;

   for (i = 0 ; i < BOARD_TEC_ID_TOTAL ; i++)
   {
      varsKey[i].state = KEY_STATE_WAIT_PRESS;
      varsKey[i].timerDebounce = 0;
      varsKey[i].timerUp = 0;
   }

   prescaler100ms = 0;
   idKeyPressed = -1;
}

extern int32_t bsp_keyboardGet(void)
{
   int32_t ret;

   ret = idKeyPressed;

   idKeyPressed = -1;

   return ret;
}

extern bool bsp_keyboardGetPressed(int32_t id, int16_t time)
{
   bool ret = false;

   if (varsKey[id].timerUp >= time)
      ret = true;

   return ret;
}

extern void bsp_keyboard_task(void)
{
   int32_t i;

   if (prescaler100ms)
      prescaler100ms--;
   else
      prescaler100ms = convertTimeMsToInternalTime(100);

   for (i = 0 ; i < BOARD_TEC_ID_TOTAL ; i++)
   {
      switch (varsKey[i].state)
      {
         case KEY_STATE_WAIT_PRESS:
            if (board_switchGet(i) == BOARD_TEC_PRESSED)
            {
               varsKey[i].timerDebounce = convertTimeMsToInternalTime(TIME_DEBOAUNCE_ACT);
               varsKey[i].state = KEY_STATE_DEBOUNCE_ACT;
            }
            break;

         case KEY_STATE_DEBOUNCE_ACT:
            if (board_switchGet(i) == BOARD_TEC_NON_PRESSED)
            {
               varsKey[i].state = KEY_STATE_WAIT_PRESS;
            }
            else if (varsKey[i].timerDebounce)
            {
               varsKey[i].timerDebounce--;
            }
            else
            {
               idKeyPressed = BOARD_TEC_ID_1+i;
               varsKey[i].timerUp = 0;
               varsKey[i].state = KEY_STATE_WAIT_NO_PRESS;
            }
            break;

         case KEY_STATE_WAIT_NO_PRESS:
            if (board_switchGet(i) == BOARD_TEC_NON_PRESSED)
            {
               varsKey[i].timerDebounce = convertTimeMsToInternalTime(TIME_DEBOAUNCE_DEACT);
               varsKey[i].state = KEY_STATE_DEBOUNCE_DEACT;
            }
            break;

         case KEY_STATE_DEBOUNCE_DEACT:
            if (board_switchGet(i) == BOARD_TEC_PRESSED)
            {
               varsKey[i].state = KEY_STATE_WAIT_NO_PRESS;
            }
            else if (varsKey[i].timerDebounce)
            {
               varsKey[i].timerDebounce--;
            }
            else
            {
               varsKey[i].state = KEY_STATE_WAIT_PRESS;
            }
            break;

         default:
            varsKey[i].state = KEY_STATE_WAIT_PRESS;
            break;
      }

      if (varsKey[i].state == KEY_STATE_WAIT_NO_PRESS)
      {
         if ((varsKey[i].timerUp < KEYBOARD_MAX_TIME_PRESSED_DS) && (prescaler100ms == 0))
            varsKey[i].timerUp++;
      }
      else
      {
         varsKey[i].timerUp = 0;
      }
   }
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
