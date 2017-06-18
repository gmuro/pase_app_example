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
#include "stdint.h"
/*==================[macros and definitions]=================================*/

#ifndef BOARD_TEC_ID_TOTAL
#define BOARD_TEC_ID_TOTAL    0
#endif

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
static uint32_t time_counter = 0;
static board_switchState_enum keys_states[BOARD_TEC_ID_TOTAL];
typedef struct
{
   int32_t counter;
   int32_t time;
   bool enable;
}soft_timer_t;

static soft_timer_t soft_timers[BOARD_TEC_ID_TOTAL];

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void start_soft_timer(int32_t id, int32_t time)
{
   soft_timers[id].counter = 0;
   soft_timers[id].time = (time/KEYBOARD_TASK_TIME_MS) + 1;
   soft_timers[id].enable = true;
}

void stop_soft_timer(int32_t id)
{
   soft_timers[id].enable = false;
}

bool is_timer_finish(int32_t id)
{
   return ((soft_timers[id].counter - soft_timers[id].time) > 0);
}

void init_keys_arrays(void)
{
   uint8_t i;
   for(i = 0; i < BOARD_TEC_ID_TOTAL;i++)
   {
      keys_states[i] = BOARD_TEC_NON_PRESSED;
      soft_timers[i].enable = false;
   }
}
extern void bsp_keyboardInit(void)
{
   init_keys_arrays();
}

board_switchId_enum check_pin_change(void)
{
   uint8_t i = 0;
   board_switchId_enum key_id = -1;
   board_switchState_enum state;
   for(i = 0; i < BOARD_TEC_ID_TOTAL;i++)
   {
      state = board_switchGet(i);
      if(state != keys_states[i])
      {
         keys_states[i] = state;
         key_id = i;
         break;
      }
   }
   return key_id;
}

extern int32_t bsp_keyboardGet(void)
{
   return check_pin_change();
}

extern bool bsp_keyboardGetPressed(int32_t id, int32_t time)
{
   //Time is expressed as a mS value. time_counter is how many times
   //bsp_keyboard_task has been called each KEYBOARD_TASK_TIME_MS mS.
   int32_t end = (time_counter * KEYBOARD_TASK_TIME_MS) + time;

   //Local_counter is used to not query GPIO at MCU_CLK
   int32_t local_counter = time_counter;

   board_switchState_enum init_state = board_switchGet(id);
   board_switchState_enum next_state = init_state;

   while(time_counter < end)
   {
      if(time_counter != local_counter)
      {
         next_state = board_switchGet(id);
         local_counter = time_counter;
         if(init_state != next_state)
         {
            return false;
         }
      }
   }
   return true;
}

extern void bsp_keyboard_task(void)
{
   time_counter++;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
