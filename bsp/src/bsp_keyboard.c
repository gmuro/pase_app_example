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

#ifndef BOARD_TEC_ID_TOTAL
#define BOARD_TEC_ID_TOTAL    0
#endif

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
static uint32_t time_counter = 0;
static board_switchState_enum keys_states[BOARD_TEC_ID_TOTAL];
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void init_keys_states(void)
{
   uint8_t i;
   for(i = 0; i < BOARD_TEC_ID_TOTAL;i++)
   {
      keys_states[i] = BOARD_TEC_NON_PRESSED;
   }
}
extern void bsp_keyboardInit(void)
{

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
   return -1;
}

extern bool bsp_keyboardGetPressed(int32_t id, int32_t time)
{
   return false;
}

extern void bsp_keyboard_task(void)
{

}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
