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

#ifndef BSP_KEYBOARD_H
#define BSP_KEYBOARD_H
/** \brief Header para MCU
 **
 ** archivo de inicilización del microcontrolador
 **
 **/

/** \addtogroup PASE_APP_EXAMPLE
 ** @{ */
/** \addtogroup BSP KEYBOARD
 ** @{ */

/*==================[inclusions]=============================================*/
#include "stdint.h"
#include "stdbool.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
#define KEYBOARD_TASK_TIME_MS    2
#define KEYBOARD_MAX_TIME_PRESSED_DS 9999

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
extern void bsp_keyboardInit(void);

/** \brief devuelve tecla que paso de estar sin presionar a presionada desde
 **        el último llamado
 **
 ** \return    identificación de tecla correspondiente a lo definido en board.
 **            Si ninguna tecla pasó de estar sin presionar a presionada desde
 **            el último llamado, devuelve -1.
 **/
extern int32_t bsp_keyboardGet(void);

/** \brief devuelve estado de tecla.
 **
 ** Devuelve el estado de la tecla luego de transcurrido el tiempo indicado.
 ** Si la tecla estuvo presionada por un tiempo mayor o igual al indicado
 ** devuelve true, sino devuelve false.
 **
 ** \param[in] id identificación de tecla.
 ** \param[in] time tiempo que debe estar presionada para retornar true
 **            en décimas de segundo
 ** \return    true si estuvo presionada un tiempo mayor o igual al indicado
 **            de lo contrario devuelve false.
 **/
extern bool bsp_keyboardGetPressed(int32_t id, int16_t time);

/** \brief función para ejecutar tareas del módulo keyboard
 **
 **/
extern void bsp_keyboard_task(void);

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef BSP_KEYBOARD_H */

