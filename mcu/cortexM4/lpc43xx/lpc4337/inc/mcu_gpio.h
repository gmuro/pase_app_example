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

#ifndef MCU_GPIO_H
#define MCU_GPIO_H
/** \brief Header para MCU
 **
 ** archivo de inicilización del microcontrolador
 **
 **/

/** \addtogroup PASE_APP_EXAMPLE
 ** @{ */
/** \addtogroup MCU GPIO
 ** @{ */

/*==================[inclusions]=============================================*/
#include "stdbool.h"
#include "stdint.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

typedef enum
{
   MCU_GPIO_PIN_ID_75 = 0,
   MCU_GPIO_PIN_ID_81,
   MCU_GPIO_PIN_ID_84,
   MCU_GPIO_PIN_ID_104,
   MCU_GPIO_PIN_ID_105,
   MCU_GPIO_PIN_ID_106,
   MCU_GPIO_PIN_ID_38,
   MCU_GPIO_PIN_ID_42,
   MCU_GPIO_PIN_ID_43,
   MCU_GPIO_PIN_ID_49,
}mcu_gpio_pinId_enum;

typedef enum
{
   MCU_GPIO_DIRECTION_INPUT = 0,
   MCU_GPIO_DIRECTION_OUTPUT,
}mcu_gpio_direction_enum;

typedef enum
{
   MCU_GPIO_EVENT_TYPE_INPUT_FALLING_EDGE = 0,
   MCU_GPIO_EVENT_TYPE_INPUT_RISING_EDGE,
}mcu_gpio_eventTypeInput_enum;

typedef void (*mcu_gpio_eventInput_callBack_type)(mcu_gpio_pinId_enum id, mcu_gpio_eventTypeInput_enum evType);

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
extern void mcu_gpio_init(void);
extern void mcu_gpio_setDirection(mcu_gpio_pinId_enum id, mcu_gpio_direction_enum dir);
extern void mcu_gpio_setOut(mcu_gpio_pinId_enum id, bool state);
extern void mcu_gpio_toggleOut(mcu_gpio_pinId_enum id);
extern bool mcu_gpio_readPin(mcu_gpio_pinId_enum id);
extern int32_t mcu_gpio_setEventInput(mcu_gpio_pinId_enum id,
                                      mcu_gpio_eventTypeInput_enum evType,
                                      mcu_gpio_eventInput_callBack_type cb);
/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef MCU_GPIO_H */

