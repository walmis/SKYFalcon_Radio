/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    STM32L1xx/cmparams.h
 * @brief   ARM Cortex-M3 parameters for the STM32L1xx.
 *
 * @defgroup ARMCMx_STM32L1xx STM32L1xx Specific Parameters
 * @ingroup ARMCMx_SPECIFIC
 * @details This file contains the Cortex-M3 specific parameters for the
 *          STM32L1xx platform.
 * @{
 */

#ifndef CMPARAMS_H
#define CMPARAMS_H

/**
 * @brief   Cortex core model.
 */
#define CORTEX_MODEL            0

/**
 * @brief   Floating Point unit presence.
 */
#define CORTEX_HAS_FPU          0

/**
 * @brief   Number of bits in priority masks.
 */
#define CORTEX_PRIORITY_BITS    3

/**
 * @brief   Number of interrupt vectors.
 * @note    This number does not include the 16 system vectors and must be
 *          rounded to a multiple of 8.
 */
#define CORTEX_NUM_VECTORS      32

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

/* Including the device CMSIS header. Note, we are not using the definitions
   from this header because we need this file to be usable also from
   assembler source files. We verify that the info matches instead.*/
#include <LPC11Uxx.h>

/*lint -save -e9029 [10.4] Signedness comes from external files, it is
  unpredictable but gives no problems.*/
#if CORTEX_MODEL != __CORTEX_M
#error "CMSIS __CORTEX_M mismatch"
#endif

#if CORTEX_PRIORITY_BITS != __NVIC_PRIO_BITS
#error "CMSIS __NVIC_PRIO_BITS mismatch"
#endif
/*lint -restore*/


#endif /* !defined(_FROM_ASM_) */

#endif /* CMPARAMS_H */

/** @} */
 
