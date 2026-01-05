/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @file
 * @brief  Pin definition for the target board.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALTargetV1
 *
 * @{
 */
#ifndef PIN_H
#define PIN_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>
#include <Io.hpp>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Pin number of all used pins. */
namespace Pin
{
    /** Pin for push button for system reset/AP mode start (ACTIVE LOW) */
    constexpr uint8_t PIN_WIFI_AND_RESET_KEY = 48U;

    /** Pin for resetting the attached Zumo robot (ACTIVE LOW) */
    constexpr uint8_t PIN_DEVICE_RESET = 5U;

    /** Pin for info LED RGB channel RED (ACTIVE LOW) */
    constexpr uint8_t INFO_LED_R = 38U;

    /** Pin for info LED RGB channel GREEN (ACTIVE LOW) */
    constexpr uint8_t INFO_LED_G = 1U;

    /** Pin for info LED RGB channel BLUE (ACTIVE LOW) */
    constexpr uint8_t INFO_LED_B = 2U;

    /** Pin for analog measurement of battery voltage */
    constexpr uint8_t PIN_BATT_MEASUREMENT = 10U;

    /** Pin for push button A */
    constexpr uint8_t PIN_BUTTON_A = 7U;

    /** Pin for push button B */
    constexpr uint8_t PIN_BUTTON_B = 15U;

    /** Pin for push button C */
    constexpr uint8_t PIN_BUTTON_C = 17U;

    /** Pin for LED A */
    constexpr uint8_t PIN_LED_A = 6U;

    /** Pin for LED B */
    constexpr uint8_t PIN_LED_B = 16U;

    /** Pin for LED C */
    constexpr uint8_t PIN_LED_C = 18U;

}; /* namespace Pin */

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* PIN_H */
/** @} */
