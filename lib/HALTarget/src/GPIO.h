/* MIT License
 *
 * Copyright (c) 2023 - 2024 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Abstraction of the GPIOs of the device.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALTarget
 *
 * @{
 */

#ifndef GPIO_H
#define GPIO_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <Logging.h>
#include <Io.hpp>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Namespace containg access to all device GPIOs.
 */
namespace GpioPins
{
    /** Pin number of all used pins. */
    namespace Pin
    {
        /** Pin for push button for system reset/AP mode start (ACTIVE LOW) */
        constexpr uint8_t PIN_WIFI_AND_RESET_KEY = 4;

        /** Pin for resetting the attached Zumo robot (ACTIVE LOW) */
        constexpr uint8_t PIN_DEVICE_RESET = 27;

        /** Pin for info LED RGB channel RED (ACTIVE LOW) */
        constexpr uint8_t INFO_LED_R = 16;

        /** Pin for info LED RGB channel GREEN (ACTIVE LOW) */
        constexpr uint8_t INFO_LED_G = 22;

        /** Pin for info LED RGB channel BLUE (ACTIVE LOW) */
        constexpr uint8_t INFO_LED_B = 21;

        /** Pin for analog measurement of battery voltage */
        constexpr uint8_t PIN_BATT_MEASUREMENT = 35;

    }; // namespace Pin

    /**
     * Digital input pin: Reset Button.
     */
    extern const DInPin<Pin::PIN_WIFI_AND_RESET_KEY, INPUT_PULLUP> resetButtonPin;

    /**
     * Digital output pin: Reset Device.
     */
    extern const DOutPin<Pin::PIN_DEVICE_RESET> resetDevicePin;

    /**
     * Digital output pin: Info LED channel RED.
     */
    extern const DOutPin<Pin::INFO_LED_R> infoLedRedPin;

    /**
     * Digital output pin: Info LED channel GREEN.
     */
    extern const DOutPin<Pin::INFO_LED_G> infoLedGreenPin;

    /**
     * Digital output pin: Info LED channel BLUE.
     */
    extern const DOutPin<Pin::INFO_LED_B> infoLedBluePin;

    /**
     * Analog input pin: Battery voltage measurement.
     */
    extern const AnalogPin<Pin::PIN_BATT_MEASUREMENT> batteryVoltagePin;

    /**
     * Initialize all i/o pins.
     */
    extern void init();

}; // namespace GPIO

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* GPIO_H */
/** @} */
