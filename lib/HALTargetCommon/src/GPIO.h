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
 * @brief  Abstraction of the GPIOs of the device.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALTargetCommon
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
#include <Logging.h>
#include <Io.hpp>
#include <Pin.h>

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
     * Digital input pin: Button A.
     */
    extern const DInPin<Pin::PIN_BUTTON_A, INPUT_PULLUP> buttonAPin;

    /**
     * Digital input pin: Button B.
     */
    extern const DInPin<Pin::PIN_BUTTON_B, INPUT_PULLUP> buttonBPin;

    /**
     * Digital input pin: Button C.
     */
    extern const DInPin<Pin::PIN_BUTTON_C, INPUT_PULLUP> buttonCPin;

    /**
     * Digital output pin: LED A.
     */
    extern const DOutPin<Pin::PIN_LED_A> ledAPin;

    /**
     * Digital output pin: LED B.
     */
    extern const DOutPin<Pin::PIN_LED_B> ledBPin;

    /**
     * Digital output pin: LED C.
     */
    extern const DOutPin<Pin::PIN_LED_C> ledCPin;

    /**
     * Initialize all i/o pins.
     */
    extern void init();

}; /* namespace GpioPins */

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* GPIO_H */
/** @} */
