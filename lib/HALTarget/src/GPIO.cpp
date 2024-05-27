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
 *  @brief  Abtraction of the GPIOs of the device.
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "GPIO.h"
#include <Util.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

using namespace GpioPins;

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/**
 * Digital input pin: Reset Button.
 */
const DInPin<Pin::PIN_WIFI_AND_RESET_KEY, INPUT_PULLUP> GpioPins::resetButtonPin;

/**
 * Digital output pin: Reset Device.
 */
const DOutPin<Pin::PIN_DEVICE_RESET> GpioPins::resetDevicePin;

/**
 * Digital output pin: Info LED channel RED.
 */
const DOutPin<Pin::INFO_LED_R> GpioPins::infoLedRedPin;

/**
 * Digital output pin: Info LED channel GREEN.
 */
const DOutPin<Pin::INFO_LED_G> GpioPins::infoLedGreenPin;

/**
 * Digital output pin: Info LED channel BLUE.
 */
const DOutPin<Pin::INFO_LED_B> GpioPins::infoLedBluePin;

/**
 * Analog input pin: Battery voltage measurement.
 */
const AnalogPin<Pin::PIN_BATT_MEASUREMENT> GpioPins::batteryVoltagePin;

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** A list of all used i/o pins, used for initialization. */
static const IoPin* ioPinList[] =
{
    &resetButtonPin,
    &resetDevicePin,
    &infoLedRedPin,
    &infoLedGreenPin,
    &infoLedBluePin,
    &batteryVoltagePin,
};

/******************************************************************************
 * Public Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Methods
 *****************************************************************************/

extern void GpioPins::init()
{
    uint8_t index = 0U;

    /* Initialize all i/o pins */
    for (index = 0U; index < UTIL_ARRAY_NUM(ioPinList); ++index)
    {
        if (nullptr != ioPinList[index])
        {
            ioPinList[index]->init();
        }
    }
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

