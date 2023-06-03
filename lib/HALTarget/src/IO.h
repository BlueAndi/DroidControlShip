/* MIT License
 *
 * Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Abstraction of the IOs of the device.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALTarget
 *
 * @{
 */

#ifndef IO_H
#define IO_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <Logging.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Namespace for specifying all used GPIOs of the ESP32 */
namespace GPIOPins
{
    /** Pin for push button for system reset/AP mode start (ACTIVE LOW) */
    static const uint8_t PIN_WIFI_AND_RESET_KEY = 4;

    /** Pin for resetting the attached Zumo robot (ACTIVE LOW) */
    static const uint8_t PIN_ROBOT_RESET = 27;

    /** Unconnected pin for reading random analog data to seed PRNG */
    static const uint8_t PIN_ANALOG_NOISE_SEED = 36;

    /** Pin for info LED RGB channel RED (ACTIVE LOW) */
    static const uint8_t INFO_LED_R = 16;

    /** Pin for info LED RGB channel GREEN (ACTIVE LOW) */
    static const uint8_t INFO_LED_G = 22;

    /** Pin for info LED RGB channel BLUE (ACTIVE LOW) */
    static const uint8_t INFO_LED_B = 21;

    /** Pin for analog measurement of battery voltage */
    static const uint8_t PIN_BATT_MEASUREMENT = 35;

    /** Pin for ZumoComSystem's Push Button A */
    static const uint8_t PIN_BUTTON_A = 33;

    /** Pin for ZumoComSystem's Push Button B */
    static const uint8_t PIN_BUTTON_B = 25;

    /** Pin for ZumoComSystem's Push Button C */
    static const uint8_t PIN_BUTTON_C = 26;

}; // namespace GPIOPins

/** Provides access to the IOs of the target. */
class IO
{
public:
    /**
     * Get IO instance
     *
     * @return Returns the IO singleton instance
     */
    static IO& getInstance()
    {
        static IO instance;
        return instance;
    }

    /**
     * Initialize IOs.
     */
    void init()
    {
        /* Set GPIO direction */
        pinMode(GPIOPins::PIN_WIFI_AND_RESET_KEY, INPUT_PULLUP);
        pinMode(GPIOPins::PIN_ROBOT_RESET, OUTPUT);
        pinMode(GPIOPins::PIN_ANALOG_NOISE_SEED, INPUT);
        pinMode(GPIOPins::INFO_LED_R, OUTPUT);
        pinMode(GPIOPins::INFO_LED_G, OUTPUT);
        pinMode(GPIOPins::INFO_LED_B, OUTPUT);
        pinMode(GPIOPins::PIN_BATT_MEASUREMENT, INPUT);
        pinMode(GPIOPins::PIN_BUTTON_A, INPUT);
        pinMode(GPIOPins::PIN_BUTTON_B, INPUT);
        pinMode(GPIOPins::PIN_BUTTON_C, INPUT);

        /* Set initial values */
        writeGPIO(GPIOPins::PIN_ROBOT_RESET, LOW);
        writeGPIO(GPIOPins::INFO_LED_R, HIGH);
        writeGPIO(GPIOPins::INFO_LED_G, HIGH);
        writeGPIO(GPIOPins::INFO_LED_B, HIGH);

        LOG_DEBUG("Initialized GPIOs");
    }

    /**
     * Reads the specified GPIO input
     *
     * @param[in] gpio The GPIO pin to be read
     * @return Returns the read GPIO value
     */
    uint8_t readGPIO(const uint8_t gpio)
    {
        return digitalRead(gpio);
    }

    /**
     * Writes to the specified GPIO output
     *
     * @param[in] gpio The GPIO pin to be written to
     * @param[in] value The value to be written
     */
    void writeGPIO(const uint8_t gpio, const uint8_t value)
    {
        digitalWrite(gpio, value);
    }

    /**
     * Reads the specified GPIO analog input and returns the value in millivolt.
     *
     * @param[in] gpio The GPIO pin to be written to
     * @return Voltage in millivolt
     */
    uint32_t readAnalogGPIOInMillivolt(const uint8_t gpio)
    {
        return analogReadMilliVolts(gpio);
    }

private:
    /**
     * Default constructor.
     */
    IO()
    {
    }

    /**
     * Destructor.
     */
    ~IO()
    {
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* IO_H */
/** @} */
