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
 * @brief  Hardware commissioning application
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef APP_H
#define APP_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <SimpleTimer.hpp>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The hardware commissioning application.
 *
 * It exercises the board peripherals in a observable way, so that a new
 * board can be validated visually, via the console log or with external
 * test equipment like a logic analyzer:
 *
 * - All LEDs are driven by a walking light pattern.
 * - Every button press/release is shown on the console and lights the
 *   associated LED as long as the button is kept pressed.
 * - A test pattern is sent periodically over the robot communication
 *   interface (USB or UART, depending on the hardware version) and every
 *   received byte is shown on the console.
 * - The robot reset line is pulsed periodically.
 * - The battery voltage is measured and shown on the console periodically.
 */
class App
{
public:
    /**
     * Constructs the commissioning application.
     */
    App() :
        m_isFatalError(false),
        m_ledChaseTimer(),
        m_ledIndex(0U),
        m_isButtonResetPressed(false),
        m_isButtonAPressed(false),
        m_isButtonBPressed(false),
        m_isButtonCPressed(false),
        m_robotStreamTimer(),
        m_robotResetTimer(),
        m_batteryTimer(),
        m_pingCounter(0U)
    {
    }

    /**
     * Destroys the commissioning application.
     */
    ~App()
    {
    }

    /**
     * Sets up the application.
     */
    void setup();

    /**
     * Processes the application periodically.
     */
    void loop();

private:
    /** Period of the LED walking light pattern in milliseconds. */
    static const uint32_t LED_CHASE_PERIOD_MS = 250U;

    /** Period of the test pattern on the robot communication interface in milliseconds. */
    static const uint32_t ROBOT_STREAM_PERIOD_MS = 1000U;

    /** Period of the robot reset line pulse in milliseconds. */
    static const uint32_t ROBOT_RESET_PERIOD_MS = 10000U;

    /** Period of the battery voltage measurement in milliseconds. */
    static const uint32_t BATTERY_PERIOD_MS = 5000U;

    /** Number of LEDs driven by the walking light pattern. */
    static const uint8_t LED_COUNT = 6U;

    /** Max. number of received bytes shown on the console per cycle. */
    static const uint8_t MAX_RX_BYTES_PER_CYCLE = 32U;

    /** Fatal error flag. */
    bool m_isFatalError;

    /** Timer for the LED walking light pattern. */
    SimpleTimer m_ledChaseTimer;

    /** Index of the currently active LED of the walking light pattern. */
    uint8_t m_ledIndex;

    /** Last known state of the reset button. */
    bool m_isButtonResetPressed;

    /** Last known state of button A. */
    bool m_isButtonAPressed;

    /** Last known state of button B. */
    bool m_isButtonBPressed;

    /** Last known state of button C. */
    bool m_isButtonCPressed;

    /** Timer for the test pattern on the robot communication interface. */
    SimpleTimer m_robotStreamTimer;

    /** Timer for the robot reset line pulse. */
    SimpleTimer m_robotResetTimer;

    /** Timer for the battery voltage measurement. */
    SimpleTimer m_batteryTimer;

    /** Counter of sent test patterns. */
    uint32_t m_pingCounter;

    /**
     * Processes the LED walking light pattern.
     */
    void processLeds();

    /**
     * Processes the buttons and lights the associated LED as long as a button
     * is kept pressed.
     */
    void processButtons();

    /**
     * Processes the robot communication interface by sending a test pattern
     * periodically and showing every received byte on the console.
     */
    void processRobotStream();

    /**
     * Processes the periodical robot reset line pulse.
     */
    void processRobotReset();

    /**
     * Processes the periodical battery voltage measurement.
     */
    void processBattery();

    /**
     * Enables exactly one LED of the walking light pattern.
     *
     * @param[in] index Index of the LED to enable. All other LEDs are disabled.
     */
    void enableLed(uint8_t index);

    /**
     * Copy construction of an instance.
     * Not allowed.
     *
     * @param[in] app Source instance.
     */
    App(const App& app);

    /**
     * Assignment of an instance.
     * Not allowed.
     *
     * @param[in] app Source instance.
     *
     * @returns Reference to App instance.
     */
    App& operator=(const App& app);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* APP_H */
/** @} */
