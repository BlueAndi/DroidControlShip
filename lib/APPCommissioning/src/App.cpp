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
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "App.h"
#include <Board.h>
#include <Logging.h>
#include <LogSinkPrinter.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/** Configuration of the logging severity if not previously defined. */
#ifndef CONFIG_LOG_SEVERITY
#define CONFIG_LOG_SEVERITY (Logging::LOG_LEVEL_INFO)
#endif /* CONFIG_LOG_SEVERITY */

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Serial interface baudrate. */
static const uint32_t SERIAL_BAUDRATE = 115200U;

/** Serial log sink */
static LogSinkPrinter gLogSinkSerial("Serial", &Serial);

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void App::setup()
{
    Board& board = Board::getInstance();

    Serial.begin(SERIAL_BAUDRATE);

    /* Register serial log sink and select it per default. */
    if (true == Logging::getInstance().registerSink(&gLogSinkSerial))
    {
        (void)Logging::getInstance().selectSink(gLogSinkSerial.getName());

        /* Set severity of logging system. */
        Logging::getInstance().setLogLevel(CONFIG_LOG_SEVERITY);
    }

    /* Initialize HAL. */
    if (false == board.init())
    {
        LOG_FATAL("HAL init failed.");
        m_isFatalError = true;

        /* Show the failure on the info LED. */
        board.getRedLed().enable(true);
    }
    else
    {
        LOG_INFO("Hardware commissioning application started.");

        m_ledChaseTimer.start(LED_CHASE_PERIOD_MS);
        m_robotStreamTimer.start(ROBOT_STREAM_PERIOD_MS);
        m_robotResetTimer.start(ROBOT_RESET_PERIOD_MS);
        m_batteryTimer.start(BATTERY_PERIOD_MS);
    }
}

void App::loop()
{
    if (false == m_isFatalError)
    {
        /* Process the robot connection, but intentionally not the network.
         * The commissioning shall run standalone without any WiFi
         * infrastructure.
         */
        Board::getInstance().getRobot().process();

        processButtons();
        processLeds();
        processRobotStream();
        processRobotReset();
        processBattery();
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void App::processLeds()
{
    if (true == m_ledChaseTimer.isTimeout())
    {
        bool isAnyButtonPressed = (true == m_isButtonResetPressed) || (true == m_isButtonAPressed) ||
                                  (true == m_isButtonBPressed) || (true == m_isButtonCPressed);

        /* The walking light pattern pauses as long as a button is kept
         * pressed, because the LEDs are used as button feedback then.
         */
        if (false == isAnyButtonPressed)
        {
            enableLed(m_ledIndex);

            ++m_ledIndex;
            m_ledIndex %= LED_COUNT;
        }

        m_ledChaseTimer.restart();
    }
}

void App::processButtons()
{
    Board& board = Board::getInstance();

    bool isButtonResetPressed = board.getButtonReset().isPressed();
    bool isButtonAPressed     = board.getButtonA().isPressed();
    bool isButtonBPressed     = board.getButtonB().isPressed();
    bool isButtonCPressed     = board.getButtonC().isPressed();

    if (m_isButtonResetPressed != isButtonResetPressed)
    {
        m_isButtonResetPressed = isButtonResetPressed;
        LOG_INFO("Button RESET %s.", (true == isButtonResetPressed) ? "pressed" : "released");

        /* Button feedback: all info LED channels on as long as the button is
         * kept pressed.
         */
        board.getRedLed().enable(isButtonResetPressed);
        board.getGreenLed().enable(isButtonResetPressed);
        board.getBlueLed().enable(isButtonResetPressed);
    }

    if (m_isButtonAPressed != isButtonAPressed)
    {
        m_isButtonAPressed = isButtonAPressed;
        LOG_INFO("Button A %s.", (true == isButtonAPressed) ? "pressed" : "released");

        /* Button feedback: LED A on as long as the button is kept pressed. */
        board.getLedA().enable(isButtonAPressed);
    }

    if (m_isButtonBPressed != isButtonBPressed)
    {
        m_isButtonBPressed = isButtonBPressed;
        LOG_INFO("Button B %s.", (true == isButtonBPressed) ? "pressed" : "released");

        /* Button feedback: LED B on as long as the button is kept pressed. */
        board.getLedB().enable(isButtonBPressed);
    }

    if (m_isButtonCPressed != isButtonCPressed)
    {
        m_isButtonCPressed = isButtonCPressed;
        LOG_INFO("Button C %s.", (true == isButtonCPressed) ? "pressed" : "released");

        /* Button feedback: LED C on as long as the button is kept pressed. */
        board.getLedC().enable(isButtonCPressed);
    }
}

void App::processRobotStream()
{
    Stream& robotStream = Board::getInstance().getRobot().getStream();

    if (true == m_robotStreamTimer.isTimeout())
    {
        char   buffer[32];
        int    written = snprintf(buffer, sizeof(buffer), "DCS ping %u\r\n", m_pingCounter);
        size_t sent    = 0U;

        if (0 < written)
        {
            sent = robotStream.write(reinterpret_cast<const uint8_t*>(buffer), static_cast<size_t>(written));
        }

        LOG_INFO("Robot stream: ping %u sent (%u bytes).", m_pingCounter, static_cast<uint32_t>(sent));
        ++m_pingCounter;

        m_robotStreamTimer.restart();
    }

    /* Show every received byte on the console, e.g. for a loopback test.
     * The number of bytes per cycle is limited to keep the application
     * responsive.
     */
    uint8_t rxCount = 0U;
    while ((0 < robotStream.available()) && (MAX_RX_BYTES_PER_CYCLE > rxCount))
    {
        int receivedByte = robotStream.read();

        if (0 > receivedByte)
        {
            break;
        }

        LOG_INFO("Robot stream: received 0x%02X.", receivedByte);
        ++rxCount;
    }
}

void App::processRobotReset()
{
    if (true == m_robotResetTimer.isTimeout())
    {
        LOG_INFO("Robot reset line pulse.");
        Board::getInstance().getRobot().reset();

        m_robotResetTimer.restart();
    }
}

void App::processBattery()
{
    if (true == m_batteryTimer.isTimeout())
    {
        Board&   board       = Board::getInstance();
        uint32_t voltage     = board.getBattery().getVoltage();
        uint8_t  chargeLevel = board.getBattery().getChargeLevel();

        LOG_INFO("Battery: %u mV (%u %%).", voltage, chargeLevel);

        m_batteryTimer.restart();
    }
}

void App::enableLed(uint8_t index)
{
    Board& board = Board::getInstance();

    board.getRedLed().enable(0U == index);
    board.getGreenLed().enable(1U == index);
    board.getBlueLed().enable(2U == index);
    board.getLedA().enable(3U == index);
    board.getLedB().enable(4U == index);
    board.getLedC().enable(5U == index);
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
