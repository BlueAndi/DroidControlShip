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
 * @brief  RemoteControl application
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "App.h"
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <Windows.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

#ifndef CONFIG_LOG_SEVERITY
#define CONFIG_LOG_SEVERITY (Logging::LOG_LEVEL_INFO)
#endif /* CONFIG_LOG_SEVERITY */

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

static void App_cmdRspChannelCallback(const uint8_t* payload, const uint8_t payloadSize);
static void App_lineSensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Serial interface baudrate. */
static const uint32_t SERIAL_BAUDRATE = 115200U;

/** Serial log sink */
static LogSinkPrinter gLogSinkSerial("Serial", &Serial);

/* Initialize channel name for sending commands. */
const char* App::CH_NAME_CMD = "REMOTE_CMD";

/* Initialize channel name for receiving command responses. */
const char* App::CH_NAME_RSP = "REMOTE_RSP";

/* YAP channel name for sending motor speeds. */
const char* App::CH_NAME_MOTOR_SPEEDS = "MOT_SPEEDS";

/* Initialize channel name for receiving line sensors data. */
const char* App::CH_NAME_LINE_SENSORS = "LINE_SENS";

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void App::setup()
{
    Serial.begin(SERIAL_BAUDRATE);

    /* Register serial log sink and select it per default. */
    if (true == Logging::getInstance().registerSink(&gLogSinkSerial))
    {
        (void)Logging::getInstance().selectSink("Serial");

        /* Set severity of logging system. */
        Logging::getInstance().setLogLevel(CONFIG_LOG_SEVERITY);

        LOG_DEBUG("LOGGER READY");
    }

    /* Initialize HAL. */
    if (false == Board::getInstance().init())
    {
        /* Log and Handle Board initialization error */
        LOG_FATAL("HAL init failed.");
        fatalErrorHandler();
    }
    else
    {
        /* Setup SerialMuxProt Channels */
        m_serialMuxProtChannelIdRemoteCtrl = m_smpServer.createChannel(CH_NAME_CMD, 1U);
        m_smpServer.subscribeToChannel(CH_NAME_RSP, App_cmdRspChannelCallback);
        m_serialMuxProtChannelIdMotorSpeeds = m_smpServer.createChannel(CH_NAME_MOTOR_SPEEDS, 4U);
        m_smpServer.subscribeToChannel(CH_NAME_LINE_SENSORS, App_lineSensorChannelCallback);
    }
}

void App::loop()
{
    /* Process Battery, Device and Network. */
    if (false == Board::getInstance().process())
    {
        /* Log and Handle Board processing error */
        LOG_FATAL("HAL process failed.");
        fatalErrorHandler();
    }

    /* Process SerialMuxProt. */
    m_smpServer.process(millis());

    /* Process Keyboard */
    keyboardHandler();
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void App::fatalErrorHandler()
{
    /* Turn on Red LED to signal fatal error. */
    Board::getInstance().getRedLed().enable(true);

    while (true)
    {
        ;
    }
}

void App::keyboardHandler()
{
    const int isPressedValue = 0x8000;
    uint8_t   buffer[4U];

    if (GetKeyState(FORWARD) & isPressedValue)
    {
        if (m_lastSentKey != FORWARD)
        {
            LOG_INFO("Forward");
            m_lastSentKey = FORWARD;
            buffer[0U]    = 0x80;
            buffer[1U]    = 0x00;
            buffer[2U]    = 0x80;
            buffer[3U]    = 0x00;
            m_smpServer.sendData(CH_NAME_MOTOR_SPEEDS, buffer, 4U);
        }
    }
    else if (GetKeyState(BACKWARD) & isPressedValue)
    {
        if (m_lastSentKey != BACKWARD)
        {
            LOG_INFO("Backward");
            m_lastSentKey = BACKWARD;
            buffer[0U]    = 0x7F;
            buffer[1U]    = 0xFF;
            buffer[2U]    = 0x7F;
            buffer[3U]    = 0xFF;
            m_smpServer.sendData(CH_NAME_MOTOR_SPEEDS, buffer, 4U);
        }
    }
    else if (GetKeyState(LEFT) & isPressedValue)
    {
        if (m_lastSentKey != LEFT)
        {
            LOG_INFO("Left");
            m_lastSentKey = LEFT;
            buffer[0U]    = 0x7F;
            buffer[1U]    = 0xFF;
            buffer[2U]    = 0x80;
            buffer[3U]    = 0x00;
            m_smpServer.sendData(CH_NAME_MOTOR_SPEEDS, buffer, 4U);
        }
    }
    else if (GetKeyState(RIGHT) & isPressedValue)
    {
        if (m_lastSentKey != RIGHT)
        {
            LOG_INFO("Right");
            m_lastSentKey = RIGHT;
            buffer[0U]    = 0x80;
            buffer[1U]    = 0x00;
            buffer[2U]    = 0x7F;
            buffer[3U]    = 0xFF;
            m_smpServer.sendData(CH_NAME_MOTOR_SPEEDS, buffer, 4U);
        }
    }
    else if (GetKeyState(CALIB_LINESENS) & isPressedValue)
    {
        if (m_lastSentKey != CALIB_LINESENS)
        {
            LOG_INFO("Calib Line Sensors");
            m_lastSentKey = CALIB_LINESENS;
            buffer[0U]    = 0x01;
            m_smpServer.sendData(CH_NAME_CMD, buffer, 1U);
        }
    }
    else if (GetKeyState(CALIB_MOTORS) & isPressedValue)
    {
        if (m_lastSentKey != CALIB_MOTORS)
        {
            LOG_INFO("Calib Motors");
            m_lastSentKey = CALIB_MOTORS;
            buffer[0U]    = 0x02;
            m_smpServer.sendData(CH_NAME_CMD, buffer, 1U);
        }
    }
    else
    {
        if ((m_lastSentKey != STOP) && (m_lastSentKey != CALIB_MOTORS) && (m_lastSentKey != CALIB_LINESENS))
        {
            LOG_INFO("Stop");
            m_lastSentKey = STOP;
            buffer[0U]    = 0x00;
            buffer[1U]    = 0x00;
            buffer[2U]    = 0x00;
            buffer[3U]    = 0x00;
            m_smpServer.sendData(CH_NAME_MOTOR_SPEEDS, buffer, 4U);
        }
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/

void App_cmdRspChannelCallback(const uint8_t* payload, const uint8_t payloadSize)
{
    LOG_DEBUG("CMD_RSP: 0x%02X", payload[0]);
}

void App_lineSensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize)
{
}
