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

        /* Setup Network. */
        Board::getInstance().getNetwork().setConfig("DCS", "localhost", 1883U, "will/dcs", "DCS Disconnected!", true);
        Board::getInstance().getNetwork().subscribe("dcs/cmd",
                                                    [this](const String& payload)
                                                    {
                                                        uint8_t buffer[1U];
                                                        buffer[0U] = 0x02;
                                                        m_smpServer.sendData(CH_NAME_CMD, buffer, 1U);
                                                    });
        Board::getInstance().getNetwork().subscribe("dcs/motorSpeeds",
                                                    [this](const String& payload)
                                                    {
                                                        uint8_t buffer[4U];

                                                        if (String("forward") == payload)
                                                        {
                                                            LOG_DEBUG("forward");
                                                            buffer[0U] = 0x80;
                                                            buffer[1U] = 0x00;
                                                            buffer[2U] = 0x80;
                                                            buffer[3U] = 0x00;
                                                        }
                                                        else if (String("backward") == payload)
                                                        {
                                                            LOG_DEBUG("backward");
                                                            buffer[0U] = 0x7F;
                                                            buffer[1U] = 0xFF;
                                                            buffer[2U] = 0x7F;
                                                            buffer[3U] = 0xFF;
                                                        }
                                                        else if (String("left") == payload)
                                                        {
                                                            LOG_DEBUG("left");
                                                            buffer[0U] = 0x7F;
                                                            buffer[1U] = 0xFF;
                                                            buffer[2U] = 0x80;
                                                            buffer[3U] = 0x00;
                                                        }
                                                        else if (String("right") == payload)
                                                        {
                                                            LOG_DEBUG("right");
                                                            buffer[0U] = 0x80;
                                                            buffer[1U] = 0x00;
                                                            buffer[2U] = 0x7F;
                                                            buffer[3U] = 0xFF;
                                                        }
                                                        else
                                                        {
                                                            LOG_DEBUG("stop");
                                                            buffer[0U] = 0x00;
                                                            buffer[1U] = 0x00;
                                                            buffer[2U] = 0x00;
                                                            buffer[3U] = 0x00;
                                                        }

                                                        if (true ==
                                                            m_smpServer.sendData(CH_NAME_MOTOR_SPEEDS, buffer, 4U))
                                                        {
                                                            LOG_DEBUG("Motor speeds sent");
                                                        }
                                                        else
                                                        {
                                                            LOG_WARNING("Failed to send motor speeds");
                                                        }
                                                    });
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
