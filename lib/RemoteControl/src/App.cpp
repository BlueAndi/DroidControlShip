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
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "App.h"
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <Util.h>
#include <Settings.h>
#include <ArduinoJson.h>

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

/* MQTT topic name for birth messages. */
const char* App::TOPIC_NAME_BIRTH = "birth";

/* MQTT topic name for will messages. */
const char* App::TOPIC_NAME_WILL = "will";

/* MQTT topic name for receiving commands. */
const char* App::TOPIC_NAME_CMD = "cmd";

/* MQTT topic name for receiving motor speeds. */
const char* App::TOPIC_NAME_MOTOR_SPEEDS = "motorSpeeds";

/* Initialize channel name for sending commands. */
const char* App::CH_NAME_CMD = "REMOTE_CMD";

/* Initialize channel name for receiving command responses. */
const char* App::CH_NAME_RSP = "REMOTE_RSP";

/* YAP channel name for sending motor speeds. */
const char* App::CH_NAME_MOTOR_SPEEDS = "MOT_SPEEDS";

/* Initialize channel name for receiving line sensors data. */
const char* App::CH_NAME_LINE_SENSORS = "LINE_SENS";

/** Default size of the JSON Document for parsing. */
static const uint32_t JSON_DOC_DEFAULT_SIZE = 1024U;

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
        if (false == Settings::getInstance().isConfigLoaded())
        {
            /* Settings shall be loaded from configuration file. */
            if (false == Settings::getInstance().loadConfigurationFile(CONFIG_FILE_PATH))
            {
                /* Log Settings error */
                LOG_ERROR("Settings could not be loaded from file. ");
                fatalErrorHandler();
            }
            else
            {
                /* Log Settings loaded */
                LOG_DEBUG("Settings loaded from file.");
            }
        }
        else
        {
            LOG_DEBUG("Settings set externally.");
        }

        /* Setup SerialMuxProt Channels */
        m_serialMuxProtChannelIdRemoteCtrl  = m_smpServer.createChannel(CH_NAME_CMD, 1U);
        m_serialMuxProtChannelIdMotorSpeeds = m_smpServer.createChannel(CH_NAME_MOTOR_SPEEDS, 4U);
        m_smpServer.subscribeToChannel(CH_NAME_RSP, App_cmdRspChannelCallback);
        m_smpServer.subscribeToChannel(CH_NAME_LINE_SENSORS, App_lineSensorChannelCallback);

        /* Setup Network. Get saved configuration.*/
        String   clientId = Settings::getInstance().getRobotName();
        String   ssid     = Settings::getInstance().getWiFiSSID();
        String   password = Settings::getInstance().getWiFiPassword();
        String   mqttAddr = Settings::getInstance().getMqttBrokerAddress();
        uint16_t mqttPort = Settings::getInstance().getMqttPort();

        /* Setup MQTT Server, Birth and Will messages. */
        if (false ==
            Board::getInstance().getNetwork().setConfig(clientId, ssid, password, mqttAddr, mqttPort, TOPIC_NAME_BIRTH,
                                                        String(clientId + String(" Connected!")), TOPIC_NAME_WILL,
                                                        String(clientId + String(" Disconnected!")), true))
        {
            LOG_ERROR("Network Configuration could not be set.");
            fatalErrorHandler();
        }

        /* Subscribe to Command Topic. */
        if (false == Board::getInstance().getNetwork().subscribe(TOPIC_NAME_CMD, [this](const String& payload)
                                                                 { cmdTopicCallback(payload); }))
        {
            LOG_ERROR("Could not subcribe to MQTT Topic: %s.", TOPIC_NAME_CMD);
            fatalErrorHandler();
        }

        /* Subscribe to Motor Speeds Topic. */
        if (false == Board::getInstance().getNetwork().subscribe(TOPIC_NAME_MOTOR_SPEEDS, [this](const String& payload)
                                                                 { motorSpeedsTopicCallback(payload); }))
        {
            LOG_ERROR("Could not subcribe to MQTT Topic: %s.", TOPIC_NAME_MOTOR_SPEEDS);
            fatalErrorHandler();
        }
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

void App::cmdTopicCallback(const String& payload)
{
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> jsonPayload;
    DeserializationError                      error = DeserializationError::Ok;

    error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON Deserialization Error %d.", error);
    }
    else
    {
        uint8_t     payloadSize = sizeof(uint8_t);
        uint8_t     buffer[payloadSize];
        JsonVariant command = jsonPayload["CMD_ID"];

        if (false == command.isNull())
        {
            buffer[0U] = command.as<uint8_t>();

            if (true == m_smpServer.sendData(CH_NAME_CMD, buffer, 1U))
            {
                LOG_DEBUG("Command %d sent.", buffer[0U]);
            }
            else
            {
                LOG_WARNING("Failed to send command %d.", buffer[0U]);
            }
        }
        else
        {
            LOG_WARNING("Received invalid command.");
        }
    }
}

void App::motorSpeedsTopicCallback(const String& payload)
{
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> jsonPayload;
    DeserializationError                      error = DeserializationError::Ok;

    error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON Deserialization Error %d.", error);
    }
    else
    {
        uint8_t     payloadSize = (2U * sizeof(int16_t));
        uint8_t     buffer[payloadSize];
        JsonVariant leftSpeed  = jsonPayload["LEFT"];
        JsonVariant rightSpeed = jsonPayload["RIGHT"];

        if ((false == leftSpeed.isNull()) && (false == rightSpeed.isNull()))
        {
            Util::int16ToByteArray(&buffer[0U * sizeof(int16_t)], sizeof(int16_t), leftSpeed.as<int16_t>());
            Util::int16ToByteArray(&buffer[1U * sizeof(int16_t)], sizeof(int16_t), rightSpeed.as<int16_t>());

            if (true == m_smpServer.sendData(CH_NAME_MOTOR_SPEEDS, buffer, payloadSize))
            {
                LOG_DEBUG("Motor speeds sent");
            }
            else
            {
                LOG_WARNING("Failed to send motor speeds");
            }
        }
        else
        {
            LOG_WARNING("Received invalid motor speeds.");
        }
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * Receives remote control command responses over SerialMuxProt channel.
 *
 * @param[in] payload       Command id
 * @param[in] payloadSize   Size of command id
 */
void App_cmdRspChannelCallback(const uint8_t* payload, const uint8_t payloadSize)
{
    uint8_t expectedPayloadSize = 1U;
    if (expectedPayloadSize == payloadSize)
    {
        LOG_DEBUG("CMD_RSP: 0x%02X", payload[0U]);
    }
    else
    {
        LOG_WARNING("CMD_RSP: Invalid payload size. Expected: %u Received: %u", expectedPayloadSize, payloadSize);
    }
}

/**
 * Receives line sensor data over SerialMuxProt channel.
 * @param[in]   payload         Line sensor data
 * @param[in]   payloadSize     Size of 5 line sensor data
 */
void App_lineSensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize)
{
    UTIL_NOT_USED(payload);
    UTIL_NOT_USED(payloadSize);
}
