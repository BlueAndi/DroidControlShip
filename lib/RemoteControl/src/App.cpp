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
#include <WiFi.h>

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

static void App_cmdRspChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData);
static void App_lineSensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData);

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

/** Default size of the JSON Document for parsing. */
static const uint32_t JSON_DOC_DEFAULT_SIZE = 1024U;

/** Buffer size for JSON serialization of birth / will message */
static const uint32_t JSON_BIRTHMESSAGE_MAX_SIZE = 64U;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void App::setup()
{
    bool      isSuccessful = false;
    Settings& settings     = Settings::getInstance();
    Board&    board        = Board::getInstance();

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
    if (false == board.init())
    {
        LOG_FATAL("HAL init failed.");
    }
    /* Settings shall be loaded from configuration file. */
    else if (false == settings.loadConfigurationFile(board.getConfigFilePath()))
    {
        LOG_FATAL("Settings could not be loaded from %s.", board.getConfigFilePath());
    }
    else
    {
        /* If the robot name is empty, use the wifi MAC address as robot name. */
        if (true == settings.getRobotName().isEmpty())
        {
            String robotName = WiFi.macAddress();

            /* Remove MAC separators from robot name. */
            robotName.replace(":", "");

            settings.setRobotName(robotName);
        }

        NetworkSettings networkSettings = {settings.getWiFiSSID(), settings.getWiFiPassword(), settings.getRobotName(),
                                           ""};

        if (false == board.getNetwork().setConfig(networkSettings))
        {
            LOG_FATAL("Network configuration could not be set.");
        }
        else
        {
            /* Setup MQTT Server, Birth and Will messages. */
            StaticJsonDocument<JSON_BIRTHMESSAGE_MAX_SIZE> birthDoc;
            char                                           birthMsgArray[JSON_BIRTHMESSAGE_MAX_SIZE];
            String                                         birthMessage;

            birthDoc["name"] = settings.getRobotName().c_str();
            (void)serializeJson(birthDoc, birthMsgArray);
            birthMessage = birthMsgArray;

            /* Setup SerialMuxProt Channels */
            m_serialMuxProtChannelIdRemoteCtrl = m_smpServer.createChannel(COMMAND_CHANNEL_NAME, COMMAND_CHANNEL_DLC);
            m_serialMuxProtChannelIdMotorSpeeds =
                m_smpServer.createChannel(SPEED_SETPOINT_CHANNEL_NAME, SPEED_SETPOINT_CHANNEL_DLC);
            m_smpServer.subscribeToChannel(COMMAND_RESPONSE_CHANNEL_NAME, App_cmdRspChannelCallback);
            m_smpServer.subscribeToChannel(LINE_SENSOR_CHANNEL_NAME, App_lineSensorChannelCallback);

            if (false == m_mqttClient.init())
            {
                LOG_FATAL("Failed to initialize MQTT client.");
            }
            else
            {
                MqttSettings mqttSettings = {settings.getRobotName(),
                                             settings.getMqttBrokerAddress(),
                                             settings.getMqttPort(),
                                             TOPIC_NAME_BIRTH,
                                             birthMessage,
                                             TOPIC_NAME_WILL,
                                             birthMessage,
                                             true};

                if (false == m_mqttClient.setConfig(mqttSettings))
                {
                    LOG_FATAL("MQTT configuration could not be set.");
                }
                /* Subscribe to Command Topic. */
                else if (false == m_mqttClient.subscribe(TOPIC_NAME_CMD,
                                                         [this](const String& payload) { cmdTopicCallback(payload); }))
                {
                    LOG_FATAL("Could not subcribe to MQTT topic: %s.", TOPIC_NAME_CMD);
                }
                /* Subscribe to Motor Speeds Topic. */
                else if (false == m_mqttClient.subscribe(TOPIC_NAME_MOTOR_SPEEDS, [this](const String& payload)
                                                         { motorSpeedsTopicCallback(payload); }))
                {
                    LOG_FATAL("Could not subcribe to MQTT topic: %s.", TOPIC_NAME_MOTOR_SPEEDS);
                }
                else
                {
                    isSuccessful = true;
                }
            }
        }
    }

    if (false == isSuccessful)
    {
        fatalErrorHandler();
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

    /* Process MQTT Communication */
    m_mqttClient.process();

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
    DeserializationError                      error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON Deserialization Error %d.", error);
    }
    else
    {
        JsonVariant command = jsonPayload["CMD_ID"];

        if (false == command.isNull())
        {
            Command cmd;
            cmd.commandId = command.as<uint8_t>();

            if (true ==
                m_smpServer.sendData(m_serialMuxProtChannelIdRemoteCtrl, reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd)))
            {
                LOG_DEBUG("Command %d sent.", cmd.commandId);
            }
            else
            {
                LOG_WARNING("Failed to send command %d.", cmd.commandId);
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
    DeserializationError                      error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON deserialization error %d.", error);
    }
    else
    {
        JsonVariant leftSpeed  = jsonPayload["LEFT"];
        JsonVariant rightSpeed = jsonPayload["RIGHT"];

        if ((false == leftSpeed.isNull()) && (false == rightSpeed.isNull()))
        {
            SpeedData motorSetpoints;
            motorSetpoints.left  = leftSpeed.as<int16_t>();
            motorSetpoints.right = rightSpeed.as<int16_t>();

            if (true == m_smpServer.sendData(m_serialMuxProtChannelIdMotorSpeeds,
                                             reinterpret_cast<uint8_t*>(&motorSetpoints), sizeof(motorSetpoints)))
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
 * @param[in] userData      User data provided by the application.
 */
void App_cmdRspChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    UTIL_NOT_USED(userData);
    if (COMMAND_RESPONSE_CHANNEL_DLC == payloadSize)
    {
        const CommandResponse* cmdRsp = reinterpret_cast<const CommandResponse*>(payload);
        LOG_DEBUG("CMD_RSP: 0x%02X", cmdRsp->response);
    }
    else
    {
        LOG_WARNING("CMD_RSP: Invalid payload size. Expected: %u Received: %u", COMMAND_RESPONSE_CHANNEL_DLC,
                    payloadSize);
    }
}

/**
 * Receives line sensor data over SerialMuxProt channel.
 * @param[in]   payload         Line sensor data
 * @param[in]   payloadSize     Size of 5 line sensor data
 * @param[in]   userData      User data provided by the application.
 */
void App_lineSensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    UTIL_NOT_USED(payload);
    UTIL_NOT_USED(payloadSize);
    UTIL_NOT_USED(userData);
}
