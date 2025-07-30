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
#include <SettingsHandler.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "RemoteControl.h"

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
static void App_currentVehicleChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData);

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

/** Buffer size for JSON serialization of birth / will message */
static const uint32_t JSON_BIRTHMESSAGE_MAX_SIZE = 64U;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void App::setup()
{
    bool             isSuccessful = false;
    SettingsHandler& settings     = SettingsHandler::getInstance();
    Board&           board        = Board::getInstance();

    Serial.begin(SERIAL_BAUDRATE);

    /* Register serial log sink and select it per default. */
    if (true == Logging::getInstance().registerSink(&gLogSinkSerial))
    {
        (void)Logging::getInstance().selectSink("Serial");

        /* Set severity of logging system. */
        Logging::getInstance().setLogLevel(CONFIG_LOG_SEVERITY);
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
            JsonDocument jsonBirthDoc;
            char         birthMsgArray[JSON_BIRTHMESSAGE_MAX_SIZE];
            String       birthMessage;

            jsonBirthDoc["name"] = settings.getRobotName().c_str();
            (void)serializeJson(jsonBirthDoc, birthMsgArray);
            birthMessage = birthMsgArray;

            /* Setup SerialMuxProt Channels */
            m_serialMuxProtChannelIdRemoteCtrl = m_smpServer.createChannel(COMMAND_CHANNEL_NAME, COMMAND_CHANNEL_DLC);
            m_serialMuxProtChannelIdMotorSpeeds =
                m_smpServer.createChannel(MOTOR_SPEED_SETPOINT_CHANNEL_NAME, MOTOR_SPEED_SETPOINT_CHANNEL_DLC);
            m_smpServer.subscribeToChannel(COMMAND_RESPONSE_CHANNEL_NAME, App_cmdRspChannelCallback);
            m_smpServer.subscribeToChannel(LINE_SENSOR_CHANNEL_NAME, App_lineSensorChannelCallback);
            m_smpServer.subscribeToChannel(CURRENT_VEHICLE_DATA_CHANNEL_NAME, App_currentVehicleChannelCallback);
            m_serialMuxProtChannelIdStatus = m_smpServer.createChannel(STATUS_CHANNEL_NAME, STATUS_CHANNEL_DLC);

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
                else if (false == m_mqttClient.subscribe(TOPIC_NAME_CMD, true,
                                                         [this](const String& payload) { cmdTopicCallback(payload); }))
                {
                    LOG_FATAL("Could not subcribe to MQTT topic: %s.", TOPIC_NAME_CMD);
                }
                /* Subscribe to Motor Speeds Topic. */
                else if (false == m_mqttClient.subscribe(TOPIC_NAME_MOTOR_SPEEDS, true, [this](const String& payload)
                                                         { motorSpeedsTopicCallback(payload); }))
                {
                    LOG_FATAL("Could not subcribe to MQTT topic: %s.", TOPIC_NAME_MOTOR_SPEEDS);
                }
                else
                {
                    isSuccessful = true;
                    m_statusTimer.start(1000U);
                }
            }
        }
    }

    if (false == isSuccessful)
    {
        LOG_FATAL("Initialization failed.");
        fatalErrorHandler();
    }
}

void App::loop()
{
    if (false == m_isFatalError)
    {
        /* Process Battery, Device and Network. */
        Board::getInstance().process();

        /* Process MQTT Communication */
        m_mqttClient.process();

        /* Process SerialMuxProt. */
        m_smpServer.process(millis());

        if (false == m_initialDataSent)
        {
            SettingsHandler& settings = SettingsHandler::getInstance();
            VehicleData      initialVehicleData;
            initialVehicleData.xPos        = settings.getInitialXPosition();
            initialVehicleData.yPos        = settings.getInitialYPosition();
            initialVehicleData.orientation = settings.getInitialHeading();

            if (true == m_smpServer.sendData(m_serialMuxProtChannelInitialVehicleData, &initialVehicleData,
                                             sizeof(initialVehicleData)))
            {
                LOG_DEBUG("Initial vehicle data sent.");
                m_initialDataSent = true;
            }
        }

        if ((true == m_statusTimer.isTimeout()) && (true == m_smpServer.isSynced()))
        {
            Status payload = {SMPChannelPayload::Status::STATUS_FLAG_OK};

            if (false == m_smpServer.sendData(m_serialMuxProtChannelIdStatus, &payload, sizeof(payload)))
            {
                LOG_WARNING("Failed to send current status to RU.");
            }

            m_statusTimer.restart();
        }
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void App::fatalErrorHandler()
{
    if (false == m_isFatalError)
    {
        /* Turn on Red LED to signal fatal error. */
        Board::getInstance().getRedLed().enable(true);
    }

    m_isFatalError = true;
}

void App::cmdTopicCallback(const String& payload)
{
    JsonDocument         jsonPayload;
    DeserializationError error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON Deserialization Error %d.", error);
    }
    else
    {
        JsonVariantConst command = jsonPayload["CMD_ID"];

        if (false == command.isNull())
        {
            bool    isValid = true;
            Command cmd;
            uint8_t cmdId = command.as<uint8_t>();

            switch (cmdId)
            {
            case 0U:
                cmd.commandId = SMPChannelPayload::CmdId::CMD_ID_IDLE;
                break;

            case 1U:
                cmd.commandId = SMPChannelPayload::CmdId::CMD_ID_START_LINE_SENSOR_CALIB;
                break;

            case 2U:
                cmd.commandId = SMPChannelPayload::CmdId::CMD_ID_START_MOTOR_SPEED_CALIB;
                break;

            case 3U:
                cmd.commandId = SMPChannelPayload::CmdId::CMD_ID_REINIT_BOARD;
                break;

            case 4U:
                cmd.commandId = SMPChannelPayload::CmdId::CMD_ID_GET_MAX_SPEED;
                break;

            case 5U:
                cmd.commandId = SMPChannelPayload::CmdId::CMD_ID_START_DRIVING;
                break;

            case 6U:
                cmd.commandId = SMPChannelPayload::CmdId::CMD_ID_SET_INIT_POS;
                LOG_WARNING("Setting initial position is not supported.");
                isValid = false;
                break;

            default:
                isValid = false;
                break;
            }

            if (false == isValid)
            {
                LOG_ERROR("Invalid command ID %d.", cmdId);
            }
            else if (true == m_smpServer.sendData(m_serialMuxProtChannelIdRemoteCtrl, reinterpret_cast<uint8_t*>(&cmd),
                                                  sizeof(cmd)))
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
    JsonDocument         jsonPayload;
    DeserializationError error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON deserialization error %d.", error);
    }
    else
    {
        JsonVariantConst leftSpeed  = jsonPayload["LEFT"];
        JsonVariantConst rightSpeed = jsonPayload["RIGHT"];

        if ((false == leftSpeed.isNull()) && (false == rightSpeed.isNull()))
        {
            MotorSpeed motorSetpoints;
            motorSetpoints.left  = leftSpeed.as<int32_t>();
            motorSetpoints.right = rightSpeed.as<int32_t>();

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
 * @param[in] userData      User data
 */
void App_cmdRspChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    UTIL_NOT_USED(userData);
    if ((nullptr != payload) && (COMMAND_RESPONSE_CHANNEL_DLC == payloadSize))
    {
        const CommandResponse* cmdRsp = reinterpret_cast<const CommandResponse*>(payload);
        LOG_DEBUG("CMD_RSP: ID: 0x%02X , RSP: 0x%02X", cmdRsp->commandId, cmdRsp->responseId);

        if (SMPChannelPayload::CmdId::CMD_ID_GET_MAX_SPEED == cmdRsp->commandId)
        {
            LOG_DEBUG("Max Speed: %d", cmdRsp->maxMotorSpeed);
        }
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
 * @param[in]   userData        User data
 */
void App_lineSensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    UTIL_NOT_USED(payload);
    UTIL_NOT_USED(payloadSize);
    UTIL_NOT_USED(userData);
}

/**
 * Receives current position and heading of the robot over SerialMuxProt channel.
 *
 * @param[in] payload       Current vehicle data. Two coordinates, one orientation and two motor speeds.
 * @param[in] payloadSize   Size of two coordinates, one orientation and two motor speeds.
 * @param[in] userData      Instance of App class.
 */
void App_currentVehicleChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    UTIL_NOT_USED(userData);

    if ((nullptr != payload) && (CURRENT_VEHICLE_DATA_CHANNEL_DLC == payloadSize))
    {
        const VehicleData* currentVehicleData = reinterpret_cast<const VehicleData*>(payload);

        LOG_DEBUG("X: %d Y: %d Heading: %d Left: %d Right: %d Center: %d", currentVehicleData->xPos,
                  currentVehicleData->yPos, currentVehicleData->orientation, currentVehicleData->left,
                  currentVehicleData->right, currentVehicleData->center);
    }
    else
    {
        LOG_WARNING("%s: Invalid payload size. Expected: %u Received: %u", CURRENT_VEHICLE_DATA_CHANNEL_NAME,
                    CURRENT_VEHICLE_DATA_CHANNEL_DLC, payloadSize);
    }
}