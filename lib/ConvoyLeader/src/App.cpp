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
 * @brief  ConvoyLeader application
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "App.h"
#include <Board.h>
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <SettingsHandler.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Util.h>
#include "StartupState.h"
#include "IdleState.h"
#include "DrivingState.h"
#include "ErrorState.h"

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
static void App_currentVehicleChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData);
static void App_statusChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData);

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

/* MQTT topic name for release messages. */
const char* App::TOPIC_NAME_RELEASE = "release";

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
    else if (MIN_BATTERY_LEVEL > board.getBattery().getChargeLevel())
    {
        LOG_FATAL("Battery too low.");
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

            if (false == settings.saveConfigurationFile(board.getConfigFilePath()))
            {
                /* Error saving settings, but it is not fatal. */
                LOG_ERROR("Settings file could not be saved.");
            }
        }

        NetworkSettings networkSettings = {settings.getWiFiSSID(), settings.getWiFiPassword(), settings.getRobotName(),
                                           ""};

        if (false == board.getNetwork().setConfig(networkSettings))
        {
            LOG_FATAL("Network configuration could not be set.");
        }
        else if (false == setupMqttClient())
        {
            LOG_FATAL("Failed to setup MQTT client.");
        }
        else if (V2VClient::PLATOON_LEADER_ID != settings.getPlatoonVehicleId())
        {
            /* Correct config.json file loaded? */
            LOG_FATAL("Platoon Vehicle ID must be 0 for the leader.");
        }
        else if (false == m_v2vClient.init(settings.getPlatoonPlatoonId(), settings.getPlatoonVehicleId()))
        {
            LOG_FATAL("Failed to initialize V2V client.");
        }
        else if (false == setupSerialMuxProt())
        {
            LOG_FATAL("Failed to setup SerialMuxProt.");
        }
        else
        {
            /* Initialize timers. */
            m_sendWaypointTimer.start(SEND_WAYPOINT_TIMER_INTERVAL);
            m_commandTimer.start(SEND_COMMANDS_TIMER_INTERVAL);
            m_motorSpeedTimer.start(SEND_MOTOR_SPEED_TIMER_INTERVAL);
            m_statusTimer.start(SEND_STATUS_TIMER_INTERVAL);

            /* Start with startup state. */
            m_systemStateMachine.setState(&StartupState::getInstance());

            isSuccessful = true;
        }
    }

    if (false == isSuccessful)
    {
        fatalErrorHandler();
    }
    else
    {
        /* Blink Green LED to signal all-good. */
        Board::getInstance().getGreenLed().enable(true);
        delay(100U);
        Board::getInstance().getGreenLed().enable(false);
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

    /* Process MQTT Communication */
    m_mqttClient.process();

    /* Process V2V Communication */
    m_v2vClient.process();

    /* Process System State Machine */
    m_systemStateMachine.process();

    /* Process periodic tasks. */
    processPeriodicTasks();
}

void App::setLatestVehicleData(const Waypoint& waypoint)
{
    m_latestVehicleData = waypoint;
}

void App::setErrorState()
{
    m_systemStateMachine.setState(&ErrorState::getInstance());
}

void App::systemStatusCallback(SMPChannelPayload::Status status)
{
    if (SMPChannelPayload::STATUS_FLAG_ERROR == status)
    {
        setErrorState();
    }

    m_statusTimeoutTimer.restart();
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

bool App::setupMqttClient()
{
    /* Setup MQTT Server, Birth and Will messages. */
    bool                                           isSuccessful = false;
    SettingsHandler&                               settings     = SettingsHandler::getInstance();
    StaticJsonDocument<JSON_BIRTHMESSAGE_MAX_SIZE> birthDoc;
    String                                         birthMessage;

    birthDoc["name"] = settings.getRobotName();

    if (0U == serializeJson(birthDoc, birthMessage))
    {
        /* Non-fatal error. Birth message will be empty. */
        LOG_ERROR("Failed to serialize birth message.");
        birthMessage.clear();
    }

    MqttSettings mqttSettings = {settings.getRobotName(),
                                 settings.getMqttBrokerAddress(),
                                 settings.getMqttPort(),
                                 TOPIC_NAME_BIRTH,
                                 birthMessage,
                                 TOPIC_NAME_WILL,
                                 birthMessage,
                                 true};

    if (false == m_mqttClient.init())
    {
        LOG_FATAL("Failed to initialize MQTT client.");
    }
    else if (false == m_mqttClient.setConfig(mqttSettings))
    {
        LOG_FATAL("MQTT configuration could not be set.");
    }
    else
    {
        /* Create Callbacks. */
        IMqttClient::TopicCallback releaseTopicCallback = [this](const String& payload)
        { IdleState::getInstance().requestRelease(); };

        IMqttClient::TopicCallback lastFollowerFeedbackCallback = [this](const String& payload)
        {
            Waypoint*   waypoint = Waypoint::deserialize(payload);
            VehicleData feedback{waypoint->xPos, waypoint->yPos,  waypoint->orientation,
                                 waypoint->left, waypoint->right, waypoint->center};

            DrivingState::getInstance().setLastFollowerFeedback(feedback);
        };

        /* Register MQTT client callbacks. */
        if (false == m_mqttClient.subscribe(TOPIC_NAME_RELEASE, true, releaseTopicCallback))
        {
            LOG_ERROR("Failed to subscribe to release topic.");
        }
        else if (false == m_mqttClient.subscribe("platoons/0/vehicles/0/feedback", false, lastFollowerFeedbackCallback))
        {
            LOG_ERROR("Failed to subscribe to last follower feedback topic.");
        }
        else
        {
            isSuccessful = true;
        }
    }

    return isSuccessful;
}

bool App::setupSerialMuxProt()
{
    bool isSuccessful = false;

    /* Channel subscription. */
    m_smpServer.subscribeToChannel(COMMAND_RESPONSE_CHANNEL_NAME, App_cmdRspChannelCallback);
    m_smpServer.subscribeToChannel(CURRENT_VEHICLE_DATA_CHANNEL_NAME, App_currentVehicleChannelCallback);

    /* Channel creation. */
    m_serialMuxProtChannelIdRemoteCtrl = m_smpServer.createChannel(COMMAND_CHANNEL_NAME, COMMAND_CHANNEL_DLC);
    m_serialMuxProtChannelIdMotorSpeeds =
        m_smpServer.createChannel(SPEED_SETPOINT_CHANNEL_NAME, SPEED_SETPOINT_CHANNEL_DLC);
    m_serialMuxProtChannelIdStatus = m_smpServer.createChannel(STATUS_CHANNEL_NAME, STATUS_CHANNEL_DLC);

    if ((0U != m_serialMuxProtChannelIdRemoteCtrl) && (0U != m_serialMuxProtChannelIdMotorSpeeds) &&
        (0U != m_serialMuxProtChannelIdStatus))
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

void App::processPeriodicTasks()
{
    if ((true == m_commandTimer.isTimeout()) && (true == m_smpServer.isSynced()))
    {
        Command payload;
        bool    isPending = StartupState::getInstance().getPendingCommand(payload);

        if (true == isPending)
        {
            if (false == m_smpServer.sendData(m_serialMuxProtChannelIdRemoteCtrl, &payload, sizeof(payload)))
            {
                LOG_WARNING("Failed to send StartupState pending command to RU.");
            }
        }

        m_commandTimer.restart();
    }

    if ((true == m_sendWaypointTimer.isTimeout()) && (true == m_mqttClient.isConnected()))
    {
        if (false == m_v2vClient.sendWaypoint(m_latestVehicleData))
        {
            LOG_WARNING("Waypoint could not be sent.");
        }

        m_sendWaypointTimer.restart();
    }

    if ((true == m_motorSpeedTimer.isTimeout()) && (true == m_smpServer.isSynced()))
    {
        int16_t   centerSpeed = 0;
        SpeedData payload;

        if (false == DrivingState::getInstance().getTopMotorSpeed(centerSpeed))
        {
            centerSpeed = 0;
        }

        payload.center = centerSpeed;

        if (false == m_smpServer.sendData(m_serialMuxProtChannelIdMotorSpeeds, &payload, sizeof(payload)))
        {
            LOG_WARNING("Failed to send motor speeds to RU.");
        }

        m_motorSpeedTimer.restart();
    }

    if ((true == m_statusTimer.isTimeout()) && (true == m_smpServer.isSynced()))
    {
        Status payload = {SMPChannelPayload::Status::STATUS_FLAG_OK};

        if (true == ErrorState::getInstance().isActive())
        {
            payload.status = SMPChannelPayload::Status::STATUS_FLAG_ERROR;
        }

        if (false == m_smpServer.sendData(m_serialMuxProtChannelIdStatus, &payload, sizeof(payload)))
        {
            LOG_WARNING("Failed to send current status to RU.");
        }

        m_statusTimer.restart();
    }

    if ((false == m_statusTimeoutTimer.isTimerRunning()) && (true == m_smpServer.isSynced()))
    {
        /* Start status timeout timer once SMP is synced the first time. */
        m_statusTimeoutTimer.start(STATUS_TIMEOUT_TIMER_INTERVAL);
    }
    else if (true == m_statusTimeoutTimer.isTimeout())
    {
        /* Not receiving status from RU. Go to error state. */
        setErrorState();
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
    if ((nullptr != payload) && (COMMAND_RESPONSE_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        App*                   application = reinterpret_cast<App*>(userData);
        const CommandResponse* cmdRsp      = reinterpret_cast<const CommandResponse*>(payload);
        LOG_DEBUG("CMD_RSP: ID: 0x%02X , RSP: 0x%02X", cmdRsp->commandId, cmdRsp->responseId);

        if (SMPChannelPayload::RSP_ID_ERROR == cmdRsp->responseId)
        {
            /* Go to error state. */
            application->setErrorState();
        }
        else if (SMPChannelPayload::CMD_ID_GET_MAX_SPEED == cmdRsp->commandId)
        {
            LOG_DEBUG("Max Speed: %d", cmdRsp->maxMotorSpeed);
            DrivingState::getInstance().setMaxMotorSpeed(cmdRsp->maxMotorSpeed);
            StartupState::getInstance().notifyCommandProcessed();
        }
        else if (SMPChannelPayload::CMD_ID_SET_INIT_POS == cmdRsp->commandId)
        {
            StartupState::getInstance().notifyCommandProcessed();
        }
    }
    else
    {
        LOG_WARNING("CMD_RSP: Invalid payload size. Expected: %u Received: %u", COMMAND_RESPONSE_CHANNEL_DLC,
                    payloadSize);
    }
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
    if ((nullptr != payload) && (CURRENT_VEHICLE_DATA_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        const VehicleData* currentVehicleData = reinterpret_cast<const VehicleData*>(payload);
        App*               application        = reinterpret_cast<App*>(userData);
        Waypoint dataAsWaypoint(currentVehicleData->xPos, currentVehicleData->yPos, currentVehicleData->orientation,
                                currentVehicleData->left, currentVehicleData->right, currentVehicleData->center);

        application->setLatestVehicleData(dataAsWaypoint);
        DrivingState::getInstance().setVehicleData(*currentVehicleData);
    }
    else
    {
        LOG_WARNING("%s: Invalid payload size. Expected: %u Received: %u", CURRENT_VEHICLE_DATA_CHANNEL_NAME,
                    CURRENT_VEHICLE_DATA_CHANNEL_DLC, payloadSize);
    }
}

/**
 * Receives current status of the RU over SerialMuxProt channel.
 *
 * @param[in] payload       Status of the RU.
 * @param[in] payloadSize   Size of the Status Flag
 * @param[in] userData      Instance of App class.
 */
void App_statusChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    if ((nullptr != payload) && (STATUS_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        const Status* currentStatus = reinterpret_cast<const Status*>(payload);
        App*          application   = reinterpret_cast<App*>(userData);
        application->systemStatusCallback(currentStatus->status);
    }
}
