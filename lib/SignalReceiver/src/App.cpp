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
 * @brief  SignalReceiver application
 * @author Paul Gramescu <paul.gramescu@gmail.com>
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
#include <CoordinateHandler.h>
#include <TrafficElement.h>
#include <TrafficHandler.h>
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

/* MQTT topic name for receiving traffic light color IDs. */
const char* App::TOPIC_NAME_TRAFFIC_LIGHT_COLORS = "trafficLightColors";

/* MQTT topic name for receiving settings. */
const char* App::TOPIC_NAME_SETTINGS = "settings";

/** Default size of the JSON Document for parsing. */
static const uint32_t JSON_DOC_DEFAULT_SIZE = 1024U;

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
        else
        {
            /* Setup MQTT Server, Birth and Will messages. */
            StaticJsonDocument<JSON_BIRTHMESSAGE_MAX_SIZE> birthDoc;
            char                                           birthMsgArray[JSON_BIRTHMESSAGE_MAX_SIZE];
            String                                         birthMessage;

            birthDoc["name"] = settings.getRobotName().c_str();
            (void)serializeJson(birthDoc, birthMsgArray);
            birthMessage = birthMsgArray;

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
                else if (false == m_mqttClient.subscribe(TOPIC_NAME_SETTINGS, false,
                                                         [this](const String& payload) { settingsCallback(payload); }))
                {
                    LOG_FATAL("Could not subcribe to MQTT topic: %s.", TOPIC_NAME_TRAFFIC_LIGHT_COLORS);
                }
                else
                {
                    /* Setup SMP channels. */
                    m_smpServer.subscribeToChannel(CURRENT_VEHICLE_DATA_CHANNEL_NAME,
                                                   App_currentVehicleChannelCallback);
                    m_smpServer.subscribeToChannel(COMMAND_RESPONSE_CHANNEL_NAME, App_cmdRspChannelCallback);
                    m_smpServer.subscribeToChannel(STATUS_CHANNEL_NAME, App_statusChannelCallback);

                    m_serialMuxProtChannelIdRemoteCtrl =
                        m_smpServer.createChannel(COMMAND_CHANNEL_NAME, COMMAND_CHANNEL_DLC);
                    m_serialMuxProtChannelIdMotorSpeeds =
                        m_smpServer.createChannel(SPEED_SETPOINT_CHANNEL_NAME, SPEED_SETPOINT_CHANNEL_DLC);
                    m_serialMuxProtChannelIdStatus = m_smpServer.createChannel(STATUS_CHANNEL_NAME, STATUS_CHANNEL_DLC);

                    if ((0U == m_serialMuxProtChannelIdRemoteCtrl) && (0U == m_serialMuxProtChannelIdMotorSpeeds) &&
                        (0U == m_serialMuxProtChannelIdStatus))
                    {
                        LOG_FATAL("Could not create SerialMuxProt Channels.");
                    }
                    else
                    {
                        m_commandTimer.start(SEND_COMMANDS_TIMER_INTERVAL);
                        m_motorSpeedTimer.start(SEND_MOTOR_SPEED_TIMER_INTERVAL);
                        m_statusTimer.start(SEND_STATUS_TIMER_INTERVAL);

                        /**
                         * Asyncronous time amount to avoid a syncronization (a time amount equal to a multiple
                         * of 5) between the RU odometry callback and traffic processing.
                         *
                         * If they are synced, there will be a cycle that will register old
                         * and new distance as equal, which is false.
                         */
                        m_processTrafficTimer.start(PROCESS_TRAFFIC_TIMER_INTERVAL);

                        /* Start with startup state. */
                        m_systemStateMachine.setState(&StartupState::getInstance());

                        isSuccessful = true;
                    }
                }
            }
        }
    }

    if (false == isSuccessful)
    {
        fatalErrorHandler();
    }
    else
    {
        LOG_DEBUG("Battery is %d.", board.getBattery().getChargeLevel());
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

    /** Process MQTT Communication */
    m_mqttClient.process();

    /* Process System State Machine */
    m_systemStateMachine.process();

    /* Process periodic tasks. */
    processPeriodicTasks();
}

void App::odometryCallback(const VehicleData& odometry)
{
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> payloadJson;
    char                                      payloadArray[JSON_DOC_DEFAULT_SIZE];

    LOG_DEBUG("RECEIVED ODOMETRY: x: %d y: %d ORIENTATION:  %d", odometry.xPos, odometry.yPos, odometry.orientation);

    /* Set new coordinates in for processing. */
    CoordinateHandler::getInstance().setCurrentOrientation(odometry.orientation);
    CoordinateHandler::getInstance().setCurrentCoordinates(odometry.xPos, odometry.yPos);
}

void App::setErrorState()
{
    m_systemStateMachine.setState(&ErrorState::getInstance());
}

void App::systemStatusCallback(SMPChannelPayload::Status status)
{
    switch (status)
    {
    case SMPChannelPayload::STATUS_FLAG_OK:
        /* Nothing to do. All good. */
        break;

    case SMPChannelPayload::STATUS_FLAG_ERROR:
        LOG_DEBUG("RU Status ERROR.");
        setErrorState();
        m_statusTimeoutTimer.stop();
        break;

    default:
        break;
    }

    m_statusTimeoutTimer.start(STATUS_TIMEOUT_TIMER_INTERVAL);
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

    if (true == m_processTrafficTimer.isTimeout())
    {
        processTraffic();
        m_processTrafficTimer.restart();
    }

    if ((false == m_statusTimeoutTimer.isTimerRunning()) && (true == m_smpServer.isSynced()))
    {
        /* Start status timeout timer once SMP is synced the first time. */
        m_statusTimeoutTimer.start(STATUS_TIMEOUT_TIMER_INTERVAL);
    }
    else if (true == m_statusTimeoutTimer.isTimeout())
    {
        /* Not receiving status from RU. Go to error state. */
        LOG_DEBUG("RU Status timeout.");
        setErrorState();
        m_statusTimeoutTimer.stop();
    }
}

void App::processTraffic()
{
    /* Process traffic only when robot is driving. */
    if ((true == DrivingState::getInstance().isActive()))
    {
        if (true == TrafficHandler::getInstance().process())
        {
            if (true == TrafficHandler::getInstance().checkLockIn())
            {
                /* Check the subscribe-once flag. */
                if (false == m_isSubscribed)
                {
                    if (TrafficHandler::getInstance().getTargetName() != nullptr)
                    {
                        lockedOnto = TrafficHandler::getInstance().getTargetName();

                        if (true == m_mqttClient.subscribe(TrafficHandler::getInstance().getTargetName(), false,
                                                           [this](const String& payload)
                                                           { trafficLightColorsCallback(payload); }))
                        {
                            LOG_DEBUG("Subscribed to Channel:%s.",
                                      TrafficHandler::getInstance().getTargetName().c_str());
                        }

                        m_isSubscribed = true;
                    }
                }
                else
                {
                    LOG_WARNING("Already subscribed to %s", TrafficHandler::getInstance().getTargetName().c_str());
                }

                /* If near, listen to signals. */
                if (true == TrafficHandler::getInstance().isNear())
                {
                    LOG_DEBUG("Near IE, listening for signals from topic %s.",
                              TrafficHandler::getInstance().getTargetName().c_str());

                    /* Processing color only when near the IE. */
                    TrafficHandler::getInstance().processColor();
                }
                else
                {
                    LOG_DEBUG("Robot has more driving towards %s to do.",
                              TrafficHandler::getInstance().getTargetName().c_str());
                    m_isListening = false;
                }
            }
            else
            {
                /* Unsub from IE. */
                if (TrafficHandler::getInstance().getTargetName() == nullptr)
                {
                    LOG_DEBUG("Target name is invalid.");
                }
                else if ((TrafficHandler::getInstance().getTargetName() != nullptr) &&
                         (TrafficHandler::getInstance().getTargetName() != ""))
                {
                    LOG_DEBUG("No longer locked onto IE, unsubbing from %s.",
                              TrafficHandler::getInstance().getTargetName().c_str());
                    m_mqttClient.unsubscribe(lockedOnto, false);
                }
                else
                {
                    LOG_DEBUG("Nothing to unsubscribe to.");
                }

                m_isListening  = false;
                m_isSubscribed = false;
            }
        }
    }
}

void App::trafficLightColorsCallback(const String& payload)
{
    /* JSON library initialization. */
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> jsonPayload;
    DeserializationError                      error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON Deserialization Error %d.", error);
    }
    else
    {
        /* Who is sending? */
        JsonVariant from = jsonPayload["FROM"];

        /* what is the color ID? */
        JsonVariant color = jsonPayload["COLOR"];

        if ((false == color.isNull()) && (false == from.isNull()))
        {
            /* If a new color has been received, deserialize it! */
            if (oldColorId.colorId != color.as<uint8_t>())
            {
                clr.colorId = color.as<uint8_t>();

                TrafficHandler::getInstance().setColorID(clr.colorId);

                oldColorId.colorId = clr.colorId;
                LOG_DEBUG("COLOR_ID: %d", clr.colorId);
            }
            else
            {
                /* Received the same color due to periodic publishing of IE. It won't be deserialized. */
            }
        }
        else
        {
            LOG_WARNING("Received invalid color.");
        }
    }
}

void App::settingsCallback(const String& payload)
{
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> jsonPayload;
    DeserializationError                      error = deserializeJson(jsonPayload, payload.c_str());

    /* Used to create topic of IE. */
    char   topic[64U];
    String topicString;

    TrafficHandler& handler = TrafficHandler::getInstance();

    String  IEname;
    int32_t receivedOrientation;
    int32_t receivedEntryX;
    int32_t receivedEntryY;
    int32_t distance;
    int32_t previousDistance;

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON deserialization error %d.", error);
    }
    else
    {
        JsonVariant name             = jsonPayload["FROM"];
        JsonVariant orientationValue = jsonPayload["TOWARDS"];
        JsonVariant xEntryValue      = jsonPayload["EX"];
        JsonVariant yEntryValue      = jsonPayload["EY"];

        if ((false == xEntryValue.isNull()) && (false == yEntryValue.isNull()))
        {
            IEname              = name.as<const char*>();
            receivedOrientation = orientationValue.as<int32_t>();
            receivedEntryX      = xEntryValue.as<int32_t>();
            receivedEntryY      = yEntryValue.as<int32_t>();
            distance            = 0;
            previousDistance    = 0;

            /* Create the topic of received IE. */
            if (0 <= snprintf(topic, 64U, "%s/trafficLightColors", IEname.c_str()))
            {
                topicString = topic;
            }

            if ((true == handler.setNewInfrastructureElement(IEname, receivedOrientation, receivedEntryX,
                                                             receivedEntryY, distance, previousDistance, topicString)))
            {
                LOG_DEBUG("Received settings from %s at X %d, Y %d pointing towards %d.", IEname.c_str(),
                          receivedEntryX, receivedEntryY, receivedOrientation);
                LOG_DEBUG("Created MQTT topic with IE: %s.", topicString.c_str());
            }
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
 * @param[in] payload       Odometry data. Two coordinates and one orientation.
 * @param[in] payloadSize   Size of two coordinates and one orientation.
 * @param[in] userData      Instance of App class.
 */
void App_currentVehicleChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    (void)userData;
    if ((nullptr != payload) && (CURRENT_VEHICLE_DATA_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        const VehicleData* odometryData = reinterpret_cast<const VehicleData*>(payload);
        App*               application  = reinterpret_cast<App*>(userData);

        application->odometryCallback(*odometryData);
    }
    else
    {
        LOG_WARNING("ODOMETRY: Invalid payload size. Expected: %u Received: %u", CURRENT_VEHICLE_DATA_CHANNEL_DLC,
                    payloadSize);
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
        LOG_DEBUG("Status received.");
        const Status* currentStatus = reinterpret_cast<const Status*>(payload);
        App*          application   = reinterpret_cast<App*>(userData);
        application->systemStatusCallback(currentStatus->status);
    }
}