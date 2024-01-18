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
#include "RemoteControl.h"
#include <Board.h>
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <SettingsHandler.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Util.h>

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
            LongitudinalController::MotorSetpointCallback motorSetpointCallback = [this](const int16_t& topCenterSpeed)
            { return this->motorSetpointCallback(topCenterSpeed); };

            m_longitudinalController.setMotorSetpointCallback(motorSetpointCallback);

            m_sendWaypointTimer.start(SEND_WAYPOINT_TIMER_INTERVAL);

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

    /* Process Longitudinal Controller */
    m_longitudinalController.process();

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

    if (LongitudinalController::STATE_STARTUP == m_longitudinalController.getState())
    {
        Command payload;
        payload.commandId = RemoteControl::CMD_ID_GET_MAX_SPEED;

        (void)m_smpServer.sendData(m_serialMuxProtChannelIdRemoteCtrl, &payload, sizeof(payload));
    }

    if (true == m_sendWaypointTimer.isTimeout())
    {
        if (false == m_v2vClient.sendWaypoint(latestVehicleData))
        {
            LOG_WARNING("Waypoint could not be sent.");
        }
        else
        {
            m_sendWaypointTimer.restart();
        }
    }
}

void App::currentVehicleChannelCallback(const VehicleData& vehicleData)
{
    latestVehicleData = Waypoint(vehicleData.xPos, vehicleData.yPos, vehicleData.orientation, vehicleData.left,
                                 vehicleData.right, vehicleData.center);

    latestVehicleData.debugPrint();

    m_longitudinalController.calculateTopMotorSpeed(latestVehicleData);
}

bool App::motorSetpointCallback(const int16_t topCenterSpeed)
{
    SpeedData payload;
    payload.center = topCenterSpeed;

    return m_smpServer.sendData(m_serialMuxProtChannelIdMotorSpeeds, &payload, sizeof(payload));
}

void App::release()
{
    Command payload;
    payload.commandId = RemoteControl::CMD_ID_START_DRIVING;

    /* Release robot. */
    if (false == m_longitudinalController.release())
    {
        LOG_WARNING("Robot could not be released.");
    }
    else if (false == m_smpServer.sendData(m_serialMuxProtChannelIdRemoteCtrl, &payload, sizeof(payload)))
    {
        LOG_WARNING("Failed to send release command to RU.");
    }
    else
    {
        LOG_INFO("Robot released.");
    }
}

void App::setMaxMotorSpeed(const int16_t maxMotorSpeed)
{
    m_longitudinalController.setMaxMotorSpeed(maxMotorSpeed);
}

void App::setLastFollowerFeedback(const Waypoint& feedback)
{
    m_longitudinalController.setLastFollowerFeedback(feedback);
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
    char                                           birthMsgArray[JSON_BIRTHMESSAGE_MAX_SIZE];
    String                                         birthMessage;

    birthDoc["name"] = settings.getRobotName();
    (void)serializeJson(birthDoc, birthMsgArray);
    birthMessage = birthMsgArray;

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
        isSuccessful =
            m_mqttClient.subscribe(TOPIC_NAME_RELEASE, true, [this](const String& payload) { this->release(); });

        isSuccessful &= m_mqttClient.subscribe("platoons/0/vehicles/0/feedback", false,
                                               [this](const String& payload)
                                               {
                                                   Waypoint* waypoint = Waypoint::deserialize(payload);

                                                   if (nullptr != waypoint)
                                                   {
                                                       this->setLastFollowerFeedback(*waypoint);
                                                       delete waypoint;
                                                   }
                                               });
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
    m_serialMuxProtChannelInitialVehicleData =
        m_smpServer.createChannel(INITIAL_VEHICLE_DATA_CHANNEL_NAME, INITIAL_VEHICLE_DATA_CHANNEL_DLC);

    if ((0U != m_serialMuxProtChannelIdRemoteCtrl) && (0U != m_serialMuxProtChannelIdMotorSpeeds) &&
        (0U != m_serialMuxProtChannelInitialVehicleData))
    {
        isSuccessful = true;
    }

    return isSuccessful;
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

        if (RemoteControl::CMD_ID_GET_MAX_SPEED == cmdRsp->commandId)
        {
            LOG_DEBUG("Max Speed: %d", cmdRsp->maxMotorSpeed);
            application->setMaxMotorSpeed(cmdRsp->maxMotorSpeed);
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
        application->currentVehicleChannelCallback(*currentVehicleData);
    }
    else
    {
        LOG_WARNING("%s: Invalid payload size. Expected: %u Received: %u", CURRENT_VEHICLE_DATA_CHANNEL_NAME,
                    CURRENT_VEHICLE_DATA_CHANNEL_DLC, payloadSize);
    }
}
