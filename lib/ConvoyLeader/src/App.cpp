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
#include <ProcessingChainFactory.h>
#include "LongitudinalController.h"
#include "LongitudinalSafetyPolicy.h"
#include "LateralController.h"
#include "LateralSafetyPolicy.h"

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

/** Buffer size for JSON serialization of birth / will message */
static const uint32_t JSON_BIRTHMESSAGE_MAX_SIZE = 64U;

/** Platoon leader vehicle ID. */
static const uint8_t PLATOON_LEADER_ID = 0U;

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
                else if (PLATOON_LEADER_ID != settings.getPlatoonVehicleId())
                {
                    /* Correct config.json file loaded? */
                    LOG_FATAL("Platoon Vehicle ID must be 0 for the leader.");
                }
                else if (false == m_v2vClient.init(settings.getPlatoonPlatoonId(), settings.getPlatoonVehicleId()))
                {
                    LOG_FATAL("Failed to initialize V2V client.");
                }
                else
                {
                    /* Setup SerialMuxProt Channels */
                    m_smpServer.subscribeToChannel(CURRENT_VEHICLE_DATA_CHANNEL_DLC_CHANNEL_NAME,
                                                   App_currentVehicleChannelCallback);
                    m_serialMuxProtChannelIdMotorSpeedSetpoints =
                        m_smpServer.createChannel(SPEED_SETPOINT_CHANNEL_NAME, SPEED_SETPOINT_CHANNEL_DLC);

                    if (0U == m_serialMuxProtChannelIdMotorSpeedSetpoints)
                    {
                        LOG_FATAL("Could not create SerialMuxProt Channel: %s.", SPEED_SETPOINT_CHANNEL_NAME);
                    }
                    else
                    {

                        ProcessingChainFactory& processingChainFactory = ProcessingChainFactory::getInstance();

                        processingChainFactory.registerLongitudinalControllerCreateFunc(LongitudinalController::create);
                        processingChainFactory.registerLongitudinalSafetyPolicyCreateFunc(
                            LongitudinalSafetyPolicy::create);
                        processingChainFactory.registerLateralControllerCreateFunc(LateralController::create);
                        processingChainFactory.registerLateralSafetyPolicyCreateFunc(LateralSafetyPolicy::create);

                        PlatoonController::InputWaypointCallback lambdaInputWaypointCallback =
                            [this](Waypoint& waypoint) { return this->inputWaypointCallback(waypoint); };
                        PlatoonController::OutputWaypointCallback lambdaOutputWaypointCallback =
                            [this](const Waypoint& waypoint) { return this->outputWaypointCallback(waypoint); };
                        PlatoonController::MotorSetpointCallback lambdaMotorSetpointCallback =
                            [this](const int16_t left, const int16_t right)
                        { return this->motorSetpointCallback(left, right); };

                        if (false == m_platoonController.init(lambdaInputWaypointCallback, lambdaOutputWaypointCallback,
                                                              lambdaMotorSetpointCallback))
                        {
                            LOG_FATAL("Could not initialize Platoon Controller.");
                        }
                        else
                        {
                            isSuccessful = true;
                        }
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

    /* Process Platoon Controller */
    m_platoonController.process();
}

void App::currentVehicleChannelCallback(const VehicleData& vehicleData)
{
    Waypoint vehicleDataAsWaypoint;

    vehicleDataAsWaypoint.xPos        = vehicleData.xPos;
    vehicleDataAsWaypoint.yPos        = vehicleData.yPos;
    vehicleDataAsWaypoint.orientation = vehicleData.orientation;
    vehicleDataAsWaypoint.left        = vehicleData.left;
    vehicleDataAsWaypoint.right       = vehicleData.right;
    vehicleDataAsWaypoint.center      = vehicleData.center;

    m_platoonController.setLatestVehicleData(vehicleDataAsWaypoint);
}

bool App::inputWaypointCallback(Waypoint& waypoint)
{
    return m_v2vClient.getNextWaypoint(waypoint);
}

bool App::outputWaypointCallback(const Waypoint& waypoint)
{
    return m_v2vClient.sendWaypoint(waypoint);
}

bool App::motorSetpointCallback(const int16_t left, const int16_t right)
{
    SpeedData payload;
    payload.left  = left;
    payload.right = right;

    return m_smpServer.sendData(m_serialMuxProtChannelIdMotorSpeedSetpoints, &payload, sizeof(payload));
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
        LOG_WARNING("%s: Invalid payload size. Expected: %u Received: %u",
                    CURRENT_VEHICLE_DATA_CHANNEL_DLC_CHANNEL_NAME, CURRENT_VEHICLE_DATA_CHANNEL_DLC, payloadSize);
    }
}
