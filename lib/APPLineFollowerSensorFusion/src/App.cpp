/* MIT License
 *
 * Copyright (c) 2023 - 2025 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Line follower Sensor Fusion application
 * @author Andreas Merkle <web@blue-andi.de>
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
#include "States/StartupState.h"
#include "States/LineSensorsCalibrationState.h"
#include "States/ErrorState.h"
#include "States/ReadyState.h"
#include "States/DrivingState.h"

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

/******************************************************************************
 * Local Variables
 *****************************************************************************/
uint32_t pingPeriodMs = 100U;    /**< Serial ping period [ms]. (5 minutes)*/
uint32_t rtcRefreshMs = 300000U; /**< RTC refresh period [ms]. (5 minutes) */

/** Serial interface baudrate. */
static const uint32_t SERIAL_BAUDRATE = 115200U;

/** Serial log sink */
static LogSinkPrinter gLogSinkSerial("Serial", &Serial);

/* MQTT topic name for birth messages. */
const char* App::TOPIC_NAME_BIRTH = "birth";

/* MQTT topic name for will messages. */
const char* App::TOPIC_NAME_WILL = "will";

/** MQTT topic name for status messages. */
const char* App::TOPIC_NAME_STATUS = "zumo/status";

/** MQTT topic name for fusion Pose */
const char* App::TOPIC_NAME_FUSION_POSE = "zumo/fusion";

/** MQTT topic name for raw Sensor data */
const char* App::TOPIC_NAME_RAW_SENSORS = "zumo/sensors";

/** MQTT topic name for receiving Space Ship Radar Pose. */
const char* App::TOPIC_NAME_RADAR_POSE = "ssr";

/** Buffer size for JSON serialization of birth / will message */
static const uint32_t JSON_BIRTHMESSAGE_MAX_SIZE = 64U;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

App::App() :
    m_initialDataSent(false),
    m_statusTimer(),
    m_serMuxChannelProvider(Board::getInstance().getRobot().getStream()),
    m_timeSync(m_serMuxChannelProvider),
    m_lineSensors(m_serMuxChannelProvider),
    m_motors(m_serMuxChannelProvider),
    m_stateMachine(),
    m_mqttClient()
{
    /* Inject dependencies into states. */
    StartupState::getInstance().injectDependencies(m_serMuxChannelProvider, m_motors);
    LineSensorsCalibrationState::getInstance().injectDependencies(m_serMuxChannelProvider);
    ReadyState::getInstance().injectDependencies(m_lineSensors);
    DrivingState::getInstance().injectDependencies(m_lineSensors, m_motors);
}

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
        NetworkSettings networkSettings = {settings.getWiFiSSID(), settings.getWiFiPassword(), settings.getRobotName(),
                                           ""};

        /* If the robot name is empty, use the wifi MAC address as robot name. */
        if (true == settings.getRobotName().isEmpty())
        {
            String robotName = WiFi.macAddress();

            /* Remove MAC separators from robot name. */
            robotName.replace(":", "");

            settings.setRobotName(robotName);
        }

        if (false == board.getNetwork().setConfig(networkSettings))
        {
            LOG_FATAL("Network configuration could not be set.");
        }
        else if (false == m_serMuxChannelProvider.init())
        {
            LOG_FATAL("SerialMuxChannelProvider init failed.");
        }
        else if (false == setupMqtt(settings.getRobotName(), settings.getMqttBrokerAddress(), settings.getMqttPort()))
        {
            LOG_FATAL("MQTT connection could not be setup.");
        }
        else
        {
            /* Start network time (SNTP) against Host and Zumo serial ping-pong. */
            m_timeSync.begin(settings.getMqttBrokerAddress().c_str(), pingPeriodMs, rtcRefreshMs);
            m_statusTimer.start(1000U);
            isSuccessful = true;
        }
    }

    if (false == isSuccessful)
    {
        LOG_FATAL("Initialization failed.");
        m_stateMachine.setState(ErrorState::getInstance());
    }
    else
    {
        LOG_INFO("Line follower Sensor Fusion application is ready.");

        /* Set initial state of the state machine. */
        m_stateMachine.setState(StartupState::getInstance());
    }
}

void App::loop()
{
    /* Process battery, device and network. */
    Board::getInstance().process();

    /* Process MQTT Communication */
    m_mqttClient.process();

    /* Process serial multiplexer. */
    m_serMuxChannelProvider.process();

    /* Process time synchronization (SNTP refresh + serial ping-pong). */
    m_timeSync.process();

    /* Process statemachine. */
    m_stateMachine.process();

    /* Send heartbeat status to Radon Ulzer controller periodically. */
    if ((true == m_statusTimer.isTimeout()) && (true == m_serMuxChannelProvider.isInSync()))
    {
        Status dcsStatus = {SMPChannelPayload::Status::STATUS_FLAG_OK};

        if (&ErrorState::getInstance() == m_stateMachine.getState())
        {
            dcsStatus.status = SMPChannelPayload::Status::STATUS_FLAG_ERROR;
        }

        if (false == m_serMuxChannelProvider.sendStatus(dcsStatus))
        {
            LOG_WARNING("Failed to send status to RU.");
        }

        m_statusTimer.restart();
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/
bool App::setupMqtt(const String& clientId, const String& brokerAddr, uint16_t brokerPort)
{
    bool         isSuccessful = false;
    JsonDocument jsonBirthDoc;
    char         birthMsgArray[JSON_BIRTHMESSAGE_MAX_SIZE];
    String       birthMessage;
    const String ssrTopic = String(TOPIC_NAME_RADAR_POSE) + "/" + clientId;

    jsonBirthDoc["name"] = clientId.c_str();
    (void)serializeJson(jsonBirthDoc, birthMsgArray);
    birthMessage = birthMsgArray;

    if (false == m_mqttClient.init())
    {
        LOG_FATAL("Failed to initialize MQTT client.");
    }
    else
    {
        MqttSettings mqttSettings = {clientId,     brokerAddr,      brokerPort,   TOPIC_NAME_BIRTH,
                                     birthMessage, TOPIC_NAME_WILL, birthMessage, true};

        if (false == m_mqttClient.setConfig(mqttSettings))
        {
            LOG_FATAL("MQTT configuration could not be set.");
        }
        /* Subscribe to Space Ship Radar Topic. */
        else if (false ==
                 m_mqttClient.subscribe(ssrTopic, false, [this](const String& payload) { SSRTopicCallback(payload); }))
        {
            LOG_FATAL("Could not subcribe to MQTT topic: %s.", TOPIC_NAME_RADAR_POSE);
        }
        else
        {
            isSuccessful = true;
            LOG_INFO("Subscribed to MQTT topic: %s.", ssrTopic.c_str());
        }
    }

    LOG_INFO("MQTT setup %s.", (true == isSuccessful) ? "successful" : "failed");
    return isSuccessful;
}

void App::SSRTopicCallback(const String& payload)
{
    JsonDocument         jsonPayload;
    DeserializationError error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON Deserialization error %d.", error);
    }
    else
    {
        JsonVariantConst xPos_mm    = jsonPayload["positionX"];  /* int : in mm */
        JsonVariantConst yPos_mm    = jsonPayload["positionY"];  /* int : in mm */
        JsonVariantConst xVel_mms   = jsonPayload["speedX"];     /* int : in mm/s */
        JsonVariantConst yVel_mms   = jsonPayload["speedY"];     /* int : in mm/s */
        JsonVariantConst angle_mrad = jsonPayload["angle"];      /* int : in mrad */
        JsonVariantConst id         = jsonPayload["identifier"]; /* int : unique id of the target */
        LOG_INFO("SSR pose: xPos=%dmm yPos=%dmm angle=%dmrad vx=%dmm/s vy=%dmm/s", xPos_mm, yPos_mm, angle_mrad,
                 xVel_mms, yVel_mms);
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
