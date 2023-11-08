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
#include "HeadingFinder.h"
#include <Board.h>
#include <Logging.h>
#include <LogSinkPrinter.h>
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

static void App_odometryChannelCallback(const uint8_t* payload, const uint8_t payloadSize);
static void App_speedChannelCallback(const uint8_t* payload, const uint8_t payloadSize);

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

/* MQTT topic name for receiving position setpoint coordinates. */
const char* App::TOPIC_NAME_POSITION_SETPOINT = "positionSetpoint";

/** Default size of the JSON Document for parsing. */
static const uint32_t JSON_DOC_DEFAULT_SIZE = 1024U;

/** Buffer size for JSON serialization of birth / will message */
static const uint32_t JSON_BIRTHMESSAGE_MAX_SIZE = 64U;

/** HeadingFinder Instance. */
static HeadingFinder gHeadingFinder;

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
                /* Subscribe to Position Setpoint Topic. */
                else if (false == m_mqttClient.subscribe(TOPIC_NAME_POSITION_SETPOINT, [this](const String& payload)
                                                         { positionTopicCallback(payload); }))
                {
                    LOG_FATAL("Could not subcribe to MQTT Topic: %s.", TOPIC_NAME_POSITION_SETPOINT);
                }
                else
                {
                    /* Setup SerialMuxProt Channels */
                    m_smpServer.subscribeToChannel(ODOMETRY_CHANNEL_NAME, App_odometryChannelCallback);
                    m_smpServer.subscribeToChannel(SPEED_CHANNEL_NAME, App_speedChannelCallback);
                    m_serialMuxProtChannelIdMotorSpeedSetpoints =
                        m_smpServer.createChannel(SPEED_SETPOINT_CHANNEL_NAME, SPEED_SETPOINT_CHANNEL_DLC);

                    if (0U == m_serialMuxProtChannelIdMotorSpeedSetpoints)
                    {
                        LOG_FATAL("Could not create SerialMuxProt Channel: %s.", SPEED_SETPOINT_CHANNEL_NAME);
                    }
                    else
                    {
                        /* Initialize HeadingFinder. */
                        gHeadingFinder.init();
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

    /* Set new target speeds. */
    sendSpeedSetpoints();
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void App::positionTopicCallback(const String& payload)
{
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> jsonPayload;
    DeserializationError                      error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON Deserialization Error %d.", error);
    }
    else
    {
        JsonVariant jsonXPos = jsonPayload["X"];
        JsonVariant jsonYPos = jsonPayload["Y"];

        if ((false == jsonXPos.isNull()) && (false == jsonYPos.isNull()))
        {
            int32_t positionX = jsonXPos.as<int32_t>();
            int32_t positionY = jsonYPos.as<int32_t>();

            LOG_DEBUG("Received position setpoint: x: %d y: %d", positionX, positionY);
            gHeadingFinder.setTargetHeading(positionX, positionY);
        }
        else
        {
            LOG_WARNING("Received invalid position.");
        }
    }
}

void App::fatalErrorHandler()
{
    /* Turn on Red LED to signal fatal error. */
    Board::getInstance().getRedLed().enable(true);

    while (true)
    {
        ;
    }
}

void App::sendSpeedSetpoints()
{
    int16_t targetSpeedLeft  = 0;
    int16_t targetSpeedRight = 0;

    if (0 != gHeadingFinder.process(targetSpeedLeft, targetSpeedRight))
    {
        SpeedData payload;
        payload.left  = targetSpeedLeft;
        payload.right = targetSpeedRight;

        if (false == m_smpServer.sendData(m_serialMuxProtChannelIdMotorSpeedSetpoints,
                                          reinterpret_cast<uint8_t*>(&payload), sizeof(payload)))
        {
            LOG_DEBUG("Could not send speed setpoints.");
        }
        else
        {
            StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> payloadJson;
            HeadingFinder::HeadingFinderData          data = gHeadingFinder.getLatestData();
            char                                      payloadArray[JSON_DOC_DEFAULT_SIZE];

            payloadJson["targetSpeedLeft"]  = targetSpeedLeft;
            payloadJson["targetSpeedRight"] = targetSpeedRight;
            payloadJson["targetHeading"]    = data.targetHeading;
            payloadJson["currentHeading"]   = data.currentHeading;

            (void)serializeJson(payloadJson, payloadArray);
            String payloadStr(payloadArray);

            m_mqttClient.publish("pid", true, payloadStr);
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
 * Receives current position and heading of the robot over SerialMuxProt channel.
 *
 * @param[in] payload       Odometry data. Two coordinates and one orientation.
 * @param[in] payloadSize   Size of two coordinates and one orientation.
 */
void App_odometryChannelCallback(const uint8_t* payload, const uint8_t payloadSize)
{
    if ((nullptr != payload) && (ODOMETRY_CHANNEL_DLC == payloadSize))
    {
        const OdometryData* odometryData = reinterpret_cast<const OdometryData*>(payload);
        LOG_DEBUG("ODOMETRY: x: %d y: %d orientation: %d", odometryData->xPos, odometryData->yPos,
                  odometryData->orientation);
        gHeadingFinder.setOdometryData(odometryData->xPos, odometryData->yPos, odometryData->orientation);
    }
    else
    {
        LOG_WARNING("ODOMETRY: Invalid payload size. Expected: %u Received: %u", ODOMETRY_CHANNEL_DLC, payloadSize);
    }
}

/**
 * Receives current motor speeds of the robot over SerialMuxProt channel.
 *
 * @param[in] payload       Motor speeds.
 * @param[in] payloadSize   Size of two motor speeds.
 */
void App_speedChannelCallback(const uint8_t* payload, const uint8_t payloadSize)
{
    if ((nullptr != payload) && (SPEED_CHANNEL_DLC == payloadSize))
    {
        const SpeedData* motorSpeedData = reinterpret_cast<const SpeedData*>(payload);
        LOG_DEBUG("SPEED: left: %d right: %d", motorSpeedData->left, motorSpeedData->right);
        gHeadingFinder.setMotorSpeedData(motorSpeedData->left, motorSpeedData->right);
    }
    else
    {
        LOG_WARNING("SPEED: Invalid payload size. Expected: %u Received: %u", SPEED_CHANNEL_DLC, payloadSize);
    }
}
