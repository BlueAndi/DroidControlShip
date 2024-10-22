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
 * @brief  SensorFusion application
 * @author Juliane Kerpe <juliane.kerpe@web.de>
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

static void App_sensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData);

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

/** MQTT topic name for sending the Position calculated by Sensor Fusion. */
const char* App::TOPIC_NAME_POSITION = "position";

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
            JsonDocument jsonBirthDoc;
            char         birthMsgArray[JSON_BIRTHMESSAGE_MAX_SIZE];
            String       birthMessage;

            jsonBirthDoc["name"] = settings.getRobotName().c_str();
            (void)serializeJson(jsonBirthDoc, birthMsgArray);
            birthMessage = birthMsgArray;

            /* Setup SerialMuxProt Channels */
            m_smpServer.subscribeToChannel(SENSORDATA_CHANNEL_NAME, App_sensorChannelCallback);

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
                else
                {
                    isSuccessful = true;
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

void App::publishSensorFusionPosition()
{
    JsonDocument payloadJson;
    char         payloadArray[JSON_DOC_DEFAULT_SIZE];

    /* Write newest Sensor Fusion Data in JSON String */
    IKalmanFilter::PositionData currentPosition = m_sensorFusion.getLatestPosition();
    payloadJson["positionX"]                    = static_cast<int32_t>(currentPosition.positionX + 0.5F);
    payloadJson["positionY"]                    = static_cast<int32_t>(currentPosition.positionY + 0.5F);
    payloadJson["angle"]                        = static_cast<int32_t>(currentPosition.angle + 0.5F);
    (void)serializeJson(payloadJson, payloadArray);
    String payloadStr(payloadArray);
    bool   wasPublishingSucessful = m_mqttClient.publish(TOPIC_NAME_POSITION, false, payloadStr);
    if (false == wasPublishingSucessful)
    {
        LOG_WARNING("Publishing Position via MQTT went wrong.");
    }
}

void App::processNewSensorData(const SensorData& newData)
{
    m_sensorFusion.estimateNewState(newData);
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * Receives sensor data for sensor fusion over SerialMuxProt channel in the order defined in SerialMuxChannels.
 * @param[in] payload      Sensor data
 * @param[in] payloadSize  Size of 8 sensor data
 * @param[in] userData     Pointer to the SensorFusion App.
 */
void App_sensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    if ((nullptr != payload) && (SENSORDATA_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        App* application = reinterpret_cast<App*>(userData);

        const SensorData* newSensorData = reinterpret_cast<const SensorData*>(payload);
        application->processNewSensorData(*newSensorData);
        application->publishSensorFusionPosition();
    }
    else
    {
        LOG_WARNING("SENSOR_DATA:: Invalid payload size. Expected: %u Received: %u", SENSORDATA_CHANNEL_DLC,
                    payloadSize);
    }
}