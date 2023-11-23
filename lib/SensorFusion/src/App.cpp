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
static void App_endlineChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData);

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
    SettingsHandler& settings = SettingsHandler::getInstance();
    Board&           board    = Board::getInstance();

    m_sensorFusion.init();

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
            m_smpServer.subscribeToChannel(SENSORDATA_CHANNEL_NAME, App_sensorChannelCallback);
            m_smpServer.subscribeToChannel(ENDLINE_CHANNEL_NAME, App_endlineChannelCallback);

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
            }
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

void App::publishSensorFusionPosition()
{
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> payloadJson;
    char                                      payloadArray[JSON_DOC_DEFAULT_SIZE];

    /* Write newest Sensor Fusion Data in JSON String */
    IKalmanFilter::PositionData currentPosition = m_sensorFusion.getLatestPosition();
    payloadJson["positionX"]                    = currentPosition.currentXPos;
    payloadJson["positionY"]                    = currentPosition.currentYPos;
    payloadJson["angle"]                        = currentPosition.currentAngle;
    (void)serializeJson(payloadJson, payloadArray);
    String payloadStr(payloadArray);
    m_mqttClient.publish(TOPIC_NAME_POSITION, false, payloadStr);
}

void App::processNewSensorData(const SensorData newData)
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
 * @param[in] payload         Sensor data
 * @param[in] payloadSize     Size of 8 sensor data
 * @param[in] userData      User data provided by the application.
 */
void App_sensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    if ((nullptr != payload) && (SENSORDATA_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        App* application = reinterpret_cast<App*>(userData);

        const SensorData* newSensorData = reinterpret_cast<const SensorData*>(payload);
        application->processNewSensorData(*newSensorData);
    }
    else
    {
        LOG_WARNING("SENSOR_DATA:: Invalid payload size. Expected: %u Received: %u", SENSORDATA_CHANNEL_DLC,
                    payloadSize);
    }
}

/**
 * Receives an End Line detection Flag over SerialMuxProt channel.
 * @param[in] payload         End Line detection flag
 * @param[in] payloadSize     Size of the data
 * @param[in] userData      User data provided by the application.
 */
void App_endlineChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    if ((nullptr != payload) && (ENDLINE_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        App* application = reinterpret_cast<App*>(userData);
        application->publishSensorFusionPosition();

        LOG_DEBUG("END_LINE: New End Line Data!");
        bool endLineHasBeenDetected = true;
    }
    else
    {
        LOG_WARNING("END_LINE: Invalid payload size. Expected: %u Received: %u", ENDLINE_CHANNEL_DLC, payloadSize);
    }
}