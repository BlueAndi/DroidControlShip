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
#include <Board.h>
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <SettingsHandler.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Util.h>
#include <CoordinateHandler.h>

#include <Participants.h>
#include <Queuer.h>

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

static void App_odometryChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData);

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

/** Sending coordinate Id only once in trigger area. */
bool gIsListening = false;

/** Save current deserialized value of COLOR ID. */
Color clr;

/** Making decisions only on change. */
Color oldColorId;

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
                /* Subscribe to Traffic Light Colors Topic. */
                else if (false == m_mqttClient.subscribe(TOPIC_NAME_TRAFFIC_LIGHT_COLORS, true,
                                                         [this](const String& payload)
                                                         { trafficLightColorsCallback(payload); }))
                {
                    LOG_FATAL("Could not subcribe to MQTT topic: %s.", TOPIC_NAME_TRAFFIC_LIGHT_COLORS);
                }
                else if (false == m_mqttClient.subscribe(TOPIC_NAME_SETTINGS, true,
                                                         [this](const String& payload) { settingsCallback(payload); }))
                {
                    LOG_FATAL("Could not subcribe to MQTT topic: %s.", TOPIC_NAME_TRAFFIC_LIGHT_COLORS);
                }
                else
                {
                    /** Setup SMP channels. */
                    m_smpServer.subscribeToChannel(ODOMETRY_CHANNEL_NAME, App_odometryChannelCallback);

                    /* Setup SerialMuxProt Channels */
                    m_serialMuxProtChannelIdTrafficLightColors =
                        m_smpServer.createChannel(TRAFFIC_LIGHT_COLORS_CHANNEL_NAME, TRAFFIC_LIGHT_COLORS_CHANNEL_DLC);

                    if (0U == m_serialMuxProtChannelIdTrafficLightColors)
                    {
                        LOG_FATAL("Could not create SerialMuxProt Channel: %s.", TRAFFIC_LIGHT_COLORS_CHANNEL_NAME);
                    }
                    else
                    {
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
    if (false == m_mqttClient.process())
    {
        LOG_DEBUG("Could not process MQTT services.");
    }

    if ((true == gIsListening) && (oldColorId.colorId != clr.colorId))
    {
        oldColorId.colorId = clr.colorId;

        sendCurrentColor();
    }
}

void App::odometryCallback(const OdometryData& odometry)
{
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> payloadJson;
    char                                      payloadArray[JSON_DOC_DEFAULT_SIZE];

    // payloadJson["x"] = odometry.xPos;
    // payloadJson["y"] = odometry.yPos;

    // (void)serializeJson(payloadJson, payloadArray);
    // String payloadStr(payloadArray);

    // if (true == m_mqttClient.publish("TL_0/coordinates", false, payloadStr))
    // {
    //     LOG_DEBUG("PUBLISHED ODOMETRY: x: %d y: %d ", odometry.xPos, odometry.yPos);
    // }

    LOG_DEBUG("RECEIVED ODOMETRY: x: %d y: %d ORIENTATION:  %d", odometry.xPos, odometry.yPos, odometry.orientation);

    CoordinateHandler::getInstance().setCurrentOrientation(odometry.orientation);
    CoordinateHandler::getInstance().setCurrentCoordinates(odometry.xPos, odometry.yPos);

    Queuer::getInstance().process();

    // if (true == CoordinateHandler::getInstance().isMovingTowards())
    // {
    //     if (true == CoordinateHandler::getInstance().checkOrientation())
    //     {
    //         LOG_DEBUG("Robot pointing towards IE.");

    //         if (CoordinateHandler::getInstance().getCurrentDistance() < 250)
    //         {
    //             LOG_DEBUG("Robot is near IE, listening for signals.");
    //             gIsListening = true;
    //         }
    //         else
    //         {
    //             gIsListening = false;
    //             LOG_DEBUG("Robot has some more driving to do.");
    //         }
    //     }
    //     else
    //     {
    //         gIsListening = false;
    //         LOG_DEBUG("Robot isn't pointing towards IE.");
    //     }
    // }
    // else
    // {
    //     gIsListening = false;
    // }
}

void App::sendCurrentColor()
{
    if (true ==
        m_smpServer.sendData(m_serialMuxProtChannelIdTrafficLightColors, reinterpret_cast<uint8_t*>(&clr), sizeof(clr)))
    {
        LOG_DEBUG("Color %d sent.", clr.colorId);
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
    /* Turn on Red LED to signal fatal error. */
    Board::getInstance().getRedLed().enable(true);

    while (true)
    {
        ;
    }
}

/**
 * Type: Rx MQTT & Tx SMP
 * Name: trafficLightColorsCallback
 */
void App::trafficLightColorsCallback(const String& payload)
{
    /** Save deserialized JSON value somewhere. */
    /** call TxSMP function later in loop() once gIsListening is true. */
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> jsonPayload;
    DeserializationError                      error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON Deserialization Error %d.", error);
    }
    else
    {
        JsonVariant color = jsonPayload["COLOR"];

        if (false == color.isNull())
        {
            clr.colorId = color.as<uint8_t>();
            LOG_DEBUG("COLOR_ID %d", clr.colorId);
        }
        else
        {
            LOG_WARNING("Received invalid color.");
        }
    }
}

/**
 * Type: Rx MQTT
 * Name: settingsCallback
 */
void App::settingsCallback(const String& payload)
{
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> jsonPayload;
    DeserializationError                      error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON deserialization error %d.", error);
    }
    else
    {
        JsonVariant robotName        = jsonPayload["FROM"];
        JsonVariant orientationValue = jsonPayload["TOWARDS"];
        JsonVariant xIntervalValue   = jsonPayload["IX"];
        JsonVariant yIntervalValue   = jsonPayload["IY"];
        JsonVariant xEntryValue      = jsonPayload["EX"];
        JsonVariant yEntryValue      = jsonPayload["EY"];

        if ((false == xIntervalValue.isNull()) && (false == yIntervalValue.isNull()) &&
            (false == xEntryValue.isNull()) && (false == yEntryValue.isNull()))
        {
            InfrastructureElement* trafficParticipant = new (std::nothrow) InfrastructureElement();

            if (nullptr != trafficParticipant)
            {
                trafficParticipant->name        = robotName.as<const char*>();
                trafficParticipant->orientation = orientationValue.as<int32_t>();
                trafficParticipant->intervX     = xIntervalValue.as<int32_t>();
                trafficParticipant->intervY     = yIntervalValue.as<int32_t>();
                trafficParticipant->entryX      = xEntryValue.as<int32_t>();
                trafficParticipant->entryY      = yEntryValue.as<int32_t>();

                if (true == Queuer::getInstance().enqueueParticipant(trafficParticipant))
                {
                    LOG_DEBUG("IE %s pointing towards %d with Trigger Area:", trafficParticipant->name,
                              trafficParticipant->orientation);
                    LOG_DEBUG("INTERVAL VALUES x:%d y:%d", trafficParticipant->intervX, trafficParticipant->intervY);
                    LOG_DEBUG("ENTRY VALUES    x:%d y:%d", trafficParticipant->entryX, trafficParticipant->entryY);
                }
            }

            Participant::getInstance().setEntryValues(xEntryValue.as<int32_t>(), yEntryValue.as<int32_t>());
            Participant::getInstance().setIntervalValues(xIntervalValue.as<int32_t>(), yIntervalValue.as<int32_t>());
            Participant::getInstance().setRequiredOrientation(orientationValue.as<int32_t>());
        }
        else
        {
            LOG_WARNING("Received invalid coordinate set.");
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
 * Type: Rx SMP
 * Name: odometryChannelCallback
 *
 * Receives current position and heading of the robot over SerialMuxProt channel.
 *
 * @param[in] payload       Odometry data. Two coordinates and one orientation.
 * @param[in] payloadSize   Size of two coordinates and one orientation.
 * @param[in] userData      Instance of App class.
 */
void App_odometryChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    (void)userData;
    if ((nullptr != payload) && (ODOMETRY_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        const OdometryData* odometryData = reinterpret_cast<const OdometryData*>(payload);
        App*                application  = reinterpret_cast<App*>(userData);

        application->odometryCallback(*odometryData);
    }
    else
    {
        LOG_WARNING("ODOMETRY: Invalid payload size. Expected: %u Received: %u", ODOMETRY_CHANNEL_DLC, payloadSize);
    }
}