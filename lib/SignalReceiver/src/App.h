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
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef APP_H
#define APP_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <Board.h>
#include <MqttClient.h>
#include <SerialMuxProtServer.hpp>
#include "SerialMuxChannels.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The Remote Control application. */
class App
{
public:
    /**
     * Construct the Remote Control application.
     */
    App() :
        m_smpServer(Board::getInstance().getDevice().getStream(), this),
        m_serialMuxProtChannelIdCoordinates(0U),
        m_serialMuxProtChannelIdTrafficLightColors(0U),
        m_mqttClient(),
        m_sendPackageTimer(),
        clr(),
        oldColorId()
    {
    }

    /**
     * Destroy the Remote Control application.
     */
    ~App()
    {
    }

    /**
     * Setup the application.
     */
    void setup();

    /**
     * Process the application periodically.
     */
    void loop();

    /**
     * Odometry Callback
     *
     * @param[in] odometry  Odometry data.
     */
    void odometryCallback(const OdometryData& odometry);

    /**
     * Sends colorId through SMP.
     */
    void sendCurrentColor();

private:
    /** Minimum battery level in percent. */
    static const uint8_t MIN_BATTERY_LEVEL = 10U;

    /** MQTT topic name for birth messages. */
    static const char* TOPIC_NAME_BIRTH;

    /** MQTT topic name for will messages. */
    static const char* TOPIC_NAME_WILL;

    /** MQTT topic name for receiving traffic light color IDs. */
    static const char* TOPIC_NAME_TRAFFIC_LIGHT_COLORS;

    /** MQTT topic name for receiving settings. */
    static const char* TOPIC_NAME_SETTINGS;

    /** SerialMuxProt Channel id receiving coordinates. */
    uint8_t m_serialMuxProtChannelIdCoordinates;

    /** SerialMuxProt Channel id sending current traffic light color ID. */
    uint8_t m_serialMuxProtChannelIdTrafficLightColors;

    /**
     * SerialMuxProt Server Instance
     *
     * @tparam tMaxChannels set to MAX_CHANNELS, defined in SerialMuxChannels.h.
     */
    SerialMuxProtServer<MAX_CHANNELS> m_smpServer;

    /**
     * MQTTClient Instance
     */
    MqttClient m_mqttClient;

private:
    /** Timer for sending coordinate sets. */
    SimpleTimer m_sendPackageTimer;

    /** Sending coordinate Id only once in trigger area. */
    bool gIsListening = false;

    /** Used for unique subscription. */
    bool gIsSubscribed = false;

    /** Save current deserialized value of COLOR ID. */
    Color clr;

    /** Sending color only on change. */
    Color oldColorId;

    String lockedOnto;

    /**
     * Handler of fatal errors in the Application.
     */
    void fatalErrorHandler();

    /**
     * Callback for Traffic Light Colors MQTT Topic.
     * @param[in] payload   Payload of the MQTT message.
     */
    void trafficLightColorsCallback(const String& payload);

    /**
     * Settings Callback
     *
     * @param[in] payload Payload of the MQTT message.
     */
    void settingsCallback(const String& paylaod);

private:
    /* Not allowed. */
    App(const App& app);            /**< Copy construction of an instance. */
    App& operator=(const App& app); /**< Assignment of an instance. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* APP_H */
/** @} */
