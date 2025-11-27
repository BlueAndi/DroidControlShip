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
 * @brief  Line follower Sensor fusion application
 * @author Tobias Haeckel <tobias.haeckel@gmx.net>
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
#include <SerialMuxProtServer.hpp>
#include <SimpleTimer.hpp>
#include <StateMachine.h>
#include "SerMuxChannelProvider.h"
#include "LineSensors.h"
#include "Motors.h"
#include <MqttClient.h>
#include "TimeSync.h"

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
    App();

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

private:
    /** MQTT topic name for birth messages. */
    static const char* TOPIC_NAME_BIRTH;

    /** MQTT topic name for will messages. */
    static const char* TOPIC_NAME_WILL;

    /** MQTT topic name for status messages. */
    static const char* TOPIC_NAME_STATUS;

    /** MQTT topic name for fusion Pose */
    static const char* TOPIC_NAME_FUSION_POSE;

    /** MQTT topic name for raw Sensor data */
    static const char* TOPIC_NAME_RAW_SENSORS;

    /** MQTT topic name for receiving Space Ship Radar Pose. */
    static const char* TOPIC_NAME_RADAR_POSE;

    /**
     * Flag for setting initial data through SMP.
     */
    bool m_initialDataSent;

    /**
     * Timer for sending system status to RU.
     */
    SimpleTimer m_statusTimer;

    /**
     * SerialMux Channel Provider handler.
     */
    SerMuxChannelProvider m_serMuxChannelProvider;

    /**
     * Time synchronization handler.
     */
    TimeSync m_timeSync;

    /**
     * Line sensors handler.
     */
    LineSensors m_lineSensors;

    /**
     * Motors handler.
     */
    Motors m_motors;

    /**
     * State machine for the application.
     */
    StateMachine m_stateMachine;

    /**
     * MQTT client handler.
     */
    MqttClient m_mqttClient;

    /**
     * Setup the MQTT connection.
     *
     * @param[in] clientId      The MQTT client id.
     * @param[in] brokerAddr    The address of the MQTT broker.
     * @param[in] brokerPort    The port of the MQTT broker.
     *
     * @return true if successful, otherwise false.
     */
    bool setupMqtt(const String& clientId, const String& brokerAddr, uint16_t brokerPort);

    /**
     * Callback for receiving Space Ship Radar Pose over MQTT topic.
     *
     * @param[in] payload   The topic payload.
     */
    void ssrTopicCallback(const String& payload);

    /**
     * Publish a combined snapshot of vehicle and line sensor data via MQTT.
     *
     * @param[in] data  Vehicle data received via SerialMux.
     */
    void publishVehicleAndSensorSnapshot(const VehicleData& data);

    /**
     * Copy construction of an instance.
     * Not allowed.
     *
     * @param[in] app Source instance.
     */
    App(const App& app);

    /**
     * Assignment of an instance.
     * Not allowed.
     *
     * @param[in] app Source instance.
     *
     * @returns Reference to App instance.
     */
    App& operator=(const App& app);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* APP_H */
/** @} */
