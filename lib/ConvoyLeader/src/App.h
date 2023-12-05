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
#include <PlatoonController.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The convoy leader application. */
class App
{
public:
    /**
     * Construct the convoy leader application.
     */
    App() :
        m_smpServer(Board::getInstance().getDevice().getStream(), this),
        m_serialMuxProtChannelIdMotorSpeedSetpoints(0U),
        m_mqttClient(),
        m_platoonController()
    {
    }

    /**
     * Destroy the convoy leader application.
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
     * Callback for the current vehicle data.
     *
     * @param[in] vehicleData Current vehicle data.
     */
    void currentVehicleChannelCallback(const VehicleData& vehicleData);

    /**
     * Input waypoint callback.
     * Called in order to get the next waypoint into the platoon controller.
     *
     * @param[out] waypoint   Next waypoint.
     *
     * @return If a waypoint is available, it returns true. Otherwise, false.
     */
    bool inputWaypointCallback(Waypoint& waypoint);

    /**
     * Output waypoint callback.
     * Called in order to send the last waypoint to the next platoon participant.
     *
     * @param[in] waypoint    Last waypoint.
     *
     * @return If the waypoint was sent successfully, returns true. Otherwise, false.
     */
    bool outputWaypointCallback(const Waypoint& waypoint);

    /**
     * Motor setpoint callback.
     * Called in order to send the motor speeds using SerialMuxProt to the robot.
     *
     * @param[in] left      Left motor speed [steps/s].
     * @param[in] right     Right motor speed [steps/s].
     *
     * @return If the motor speeds were sent successfully, returns true. Otherwise, false.
     */
    bool motorSetpointCallback(const int16_t left, const int16_t right);

private:
    /** Minimum battery level in percent. */
    static const uint8_t MIN_BATTERY_LEVEL = 10U;

    /** MQTT topic name for birth messages. */
    static const char* TOPIC_NAME_BIRTH;

    /** MQTT topic name for will messages. */
    static const char* TOPIC_NAME_WILL;

    /** MQTT topic name for receiving position setpoint coordinates. */
    static const char* TOPIC_NAME_POSITION_SETPOINT;

    /** SerialMuxProt Channel id sending sending motor speed setpoints. */
    uint8_t m_serialMuxProtChannelIdMotorSpeedSetpoints;

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

    /**
     * Platoon controller instance.
     */
    PlatoonController m_platoonController;

private:
    /**
     * Callback for Position Setpoint MQTT Topic.
     * @param[in] payload   Payload of the MQTT message.
     */
    void positionTopicCallback(const String& payload);

    /**
     * Handler of fatal errors in the Application.
     */
    void fatalErrorHandler();

    /**
     * Send speed setpoints using SerialMuxProt.
     */
    void sendSpeedSetpoints();

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
