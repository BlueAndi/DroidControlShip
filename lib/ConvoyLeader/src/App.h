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
#include <SimpleTimer.hpp>
#include <StateMachine.h>
#include <V2VClient.h>
#include "SerialMuxChannels.h"
#include "LongitudinalController.h"

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
        m_serialMuxProtChannelIdRemoteCtrl(0U),
        m_serialMuxProtChannelIdMotorSpeeds(0U),
        m_smpServer(Board::getInstance().getDevice().getStream(), this),
        m_mqttClient(),
        m_v2vClient(m_mqttClient),
        m_longitudinalController(),
        m_sendWaypointTimer()
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
     * Motor setpoint callback.
     * Called in order to send the motor speeds using SerialMuxProt to the robot.
     *
     * @param[in] topCenterSpeed  Center motor speed [steps/s].
     *
     * @return If the motor speed was sent successfully, returns true. Otherwise, false.
     */
    bool motorSetpointCallback(const int16_t topCenterSpeed);

    /**
     * Release robot and start driving.
     */
    void release();

    /**
     * Set max motor speed.
     *
     * @param[in] maxMotorSpeed    Max motor speed [steps/s].
     */
    void setMaxMotorSpeed(const int16_t maxMotorSpeed);

    /**
     * Set incoming feedback from last follower.
     *
     * @param[in] feedback Feedback from last follower.
     */
    void setLastFollowerFeedback(const Waypoint& feedback);

private:
    /** Minimum battery level in percent. */
    static const uint8_t MIN_BATTERY_LEVEL = 10U;

    /** Send waypoint timer interval in ms. */
    static const uint32_t SEND_WAYPOINT_TIMER_INTERVAL = 500U;

    /** Send commands timer interval in ms. */
    static const uint32_t SEND_COMMANDS_TIMER_INTERVAL = 100U;

    /** MQTT topic name for birth messages. */
    static const char* TOPIC_NAME_BIRTH;

    /** MQTT topic name for will messages. */
    static const char* TOPIC_NAME_WILL;

    /** MQTT topic name for release messages. */
    static const char* TOPIC_NAME_RELEASE;

    /** SerialMuxProt Channel id for sending remote control commands. */
    uint8_t m_serialMuxProtChannelIdRemoteCtrl;

    /** SerialMuxProt Channel id for sending motor speeds. */
    uint8_t m_serialMuxProtChannelIdMotorSpeeds;

    /**
     * SerialMuxProt Server Instance
     *
     * @tparam tMaxChannels set to MAX_CHANNELS, defined in SerialMuxChannels.h.
     */
    SMPServer m_smpServer;

    /**
     * MQTTClient Instance
     */
    MqttClient m_mqttClient;

    /**
     * V2V client instance.
     */
    V2VClient m_v2vClient;

    /** The system state machine. */
    StateMachine m_systemStateMachine;

    /**
     * Longitudinal controller.
     */
    LongitudinalController m_longitudinalController;

    /**
     * Latest vehicle data from RU.
     */
    Waypoint m_latestVehicleData;

    /**
     * Send waypoint timer.
     */
    SimpleTimer m_sendWaypointTimer;

    /**
     * Timer for sending initial commands.
     */
    SimpleTimer m_commandTimer;

private:
    /**
     * Handler of fatal errors in the Application.
     */
    void fatalErrorHandler();

    /**
     * Setup the MQTT client.
     *
     * @return If successful returns true, otherwise false.
     */
    bool setupMqttClient();

    /**
     * Setup the SerialMuxProt channels.
     *
     * @return If successful returns true, otherwise false.
     */
    bool setupSerialMuxProt();

    /**
     * Process periodic tasks.
     */
    void processPeriodicTasks();

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
