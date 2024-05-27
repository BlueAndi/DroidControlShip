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
#include <V2VCommManager.h>
#include "SerialMuxChannels.h"

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
        m_serialMuxProtChannelIdStatus(0U),
        m_smpServer(Board::getInstance().getRobot().getStream(), this),
        m_mqttClient(),
        m_v2vCommManager(m_mqttClient),
        m_systemStateMachine(),
        m_latestVehicleData(),
        m_sendWaypointTimer(),
        m_commandTimer(),
        m_motorSpeedTimer(),
        m_statusTimer(),
        m_statusTimeoutTimer(),
        m_lastV2VStatus(V2VCommManager::V2VStatus::V2V_STATUS_NOT_INIT),
        m_lastRUStatus(SMPChannelPayload::Status::STATUS_FLAG_OK),
        m_lastWaypointSent()
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
     * Set latest vehicle data from RU.
     *
     * @param[in] waypoint  Latest vehicle data from RU.
     */
    void setLatestVehicleData(const Waypoint& waypoint);

    /**
     * Set error state.
     */
    void setErrorState();

    /**
     * System Status callback.
     *
     * @param[in] status    System status
     */
    void systemStatusCallback(SMPChannelPayload::Status status);

private:
    /** Minimum battery level in percent. */
    static const uint8_t MIN_BATTERY_LEVEL = 10U;

    /** Send waypoint timer interval in ms. */
    static const uint32_t SEND_WAYPOINT_TIMER_INTERVAL = 50U;

    /** Send commands timer interval in ms. */
    static const uint32_t SEND_COMMANDS_TIMER_INTERVAL = 100U;

    /** Send motor speed timer interval in ms. */
    static const uint32_t SEND_MOTOR_SPEED_TIMER_INTERVAL = 100U;

    /** Send status timer interval in ms. */
    static const uint32_t SEND_STATUS_TIMER_INTERVAL = 1000U;

    /** Status timeout timer interval in ms. */
    static const uint32_t STATUS_TIMEOUT_TIMER_INTERVAL = 2U * SEND_STATUS_TIMER_INTERVAL;

    /** Distance interval between waypoints in mm. */
    static const int32_t WAYPOINT_DISTANCE_INTERVAL = 100;

    /** MQTT topic name for birth messages. */
    static const char* TOPIC_NAME_BIRTH;

    /** MQTT topic name for will messages. */
    static const char* TOPIC_NAME_WILL;

    /** SerialMuxProt Channel id for sending remote control commands. */
    uint8_t m_serialMuxProtChannelIdRemoteCtrl;

    /** SerialMuxProt Channel id for sending motor speeds. */
    uint8_t m_serialMuxProtChannelIdMotorSpeeds;

    /** SerialMuxProt Channel id for sending system status. */
    uint8_t m_serialMuxProtChannelIdStatus;

    /**
     * SerialMuxProt Server Instance
     */
    SMPServer m_smpServer;

    /**
     * MQTTClient Instance
     */
    MqttClient m_mqttClient;

    /**
     * V2V communication manager instance.
     */
    V2VCommManager m_v2vCommManager;

    /**
     * The system state machine.
     */
    StateMachine m_systemStateMachine;

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

    /**
     * Timer for sending motor speed to RU.
     */
    SimpleTimer m_motorSpeedTimer;

    /**
     * Timer for sending system status to RU.
     */
    SimpleTimer m_statusTimer;

    /**
     * Timer for timeout of system status of RU.
     */
    SimpleTimer m_statusTimeoutTimer;

    /** Last V2V Communication Manager status. */
    V2VCommManager::V2VStatus m_lastV2VStatus;

    /** Last system status of RU. */
    SMPChannelPayload::Status m_lastRUStatus;

    /** Last Waypoint sent. */
    Waypoint m_lastWaypointSent;

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

    /**
     * Process V2V communication.
     */
    void processV2VCommunication();

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
