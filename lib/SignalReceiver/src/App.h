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
#include "SerialMuxChannels.h"
#include <StateMachine.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The signal receiver application. */
class App
{
public:
    /**
     * Construct the signal receiver application.
     */
    App() :
        m_serialMuxProtChannelIdRemoteCtrl(0U),
        m_serialMuxProtChannelIdMotorSpeeds(0U),
        m_serialMuxProtChannelIdStatus(0U),
        m_smpServer(Board::getInstance().getDevice().getStream(), this),
        m_serialMuxProtChannelIdTrafficLightColors(0U),
        m_mqttClient(),
        m_systemStateMachine(),
        m_commandTimer(),
        m_motorSpeedTimer(),
        m_processTrafficTimer(),
        m_statusTimer(),
        m_statusTimeoutTimer(),
        m_isListening(false),
        m_isSubscribed(false),
        clr(),
        oldColorId(),
        lockedOnto()
    {
    }

    /**
     * Destroy the signal receiver application.
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
     * Set error state.
     */
    void setErrorState();

    /**
     * System Status callback.
     *
     * @param[in] status    System status
     */
    void systemStatusCallback(SMPChannelPayload::Status status);

    /**
     * Odometry Callback
     *
     * @param[in] odometry  Odometry data.
     */
    void odometryCallback(const VehicleData& odometry);

private:
    /** Minimum battery level in percent. */
    static const uint8_t MIN_BATTERY_LEVEL = 10U;

    /** Send commands timer interval in ms. */
    static const uint32_t SEND_COMMANDS_TIMER_INTERVAL = 100U;

    /** Send motor speed timer interval in ms. */
    static const uint32_t SEND_MOTOR_SPEED_TIMER_INTERVAL = 100U;

    /** Send status timer interval in ms. */
    static const uint32_t SEND_STATUS_TIMER_INTERVAL = 1000U;

    /** Process traffic timer interval in ms. */
    static const uint32_t PROCESS_TRAFFIC_TIMER_INTERVAL = 96U;

    /** Status timeout timer interval in ms. */
    static const uint32_t STATUS_TIMEOUT_TIMER_INTERVAL = 4U * SEND_STATUS_TIMER_INTERVAL;

    /** MQTT topic name for birth messages. */
    static const char* TOPIC_NAME_BIRTH;

    /** MQTT topic name for will messages. */
    static const char* TOPIC_NAME_WILL;

    /** MQTT topic name for receiving traffic light color IDs. */
    static const char* TOPIC_NAME_TRAFFIC_LIGHT_COLORS;

    /** MQTT topic name for receiving settings. */
    static const char* TOPIC_NAME_SETTINGS;

    /** SerialMuxProt Channel id for sending remote control commands. */
    uint8_t m_serialMuxProtChannelIdRemoteCtrl;

    /** SerialMuxProt Channel id for sending motor speeds. */
    uint8_t m_serialMuxProtChannelIdMotorSpeeds;

    /** SerialMuxProt Channel id for sending system status. */
    uint8_t m_serialMuxProtChannelIdStatus;

    /** SerialMuxProt Channel id sending current traffic light color ID. */
    uint8_t m_serialMuxProtChannelIdTrafficLightColors;

    /**
     * SerialMuxProt Server Instance
     */
    SMPServer m_smpServer;

    /**
     * MQTTClient Instance
     */
    MqttClient m_mqttClient;

    /**
     * The system state machine.
     */
    StateMachine m_systemStateMachine;

    /**
     * Timer for sending initial commands.
     */
    SimpleTimer m_commandTimer;

    /**
     * Timer for sending motor speed to RU.
     */
    SimpleTimer m_motorSpeedTimer;

    /**
     * Time for processing trffic.
     */
    SimpleTimer m_processTrafficTimer;

    /**
     * Timer for sending system status to RU.
     */
    SimpleTimer m_statusTimer;

    /**
     * Timer for timeout of system status of RU.
     */
    SimpleTimer m_statusTimeoutTimer;

    /** Sending color Id only when near IE. */
    bool m_isListening = false;

    /** Used for unique subscription. */
    bool m_isSubscribed = false;

    /** Save current deserialized value of COLOR ID. */
    Color clr;

    /** Sending color only on change. */
    Color oldColorId;

    /** The topic name of the IE the robot is locked onto.*/
    String lockedOnto;

    /**
     * Handler of fatal errors in the Application.
     */
    void fatalErrorHandler();

    /**
     * Process periodic tasks.
     */
    void processPeriodicTasks();

    /**
     * Process traffic. This function will trigger
     * subcriptions and unsubscriptions to IE topics based on
     * distance between the robot and IEs.
     */
    void processTraffic();

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
    void settingsCallback(const String& payload);

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
