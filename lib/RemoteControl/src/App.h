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
        m_smpServer(Board::getInstance().getDevice().getStream()),
        m_serialMuxProtChannelIdRemoteCtrl(0U),
        m_serialMuxProtChannelIdMotorSpeeds(0U),
        m_mqttClient()
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

private:
    /** MQTT topic name for birth messages. */
    static const char* TOPIC_NAME_BIRTH;

    /** MQTT topic name for will messages. */
    static const char* TOPIC_NAME_WILL;

    /** MQTT topic name for receiving commands. */
    static const char* TOPIC_NAME_CMD;

    /** MQTT topic name for receiving motor speeds. */
    static const char* TOPIC_NAME_MOTOR_SPEEDS;

    /** SerialMuxProt Channel id sending remote control commands. */
    uint8_t m_serialMuxProtChannelIdRemoteCtrl;

    /** SerialMuxProt Channel id sending sending motor speeds. */
    uint8_t m_serialMuxProtChannelIdMotorSpeeds;

    /**
     * SerialMuxProt Server Instance
     *
     * @tparam tMaxChannels set to 10, as App does not require
     * more channels for external communication.
     */
    SerialMuxProtServer<10U> m_smpServer;

    /**
     * MQTTClient Instance
     */
    MqttClient m_mqttClient;

private:
    /**
     * Handler of fatal errors in the Application.
     */
    void fatalErrorHandler();

    /**
     * Callback for Command MQTT Topic.
     * @param[in] payload   Payload of the MQTT message.
     */
    void cmdTopicCallback(const String& payload);

    /**
     * Callback for Motor Speeds MQTT Topic.
     * @param[in] payload   Payload of the MQTT message.
     */
    void motorSpeedsTopicCallback(const String& payload);

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
