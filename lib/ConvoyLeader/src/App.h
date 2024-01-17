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
#include <SerialMuxProtServer.hpp>
#include "SerialMuxChannels.h"
#include <V2VClient.h>

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
        m_serialMuxProtChannelInitialVehicleData(0U),
        m_smpServer(Board::getInstance().getDevice().getStream(), this),
        m_mqttClient(),
        m_v2vClient(m_mqttClient),
        m_initialDataSent(false)
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

private:
    /** Minimum battery level in percent. */
    static const uint8_t MIN_BATTERY_LEVEL = 10U;

    /** MQTT topic name for birth messages. */
    static const char* TOPIC_NAME_BIRTH;

    /** MQTT topic name for will messages. */
    static const char* TOPIC_NAME_WILL;

    /** SerialMuxProt Channel id for sending remote control commands. */
    uint8_t m_serialMuxProtChannelIdRemoteCtrl;

    /** SerialMuxProt Channel id for sending motor speeds. */
    uint8_t m_serialMuxProtChannelIdMotorSpeeds;

    /** SerialMuxProt Channel id for sending initial position data. */
    uint8_t m_serialMuxProtChannelInitialVehicleData;

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
     * V2V client instance.
     */
    V2VClient m_v2vClient;

    /**
     * Flag for setting initial data through SMP.
     */
    bool m_initialDataSent;

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
