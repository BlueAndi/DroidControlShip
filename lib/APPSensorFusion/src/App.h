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
 * @brief  SensorFusion application
 * @author Juliane Kerpe <juliane.kerpe@web.de>
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
#include "SensorFusion.h"
#include "SerialMuxChannels.h"
#include <MqttClient.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The Sensor Fusion application. */
class App
{
public:
    /**
     * Construct the Sensor Fusion application.
     */
    App() :
        m_sensorFusion(),
        m_smpServer(Board::getInstance().getRobot().getStream()),
        m_mqttClient(),
        m_isFatalError(false)
    {
        m_smpServer.setUserData(this);
    }

    /**
     * Destroy the Sensor Fusion application.
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
     * Publish Position calculated by Sensor Fusion via MQTT.
     */
    void publishSensorFusionPosition();

    /**
     * Process the Receiving of New Sensor Data via SerialMuxProt
     *
     * @param[in] newData New Sensor Data.
     */
    void processNewSensorData(const SensorData& newData);

private:
    /** Minimum battery level in percent. */
    static const uint8_t MIN_BATTERY_LEVEL = 10U;

    SensorFusion m_sensorFusion; /**< Instance of the SensorFusion algorithm. */

    /** MQTT topic name for birth messages. */
    static const char* TOPIC_NAME_BIRTH;

    /** MQTT topic name for will messages. */
    static const char* TOPIC_NAME_WILL;

    /** MQTT topic name for sending Position Data. */
    static const char* TOPIC_NAME_POSITION;

    /**
     * MQTTClient Instance
     */
    MqttClient m_mqttClient;

    /**
     * SerialMuxProt Server Instance
     *
     * @tparam tMaxChannels set to MAX_CHANNELS, defined in SerialMuxChannels.h.
     */
    SMPServer m_smpServer;

    /**
     * Is fatal error happened?
     */
    bool m_isFatalError;

    /**
     * Handler of fatal errors in the Application.
     */
    void fatalErrorHandler();

private:
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
