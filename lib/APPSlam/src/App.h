/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
 * @file
 * @brief  SLAM application
 * @author Jonas Hochhaus<jonas.hochhaus01@gmail.com>
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
#include "SerialMuxChannels.h"
#include <SimpleTimer.hpp>
#include <WiFiClient.h>


/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The SLAM application. */
class App
{
public:
    /**
     * Construct the SLAM application.
     */
    App() :
        m_smpServer(Board::getInstance().getRobot().getStream()),
        m_serialMuxProtChannelIdRemoteCtrl(0U),
        m_serialMuxProtChannelIdMotorSpeeds(0U),
        m_serialMuxProtChannelIdRobotSpeeds(0U),
        m_serialMuxProtChannelIdStatus(0U),
        m_initialDataSent(false),
        m_tcpRxBuffer(),
        m_lastReconnectAttemptTimeMs(0U),
        m_statusTimer(),
        m_isFatalError(false)
    {
        m_smpServer.setUserData(this);
    }

    /**
     * Destroy the SLAM application.
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
     * Handle vehicle data received from SerialMuxProt and forward to ROS2 server.
     *
     * @param[in] vehicleData Pointer to vehicle data.
     */
    void handleVehicleData(const VehicleData* vehicleData);

private:
    /** SerialMuxProt Channel ID for sending remote control commands. */
    uint8_t m_serialMuxProtChannelIdRemoteCtrl;

    /** SerialMuxProt Channel ID for sending motor speeds. */
    uint8_t m_serialMuxProtChannelIdMotorSpeeds;

    /** SerialMuxProt Channel ID for sending robot speed setpoints (linear + angular). */
    uint8_t m_serialMuxProtChannelIdRobotSpeeds;

    /** SerialMuxProt Channel ID for sending system status. */
    uint8_t m_serialMuxProtChannelIdStatus;

    /** TCP client for communication with the ROS2 server. */
    WiFiClient m_tcpClient;

    /**
     * SerialMuxProt Server Instance
     *
     * @tparam tMaxChannels set to MAX_CHANNELS, defined in SerialMuxChannels.h.
     */
    SMPServer m_smpServer;

    /**
     * Flag for setting initial data through SMP.
     */
    bool m_initialDataSent;

    /** TCP receive buffer. */
    String m_tcpRxBuffer;

    /** Timestamp of the last TCP reconnect attempt [ms]. */
    uint32_t m_lastReconnectAttemptTimeMs;

    /**
     * Timer for sending system status to RU.
     */
    SimpleTimer m_statusTimer;

    /**
     * Is fatal error happened?
     */
    bool m_isFatalError;

private:
    /**
     * Handler of fatal errors in the Application.
     */
    void fatalErrorHandler();

    /**
     * Setup the SerialMuxProt Server.
     *
     * @returns true if successful, otherwise false.
     */
    bool setupSerialMuxProtServer();

    /**
     * Connect to the ROS2 server via TCP.
     *
     * @returns true if successful, otherwise false.
     */
    bool connectToROS2Server();

    /**
     * Send a newline-terminated TCP packet to the ROS2 server.
     *
     * @param[in] payload Payload to send.
     */
    void sendPacket(const String& payload);

    /**
     * Receive and buffer TCP packets from the ROS2 server.
     * Processes complete newline-terminated lines.
     */
    void receivePackets();

    /**
     * Parse and process one incoming JSON line from the ROS2 server.
     * Expects a JSON object with "linear" (mm/s) and "angular" (mrad/s) velocity fields.
     * Logs a warning if parsing fails or the expected fields are missing.
     *
     * @param[in] line JSON string to parse and process.
     */
    void processIncomingLine(const String& line);

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
