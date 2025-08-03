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
 * @brief  Turtle Application.
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
#include <SimpleTimer.hpp>
#include "MicroRosClient.h"
#include "SerialMuxChannels.h"

#include <geometry_msgs/msg/twist.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The Turtle application. */
class App
{
public:
    /**
     * Construct the Turtle application.
     */
    App() :
        m_ros(),
        m_serialMuxProtChannelIdStatus(0U),
        m_serialMuxProtChannelIdTurtle(0U),
        m_smpServer(Board::getInstance().getRobot().getStream()),
        m_statusTimer(),
        m_turtleMovementTimer(),
        m_turtleSpeedSetpoint(),
        m_isNewTurtleSpeedSetpoint(true),
        m_isFatalError(false)
    {
        m_smpServer.setUserData(this);
    }

    /**
     * Destroy the Turtle application.
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
    /**
     * Interval for sending system status to RU.
     */
    static const uint32_t STATUS_TIMER_INTERVAL = 1000U;

    /**
     * Interval of the duration of a turtle step.
     */
    static const uint32_t TURTLE_STEP_TIMER_INTERVAL = 1000U;

    /**
     * Instance of the MicroRosClient.
     */
    MicroRosClient m_ros;

    /**
     * ROS topic name for the velocity commands.
     */
    static const char* TOPIC_NAME_CMD_VEL;

    /**
     * SerialMuxProt Channel id for sending system status.
     */
    uint8_t m_serialMuxProtChannelIdStatus;

    /**
     * SerialMuxProt Channel id for sending Turtle speeds.
     */
    uint8_t m_serialMuxProtChannelIdTurtle;

    /**
     * SerialMuxProt Server Instance
     */
    SMPServer m_smpServer;

    /**
     * Timer for sending system status to RU.
     */
    SimpleTimer m_statusTimer;

    /**
     * Timer for the movement of the turtle.
     */
    SimpleTimer m_turtleMovementTimer;

    /**
     * Turtle speed setpoint.
     */
    geometry_msgs__msg__Twist m_turtleSpeedSetpoint;

    /**
     * Flag: New turtle speed setpoint available.
     */
    bool m_isNewTurtleSpeedSetpoint;

    /**
     * Is fatal error happened?
     */
    bool m_isFatalError;

    /**
     * Handler of fatal errors in the Application.
     */
    void fatalErrorHandler();

    /**
     * Setup the Micro ROS Client.
     *
     * @returns true if successful, otherwise false.
     */
    bool setupMicroRosClient();

    /**
     * Setup the SerialMuxProt Server.
     *
     * @returns true if successful, otherwise false.
     */
    bool setupSerialMuxProtServer();

    /**
     * Handle the Turtle behavior.
     */
    void handleTurtle();

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
