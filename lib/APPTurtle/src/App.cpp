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
 * @brief  Turtle application.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "App.h"
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <SettingsHandler.h>
#include <WiFi.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

#ifndef CONFIG_LOG_SEVERITY
#define CONFIG_LOG_SEVERITY (Logging::LOG_LEVEL_INFO)
#endif /* CONFIG_LOG_SEVERITY */

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Serial interface baudrate. */
static const uint32_t SERIAL_BAUDRATE = 115200U;

/** Serial log sink */
static LogSinkPrinter gLogSinkSerial("Serial", &Serial);

/* ROS topic name for the velocity commands. */
const char* App::TOPIC_NAME_CMD_VEL = "cmd_vel";

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void App::setup()
{
    bool             isSuccessful = false;
    SettingsHandler& settings     = SettingsHandler::getInstance();
    Board&           board        = Board::getInstance();

    Serial.begin(SERIAL_BAUDRATE);

    /* Register serial log sink and select it per default. */
    if (true == Logging::getInstance().registerSink(&gLogSinkSerial))
    {
        (void)Logging::getInstance().selectSink("Serial");

        /* Set severity of logging system. */
        Logging::getInstance().setLogLevel(CONFIG_LOG_SEVERITY);
    }

    /* Initialize HAL. */
    if (false == board.init())
    {
        LOG_FATAL("HAL init failed.");
    }
    /* Settings shall be loaded from configuration file. */
    else if (false == settings.loadConfigurationFile(board.getConfigFilePath()))
    {
        LOG_FATAL("Settings could not be loaded from %s.", board.getConfigFilePath());
    }
    else
    {
        /* If the robot name is empty, use the wifi MAC address as robot name. */
        if (true == settings.getRobotName().isEmpty())
        {
            String robotName("MAC");
            robotName += WiFi.macAddress();

            /* Remove MAC separators from robot name. */
            robotName.replace(":", "");

            settings.setRobotName(robotName);
        }

        NetworkSettings networkSettings = {settings.getWiFiSSID(), settings.getWiFiPassword(), settings.getRobotName(),
                                           ""};
        IPAddress       microROSAgentIPAdress;

        if (false == microROSAgentIPAdress.fromString(settings.getMicroROSAgentAddress()))
        {
            LOG_FATAL("Invalid Micro-ROS agent IP-address: %s", settings.getMicroROSAgentAddress().c_str());
        }
        else if (false == board.getNetwork().setConfig(networkSettings))
        {
            LOG_FATAL("Network configuration could not be set.");
        }
        else if (false == m_ros.setConfiguration(settings.getRobotName(), "", microROSAgentIPAdress,
                                                 settings.getMicroROSAgentPort()))
        {
            LOG_FATAL("Micro-ROS agent could not be configured.");
        }
        else if (false == setupMicroRosClient())
        {
            LOG_FATAL("Micro-ROS client could not be setup.");
        }
        else if (false == setupSerialMuxProtServer())
        {
            LOG_FATAL("SerialMuxProt server could not be setup.");
        }
        else
        {
            /* Trigger immediately. */
            m_turtleMovementTimer.start(0U);
            isSuccessful = true;
        }
    }

    if (false == isSuccessful)
    {
        LOG_FATAL("Initialization failed.");
        fatalErrorHandler();
    }
    else
    {
        /* Blink Green LED to signal all-good. */
        Board::getInstance().getGreenLed().enable(true);
        delay(100U);
        Board::getInstance().getGreenLed().enable(false);

        LOG_INFO(settings.getRobotName() + " is ready.");
    }
}

void App::loop()
{
    if (false == m_isFatalError)
    {
        /* Process Battery, Device and Network. */
        Board::getInstance().process();

        /* Process Micro-ROS client. */
        m_ros.process();

        /* Process SerialMuxProt. */
        m_smpServer.process(millis());

        if (true == m_smpServer.isSynced())
        {
            if (true == m_statusTimer.isTimeout())
            {
                Status payload = {SMPChannelPayload::Status::STATUS_FLAG_OK};

                if (false == m_smpServer.sendData(m_serialMuxProtChannelIdStatus, &payload, sizeof(payload)))
                {
                    LOG_WARNING("Failed to send current status to RU.");
                }

                m_statusTimer.restart();
            }

            handleTurtle();
        }
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void App::fatalErrorHandler()
{
    if (false == m_isFatalError)
    {
        /* Turn on Red LED to signal fatal error. */
        Board::getInstance().getRedLed().enable(true);
    }

    m_isFatalError = true;
}

bool App::setupMicroRosClient()
{
    bool isSuccessful = false;

    /* Subscriber for Geometry Twist ROS messages. */
    typedef Subscriber<geometry_msgs__msg__Twist> CmdVelSubscriber;

    /* Create the Subscriber Callback. */
    CmdVelSubscriber::RosTopicCallback twistCallback = [this](const geometry_msgs__msg__Twist* msgData)
    {
        if (nullptr == msgData)
        {
            LOG_ERROR("TwistCallback received nullptr.");
        }
        else
        {
            LOG_DEBUG("Twist data received.");

            /* Short blink to indicate reception. */
            Board::getInstance().getBlueLed().enable(true);

            m_turtleSpeedSetpoint      = *msgData;
            m_isNewTurtleSpeedSetpoint = true;

            Board::getInstance().getBlueLed().enable(false);
        }
    };

    /* Create instance of CmdVelSubscriber. Will be deleted by the MicroRosClient. */
    CmdVelSubscriber* twistSub = new (std::nothrow)
        CmdVelSubscriber(TOPIC_NAME_CMD_VEL, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), twistCallback);

    /* Register the subscriber. */
    if (nullptr == twistSub)
    {
        LOG_ERROR("Could not create instance of CmdVelSubscriber");
    }
    else if (false == m_ros.registerSubscriber(twistSub))
    {
        LOG_ERROR("Could not register the CmdVelSubscriber.");
    }
    else
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

bool App::setupSerialMuxProtServer()
{
    bool isSuccessful = false;

    m_serialMuxProtChannelIdStatus = m_smpServer.createChannel(STATUS_CHANNEL_NAME, STATUS_CHANNEL_DLC);
    m_serialMuxProtChannelIdTurtle =
        m_smpServer.createChannel(ROBOT_SPEED_SETPOINT_CHANNEL_NAME, ROBOT_SPEED_SETPOINT_CHANNEL_DLC);

    if ((0U == m_serialMuxProtChannelIdStatus) || (0U == m_serialMuxProtChannelIdTurtle))
    {
        LOG_ERROR("Failed to create SerialMuxProt channels.");
    }
    else
    {
        isSuccessful = true;
        m_statusTimer.start(STATUS_TIMER_INTERVAL);
    }

    return isSuccessful;
}

void App::handleTurtle()
{
    /* Check for new data. */
    if (true == m_isNewTurtleSpeedSetpoint)
    {
        RobotSpeed    payload;
        const int32_t MILLI_CONVERSION_FACTOR = 1000;
        int32_t       linearSpeed = m_turtleSpeedSetpoint.linear.x * MILLI_CONVERSION_FACTOR; /* Linear speed in mm/s */
        int32_t angularSpeed = m_turtleSpeedSetpoint.angular.z * MILLI_CONVERSION_FACTOR; /* Angular speed in mrad/s */

        payload.linearCenter = linearSpeed;
        payload.angular      = angularSpeed;

        LOG_DEBUG("Linear speed: %d mm/s, Angular speed: %d mrad/s", payload.linearCenter, payload.angular);

        if (false == m_smpServer.sendData(m_serialMuxProtChannelIdTurtle, &payload, sizeof(payload)))
        {
            LOG_WARNING("Failed to send motor speeds to RU.");
        }
        else
        {
            m_isNewTurtleSpeedSetpoint = false;
            m_turtleMovementTimer.start(TURTLE_STEP_TIMER_INTERVAL);
        }
    }

    if (true == m_turtleMovementTimer.isTimeout())
    {
        RobotSpeed payload;
        payload.linearCenter = 0;
        payload.angular      = 0;

        if (false == m_smpServer.sendData(m_serialMuxProtChannelIdTurtle, &payload, sizeof(payload)))
        {
            LOG_WARNING("Failed to send motor speeds to RU.");
        }

        m_turtleMovementTimer.stop();
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
