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
 * @brief  Turtle application.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "App.h"
#include <Board.h>
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <SettingsHandler.h>
#include <WiFi.h>

#include <geometry_msgs/msg/twist.h>

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

        LOG_DEBUG("LOGGER READY");
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
            String robotName = WiFi.macAddress();

            /* Remove MAC separators from robot name. */
            robotName.replace(":", "");

            settings.setRobotName(robotName);
        }

        NetworkSettings networkSettings = {settings.getWiFiSSID(), settings.getWiFiPassword(), settings.getRobotName(),
                                           ""};

        if (false == board.getNetwork().setConfig(networkSettings))
        {
            LOG_ERROR("Network configuration could not be set.");
        }
        else if (false == m_ros.setConfiguration(settings.getRobotName(), "", settings.getMqttBrokerAddress(),
                                                 settings.getMqttPort()))
        {
            LOG_ERROR("Micro-ROS Agent could not be configured.");
        }
        else
        {
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

                    /* Process data. */
                    LOG_DEBUG("Linear: %f %f %f", msgData->linear.x, msgData->linear.y, msgData->linear.z);
                    LOG_DEBUG("Angular: %f %f %f", msgData->angular.x, msgData->angular.y, msgData->angular.z);

                    Board::getInstance().getBlueLed().enable(false);
                }
            };

            /* Create instance of CmdVelSubscriber. Will be deleted by the MicroRosClient. */
            CmdVelSubscriber* twistSub = new (std::nothrow) CmdVelSubscriber(
                TOPIC_NAME_CMD_VEL, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), twistCallback);

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

        LOG_INFO("Turtle is ready.");
    }
}

void App::loop()
{
    Board& board = Board::getInstance();

    /* Process Battery, Device and Network. */
    if (false == board.process())
    {
        /* Log and Handle Board processing error */
        LOG_FATAL("HAL process failed.");
        fatalErrorHandler();
    }

    /* Process Micro ROS client. */
    if (false == m_ros.process())
    {
        LOG_FATAL("ROS process failed.");
        fatalErrorHandler();
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
    /* Turn on Red LED to signal fatal error. */
    Board::getInstance().getRedLed().enable(true);

    while (true)
    {
        ;
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
