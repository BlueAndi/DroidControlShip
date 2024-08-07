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

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

#ifndef CONFIG_LOG_SEVERITY
#define CONFIG_LOG_SEVERITY (Logging::LOG_LEVEL_INFO)
#endif /* CONFIG_LOG_SEVERITY */

#define RCCHECK(fn)                                                                                                    \
    {                                                                                                                  \
        rcl_ret_t temp_rc = fn;                                                                                        \
        if ((temp_rc != RCL_RET_OK))                                                                                   \
        {                                                                                                              \
            LOG_FATAL("Function %s failed with error code: %d", #fn, temp_rc);                                         \
            fatalErrorHandler();                                                                                       \
        }                                                                                                              \
    }

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/**
 * Handler for motor speeds topic subscriber callback
 */
static void App_motorSpeedsTopicCallback(const void* msgin);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Serial interface baudrate. */
static const uint32_t SERIAL_BAUDRATE = 115200U;

/** Serial log sink */
static LogSinkPrinter gLogSinkSerial("Serial", &Serial);

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
        else
        {
            /* Initialize micro-ROS */
            char ssid[settings.getWiFiSSID().length() + 1];
            char pass[settings.getWiFiPassword().length() + 1];
            settings.getWiFiSSID().toCharArray(ssid, sizeof(ssid));
            settings.getWiFiPassword().toCharArray(pass, sizeof(pass));
            settings.getMqttBrokerAddress();

            IPAddress ipaddr;
            ipaddr.fromString(settings.getMqttBrokerAddress());

            LOG_INFO("Start micro-ROS Transport init...");

            // connects to configured Wifi AP in a loop
            set_microros_wifi_transports(ssid, pass, ipaddr, settings.getMqttPort());

            LOG_INFO("ROS Transport init complete");

            LOG_INFO("Starting ROS init ...");
            m_allocator = rcl_get_default_allocator();
            rclc_support_t     support;
            rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

            RCCHECK(rclc_support_init(&support, 0, NULL, &m_allocator));

            RCCHECK(rclc_node_init_default(&m_node, "zumo_node", "", &support));

            RCCHECK(rclc_subscription_init_default(&m_subscriber, &m_node,
                                                   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd"));

            RCCHECK(rclc_executor_init(&m_executor, &support.context, 1, &m_allocator));

            RCCHECK(rclc_executor_add_subscription(&m_executor, &m_subscriber, &m_msg, App_motorSpeedsTopicCallback,
                                                   ON_NEW_DATA));

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

        LOG_INFO("Turtle is ready.");
    }
}

void App::loop()
{
    /* Process Battery, Device and Network. */
    if (false == Board::getInstance().process())
    {
        /* Log and Handle Board processing error */
        LOG_FATAL("HAL process failed.");
        fatalErrorHandler();
    }

    // I try spinning, that's a good trick!
    rclc_executor_spin(&m_executor);
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

static void App_motorSpeedsTopicCallback(const void* msgin)
{
    // TODO implement message converstion to DifferentialDrive SerialMuxProt message
    LOG_INFO("MOTOR_SPEEDS_TOPIC_CALLBACK");
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
