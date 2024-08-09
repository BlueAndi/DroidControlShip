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
 * @brief  Abstraction of Micro-ROS Client.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef MICRO_ROS_AGENT_H
#define MICRO_ROS_AGENT_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Micro-ROS Client.
 */
class MicroRosClient
{
public:
    /**
     * Default Constructor
     */
    MicroRosClient() : m_isConfigured(false), m_node(), m_subscriber(), m_executor(), m_allocator(), m_msg()
    {
    }

    /**
     * Default destructor.
     */
    ~MicroRosClient()
    {
    }

    /**
     * Set the Agent configuration.
     *
     * @param[in] address IP address of the Micro-ROS agent.
     * @param[in] port Port of the Micro-ROS agent.
     *
     * @returns If the parameters are valid, returns true. Otherwise, false.
     */
    bool setAgent(const String& ipAddress, uint16_t port);

    /**
     * Process the Micro-Ros node and its executors.
     *
     * @returns If connection to the agent cannot be established, returns false. Otherwise, true.
     */
    bool process();

private:
    /**
     * Server configuration. Contains IP address and port of the Agent.
     */
    micro_ros_agent_locator m_agentConfiguration;

    /**
     * Flag: is the client configured?
     */
    bool m_isConfigured;

    /**
     * Micro-ROS node handle
     */
    rcl_node_t m_node;

    /**
     * Micro-ROS subscriber for cmd_vel topic
     */
    rcl_subscription_t m_subscriber;

    /**
     * Micro-ROS executor
     */
    rclc_executor_t m_executor;

    /**
     * Micro-ROS allocator
     */
    rcl_allocator_t m_allocator;

    /**
     * Micro-ROS message for cmd_vel topic
     */
    geometry_msgs__msg__Twist m_msg;

    /**
     * Configure the client.
     */
    bool configureClient();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* MICRO_ROS_AGENT_H */
/** @} */
