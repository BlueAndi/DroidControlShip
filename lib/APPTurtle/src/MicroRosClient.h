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

#include "Subscriber.h"
#include "CustomRosTransport.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

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
    MicroRosClient();

    /**
     * Default destructor.
     */
    ~MicroRosClient();

    /**
     * Set the Client configuration.
     *
     * @param[in] nodeName Name of the ROS2 Node.
     * @param[in] nodeNamespace Namespace of the ROS2 Node. Can be empty.
     * @param[in] agentIpAddress IP address of the Micro-ROS agent.
     * @param[in] agentPort Port of the Micro-ROS agent.
     *
     * @returns If the parameters are valid, returns true. Otherwise, false.
     */
    bool setConfiguration(const String& nodeName, const String& nodeNamespace, const String& agentIpAddress,
                          uint16_t agentPort);

    /**
     * Process the Micro-Ros node and its executors.
     *
     * @returns If connection to the agent cannot be established, returns false. Otherwise, true.
     */
    bool process();

    /**
     * Register a subscriber to a ROS Topic.
     *
     * @param[in] subscriber Pointer to a new subscriber. It shall be instanced using new. The MicroRosClient will
     * delete the pointer once it is no longer used. Checks if the instance is nullptr.
     *
     * @returns If succesfully created, returns true. Otherwise, false.
     */
    bool registerSubscriber(BaseSubscriber* subscriber);

private:
    /**
     * Name of the ROS2 Node.
     */
    String m_nodeName;

    /**
     * Namespace of the ROS2 Node.
     */
    String m_nodeNamespace;

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
     * Micro-ROS executor
     */
    rclc_executor_t m_executor;

    /**
     * Micro-ROS allocator
     */
    rcl_allocator_t m_allocator;

    /**
     * Array of pointers to subscribers.
     */
    BaseSubscriber* m_subscribers[RMW_UXRCE_MAX_SUBSCRIPTIONS];

    /**
     * Counter of subscribtions.
     */
    size_t m_numberOfHandles;

    /**
     * Configure the client.
     *
     * @returns If succesfully configured, returns true. Otherwise, false.
     */
    bool configureClient();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* MICRO_ROS_AGENT_H */
/** @} */
