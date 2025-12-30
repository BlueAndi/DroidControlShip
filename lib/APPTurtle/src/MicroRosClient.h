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

#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <SimpleTimer.hpp>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The Micro-ROS client handles the communication with the Micro-ROS agent.
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
    bool setConfiguration(const String& nodeName, const String& nodeNamespace, const IPAddress& agentIpAddress,
                          uint16_t agentPort);

    /**
     * Register a subscriber to a ROS Topic.
     *
     * @param[in] subscriber Pointer to a new subscriber. It shall be instanced using new. The MicroRosClient will
     *                       delete the pointer once it is no longer used. Checks if the instance is nullptr.
     *
     * @returns If successfully created, returns true. Otherwise, false.
     */
    bool registerSubscriber(BaseSubscriber* subscriber);

    /**
     * Process the Micro-ROS node and its executors.
     */
    void process();

private:
    /**
     * The Micro-ROS agent will be periodically pinged in waiting state.
     * This is the period time in ms.
     */
    static const uint32_t MICRO_ROS_AGENT_PING_PERIOD_LONG = 500U;
    /**
     * The Micro-ROS agent will be periodically pinged to detect connection loss.
     * This is the period time in ms.
     */
    static const uint32_t MICRO_ROS_AGENT_PING_PERIOD_SHORT = 200U;

    /**
     * The Micro-ROS agent ping operation timeout is ms.
     */
    static const int32_t MICRO_ROS_AGENT_PING_TIMEOUT = 200;

    /**
     * The Micro-ROS agent ping operation attempts. Keep 1, because the
     * retry mechanism is handled by our client.
     */
    static const uint8_t MICRO_ROS_AGENT_PING_ATTEMPTS = 1U;

    /**
     * DDS queue check timeout in ms.
     */
    static const uint64_t DDS_QUEUE_CHECK_TIMEOUT = 8U;

    /**
     * The connection states with the Micro-ROS agent.
     */
    enum State
    {
        STATE_WAIT_FOR_AGENT = 0, /**< Waiting for Micro-ROS agent. */
        STATE_CONNECTING,         /**< Connecting to Micro-ROS agent. */
        STATE_CONNECTED,          /**< Connection with Micro-ROS agent established. */
        STATE_DISCONNECTED        /**< Disconnected or connection lost. */
    };

    /**
     * Connection state with Micro-ROS agent.
     */
    State m_state;

    /**
     * Name of the ROS2 Node.
     */
    String m_nodeName;

    /**
     * Namespace of the ROS2 Node.
     */
    String m_nodeNamespace;

    /**
     * Custom Micro-ROS transport.
     */
    CustomRosTransport m_customRosTransport;

    /**
     * Micro-ROS node handle
     */
    rcl_node_t m_node;

    /**
     * Micro-ROS support structure
     */
    rclc_support_t m_support;

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
     * Timer used for periodically operations.
     */
    SimpleTimer m_timer;

    /**
     * Setup the custom transport protocol.
     *
     * @param[in] ipAddress The Micro-ROS agent IP-address.
     * @param[in] port      The Micro-ROS agent port.
     *
     * @return If successful it will return true otherwise false.
     */
    bool setupCustomTransport(const IPAddress& ipAddress, uint16_t port);

    /**
     * Create all Micro-ROS required entities.
     * Note, it will allocate memory for the entities. If it fails, it will
     * automatically clean-up.
     *
     * @return If successful created, it will return true otherwise false.
     */
    bool createEntities();

    /**
     * Destroy all Micro-ROS entities.
     * Note, it will release the memory for the entities.
     */
    void destroyEntities();

    /**
     * Subscribe all topics.
     */
    void subscribe();

    /**
     * Unsubscribe all topis.
     */
    void unsubscribe();

    /**
     * Process waiting state.
     */
    void waitingForAgentState();

    /**
     * Process connecting state.
     */
    void connectingState();

    /**
     * Process connected state.
     */
    void connectedState();

    /**
     * Process disconnected state.
     */
    void disconnectedState();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* MICRO_ROS_AGENT_H */
/** @} */
