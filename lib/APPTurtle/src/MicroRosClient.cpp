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
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "MicroRosClient.h"
#include <Logging.h>
#include <Board.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

MicroRosClient::MicroRosClient() :
    m_state(STATE_WAIT_FOR_AGENT),
    m_nodeName(),
    m_nodeNamespace(),
    m_customRosTransport(),
    m_node(),
    m_support(),
    m_executor(),
    m_allocator(),
    m_subscribers{nullptr},
    m_numberOfHandles(0U)
{
}

MicroRosClient::~MicroRosClient()
{
    unsubscribe();
    destroyEntities();
}

bool MicroRosClient::setConfiguration(const String& nodeName, const String& nodeNamespace,
                                      const IPAddress& agentIpAddress, uint16_t agentPort)
{
    bool isSuccessful = false;

    if (true == nodeName.isEmpty())
    {
        LOG_ERROR("Node name cannot be empty!");
    }
    else if (STATE_WAIT_FOR_AGENT != m_state)
    {
        LOG_ERROR("Set configuration in invalid state: %d", m_state);
    }
    else
    {
        m_nodeName      = nodeName;
        m_nodeNamespace = nodeNamespace;

        isSuccessful = setupCustomTransport(agentIpAddress, agentPort);
    }

    return isSuccessful;
}

bool MicroRosClient::registerSubscriber(BaseSubscriber* subscriber)
{
    bool isSuccessful = false;

    if (nullptr == subscriber)
    {
        LOG_ERROR("Subscriber is nullptr.");
    }
    else if (STATE_WAIT_FOR_AGENT != m_state)
    {
        LOG_ERROR("Subscriptions must be performed before calling process() for the first time!");
    }
    else if (RMW_UXRCE_MAX_SUBSCRIPTIONS <= m_numberOfHandles)
    {
        LOG_ERROR("Maximum number of subscribtions reached.");
    }
    else
    {
        m_subscribers[m_numberOfHandles] = subscriber;
        m_numberOfHandles++;

        isSuccessful = true;
    }

    return isSuccessful;
}

void MicroRosClient::process()
{
    switch (m_state)
    {
    case STATE_WAIT_FOR_AGENT:
        waitingForAgentState();
        break;

    case STATE_CONNECTING:
        connectingState();
        break;

    case STATE_CONNECTED:
        connectedState();
        break;

    case STATE_DISCONNECTED:
        disconnectedState();
        break;

    default:
        /* Should never happend. */
        break;
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool MicroRosClient::setupCustomTransport(const IPAddress& ipAddress, uint16_t port)
{
    bool isSuccessful = false;

    m_customRosTransport.init(ipAddress, port);

    if (RCL_RET_OK != rmw_uros_set_custom_transport(false, (void*)&m_customRosTransport, CustomRosTransport::open,
                                                    CustomRosTransport::close, CustomRosTransport::write,
                                                    CustomRosTransport::read))
    {
        LOG_ERROR("Failed to set custom transport for Micro-ROS.");
    }
    else
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

bool MicroRosClient::createEntities()
{
    bool isSuccessful = false;

    if (false == m_nodeName.isEmpty())
    {
        m_allocator = rcl_get_default_allocator();

        if (RCL_RET_OK != rclc_support_init(&m_support, 0, NULL, &m_allocator))
        {
            LOG_ERROR("Failed to initialize support structure.");
        }
        else if (RCL_RET_OK != rclc_node_init_default(&m_node, m_nodeName.c_str(), m_nodeNamespace.c_str(), &m_support))
        {
            LOG_ERROR("Failed to initialize node.");
        }
        else if (RCL_RET_OK != rclc_executor_init(&m_executor, &m_support.context, m_numberOfHandles, &m_allocator))
        {
            LOG_ERROR("Failed to initialize executor.");
        }
        else
        {
            isSuccessful = true;
        }

        /* If necessary, it will be cleaned-up. */
        if (false == isSuccessful)
        {
            destroyEntities();
        }
    }

    return isSuccessful;
}

void MicroRosClient::destroyEntities()
{
    rcl_ret_t ret = rclc_executor_fini(&m_executor);

    ret += rcl_node_fini(&m_node);
    ret += rclc_support_fini(&m_support);

    if (RCL_RET_OK != ret)
    {
        LOG_WARNING("Error while destroying entities.");
    }
}

void MicroRosClient::subscribe()
{
    for (size_t idx = 0; idx < RMW_UXRCE_MAX_SUBSCRIPTIONS; idx++)
    {
        BaseSubscriber* currentSubscriber = m_subscribers[idx];

        if (nullptr != currentSubscriber)
        {
            if (false == currentSubscriber->init(&m_node, &m_executor))
            {
                LOG_WARNING("Failed to create subscriber for topic %s", currentSubscriber->m_topicName.c_str());
            }
        }
    }
}

void MicroRosClient::unsubscribe()
{
    /* Remove all subscribtions. */
    for (size_t idx = 0; idx < RMW_UXRCE_MAX_SUBSCRIPTIONS; idx++)
    {
        BaseSubscriber* currentSubscriber = m_subscribers[idx];

        if (nullptr != currentSubscriber)
        {
            rcl_ret_t ret = rcl_subscription_fini(&currentSubscriber->m_subscriber, &m_node);

            if (RCL_RET_OK != ret)
            {
                LOG_WARNING("Error while unsubscribing.");
            }

            delete currentSubscriber;
            m_subscribers[idx] = nullptr;
        }
    }
}

void MicroRosClient::waitingForAgentState()
{
    INetwork& network = Board::getInstance().getNetwork();

    /* Network is up and running, as well client is already configured? */
    if ((true == network.isUp()) && (0U != network.getIp()) && (false == m_nodeName.isEmpty()))
    {
        /* First time entered or
         * ping agent again?
         */
        if ((false == m_timer.isTimerRunning()) || (true == m_timer.isTimeout()))
        {
            LOG_INFO("Ping Micro-ROS agent %s:%u ...", m_customRosTransport.getIPAddressAsStr().c_str(),
                     m_customRosTransport.getPort());

            if (RMW_RET_OK == rmw_uros_ping_agent(MICRO_ROS_AGENT_PING_TIMEOUT, MICRO_ROS_AGENT_PING_ATTEMPTS))
            {
                m_timer.stop();
                m_state = STATE_CONNECTING;
            }
            else if (false == m_timer.isTimerRunning())
            {
                m_timer.start(MICRO_ROS_AGENT_PING_PERIOD_LONG);
            }
            else
            {
                m_timer.restart();
            }
        }
    }
}

void MicroRosClient::connectingState()
{
    LOG_INFO("Connecting to Micro-ROS agent %s:%u ...", m_customRosTransport.getIPAddressAsStr().c_str(),
             m_customRosTransport.getPort());

    if (false == createEntities())
    {
        m_state = STATE_WAIT_FOR_AGENT;
    }
    else
    {
        LOG_INFO("Connected with Micro-ROS agent %s:%u.", m_customRosTransport.getIPAddressAsStr().c_str(),
                 m_customRosTransport.getPort());

        subscribe();

        /* Periodically verify that the connection is still established. */
        m_timer.start(MICRO_ROS_AGENT_PING_PERIOD_SHORT);

        m_state = STATE_CONNECTED;
    }
}

void MicroRosClient::connectedState()
{
    INetwork& network          = Board::getInstance().getNetwork();
    bool      isConnectionLost = false;

    /* Network down? */
    if (false == network.isUp())
    {
        isConnectionLost = true;
    }
    /* Periodically verify that the connection is still established. */
    else if (true == m_timer.isTimeout())
    {
        if (RMW_RET_OK != rmw_uros_ping_agent(MICRO_ROS_AGENT_PING_TIMEOUT, MICRO_ROS_AGENT_PING_ATTEMPTS))
        {
            isConnectionLost = true;
        }
        else
        {
            m_timer.restart();
        }
    }

    /* Connection lost? */
    if (true == isConnectionLost)
    {
        LOG_INFO("Connection lost with Micro-ROS agent.");

        m_timer.stop();
        m_state = STATE_DISCONNECTED;
    }
    /* Connection is still established, execute normal operation. */
    else
    {
        /* Anything in DDS queue? */
        (void)rclc_executor_spin_some(&m_executor, RCL_MS_TO_NS(DDS_QUEUE_CHECK_TIMEOUT));
    }
}

void MicroRosClient::disconnectedState()
{
    /* Clean-up */
    unsubscribe();
    destroyEntities();

    m_state = STATE_WAIT_FOR_AGENT;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
