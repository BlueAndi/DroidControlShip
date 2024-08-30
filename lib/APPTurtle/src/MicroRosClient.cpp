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
    m_nodeName(""),
    m_nodeNamespace(""),
    m_agentConfiguration(),
    m_isConfigured(false),
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
    rcl_ret_t ret = RCL_RET_OK;

    /* Cleanup of Subscribtions. */
    for (size_t idx = 0; idx < RMW_UXRCE_MAX_SUBSCRIPTIONS; idx++)
    {
        BaseSubscriber* currentSubscriber = m_subscribers[idx];

        if (nullptr != m_subscribers[idx])
        {
            ret += rcl_subscription_fini(&currentSubscriber->m_subscriber, &m_node);

            delete currentSubscriber;
        }
    }

    /* Cleanup of node. */
    ret += rcl_node_fini(&m_node);

    if (RCL_RET_OK != ret)
    {
        LOG_ERROR("Failed to destroy the nodes.");
    }
}

bool MicroRosClient::setConfiguration(const String& nodeName, const String& nodeNamespace, const String& agentIpAddress,
                                      uint16_t agentPort)
{
    bool isSuccessful = false;

    if (true == nodeName.isEmpty())
    {
        LOG_ERROR("Node name cannot be empty!");
    }
    else if (true == agentIpAddress.isEmpty())
    {
        LOG_ERROR("Agent IP cannot be empty!");
    }
    else
    {
        m_nodeName      = nodeName;
        m_nodeNamespace = nodeNamespace;
        m_agentConfiguration.address.fromString(agentIpAddress);
        m_agentConfiguration.port = agentPort;
        isSuccessful              = true;
    }

    return isSuccessful;
}

bool MicroRosClient::process()
{
    bool      isSuccessful = true;
    INetwork& network      = Board::getInstance().getNetwork();

    if ((true == network.isUp()) && (0U != network.getIp()))
    {
        if (false == m_isConfigured)
        {
            m_isConfigured = configureClient();

            if (false == m_isConfigured)
            {
                isSuccessful = false;
            }
            else
            {
                LOG_DEBUG("MicroROSClient configured.");
            }
        }
        else
        {
            if (RCL_RET_ERROR == rclc_executor_spin_some(&m_executor, RCL_MS_TO_NS(5)))
            {
                isSuccessful = false;
            }
        }
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
    else if (true == m_isConfigured)
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

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool MicroRosClient::configureClient()
{
    bool               isSuccessful = false;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    m_allocator                     = rcl_get_default_allocator();

    if ((true == m_nodeName.isEmpty()))
    {
        LOG_ERROR("Node name is empty.");
    }
    else if (RCL_RET_OK != rmw_uros_set_custom_transport(false, (void*)&m_agentConfiguration, CustomRosTransport::open,
                                                         CustomRosTransport::close, CustomRosTransport::write,
                                                         CustomRosTransport::read))
    {
        LOG_ERROR("Failed to set custom transport for Micro-ROS.");
    }
    else if (RCL_RET_OK != rclc_support_init(&m_support, 0, NULL, &m_allocator))
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

        for (size_t idx = 0; idx < RMW_UXRCE_MAX_SUBSCRIPTIONS; idx++)
        {
            BaseSubscriber* currentSubscriber = m_subscribers[idx];

            if (nullptr != currentSubscriber)
            {
                if (false == currentSubscriber->init(&m_node, &m_executor))
                {
                    LOG_DEBUG("Failed to create subscriber for topic %s", currentSubscriber->m_topicName.c_str());
                    isSuccessful = false;
                }
            }
        }
    }

    return isSuccessful;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
