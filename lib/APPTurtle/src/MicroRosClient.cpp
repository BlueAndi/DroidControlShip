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

bool MicroRosClient::setConfiguration(const String& nodeName, const String& nodeNamespace, const String& agentIpAddress,
                                      uint16_t agentPort)
{
    m_nodeName      = nodeName;
    m_nodeNamespace = nodeNamespace;
    m_agentConfiguration.address.fromString(agentIpAddress);
    m_agentConfiguration.port = agentPort;
    return true;
}

bool MicroRosClient::process()
{
    bool      isSuccessful = true;
    INetwork& network      = Board::getInstance().getNetwork();

    if ((true == network.isUp()) && (false == network.getIp().isEmpty()))
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
            if (RCL_RET_ERROR == rclc_executor_spin_some(&m_executor, RCL_MS_TO_NS(100)))
            {
                isSuccessful = false;
            }
        }
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
    rclc_support_t     support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    m_allocator                     = rcl_get_default_allocator();

    uint8_t numberOfHandles =
        RMW_UXRCE_MAX_SUBSCRIPTIONS + RMW_UXRCE_MAX_SERVICES + RMW_UXRCE_MAX_CLIENTS + RMW_UXRCE_MAX_GUARD_CONDITION;

    if ((true == m_nodeName.isEmpty()))
    {
        LOG_ERROR("Node name is empty.");
    }
    else if (RCL_RET_OK != rmw_uros_set_custom_transport(false, (void*)&m_agentConfiguration, platformio_transport_open,
                                                         platformio_transport_close, platformio_transport_write,
                                                         platformio_transport_read))
    {
        LOG_ERROR("Failed to set custom transport for Micro-ROS.");
    }
    else if (RCL_RET_OK != rclc_support_init(&support, 0, NULL, &m_allocator))
    {
        LOG_ERROR("Failed to initialize support structure.");
    }
    else if (RCL_RET_OK != rclc_node_init_default(&m_node, m_nodeName.c_str(), m_nodeNamespace.c_str(), &support))
    {
        LOG_ERROR("Failed to initialize node.");
    }
    else if (RCL_RET_OK != rclc_executor_init(&m_executor, &support.context, numberOfHandles, &m_allocator))
    {
        LOG_ERROR("Failed to initialize executor.");
    }
    else
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
