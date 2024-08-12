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

/**
 * Callback for the cmd topic.
 */
static void cmdTopicCallback(const void* msgin, void* context);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

bool MicroRosClient::setAgent(const String& ipAddress, uint16_t port)
{
    m_agentConfiguration.address.fromString(ipAddress);
    m_agentConfiguration.port = port;
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
        }
        else
        {
            if (RCL_RET_ERROR != rclc_executor_spin_some(&m_executor, RCL_MS_TO_NS(100)))
            {
                isSuccessful = true;
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

    if (RCL_RET_OK != rmw_uros_set_custom_transport(false, (void*)&m_agentConfiguration, platformio_transport_open,
                                                    platformio_transport_close, platformio_transport_write,
                                                    platformio_transport_read))
    {
    }
    else if (RCL_RET_OK != rclc_support_init(&support, 0, NULL, &m_allocator))
    {
    }
    else if (RCL_RET_OK != rclc_node_init_default(&m_node, "zumo_node", "", &support))
    {
    }
    else if (RCL_RET_OK != rclc_subscription_init_default(
                               &m_subscriber, &m_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd"))
    {
    }
    else if (RCL_RET_OK != rclc_executor_init(&m_executor, &support.context, 1, &m_allocator))
    {
    }
    else if (RCL_RET_OK != rclc_executor_add_subscription_with_context(&m_executor, &m_subscriber, &m_msg,
                                                                       cmdTopicCallback, this, ON_NEW_DATA))
    {
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

static void cmdTopicCallback(const void* msgin, void* context)
{
    (void)msgin;
    (void)context;
    LOG_INFO("CMD Topic Callback");
    Board::getInstance().getBlueLed().enable(true);
    delay(50U);
    Board::getInstance().getBlueLed().enable(false);
}
