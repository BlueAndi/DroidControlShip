/* MIT License
 *
 * Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Network realization
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Network.h"
#include <Logging.h>
#include <WiFi.h>
#include <SimpleTimer.hpp>

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

Network::Network() : INetwork(), m_configSet(false), m_wiFiSSID(""), m_wiFiPassword(""), m_isWiFiConfigured(false)
{
}

Network::~Network()
{
}

bool Network::init()
{
    return true;
}

bool Network::setConfig(const String& ssid, const String& password)
{
    if (true == ssid.isEmpty())
    {
        /* Check only performed on target. */
        LOG_ERROR("WiFi SSID is empty.");
    }
    else
    {
        m_wiFiSSID     = ssid;
        m_wiFiPassword = password;
        m_configSet    = true;
    }

    return m_configSet;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool Network::init()
{
    if (true == m_configSet)
    {
        if (WL_CONNECT_FAILED == WiFi.begin(m_wiFiSSID.c_str(), m_wiFiPassword.c_str()))
        {
            LOG_ERROR("Failed to connect to WiFi.");
        }
        else
        {
            m_isWiFiConfigured = true;
        }
    }
    return m_isWiFiConfigured;
}

bool Network::manageConnection()
{
    bool isSuccess = true;

    if (true == m_isWiFiConfigured)
    {
        if (WL_CONNECTED != WiFi.status())
        {
            if (false == m_wifiTimeoutTimer.isTimerRunning())
            {
                /* Start timeout timer. */
                LOG_DEBUG("Connection to WiFi lost. Reconnecting...");
                m_wifiTimeoutTimer.start(WIFI_TIMEOUT);
            }
            else if (true == m_wifiTimeoutTimer.isTimeout())
            {
                LOG_ERROR("WiFi Connection Timed-out!");
                isSuccess = false;
            }
            else
            {
                /* WiFi is connecting. Do nothing. */
            }
        }
        else
        {
            if (true == m_wifiTimeoutTimer.isTimerRunning())
            {
                /* Stop timer. */
                m_wifiTimeoutTimer.stop();
            }
        }
    }

    return isSuccess;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
