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

Network::Network() :
    INetwork(),
    m_state(STATE_UNINITIALIZED),
    m_configSet(false),
    m_wiFiSSID("ZumoAP"),
    m_wiFiPassword("zumopass")
{
}

Network::~Network()
{
}

bool Network::init()
{
    bool isSuccess = true;

    if (STATE_UNINITIALIZED != m_state)
    {
        isSuccess = false;
    }
    else
    {
        m_state = STATE_SETUP;
    }

    return isSuccess;
}

void Network::process()
{
    switch (m_state)
    {
    case STATE_UNINITIALIZED:
        /* Nothing to do. */
        break;

    case STATE_SETUP:
        handleStationSetup();
        break;

    case STATE_CONNECTING:
        handleConnectingState();
        break;

    case STATE_CONNECTED:
        manageConnection();
        break;

    case STATE_DISCONNECTED:
        switchToAPMode();
        break;

    case STATE_AP_SETUP:
        handleAPSetup();
        break;

    case STATE_AP_UP:
        handleAPState();
        break;

    default:
        /* Should never be called - defensive code. */
        break;
    }
}

bool Network::setConfig(const NetworkSettings& settings)
{
    if (true == settings.ssid.isEmpty())
    {
        /* Check only performed on target. */
        LOG_ERROR("WiFi SSID is empty.");
    }
    else
    {
        m_wiFiSSID     = settings.ssid;
        m_wiFiPassword = settings.password;
        m_apSSID       = settings.apSsid;
        m_apPassword   = settings.apPassword;
        m_configSet    = true;
    }

    return m_configSet;
}

bool Network::isUp() const
{
    /* Network is not in a setup or initialized state. */
    return ((STATE_UNINITIALIZED != m_state) && (STATE_SETUP != m_state) && (STATE_AP_SETUP != m_state));
}

IPAddress Network::getIp() const
{
    IPAddress ipAddr;

    if (STATE_CONNECTED == m_state)
    {
        ipAddr = WiFi.localIP();
    }
    else if (STATE_AP_UP == m_state)
    {
        ipAddr = WiFi.softAPIP();
    }
    else
    {
        ipAddr = IPAddress();
    }

    return ipAddr;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void Network::handleStationSetup()
{
    if (true == m_configSet)
    {
        if (WL_CONNECT_FAILED == WiFi.begin(m_wiFiSSID.c_str(), m_wiFiPassword.c_str()))
        {
            LOG_ERROR("Failed to connect to WiFi.");
            m_state = STATE_DISCONNECTED;
        }
        else
        {
            m_state = STATE_CONNECTING;
        }
    }
}

void Network::handleConnectingState()
{
    wl_status_t currentStatus = WiFi.status();

    if (WL_CONNECTED == currentStatus)
    {
        if (true == m_wifiTimeoutTimer.isTimerRunning())
        {
            /* Stop timer. */
            m_wifiTimeoutTimer.stop();
        }

        LOG_DEBUG("WiFi connection established successfully with %s.", m_wiFiSSID);
        m_state = STATE_CONNECTED;
    }
    else if (WL_NO_SSID_AVAIL == currentStatus)
    {
        if (true == m_wifiTimeoutTimer.isTimerRunning())
        {
            /* Stop timer. */
            m_wifiTimeoutTimer.stop();
        }

        LOG_DEBUG("Cannot find network with provided SSID (%s).", m_wiFiSSID);
        m_state = STATE_DISCONNECTED;
    }
    else
    {
        if (false == m_wifiTimeoutTimer.isTimerRunning())
        {
            /* Start timeout timer. */
            m_wifiTimeoutTimer.start(WIFI_TIMEOUT);
        }
        else if (true == m_wifiTimeoutTimer.isTimeout())
        {
            LOG_ERROR("WiFi Connection Timed-out!");
            m_state = STATE_DISCONNECTED;
        }
        else
        {
            /* WiFi is connecting. Do nothing. */
        }
    }
}

void Network::manageConnection()
{
    if (m_state != STATE_CONNECTED)
    {
        LOG_DEBUG("Connection to WiFi lost. Reconnecting...");
        m_state = STATE_CONNECTING;
    }
}

void Network::switchToAPMode()
{
    if (WiFi.disconnect() && WiFi.mode(WIFI_AP) && WIFI_MODE_AP == WiFi.getMode())
    {
        m_state = STATE_AP_SETUP;
    }
}

void Network::handleAPSetup()
{
    if (WIFI_AP != WiFi.getMode())
    {
        WiFi.mode(WIFI_AP);
    }
    else
    {
        bool isSuccessful = WiFi.softAP(m_apSSID.c_str(), m_apPassword.c_str());

        LOG_DEBUG("Setting up Access Point (%s, %s).", m_apSSID.c_str(), m_apPassword.c_str());

        if (true == isSuccessful)
        {
            m_state = STATE_AP_UP;
            LOG_DEBUG("Access Point up.");
        }
    }
}

void Network::handleAPState()
{
    /* Don't need to do anything for now. */
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
