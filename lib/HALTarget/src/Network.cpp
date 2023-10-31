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

Network::Network() : INetwork(), m_state(STATE_UNINITIALIZED), m_configSet(false), m_wiFiSSID(""), m_wiFiPassword("")
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
        isSuccess == false;
    }
    else
    {
        m_state = STATE_SETUP;
    }
    return true;
}

bool Network::process()
{
    bool isSuccess = false;

    switch (m_state)
    {
    case STATE_UNINITIALIZED:
        /* Nothing to do. */
        isSuccess = true;
        break;
    case STATE_SETUP:
        isSuccess = handleStationSetup();
        break;

    case STATE_CONNECTING:
        isSuccess = handleConnectingState();
        break;

    case STATE_CONNECTED:
        isSuccess = manageConnection();
        break;

    case STATE_DISCONNECTED:
        isSuccess = switchToAPMode();
        break;

    case STATE_AP_SETUP:
        isSuccess = handleAPSetup();
        break;

    case STATE_AP_UP:
        isSuccess = handleAPState();
        break;

    default:
        /* Should never be called - defensive code. */
        break;
    }

    return isSuccess;
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

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool Network::handleStationSetup()
{
    bool isSuccess = false;

    if (true == m_configSet)
    {
        if (WL_CONNECT_FAILED == WiFi.begin(m_wiFiSSID.c_str(), m_wiFiPassword.c_str()))
        {
            LOG_ERROR("Failed to connect to WiFi.");
            m_state   = STATE_AP_SETUP;
            isSuccess = true;
        }
        else
        {
            m_state   = STATE_CONNECTING;
            isSuccess = true;
        }
    }
    return isSuccess;
}

bool Network::handleConnectingState()
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
    return true;
}

bool Network::manageConnection()
{
    if (m_state != STATE_CONNECTED)
    {
        LOG_DEBUG("Connection to WiFi lost. Reconnecting...");
        m_state = STATE_CONNECTING;
    }

    return true;
}

bool Network::switchToAPMode()
{
    WiFi.mode(WIFI_AP);
    m_state = STATE_AP_SETUP;
    return true;
}

bool Network::handleAPSetup()
{
    bool setupSuccess = false;

    if (WIFI_AP != WiFi.getMode())
    {
        WiFi.mode(WIFI_AP);
    }

    /* get AP ssid and pw */
    String ssid;
    String password;
    if (false == m_apSSID.isEmpty())
    {
        ssid = m_apSSID;
    }
    else
    {
        /* Use default AP name, if none is provided. */
        ssid = String("Zumo_AP");
    }
    if (false == m_apPassword.isEmpty())
    {
        password = m_apPassword;
    }
    else
    {
        /* Use default AP password, if none is provided. */
        password = String("zumopass");
    }

    LOG_DEBUG("Setting up Access Point.");

    /* setup AP and return success */
    setupSuccess = WiFi.softAP(ssid.c_str(), password.c_str());

    if (setupSuccess)
    {
        m_state = STATE_AP_UP;
        LOG_DEBUG("Access Point up.");
    }
    return setupSuccess;
}

bool Network::handleAPState()
{
    /* Don't need to do anything for now. */
    return true;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
