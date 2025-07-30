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
 * @brief  Network realization
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Network.h"
#include <Logging.h>

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

Network::Network() : INetwork(), m_state(STATE_UNINITIALIZED), m_configSet(false)
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
    /* Settings have no effect in the simulation but have to be set nonetheless. */
    m_configSet = true;
    return m_configSet;
}

bool Network::isUp() const
{
    /* Simulation link always returns true. */
    return true;
}

IPAddress Network::getIp() const
{
    /* Simulation uses localhost IP. */
    return IPAddress(127, 0, 0, 1);
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
        m_state = STATE_CONNECTING;
    }
}

void Network::handleConnectingState()
{
    /* Act like a connection was successfully established in simulation. */
    m_state = STATE_CONNECTED;
}

void Network::manageConnection()
{
    /* Do nothing in simulation. */
}

void Network::switchToAPMode()
{
    /* Don't need to handle AP mode in simulation. */
    m_state = STATE_AP_SETUP;
}

void Network::handleAPSetup()
{
    /* Don't need to handle AP mode in simulation. */
    m_state = STATE_AP_UP;
}

void Network::handleAPState()
{
    /* Don't need to handle AP mode in simulation. */
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
