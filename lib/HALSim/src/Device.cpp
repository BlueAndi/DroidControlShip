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
 * @brief  Device realization
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Device.h"

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

/* Default server address of the device. */
const char* Device::DEFAULT_SERVER_ADDRESS = "localhost";

/* Default server port of the device. */
const char* Device::DEFAULT_SERVER_PORT = "65432";

/* Maximum number of connection retries. */
const uint8_t Device::MAX_CONN_RETRY_COUNT = 2U;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

bool Device::init()
{
    return m_socket.init(m_address, m_port);
}

bool Device::process()
{
    bool isSuccess = true;

    /* Process SocketClient. */
    if (false == m_socket.process())
    {
        m_retryConnectionCounter++;

        /* Device can retry to connect before returning an error. */
        if (MAX_CONN_RETRY_COUNT <= m_retryConnectionCounter)
        {
            isSuccess = false;
        }
    }
    else
    {
        /* Reset connection counter. */
        if (0U < m_retryConnectionCounter)
        {
            m_retryConnectionCounter = 0U;
        }
    }

    return isSuccess;
}

Stream& Device::getStream()
{
    return m_socket;
}

void Device::reset()
{
    /* Not Implemented. */
}

void Device::enterBootloader()
{
    /* Not Implemented. */
}

void Device::setServer(const char* address, const char* port)
{
    if (nullptr == address)
    {
        m_address = DEFAULT_SERVER_ADDRESS;
    }
    else
    {
        m_address = address;
    }

    if (nullptr == port)
    {
        m_port = DEFAULT_SERVER_PORT;
    }
    else
    {
        m_port = port;
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
