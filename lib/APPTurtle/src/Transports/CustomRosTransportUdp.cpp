/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Custom Micro-ROS transport over UDP.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "CustomRosTransportUdp.h"

#include <SimpleTimer.hpp>
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

const String CustomRosTransportUdp::m_protocolName("UDP");

/******************************************************************************
 * Public Methods
 *****************************************************************************/

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool CustomRosTransportUdp::open(void)
{
    bool      isOpen = false;
    const int UDP_OK = 1;

    if (UDP_OK != m_udpClient.begin(m_port))
    {
        LOG_ERROR("UDP begin error");
    }
    else
    {
        isOpen = true;
    }

    return isOpen;
}

bool CustomRosTransportUdp::close()
{
    m_udpClient.stop();

    return true;
}

size_t CustomRosTransportUdp::write(const uint8_t* buffer, size_t size, uint8_t* errorCode)
{
    size_t sent = 0U;

    if ((nullptr == buffer) || (0U == size) || (nullptr == errorCode))
    {
        LOG_ERROR("One or more parameters are invalid.");
    }
    else
    {
        const int UDP_OK = 1;
        int       ret    = UDP_OK;

        ret = m_udpClient.beginPacket(m_address, m_port);

        if (UDP_OK != ret)
        {
            LOG_ERROR("UDP beginPacket error");
        }
        else
        {
            size_t bytesToWrite = m_udpClient.write(buffer, size);

            if (bytesToWrite != size)
            {
                LOG_ERROR("UDP write error");
            }
            else
            {
                ret = m_udpClient.endPacket();

                if (UDP_OK != ret)
                {
                    LOG_ERROR("UDP endPacket error");
                }
                else
                {
                    sent = bytesToWrite;
                }
            }
        }

        if (UDP_OK != ret)
        {
            int writeErrorCode = m_udpClient.getWriteError();
            *errorCode         = writeErrorCode;
            LOG_ERROR("UDP error: %d", writeErrorCode);
        }

        m_udpClient.flush();
    }

    return sent;
}

size_t CustomRosTransportUdp::read(uint8_t* buffer, size_t size, int timeout, uint8_t* errorCode)
{
    size_t readBytes = 0U;

    if ((nullptr == buffer) || (0U == size) || (nullptr == errorCode))
    {
        LOG_ERROR("One or more parameters are invalid.");
    }
    else
    {
        SimpleTimer readTimer;

        readTimer.start(timeout);

        while (false == readTimer.isTimeout())
        {
            if (0 < m_udpClient.parsePacket())
            {
                break;
            }

#if defined(TARGET_NATIVE)
            /*
             * SimpleTimer requires simulation time to progress
             * otherwise this loop will block until a packet was received.
             * TODO This is a workaround which needs to be discussed further.
             */
            Board::getInstance().stepTime();
#endif
        }

        if (0 == m_udpClient.available())
        {
            *errorCode = 1;
        }
        else
        {
            readBytes = m_udpClient.read(buffer, size);
        }
    }

    return readBytes;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
