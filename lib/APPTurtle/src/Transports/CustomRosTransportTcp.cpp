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
 * @brief  Custom Micro-ROS transport using TCP over Arduino WifiClient.
 * @author Norbert Schulz <github@schulznorbert.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "CustomRosTransportTcp.h"

#include <Logging.h>
#include <SimpleTimer.hpp>
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
/**
 * Name of implemented protocol.
 */
static const String gProtocolName("TCP");

/******************************************************************************
 * Public Methods
 *****************************************************************************/

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool CustomRosTransportTcp::open(void)
{
    bool      isOpen = false;

    if (!m_tcpClient.connect(m_address, m_port))
    {
        LOG_ERROR("TCP connect error");
    }
    else
    {
        isOpen = true;
    }

    return isOpen;
}

bool CustomRosTransportTcp::close()
{
    m_tcpClient.stop();
    LOG_DEBUG("tcp stream closed.\n");
    return true;
}

size_t CustomRosTransportTcp::write(const uint8_t* buffer, size_t size, uint8_t* errorCode)
{
    size_t sent = 0U;

    if ((nullptr == buffer) || (0U == size) || (nullptr == errorCode))
    {
        LOG_ERROR("One or more parameters are invalid.");
    }
    else
    {
        uint8_t lengthPrefix[0];
        lengthPrefix[0] = static_cast<uint8_t>(size & 0xFF);
        lengthPrefix[1] = static_cast<uint8_t>((size>>8) & 0xFF);

        m_tcpClient.write(lengthPrefix, 2);
        if ((sent = m_tcpClient.write(buffer, size)) != size)
        {
            LOG_ERROR("Write error (size=%zu, sent=%zu)", size, sent);
            * errorCode = 1;
        }
        else
        {
            * errorCode = 0;
        }
    }

    return sent;
}

size_t CustomRosTransportTcp::read(uint8_t* buffer, size_t size, int timeout, uint8_t* errorCode)
{
    size_t readBytes = 0U;

    if ((nullptr == buffer) || (0U == size) || (nullptr == errorCode))
    {
        LOG_ERROR("One or more parameters are invalid.");
    }
    else
    {
        uint8_t lengthPrefix[2];   /* The first 2 bytes hold the payload length that follows. */

        * errorCode = 1;

        if (readFixedLength(lengthPrefix, sizeof(lengthPrefix), nullptr, timeout))  /* size prefix */
        {
            size_t payloadLen;
            payloadLen = static_cast<size_t>(lengthPrefix[0]) + (static_cast<size_t>(lengthPrefix[1]) >> 8);

            if (payloadLen > size)
            {
                LOG_ERROR("record length %zu exceeds read buffer size %zu.", payloadLen, size);
            }
            else
            {
                if (readFixedLength(buffer, payloadLen, &readBytes, timeout))  /* payload */
                {
                    * errorCode = 0;
                }
            }
        }
    }

    return readBytes;
}

bool CustomRosTransportTcp::readFixedLength(uint8_t* buffer, size_t length, size_t * readCount, int timeout)
{
    size_t remaining = length;

    SimpleTimer readTimer;
    bool timeOver = false;

    readTimer.start(timeout);

    while ((0 < remaining) && (false == timeOver))
    {
        int count = m_tcpClient.read(buffer, remaining);
        if (-1 == count)
        {
            break;
        }

        remaining -= count;
        buffer += count;

#if defined(TARGET_NATIVE)
        /*
         * SimpleTimer requires simulation time to progress
         * otherwise this loop will block until a packet was received.
         * TODO This is a workaround which needs to be discussed further.
         */
        Board::getInstance().stepTime();
#endif
        if (readTimer.isTimeout())
        {
            timeOver = true;
        }
    }


    if (nullptr != readCount)
    {
        * readCount = length - remaining;
    }

    return remaining ? false : true;
}

const String& CustomRosTransportTcp::getProtocolName() const
{
    return gProtocolName;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
