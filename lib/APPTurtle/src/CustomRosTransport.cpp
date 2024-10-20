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
 * @brief  Custom Micro-ROS transport over UDP.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "CustomRosTransport.h"
#include <Util.h>
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

/******************************************************************************
 * Public Methods
 *****************************************************************************/

bool CustomRosTransport::open(uxrCustomTransport* transport)
{
    bool isOpen = false;

    if (nullptr == transport)
    {
        LOG_ERROR("Transport is nullptr.");
    }
    else
    {
        const CustomRosTransport* tthis  = reinterpret_cast<CustomRosTransport*>(transport->args);
        const int                 UDP_OK = 1;

        if (UDP_OK != udpClient.begin(tthis->m_port))
        {
            LOG_ERROR("UDP begin error");
        }
        else
        {
            isOpen = true;
        }
    }

    return isOpen;
}

bool CustomRosTransport::close(uxrCustomTransport* transport)
{
    UTIL_NOT_USED(transport);

    udpClient.stop();

    return true;
}

size_t CustomRosTransport::write(uxrCustomTransport* transport, const uint8_t* buffer, size_t size, uint8_t* errorCode)
{
    size_t sent = 0U;

    if ((nullptr == transport) || (nullptr == buffer) || (0U == size) || (nullptr == errorCode))
    {
        LOG_ERROR("One or more parameters are invalid.");
    }
    else
    {
        const CustomRosTransport* tthis  = static_cast<CustomRosTransport*>(transport->args);
        const int                 UDP_OK = 1;
        int                       ret    = UDP_OK;

        ret = udpClient.beginPacket(tthis->m_address, tthis->m_port);

        if (UDP_OK != ret)
        {
            LOG_ERROR("UDP beginPacket error");
        }
        else
        {
            size_t bytesToWrite = udpClient.write(buffer, size);

            if (bytesToWrite != size)
            {
                LOG_ERROR("UDP write error");
            }
            else
            {
                ret = udpClient.endPacket();

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
            int writeErrorCode = udpClient.getWriteError();
            *errorCode         = writeErrorCode;
            LOG_ERROR("UDP error: %d", writeErrorCode);
        }

        udpClient.flush();
    }

    return sent;
}

size_t CustomRosTransport::read(uxrCustomTransport* transport, uint8_t* buffer, size_t size, int timeout,
                                uint8_t* errorCode)
{
    size_t readBytes = 0U;

    if ((nullptr == transport) || (nullptr == buffer) || (0U == size) || (nullptr == errorCode))
    {
        LOG_ERROR("One or more parameters are invalid.");
    }
    else
    {
        SimpleTimer readTimer;
        readTimer.start(timeout);

        while (false == readTimer.isTimeout())
        {
            if (0 < udpClient.parsePacket())
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

        if (0 == udpClient.available())
        {
            *errorCode = 1;
        }
        else
        {
            readBytes = udpClient.read(buffer, size);
        }
    }

    return readBytes;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/
