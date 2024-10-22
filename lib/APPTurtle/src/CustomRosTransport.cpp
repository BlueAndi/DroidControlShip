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

/**
 * Unwrap pointer to CustomRosTransport from uxrCustomTransport structure.
 * 
 * This is used by the static C-Languge entry points to forward the request
 * to the matching C++ member function.
 *
 * @param[in] transport The custom transport data structure pointer.
 *
 * @return The this pointer to transport owning CustomRosTransport class.
 */
static CustomRosTransport* toThis(const uxrCustomTransport* transport);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

bool CustomRosTransport::open(uxrCustomTransport* transport)
{
    CustomRosTransport * tp = toThis(transport);
    return (nullptr != tp) ? tp->open() : false;
}

bool CustomRosTransport::close(uxrCustomTransport* transport)
{
    CustomRosTransport * tp = toThis(transport);
    return (nullptr != tp) ? tp->close() : false;
}

size_t CustomRosTransport::write(uxrCustomTransport* transport, const uint8_t* buffer, size_t size, uint8_t* errorCode)
{
    CustomRosTransport * tp = toThis(transport);
    return (nullptr != tp) ? tp->write(buffer, size, errorCode) : 0U;
}

size_t CustomRosTransport::read(uxrCustomTransport* transport, uint8_t* buffer, size_t size, int timeout,
                                uint8_t* errorCode)
{
    CustomRosTransport * tp = toThis(transport);
    return (nullptr != tp) ? tp->read(buffer, size, timeout, errorCode) : 0U;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

static CustomRosTransport* toThis(const uxrCustomTransport* transport)
{
    CustomRosTransport* transportClass = nullptr;
    
    if (nullptr == transport)
    {
        LOG_FATAL("Invalid uxrCustomTransport pointer.");
    }
    else
    {
        transportClass = reinterpret_cast<CustomRosTransport*>(transport->args);
    }

    return transportClass;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool CustomRosTransport::open(void)
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

bool CustomRosTransport::close()
{
    m_udpClient.stop();

    return true;
}

size_t CustomRosTransport::write(const uint8_t* buffer, size_t size, uint8_t* errorCode)
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

size_t CustomRosTransport::read(uint8_t* buffer, size_t size, int timeout, uint8_t* errorCode)
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
