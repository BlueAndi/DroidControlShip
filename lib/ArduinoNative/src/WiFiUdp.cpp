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
 * @brief  UDP Client for Native platform. Emulation of WiFiUDP.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "WiFiUdp.h"
#include <Util.h>
#include <Logging.h>

#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

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

uint8_t WiFiUDP::begin(uint16_t port)
{
    m_socket = socket(AF_INET, SOCK_DGRAM, 0);

    int flags = fcntl(m_socket, F_GETFL, 0);
    if (-1 == fcntl(m_socket, F_SETFL, flags | O_NONBLOCK))
    {
        LOG_ERROR("socket configuration failed %d", errno);
    }

    if (-1 == m_socket)
    {
        LOG_ERROR("socket creation failed %d", errno);
    }
    else
    {
        m_servaddr.sin_family = AF_INET;
        return 1;
    }

    return 0;
}

void WiFiUDP::stop()
{
    if (-1 != m_socket)
    {
        m_outBufferLength = 0;
        m_inBufferLength  = 0;
        close(m_socket);
    }
}

int WiFiUDP::beginPacket(IPAddress ip, uint16_t port) // NOLINT(performance-unnecessary-value-param)
{
    m_servaddr.sin_port        = htons(port);
    m_servaddr.sin_addr.s_addr = ip;
    m_outBufferLength          = 0;

    return 1;
}

size_t WiFiUDP::write(const uint8_t* buffer, size_t length)
{
    size_t bufferSize = sizeof(m_outBuffer);

    /* calculate chunk of buffer that still fits in local buffer */
    m_outBufferLength = std::min(length, bufferSize);

    /* TODO determine if dropping bytes exceeding the buffer limit is okay. Cyclically calling endPacket not needed
     * because this is checked by the caller currently */
    memcpy(m_outBuffer, buffer, m_outBufferLength);

    return m_outBufferLength;
}

int WiFiUDP::endPacket()
{
    int len = sendto(m_socket, reinterpret_cast<const char*>(m_outBuffer), m_outBufferLength, 0,
                     (const struct sockaddr*)&m_servaddr, sizeof(m_servaddr));

    if (-1 == len)
    {
        LOG_ERROR("sendto failed %d", errno);
    }
    else
    {
        return 1;
    }
    return 0;
}

int WiFiUDP::getWriteError()
{
    return (int)errno;
}

void WiFiUDP::flush()
{
}

int WiFiUDP::parsePacket()
{
    struct sockaddr_in senderAddr;
    socklen_t          senderAddrSize = sizeof(senderAddr);
    ssize_t            packetSize     = 0;

    /* read socket non blocking */
    packetSize = recvfrom(m_socket, m_inBuffer, sizeof(m_inBuffer), MSG_DONTWAIT, (struct sockaddr*)&senderAddr,
                          &senderAddrSize);

    /* determine whether the error was a timeout */
    if (-1 == packetSize)
    {
        if ((EAGAIN == errno) || (errno == EWOULDBLOCK))
        {
            return 0;
        }

        LOG_ERROR("recvfrom failed %d", errno);
        return 0;
    }
    else /* receiving successful */
    {
        if (0 < packetSize)
        {
            m_inBufferLength = packetSize;
        }
    }

    return packetSize;
}

int WiFiUDP::available() const
{
    return m_inBufferLength;
}

int WiFiUDP::read(uint8_t* buffer, size_t size)
{
    return readBytes(buffer, size);
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void WiFiUDP::print(const char str[])
{
    /* Not implemented. */
    UTIL_NOT_USED(str);
}

void WiFiUDP::print(uint8_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::print(uint16_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::print(uint32_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::print(int8_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::print(int16_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::print(int32_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(const char str[])
{
    /* Not implemented. */
    UTIL_NOT_USED(str);
}

void WiFiUDP::println(uint8_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(uint16_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(uint32_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(int8_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(int16_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(int32_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

size_t WiFiUDP::readBytes(uint8_t* buffer, size_t length)
{
    if ((0 == m_inBufferLength) || (0 == length))
    {
        return 0;
    }

    size_t bytesToRead = std::min(length, m_inBufferLength);

    if (WIFIUDP_MAXLINE > bytesToRead)
    {
        memcpy(buffer, m_inBuffer, bytesToRead);

        m_inBufferLength -= bytesToRead;
        memmove(m_inBuffer, m_inBuffer + bytesToRead, m_inBufferLength);
    }

    return bytesToRead;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
