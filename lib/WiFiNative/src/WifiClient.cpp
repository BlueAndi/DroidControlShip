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
 *  @brief WifiClient class for ArduinoNative.
 *  @author Norbert Schulz <github@schulznorbert.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "WiFiClient.h"
#include <Logging.h>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/fcntl.h>
#include <netinet/tcp.h>
#include <netinet/in.h>

/******************************************************************************
 * Macros
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

uint8_t WiFiClient::connect(const IPAddress& addr, uint16_t port)
{
    uint8_t retval = 0U;

    if (0U != connected())
    {
        /* Connect is called on an already connected client.
         * Handle it as re-connect by closing the former socket connection.
         */
        stop();
    }

    if (SOCK_INVAL != (m_poll.fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)))
    {
        struct sockaddr_in serverAddr;

        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family      = AF_INET;
        serverAddr.sin_addr.s_addr = htonl(addr);
        serverAddr.sin_port        = htons(port);

        if (0 == ::connect(m_poll.fd, reinterpret_cast<struct sockaddr*>(&serverAddr), sizeof(serverAddr)))
        {
            const int one = 1;

            if (-1 == ::setsockopt(m_poll.fd, SOL_TCP, TCP_NODELAY, &one, sizeof(one)))
            {
                LOG_ERROR("%s:%s", "setsockopt", strerror(errno));
            }
            else if (-1 == ::fcntl(m_poll.fd, F_SETFL, ::fcntl(m_poll.fd, F_GETFL) | O_NONBLOCK))
            {
                LOG_ERROR("%s:%s", "fcntl", strerror(errno));
            }
            else 
            {
                retval = 1U; /* success*/
            }
        }
        else
        {
            LOG_ERROR("%s:%s", "connect", strerror(errno));
        }

        if (0U == retval)
        {
            stop();  /* One of the system calls above failed. */
        }
    }
    else
    {
        LOG_ERROR("%s:%s", "socket", strerror(errno));
    }

    return retval;
}

void WiFiClient::stop()
{
    if (0U != connected())
    {
        if (0 != ::close(m_poll.fd))
        {
            LOG_ERROR("%s:%s", "close", strerror(errno));
        }

        m_poll.fd = SOCK_INVAL;
    }
}

size_t WiFiClient::write(const uint8_t* buffer, size_t size)
{
    size_t remaining = size;
    uint32_t retries = 0;

    if ((0U < size) && (0U != connected()))
    {
        while ((0U < remaining) && (retries < SOCK_WRITE_RETRY))
        {
            ssize_t written = ::send(m_poll.fd, buffer, remaining, MSG_DONTWAIT);
            if (0 > written)
            {
                if ((EAGAIN == errno) || (EWOULDBLOCK == errno))
                {
                    written = 0; /* Not an error, just retry indication. */
                    usleep(SOCK_WRITE_TMO_US);
                }
                else
                {
                    LOG_ERROR("%s:%s", "send", strerror(errno));
                    break;
                }
            }

            remaining -= written;
            buffer += written;
            
            ++retries;
        }
    }

    return size - remaining;
}

int WiFiClient::read(uint8_t* buffer, size_t size)
{
    int retval = -1;

    if (0U != connected())
    {
        if (-1 != ::poll(&m_poll, 1, 10))
        {
            if (0 != (POLLIN & m_poll.revents))
            {
                ssize_t result = ::recv(m_poll.fd, buffer, size, 0);
                if (-1 != result)
                {
                    retval = result; /* Success! */
                }
                else
                {
                    LOG_ERROR("%s:%s", "recv", strerror(errno));
                }
            }
            else
            {
                retval = 0;
            }
        }
        else
        {
            LOG_ERROR("%s:%s", "poll", strerror(errno));
        }
    }

    return retval;
}