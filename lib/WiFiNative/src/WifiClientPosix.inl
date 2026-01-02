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
 *  @file
 * @brief WifiClient class for ArduinoNative.
 *  @author Norbert Schulz <github@schulznorbert.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <unistd.h>
#include <sys/socket.h>
#include <sys/fcntl.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <poll.h>

#include <map>

#include "WiFiClient.h"
#include <Logging.h>
/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/**
 * WifiClient instance to pollfd (socket data) mapping.
 * Local definition as stored type is OS dependend, and WifiClient is generic.
 */
static std::map<const WiFiClient*, struct pollfd> gConnections;

/**
 * The invalid socket value.
 */
static const int SOCK_INVAL = -1;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

WiFiClient::~WiFiClient()
{
    stop(); /* Cleanup eventually open socket. */
}

uint8_t WiFiClient::connected() const
{
    bool isConnected = false;
    auto iterSock    = gConnections.find(this);

    if (gConnections.end() != iterSock)
    {
        isConnected = (iterSock->second.fd != SOCK_INVAL);
    }

    return isConnected;
}

uint8_t WiFiClient::connect(const IPAddress& addr, uint16_t port)
{
    uint8_t retval   = 0U;
    int     socketFd = SOCK_INVAL;

    if (0U != connected())
    {
        /* Connect is called on an already connected client.
         * Handle it as re-connect by closing the former socket connection.
         */
        stop();
    }

    socketFd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (SOCK_INVAL != socketFd)
    {
        struct sockaddr_in serverAddr;

        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family      = AF_INET;
        serverAddr.sin_addr.s_addr = htonl(addr);
        serverAddr.sin_port        = htons(port);

        if (0 == ::connect(socketFd, reinterpret_cast<struct sockaddr*>(&serverAddr), sizeof(serverAddr)))
        {
            const int enableOptVal = 1;

            if (-1 == ::setsockopt(socketFd, IPPROTO_TCP, TCP_NODELAY, &enableOptVal, sizeof(enableOptVal)))
            {
                LOG_ERROR("%s:%s", "setsockopt", strerror(errno));
            }
            else if (-1 == ::fcntl(socketFd, F_SETFL, ::fcntl(socketFd, F_GETFL) | O_NONBLOCK))
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
            stop(); /* One of the system calls above failed. */
        }
        else
        {
            /* Store poll data and listen to input data event. */
            gConnections[this] = {.fd = socketFd, .events = POLLIN, .revents = 0};
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
        auto iterSock = gConnections.find(this);

        if (gConnections.end() != iterSock)
        {
            if (0 != ::close(iterSock->second.fd))
            {
                LOG_ERROR("%s:%s", "close", strerror(errno));
            }
            (void)gConnections.erase(iterSock);
        }
    }
}

size_t WiFiClient::write(const uint8_t* buffer, size_t size)
{
    size_t   remaining = size;
    uint32_t retries   = 0;

    if ((0U < size) && (0U != connected()))
    {
        auto pollFd = gConnections[this];

        while ((0U < remaining) && (retries < SOCK_WRITE_RETRY))
        {
            ssize_t written = ::send(pollFd.fd, buffer, remaining, MSG_DONTWAIT);
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
    int retval = -2;

    if (0U != connected())
    {
        auto pollFd = gConnections[this];

        if (-1 != ::poll(&pollFd, 1, 10))
        {
            if (0 != (POLLIN & pollFd.revents))
            {
                ssize_t result = ::recv(pollFd.fd, buffer, size, 0);
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
