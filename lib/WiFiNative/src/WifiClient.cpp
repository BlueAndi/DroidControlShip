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
    uint8_t retval = 0;

    if (connected())
    {
        stop();
    }

    if (SOCK_INVAL != (m_poll.fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)))
    {
        struct sockaddr_in serverAddr;

        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = htonl(addr);
        serverAddr.sin_port = htons(port);

        if (0 == ::connect(
                        m_poll.fd, 
                        reinterpret_cast<struct sockaddr *>(&serverAddr),
                        sizeof(serverAddr) ))
        {
            const int one = 1;
            
            ::setsockopt(m_poll.fd, SOL_TCP, TCP_NODELAY, &one, sizeof(one));
            ::fcntl(m_poll.fd, F_SETFL, O_NONBLOCK);

            retval = 1; /* success*/
        }
        else
        {
            stop();
        }
    }

    return retval;
}

void WiFiClient::stop()
{
    if (connected())
    {
        ::close(m_poll.fd);
        m_poll.fd = SOCK_INVAL;
    }
}

size_t WiFiClient::write(const uint8_t* buffer, size_t size)
{
    size_t remaining = size;
 
    if ((0 < size) && connected())
    {
        while (0 < remaining) {
            ssize_t written = ::send(m_poll.fd, buffer, remaining, 0);
    
            if (0 > written)   /* error */
            {
                break;
            }

            remaining -= written;
            buffer += written;
        }
    }

    return size - remaining;
}

int WiFiClient::read(uint8_t* buffer, size_t size)
{
    int retval = -1;

    if (connected())
    {
        if (-1 != ::poll(&m_poll, 1, 10))
        {
            if (0 != (POLLIN & m_poll.revents))
            {
                ssize_t  result = ::recv(m_poll.fd, buffer, size, 0);
                if (-1 != result)
                {
                    retval = result;  /* Success! */
                }
                else
                {
                    perror("recv");
                }
            }
            else
            {
                retval = 0;
            }
        }
        else
        {
            perror("poll");
        }
    }

    return retval;
}