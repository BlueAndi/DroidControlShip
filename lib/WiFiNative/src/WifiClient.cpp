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
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <sys/select.h>
#include <netinet/tcp.h>

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

    if (SOCK_INVAL != m_socket)
    {
        stop();
    }

    if (SOCK_INVAL != (m_socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)))
    {
        m_servaddr.sin_family = AF_INET;
        m_servaddr.sin_addr.s_addr = htonl(addr);
        m_servaddr.sin_port = htons(port);

        if (0 == ::connect(
                        m_socket, 
                        reinterpret_cast<struct sockaddr *>(&m_servaddr),
                        sizeof(m_servaddr) ))
        {
            const int one = 1;
            
            ::setsockopt(m_socket, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            ::fcntl(m_socket, F_SETFL, O_NONBLOCK);
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
    if (SOCK_INVAL != m_socket)
    {
        ::shutdown(m_socket, SHUT_RDWR);
        ::close(m_socket);
        m_socket = SOCK_INVAL;
    }
}


size_t WiFiClient::write(const uint8_t* buffer, size_t size)
{
    size_t remaining = size;
 
    if ((0 < size) && (SOCK_INVAL != m_socket))
    {
        while (0 < remaining) {
            ssize_t written = ::send(m_socket, buffer, remaining, 0);
    
            if (0 > written)   /* error */
            {
                break;
                /* TODO: Signal error upwards - how ?*/
            }

            remaining -= written;
            buffer += written;
        }
    }

    return size - remaining;
}

int WiFiClient::available() const
{
    int count = 0;

    if (SOCK_INVAL != m_socket)
    {   
        ioctl(m_socket, FIONREAD, &count);
    }

    return count;
}

int WiFiClient::read(uint8_t* buffer, size_t size)
{
    int retval = -1;


    if (SOCK_INVAL != m_socket)
    {
        fd_set readFdSet;
        FD_ZERO(&readFdSet);
        FD_SET(m_socket, &readFdSet);

        struct timeval tv;
        tv.tv_sec = 100 / 1000;
        tv.tv_usec = (100 % 1000) * 1000;

        int selectRet = select(1, &readFdSet, NULL, NULL, &tv);

        ssize_t  result = ::recv(m_socket, buffer, size, 0);
        if (-1 == result)
        {
            if ((EAGAIN == errno) || (EWOULDBLOCK == errno))
            {
                retval = 0; /* No error, just no data received yet. */
            }
        } 
        else
        {
            retval = result;  /* Success! */
        }
    }

    return retval;
}
