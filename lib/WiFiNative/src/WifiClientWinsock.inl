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
 *  @brief WifiClient class for ArduinoNative (Winwsock 2 variant)
 *  @author Norbert Schulz <github@schulznorbert.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <map>

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#include <unistd.h>

#include <WiFiClient.h>
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
 * WifiClient instance to socket mapping.
 * Local definition as stored type is OS dependend, and WifiClient is generic.
 */
static std::map<const WiFiClient*, SOCKET> gSockets;

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
    auto iterSock    = gSockets.find(this);

    if (gSockets.end() != iterSock)
    {
        isConnected = (iterSock->second != INVALID_SOCKET);
    }

    return isConnected;
}

uint8_t WiFiClient::connect(const IPAddress& addr, uint16_t port)
{
    uint8_t retval       = 0U;
    SOCKET  socketHandle = INVALID_SOCKET;

    if (0U != connected())
    {
        /* Connect is called on an already connected client.
         * Handle it as a re-connect by closing the former connection.
         */
        stop();
    }

    socketHandle = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (INVALID_SOCKET != socketHandle)
    {
        struct sockaddr_in serverAddr;

        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family      = AF_INET;
        serverAddr.sin_addr.s_addr = htonl(addr);
        serverAddr.sin_port        = htons(port);

        if (0 == ::connect(socketHandle, reinterpret_cast<struct sockaddr*>(&serverAddr), sizeof(serverAddr)))
        {
            DWORD nodelay = TRUE;

            if (SOCKET_ERROR == setsockopt(socketHandle, IPPROTO_TCP, TCP_NODELAY,
                                           reinterpret_cast<const char*>(&nodelay), sizeof(nodelay)))
            {
                LOG_ERROR("setsockopt error %ld\n", WSAGetLastError());
            }
            else
            {
                u_long mode = 0U;
                int    rt   = ioctlsocket(socketHandle, FIONBIO, &mode); /* Enable non-blocking IO. */

                if (rt != NO_ERROR)
                {
                    LOG_ERROR("ioctlsocket error: %ld\n", rt);
                }
                else
                {
                    retval = 1U; /* success*/
                }
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
            gSockets[this] = socketHandle; /* Store open socket for this class. */
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
        auto iterSock = gSockets.find(this);

        if (gSockets.end() != iterSock)
        {
            if (SOCKET_ERROR == closesocket(iterSock->second))
            {
                LOG_ERROR("closesocket error %d", WSAGetLastError());
            }
            (void)gSockets.erase(iterSock);
        }
    }
}

size_t WiFiClient::write(const uint8_t* buffer, size_t size)
{
    size_t   remaining = size;
    uint32_t retries   = 0;

    if ((0U < size) && (0U != connected()))
    {
        SOCKET socket = gSockets[this];

        while ((0U < remaining) && (retries < SOCK_WRITE_RETRY))
        {
            int written = ::send(socket, reinterpret_cast<const char*>(buffer), remaining, 0);
            if (SOCKET_ERROR == written)
            {
                int lastError = WSAGetLastError();
                if (WSAEWOULDBLOCK == lastError)
                {
                    usleep(SOCK_WRITE_TMO_US);
                    written = 0; /* Not an error, just retry indication. */
                }
                else
                {
                    LOG_ERROR("send error %d", lastError);
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
        SOCKET socket    = gSockets[this];
        int    selectRet = 0;
        fd_set socketSet;

        timeval timeout{.tv_sec = 0, .tv_usec = 10 * 1000}; /* 10 milliseconds. */
        FD_ZERO(&socketSet);
        FD_SET(socket, &socketSet);
        selectRet = select(1, &socketSet, nullptr, nullptr, &timeout);

        if (SOCKET_ERROR == selectRet)
        {
            LOG_ERROR("select error %d ", WSAGetLastError());
        }
        else if (1 == selectRet) /* read signaled */
        {
            ssize_t result = ::recv(socket, reinterpret_cast<char*>(buffer), size, 0);
            if (-1 != result)
            {
                retval = result; /* Success! */
            }
            else
            {
                LOG_ERROR("%s:%s", "recv", strerror(errno));
            }
        }
        else /* no data */
        {
            retval = 0;
        }
    }

    return retval;
}
