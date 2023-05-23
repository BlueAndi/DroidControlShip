/* MIT License
 *
 * Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Socket Client for Inter-Process Communication
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "SocketClient.h"
#include <stdio.h>
#include <queue>

#ifdef _WIN32
#undef UNICODE
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <arpa/inet.h>  /* definition of inet_ntoa */
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
#include <cstring>  /* definition of memset for tests. */
#endif

/******************************************************************************
 * Macros
 *****************************************************************************/

#ifndef _WIN32
#define INVALID_SOCKET (SOCKET)(~0)
#define SOCKET_ERROR   (-1)
#endif

/******************************************************************************
 * Types and classes
 *****************************************************************************/

#ifndef _WIN32
typedef unsigned int UINT_PTR;
typedef UINT_PTR     SOCKET;
#endif

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** SocketClient Members. PIMPL Idiom. */
struct SocketClient::SocketClientImpl
{
    /**
     * File Descriptor of the Client Socket.
     */
    SOCKET m_serverSocket;

    /**
     * Queue for the received bytes.
     */
    std::queue<uint8_t> m_rcvQueue;

    /**
     * Socket server address information.
     */
    struct addrinfo* m_addrInfo;

    /**
     * Construct an SocketClientImpl instance.
     */
    SocketClientImpl() : m_serverSocket(INVALID_SOCKET), m_rcvQueue(), m_addrInfo(nullptr)
    {
    }
};

/******************************************************************************
 * Public Methods
 *****************************************************************************/

SocketClient::SocketClient() : Stream(), m_members(new SocketClientImpl)
{
}

SocketClient::~SocketClient()
{
    /* Sockets are closed before deleting m_members. */
    closeListeningSocket();

    if (nullptr != m_members)
    {
        delete m_members;
    }
}

bool SocketClient::init(const char* serverAddress, const char* portNumber)
{
    int             result    = 0;
    bool            isSuccess = false;
    struct addrinfo hints;

    if (nullptr != m_members)
    {
        memset(&hints, 0, sizeof(struct addrinfo));

        hints.ai_family   = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;

#ifdef _WIN32
        WSADATA wsaData;

        /* Initialize Winsock */
        result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (0 != result)
        {
            printf("WSAStartup failed with error: %d\n", result);
        }

#endif

        if (0 == result)
        {
            /* Resolve the server address and port */
            result = getaddrinfo(serverAddress, portNumber, &hints, &m_members->m_addrInfo);
            if (0 != result)
            {
                printf("getaddrinfo failed with error: %d\n", result);
                closeListeningSocket();
            }
            else
            {
                isSuccess = true;
            }
        }
    }

    return isSuccess;
}

void SocketClient::print(const char str[])
{
    /* Not implemented*/
    (void)str;
}

void SocketClient::print(uint8_t value)
{
    /* Not implemented*/
    (void)value;
}

void SocketClient::print(uint16_t value)
{
    /* Not implemented*/
    (void)value;
}

void SocketClient::print(uint32_t value)
{
    /* Not implemented*/
    (void)value;
}

void SocketClient::print(int8_t value)
{
    /* Not implemented*/
    (void)value;
}

void SocketClient::print(int16_t value)
{
    /* Not implemented*/
    (void)value;
}

void SocketClient::print(int32_t value)
{
    /* Not implemented*/
    (void)value;
}

void SocketClient::println(const char str[])
{
    /* Not implemented*/
    (void)str;
}

void SocketClient::println(uint8_t value)
{
    /* Not implemented*/
    (void)value;
}

void SocketClient::println(uint16_t value)
{
    /* Not implemented*/
    (void)value;
}

void SocketClient::println(uint32_t value)
{
    /* Not implemented*/
    (void)value;
}

void SocketClient::println(int8_t value)
{
    /* Not implemented*/
    (void)value;
}

void SocketClient::println(int16_t value)
{
    /* Not implemented*/
    (void)value;
}

void SocketClient::println(int32_t value)
{
    /* Not implemented*/
    (void)value;
}

size_t SocketClient::write(const uint8_t* buffer, size_t length)
{
    size_t bytesSent = 0;

    if (nullptr != m_members)
    {
        /* Echo the buffer back to the sender */
        if (INVALID_SOCKET != m_members->m_serverSocket)
        {
            int result = send(m_members->m_serverSocket, reinterpret_cast<const char*>(buffer), length, 0);
            if (SOCKET_ERROR == result)
            {
                printf("send failed\n");
                /* Error on the socket. Client is now invalid. */
                m_members->m_serverSocket = INVALID_SOCKET;
            }
            else
            {
                bytesSent = result;
            }
        }
    }
    return bytesSent;
}

int SocketClient::available() const
{
    return (nullptr != m_members) ? m_members->m_rcvQueue.size() : 0;
}

size_t SocketClient::readBytes(uint8_t* buffer, size_t length)
{
    size_t count = 0;

    for (count = 0; count < length; count++)
    {
        uint8_t byte;
        if (false == getByte(byte))
        {
            break;
        }
        else
        {
            buffer[count] = byte;
        }
    }

    return count;
}

bool SocketClient::process()
{
    bool isConnected = false;

    if (nullptr != m_members)
    {
        if (INVALID_SOCKET != m_members->m_serverSocket)
        {
            fd_set         readFDS, writeFDS, exceptFDS;
            int            result;
            struct timeval timeout;

            timeout.tv_sec  = 0;
            timeout.tv_usec = 10;

            FD_ZERO(&readFDS);
            FD_ZERO(&writeFDS);
            FD_ZERO(&exceptFDS);

            FD_SET(m_members->m_serverSocket, &readFDS);

            result = select(m_members->m_serverSocket + 1, &readFDS, &writeFDS, &exceptFDS, &timeout);

            if (0 < result)
            {
                /* Server Ready to read */
                if (FD_ISSET(m_members->m_serverSocket, &readFDS))
                {
                    const size_t bufferLength = UINT8_MAX;
                    char         recvbuf[bufferLength];
                    int          result = recv(m_members->m_serverSocket, recvbuf, bufferLength, 0);

                    if (0 < result)
                    {
                        for (uint8_t idx = 0; idx < result; idx++)
                        {
                            m_members->m_rcvQueue.push(static_cast<uint8_t>(recvbuf[idx]));
                        }
                    }
                    else
                    {
                        /* Client disconnected or error on the socket. */
                        closeListeningSocket();
                        m_members->m_serverSocket = INVALID_SOCKET;
                    }
                }
            }

            isConnected = true;
        }
        else
        {
            isConnected = connectSocket();
        }
    }

    return isConnected;
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool SocketClient::connectSocket()
{
    bool             isConnected = false;
    struct addrinfo* itr         = nullptr;

    if ((nullptr == m_members) || (nullptr == m_members->m_addrInfo))
    {
        /* No address configuration found. */
        closeListeningSocket();
    }
    else if (INVALID_SOCKET != m_members->m_serverSocket)
    {
        /* Already connected to socket server. */
        isConnected = true;
    }
    else
    {
        /* Attempt to connect to an address until one succeeds. */
        for (itr = m_members->m_addrInfo; itr != NULL; itr = itr->ai_next)
        {
            /* Create a SOCKET for connecting to server */
            m_members->m_serverSocket = socket(itr->ai_family, itr->ai_socktype, itr->ai_protocol);
            if (INVALID_SOCKET == m_members->m_serverSocket)
            {
                printf("Socket creation failed\n");
                break;
            }

            /* Connect to server. */
            if (SOCKET_ERROR == connect(m_members->m_serverSocket, itr->ai_addr, (int)itr->ai_addrlen))
            {
                m_members->m_serverSocket = INVALID_SOCKET;
                continue;
            }
            break;
        }

        /* Check validity of socket. */
        if (INVALID_SOCKET != m_members->m_serverSocket)
        {
            isConnected = true;
        }
    }

    return isConnected;
}

void SocketClient::closeListeningSocket()
{
    if (nullptr != m_members)
    {
        /* Close the listening socket. */
        if (INVALID_SOCKET != m_members->m_serverSocket)
        {
#ifdef _WIN32
            closesocket(m_members->m_serverSocket);
#else
            close(m_members->m_serverSocket);
#endif
        }

        if (nullptr != m_members->m_addrInfo)
        {
            freeaddrinfo(m_members->m_addrInfo);
        }
    }

#ifdef _WIN32
    /* Terminate the use of the Winsock 2 DLL. */
    WSACleanup();
#endif
}

bool SocketClient::getByte(uint8_t& byte)
{
    if (nullptr != m_members)
    {
        if (false == m_members->m_rcvQueue.empty())
        {
            byte = m_members->m_rcvQueue.front();
            m_members->m_rcvQueue.pop();
            return true;
        }
    }
    return false;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
