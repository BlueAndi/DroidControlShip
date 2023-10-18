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
 * 
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "Stream.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Socket Client for Inter-Process Communication.
 */
class SocketClient : public Stream
{
public:
    /**
     * Construct a SocketClient.
     */
    SocketClient();

    /**
     * Destruct the SocketClient.
     */
    virtual ~SocketClient();

    /**
     * Initialize the SocketClient.
     * @param[in] serverAddress Address of Server.
     * @param[in] portNumber Port number to set the Socket to.
     * @returns true if server has been succesfully set-up.
     */
    bool init(const char* serverAddress, const char* portNumber);

    /**
     * Print argument to the Output Stream.
     * @param[in] str Argument to print.
     */
    void print(const char str[]) final;

    /**
     * Print argument to the Output Stream.
     * @param[in] value Argument to print.
     */
    void print(uint8_t value) final;

    /**
     * Print argument to the Output Stream.
     * @param[in] value Argument to print.
     */
    void print(uint16_t value) final;

    /**
     * Print argument to the Output Stream.
     * @param[in] value Argument to print.
     */
    void print(uint32_t value) final;

    /**
     * Print argument to the Output Stream.
     * @param[in] value Argument to print.
     */
    void print(int8_t value) final;

    /**
     * Print argument to the Output Stream.
     * @param[in] value Argument to print.
     */
    void print(int16_t value) final;

    /**
     * Print argument to the Output Stream.
     * @param[in] value Argument to print.
     */
    void print(int32_t value) final;

    /**
     * Print argument to the Output Stream.
     * Appends Carriage Return at the end of the argument.
     * @param[in] str Argument to print.
     */
    void println(const char str[]) final;

    /**
     * Print argument to the Output Stream.
     * Appends Carriage Return at the end of the argument.
     * @param[in] value Argument to print.
     */
    void println(uint8_t value) final;

    /**
     * Print argument to the Output Stream.
     * Appends Carriage Return at the end of the argument.
     * @param[in] value Argument to print.
     */
    void println(uint16_t value) final;

    /**
     * Print argument to the Output Stream.
     * Appends Carriage Return at the end of the argument.
     * @param[in] value Argument to print.
     */
    void println(uint32_t value) final;

    /**
     * Print argument to the Output Stream.
     * Appends Carriage Return at the end of the argument.
     * @param[in] value Argument to print.
     */
    void println(int8_t value) final;

    /**
     * Print argument to the Output Stream.
     * Appends Carriage Return at the end of the argument.
     * @param[in] value Argument to print.
     */
    void println(int16_t value) final;

    /**
     * Print argument to the Output Stream.
     * Appends Carriage Return at the end of the argument.
     * @param[in] value Argument to print.
     */
    void println(int32_t value) final;

    /**
     * Send a message to the socket.
     * @param[in] buffer Byte buffer to send
     * @param[in] length Number of bytes to send
     * @returns Number of bytes written
     */
    size_t write(const uint8_t* buffer, size_t length) final;

    /**
     * Check if any data has been received.
     * @returns number of available bytes.
     */
    int available() const final;

    /**
     * Read bytes into a buffer.
     * @param[in] buffer Array to write bytes to.
     * @param[in] length number of bytes to be read.
     * @returns Number of bytes read from Stream.
     */
    size_t readBytes(uint8_t* buffer, size_t length) final;

    /**
     * Process the receiving of messages and client connections.
     * 
     * @returns true if connection is active. Otherwise, false.
     */
    bool process();

private:
    /** Maximum amount of available bytes allowed before issuing a warning. */
    static const uint16_t CANARY_WARNING_LIMIT = 500U;

    /** Struct for Implementation of PIMPL Idiom. */
    struct SocketClientImpl;

    /** SocketClient Members. PIMPL Idiom. */
    SocketClientImpl* m_members;

    /**
     * Connect to the socket server.
     * 
     * @returns true if successfully connected to socket server. Otherwise, false.
     */
    bool connectSocket();

    /**
     * Close the listening socket connection.
     */
    void closeListeningSocket();

    /**
     * Get a Byte from the receiving buffer, if any.
     * @param[out] byte buffer to write the byte to.
     * @returns If a received byte has been succesfully written to the buffer, returns true. Otherwise, false.
     */
    bool getByte(uint8_t& byte);

private:
    /* Not allowed. */
    SocketClient(const SocketClient& srv);
    SocketClient& operator=(const SocketClient& srv);
};

#endif /* SOCKET_SERVER_H_ */
/** @} */
