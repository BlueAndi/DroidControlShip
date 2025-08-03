/* MIT License
 *
 * Copyright (c) 2023 - 2025 Andreas Merkle <web@blue-andi.de>
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
 * @brief  UDP Client for Native platform. Emulation of WiFiUdp.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALSim
 *
 * @{
 */
#ifndef WIFI_UDP_H
#define WIFI_UDP_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <Arduino.h>
#include <Stream.h>
#include <IPAddress.h>
#include <netinet/in.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * UDP client class.
 */
class WiFiUDP : public Stream
{
public:
    /**
     * Default constructor.
     */
    WiFiUDP() :
        Stream(),
        m_socket(-1),
        m_servaddr(),
        m_outBuffer{0},
        m_outBufferLength(0),
        m_inBuffer{0},
        m_inBufferLength(0)
    {
    }

    /**
     * Default Destructor.
     */
    virtual ~WiFiUDP()
    {
    }

    /**
     * Begin UDP client.
     * @param[in] port Port number.
     * @return If successful returns 1, otherwise 0.
     */
    uint8_t begin(uint16_t port);

    /**
     * Stop the UDP client.
     */
    void stop();

    /**
     * Begin an UDP packet.
     *
     * @param[in] ip IP address to connect to.
     * @param[in] port Port number.
     *
     * @return If successful returns 1, otherwise 0.
     */
    int beginPacket(IPAddress ip, uint16_t port);

    /**
     * Write bytes to stream.
     * @param[in] buffer Byte Array to send.
     * @param[in] length Length of Buffer.
     * @returns Number of bytes written
     */
    size_t write(const uint8_t* buffer, size_t length) final;

    /**
     * End the UDP packet.
     *
     * @return If successful returns 1, otherwise 0.
     */
    int endPacket();

    /**
     * Get the error code for the write operation.
     *
     * @return Error code.
     */
    int getWriteError();

    /**
     * Flush the UDP client.
     */
    void flush();

    /**
     * Parse the incoming packet.
     *
     * @return Packet size.
     */
    int parsePacket();

    /**
     * Check if there are available bytes in the Stream.
     * @returns Number of available bytes.
     */
    int available() const final;

    /**
     * Read bytes from the incoming packet into a buffer.
     *
     * @param[out] buffer Buffer to read into.
     * @param[in] size Size of the buffer.
     *
     * @return Number of bytes read.
     */
    int read(uint8_t* buffer, size_t size);

private:
    static const uint32_t WIFIUDP_MAXLINE = 1024U;      /**< Internal buffer size */
    int                   m_socket;                     /**< Socket file descriptor. */
    struct sockaddr_in    m_servaddr;                   /**< Server address. */
    uint8_t               m_outBuffer[WIFIUDP_MAXLINE]; /**< Buffer for outgoing packets. */
    size_t                m_outBufferLength;            /**< Length of the output buffer. */
    uint8_t               m_inBuffer[WIFIUDP_MAXLINE];  /**< Buffer for incoming packets. */
    size_t                m_inBufferLength;             /**< Length of the input buffer. */

private:
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
     * Read bytes into a buffer.
     * @param[in] buffer Array to write bytes to.
     * @param[in] length number of bytes to be read.
     * @returns Number of bytes read from Stream.
     */
    size_t readBytes(uint8_t* buffer, size_t length) final;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* WIFI_UDP_H */
/** @} */
