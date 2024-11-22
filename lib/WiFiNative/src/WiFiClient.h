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
 * @brief  Wifi Client for Native platform. Emulation of Arduino WiFiClient.
 * @author Norbert Schulz <github@schulznorbert.de>
 *
 * @addtogroup HALSim
 *
 * @{
 */
#ifndef WIFI_CLIENT_H
#define WIFI_CLIENT_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <Arduino.h>
#include <IPAddress.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * WiFiClient client class compatible with Arduino. 
 * 
 * Only a subset of API functions is supported, based on the actual demand.
 */
class WiFiClient
{
    public:

    /**
     * Default constructor.
     */
    WiFiClient()
    {
    }

    /**
     * Destructor.
     */
    virtual ~WiFiClient();

    /**
     * Establish a connection.
     * 
     * @param[in] addr Port number.
     * @param[in] port Port number.
     * @return If successful returns 1, otherwise 0.
     */
    uint8_t connect(const IPAddress& addr, uint16_t port);

    /**
     * Indicate if client is connected.
     * 
     * @return If successful returns 1, otherwise 0.
     */
    uint8_t connected() const;

    /**
     * Stop the WiF Client.
     */
    void stop();

    /**
     * Write bytes to stream.
     *
     * The write uses non blocking socket mode, but retries on a
     * EWOULDBLOCK result. The constants SOCK_WRITE_RETRY and
     * SOCK_WRITE_TMO_US define how often to retry and the 
     * time delay in between.
     * 
     * @param[in] buffer Byte Array to send.
     * @param[in] size Length of Buffer.
     * 
     * @returns Number of bytes written or 0 on error.
     */
    size_t write(const uint8_t* buffer, size_t size);


    /**
     * Check if there are available bytes in the Stream.
     * 
     * @returns Number of available bytes.
     */
    int available() const;

    /**
     * Read bytes from the incoming packet into a buffer.
     *
     * @param[out] buffer Buffer to read into.
     * @param[in] size Length of the buffer.
     *
     * @return Number of bytes read or -1 on error.
     */
    int read(uint8_t* buffer, size_t size);

private:
    static const uint32_t SOCK_WRITE_RETRY = 4U;    /**< How often to retry sending.  */
    static const uint32_t SOCK_WRITE_TMO_US = 250U; /**< Delay between write attemps. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* WIFI_CLIENT_H */
/** @} */
