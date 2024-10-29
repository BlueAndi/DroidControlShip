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
 * @brief  Custom Micro-ROS transport using TCP over Arduino WifiClient.
 * @author Norbert Schulz <github@schulznorbert.de>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef CUSTOM_ROS_TRANSPORT_TCP_H
#define CUSTOM_ROS_TRANSPORT_TCP_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "CustomRosTransportBase.h"

#include <WiFiClient.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/
typedef class CustomRosTransportTcp CustomRosTransport;

/**
 * Micro-ROS custom transport adaption for TCP.
 *
 * TCP protocol uses a record format with a 16 bit length field before the
 * payload.
 * 
 *  +----------------+----------------+-----..........------+
 *  |    1 Byte      |    1 Bytes     |    "Lenght" Bytes   |
 *  +----------------+----------------+-----..........------+
 *  | Length [0..7]  | Length [8..15] |      < payload >    |
 *  +----------------+----------------+-----..........------+
 */
class CustomRosTransportTcp : public CustomRosTransportBase
{
public:
    /**
     * Constructs a custom Micro-ROS transport.
     */
    CustomRosTransportTcp() : CustomRosTransportBase(), m_tcpClient()
    {
    }

    /**
     * Destroys custom Micro-ROS transport.
     *
     */
    virtual ~CustomRosTransportTcp() final
    {
    }

    /** 
     * Get protocol name used by this trandport.
     * @return protocol name
     */
    virtual const String& getProtocolName() const final;

private:
    /**
     * Open and initialize the custom transport.
     *
     * @return A boolean indicating if the opening was successful.
     */
    bool open(void) final;

    /**
     * Close the custom transport.
     *
     * @return A boolean indicating if the closing was successful.
     */
    bool close(void) final;

    /**
     * Write data to the custom transport.
     *
     * @param[in]  buffer The buffer to write.
     * @param[in]  size The size of the buffer.
     * @param[out] errorCode The error code.
     *
     * @return The number of bytes written.
     */
    size_t write(const uint8_t* buffer, size_t size, uint8_t* errorCode) final;

    /**
     * Read data from the custom transport.
     *
     * @param[out] buffer The buffer to read into.
     * @param[in]  size The size of the buffer.
     * @param[in]  timeout The timeout in milliseconds.
     * @param[out] errorCode The error code.
     *
     * @return The number of bytes read.
     */
    size_t read(uint8_t* buffer, size_t size, int timeout, uint8_t* errorCode) final;

    /**
     * Try to read a fixed number of bytes.
     * 
     * @param[out] buffer The buffer to read into.
     * @param[in]  size The size of the buffer.
     * @param[in]  readCount return bytes read if not a nullptr.
     * @param[in]  timeout The timeout in milliseconds.
     * 
     * @return true of all requested bytes received.
     */
    bool readFixedLength(uint8_t* buffer, size_t length, size_t * readCount, int timeout);

private:
    WiFiClient   m_tcpClient; /**< TCP client */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* CUSTOM_ROS_TRANSPORT_TCP_H */
/** @} */
