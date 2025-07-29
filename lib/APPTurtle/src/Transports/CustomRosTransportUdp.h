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
 * @brief  Custom Micro-ROS transport using UDP over Arduino WiFiUdp.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef CUSTOM_ROS_TRANSPORT_UDP_H
#define CUSTOM_ROS_TRANSPORT_UDP_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "CustomRosTransportBase.h"

#include <WiFiUdp.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** 
 * Map this transport to the class name used in MicroRosClient.
 * The used transport is a compile time decision and this typedef
 * avoids the use of ifdef's.
 */
typedef class CustomRosTransportUdp CustomRosTransport;

/**
 * Micro-ROS custom transport adaption.
 *
 * The static functions are used as these are called from C-language
 */
class CustomRosTransportUdp : public CustomRosTransportBase
{
public:
    /**
     * Constructs a custom Micro-ROS transport.
     */
    CustomRosTransportUdp() : CustomRosTransportBase(), m_udpClient()
    {
    }

    /**
     * Destroys custom Micro-ROS transport.
     *
     */
    ~CustomRosTransportUdp() final
    {
    }

    /** 
     * Get protocol name used by this transport.
     * @return Protocol name
     */
    const String& getProtocolName() const final
    {
        return m_protocolName;
    }

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

    WiFiUDP               m_udpClient; /**< UDP client */
    static const String   m_protocolName;  /**< This protocol name. */

};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* CUSTOM_ROS_TRANSPORT_UDP_H */
/** @} */
