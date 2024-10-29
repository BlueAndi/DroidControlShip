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
 * @brief Custom Transport class with C-language interface for microros.
 * @author Norbert Schulz <github@schulznorbert.de>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef CUSTOM_ROS_TRANSPORT_BASE_H
#define CUSTOM_ROS_TRANSPORT_BASE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <IPAddress.h>
#include <rmw_microros/rmw_microros.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Micro-ROS custom transport adaption.
 *
 * The static functions are needed as these are called from C-language.
 */
class CustomRosTransportBase
{
public:
    /**
     * Constructs a custom Micro-ROS transport.
     */
    CustomRosTransportBase() : m_address(), m_port(DEFAULT_PORT)
    {
    }

    /**
     * Destroys custom Micro-ROS transport.
     *
     */
    ~CustomRosTransportBase()
    {
    }

    /**
     * Initialize custom ROS transport with agent address and port.
     *
     * @param[in] addr  Micro-ROS agent IP-address
     * @param[in] port  Micro-ROS agent port
     */
    void init(const IPAddress& addr, uint16_t port)
    {
        m_address = addr;
        m_port    = port;
    }

    /**
     * Get IP-address of Micro-ROS agent.
     *
     * @return IP-address
     */
    const IPAddress& getIPAddress() const
    {
        return m_address;
    }

    /**
     * Get IP-address of Micro-ROS agent as string
     *
     * @return IP-address as string
     */
    const String getIPAddressAsStr() const
    {
        return m_address.toString();
    }

    /**
     * Get port of Micro-ROS agent.
     *
     * @return Port
     */
    uint16_t getPort() const
    {
        return m_port;
    }

    /**
     * Open and initialize the custom transport (C-Entry Point).
     * https://micro.ros.org/docs/tutorials/advanced/create_custom_transports/
     *
     * @param[in] transport The arguments passed through uxr_init_custom_transport.
     *
     * @return A boolean indicating if the opening was successful.
     */
    static bool open(uxrCustomTransport* transport);

    /**
     * Close the custom transport (C-Entry Point).
     * https://micro.ros.org/docs/tutorials/advanced/create_custom_transports/
     *
     * @param[in] transport The arguments passed through uxr_init_custom_transport.
     *
     * @return A boolean indicating if the closing was successful.
     */
    static bool close(uxrCustomTransport* transport);

    /**
     * Write data to the custom transport (C-Entry Point).
     * https://micro.ros.org/docs/tutorials/advanced/create_custom_transports/
     *
     * @param[in]  transport The arguments passed through uxr_init_custom_transport.
     * @param[in]  buffer The buffer to write.
     * @param[in]  size The size of the buffer.
     * @param[out] errorCode The error code.
     *
     * @return The number of bytes written.
     */
    static size_t write(uxrCustomTransport* transport, const uint8_t* buffer, size_t size, uint8_t* errorCode);

    /**
     * Read data from the custom transport (C-Entry Point).
     * https://micro.ros.org/docs/tutorials/advanced/create_custom_transports/
     *
     * @param[in]  transport The arguments passed through uxr_init_custom_transport.
     * @param[out] buffer The buffer to read into.
     * @param[in]  size The size of the buffer.
     * @param[in]  timeout The timeout in milliseconds.
     * @param[out] errorCode The error code.
     *
     * @return The number of bytes read.
     */
    static size_t read(uxrCustomTransport* transport, uint8_t* buffer, size_t size, int timeout, uint8_t* errorCode);

    /** 
     * Get protocol name used by this trandport.
     * @return protocol name
     */
    virtual const String& getProtocolName() const = 0;

protected:
    /**
     * Open and initialize the custom transport.
     *
     * @return A boolean indicating if the opening was successful.
     */
    virtual bool open(void) = 0;

    /**
     * Close the custom transport.
     *
     * @return A boolean indicating if the closing was successful.
     */
    virtual bool close(void) = 0;

    /**
     * Write data to the custom transport.
     *
     * @param[in]  buffer The buffer to write.
     * @param[in]  size The size of the buffer.
     * @param[out] errorCode The error code.
     *
     * @return The number of bytes written.
     */
    virtual size_t write(const uint8_t* buffer, size_t size, uint8_t* errorCode) = 0;

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
    virtual size_t read(uint8_t* buffer, size_t size, int timeout, uint8_t* errorCode) = 0;

    /**
     * Default Micro-ROS agent port.
     */
    static const int DEFAULT_PORT = 8888;

    IPAddress m_address;   /**< IP address of the Micro-ROS agent. */
    uint16_t  m_port;      /**< Port of the Micro-ROS agent. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* CUSTOM_ROS_TRANSPORT_BASE_H */
/** @} */
