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
 * @brief  Custom Micro-ROS transport.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef CUSTOM_ROS_TRANSPORT_H
#define CUSTOM_ROS_TRANSPORT_H

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
 * Micro-ROS agent locator struct.
 * Constains the IP address and port of the agent.
 */
struct micro_ros_agent_locator
{
    IPAddress address; /**< IP address of the agent */
    int       port;    /**< Port of the agent */
};

/**
 * Class like definition of ROS2 custom transport functions.
 *
 * Only static functions used as these are called from C-language
 */
class CustomRosTransport
{
public:
    /**
     * Open and initialize the custom transport.
     * https://micro.ros.org/docs/tutorials/advanced/create_custom_transports/
     *
     * @param[in] transport The arguments passed through uxr_init_custom_transport.
     *
     * @return A boolean indicating if the opening was successful.
     */
    static bool open(uxrCustomTransport* transport);

    /**
     * Close the custom transport.
     * https://micro.ros.org/docs/tutorials/advanced/create_custom_transports/
     *
     * @param[in] transport The arguments passed through uxr_init_custom_transport.
     *
     * @return A boolean indicating if the closing was successful.
     */
    static bool close(uxrCustomTransport* transport);

    /**
     * Write data to the custom transport.
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
     * Read data from the custom transport.
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
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* CUSTOM_ROS_TRANSPORT_H */
/** @} */
