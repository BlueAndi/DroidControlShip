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
 * @brief  Device realization
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef DEVICE_H
#define DEVICE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "IDevice.h"
#include "SocketClient.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class provides access to the simulation device. */
class Device : public IDevice
{
public:
    /**
     * Constructs the device adapter.
     */
    Device() : IDevice(), m_retryConnectionCounter(0U), m_socket()
    {
    }

    /**
     * Destroys the device adapter.
     */
    virtual ~Device()
    {
    }

    /**
     * Initialize device driver.
     *
     * @return If successfully initialized, returns true. Otherwise, false.
     */
    bool init() final;

    /**
     * Process communication with the device.
     *
     * @return If communication is successful, returns true. Otherwise, false.
     */
    bool process() final;

    /**
     * Get comunication Stream.
     *
     * @return Device data Stream.
     */
    Stream& getStream() final;

    /**
     * Reset the device.
     */
    void reset() final;

private:
    /** Default server address of the device. */
    static const char* DEFAULT_SERVER_ADDRESS;

    /** Default server port of the device. */
    static const char* DEFAULT_SERVER_PORT;

    /** Maximum number of connection retries. */
    static const uint8_t MAX_CONN_RETRY_COUNT;

    /** Retry connection counter. */
    uint8_t m_retryConnectionCounter;

    /**
     * Socket stream for communication with the device.
     */
    SocketClient m_socket;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* DEVICE_H */
/** @} */
