/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
 * @file
 * @brief  Receiver realization
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef RECEIVER_H
#define RECEIVER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "IReceiver.h"

#include <webots/Receiver.hpp>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * This class provides a receiver to communicate with other robots or the supervisor
 * in the simulation.
 */
class Receiver : public IReceiver
{
public:
    /**
     * Constructs the receiver adapter.
     *
     * @param[in] receiver   The receiver of the simulated robot.
     */
    Receiver(webots::Receiver* receiver) : IReceiver(), m_receiver(receiver), m_dataRemains(0U)
    {
    }

    /**
     * Destroys the receiver adapter.
     */
    ~Receiver()
    {
    }

    /**
     * Set channel which to receive data from.
     *
     * @param[in] channel   The channel which to use.
     */
    void setChannel(int32_t channel) override;

    /**
     * Receives data from the configured channel.
     *
     * @param[in] data  Data buffer.
     * @param[in] size  Data buffer size in bytes.
     *
     * @return Number of bytes read from stream.
     */
    size_t receive(void* data, size_t size) override;

    /**
     * Check if any data has been received.
     *
     * @return Number of available bytes.
     */
    int available() const override;

private:
    webots::Receiver* m_receiver;    /**< The receiver on the simulated robot. */
    size_t            m_dataRemains; /**< Number of bytes which are remaining in head packet. */

    /* Default constructor not allowed. */
    Receiver();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* RECEIVER_H */
/** @} */
