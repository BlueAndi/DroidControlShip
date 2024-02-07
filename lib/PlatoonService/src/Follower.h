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
 * @brief  Follower class for V2V communication.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup PlatoonService
 *
 * @{
 */
#ifndef FOLLOWER_H
#define FOLLOWER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Follower class. */
class Follower
{
public:
    /**
     * Follower Constructor.
     */
    Follower() : m_lastHeartbeatTimestamp(0U), m_status(0U)
    {
    }

    /**
     * Follower Destructor.
     */
    ~Follower()
    {
    }

    /**
     * Get the last heartbeat timestamp.
     *
     * @return Last heartbeat timestamp.
     */
    uint32_t getLastHeartbeatTimestamp() const
    {
        return m_lastHeartbeatTimestamp;
    }

    /**
     * Set the last heartbeat timestamp.
     *
     * @param[in] timestamp    Last heartbeat timestamp.
     */
    void setLastHeartbeatTimestamp(uint32_t timestamp)
    {
        m_lastHeartbeatTimestamp = timestamp;
    }

    /**
     * Get the status.
     *
     * @return Status.
     */
    uint8_t getStatus() const
    {
        return m_status;
    }

    /**
     * Set the status.
     *
     * @param[in] status    Status.
     */
    void setStatus(uint8_t status)
    {
        m_status = status;
    }

private:
    /** Last heartbeat timestamp. */
    uint32_t m_lastHeartbeatTimestamp;

    /** Status. */
    uint8_t m_status;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* FOLLOWER_H */
/** @} */
