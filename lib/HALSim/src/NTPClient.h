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
 * @brief  Simulated NTP client
 * @author Tobias Haeckel <tobias.haeckel@gmx.net>
 *
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef NTPCLIENT_H
#define NTPCLIENT_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include "WiFiUdp.h"

/******************************************************************************
 * Classes
 *****************************************************************************/

/**
 * @brief Simulated NTP client for native builds (no actual network usage).
 */
class NTPClient
{
public:
    /**
     * @brief Constructs the simulated NTP client.
     *
     * @param[in] udp               Reference to WiFiUDP instance (unused).
     * @param[in] poolServerName    Symbolic NTP server name.
     * @param[in] timeOffset        Time offset in seconds applied to host time.
     * @param[in] updateInterval    Update interval in milliseconds (unused).
     */
    NTPClient(WiFiUDP& udp, const char* poolServerName, int32_t timeOffset, uint32_t updateInterval);

    /**
     * @brief Initializes the simulated client.
     * No action in simulation mode.
     */
    void begin();

    /**
     * @brief Sets the time offset relative to UTC.
     *
     * @param [in] seconds Time offset in seconds relative to UTC.
     */
    void setTimeOffset(int32_t seconds);

    /**
     * @brief Sets the pool server name.
     *
     * @param[in] server Symbolic NTP server name (no effect in Simulation).
     */
    void setPoolServerName(const char* server);

    /**
     * @brief Performs a simulated update attempt.
     *
     * @return Always true.
     */
    bool update();

    /**
     * @brief Returns current epoch time in seconds (UTC + offset).
     *
     * @return Epoch time in seconds.
     */
    uint32_t getEpochTime();

private:
    WiFiUDP&    m_udp;              /**< Reference to UDP interface (not used). */
    const char* m_server;           /**< Symbolic NTP pool server name. */
    int32_t     m_timeOffsetSec;    /**< Offset applied to epoch time (seconds). */
    uint32_t    m_updateIntervalMs; /**< Update interval (unused). */
};

#endif /* NTPCLIENT_H */

/** @} */ //