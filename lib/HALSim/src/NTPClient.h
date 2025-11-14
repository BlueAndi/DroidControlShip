/*
 * MIT License
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
 * @brief Lightweight NTPClient replacement used in native simulation builds.
 *
 * This module provides a minimal emulation of the Arduino NTPClient interface.
 * It is intended to support the TimeSync component in simulation environments
 * where no real NTP communication takes place. Instead, the client returns the
 * host system time plus a configurable offset.
 *
 * @author Tobias Haeckel <tobias.haeckel@gmx.net>
 *
 * @addtogroup HALSim
 * @{
 */

#ifndef NTPCLIENT_H
#define NTPCLIENT_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <chrono>
#include <cstdint>
#include "WiFiUdp.h"

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * @brief Simulated NTP client for native builds.
 *
 * This class mimics the essential parts of the Arduino NTPClient API, allowing
 * system time retrieval for modules that require timestamps. No network
 * communication is performed; the class uses the host's system clock.
 */
class NTPClient
{
public:
    /**
     * @brief Constructs a simulated NTP client.
     *
     * @param[in] udp               Reference to WiFiUDP instance (unused).
     * @param[in] poolServerName    Symbolic NTP server name.
     * @param[in] timeOffset        Time offset in seconds applied to host time.
     * @param[in] updateInterval    Update interval in milliseconds (unused).
     */
    NTPClient(WiFiUDP& udp, const char* poolServerName, long timeOffset, unsigned long updateInterval) :
        m_udp(udp),
        m_server(poolServerName),
        m_timeOffsetSec(timeOffset),
        m_updateIntervalMs(updateInterval)
    {
        (void)m_updateIntervalMs;
    }

    /**
     * @brief Initializes the simulated client.
     *
     * No action is performed in simulation mode.
     */
    void begin()
    {
        /* No-op in simulation. */
    }

    /**
     * @brief Sets the time offset relative to the host system time.
     *
     * @param[in] seconds New offset in seconds.
     */
    void setTimeOffset(long seconds)
    {
        m_timeOffsetSec = seconds;
    }

    /**
     * @brief Sets the pool server name (for API compatibility only).
     *
     * @param[in] server Pointer to a C-string with the server name.
     */
    void setPoolServerName(const char* server)
    {
        m_server = server;
    }

    /**
     * @brief Performs an update attempt.
     *
     * Always returns true in simulation mode.
     *
     * @return true Always.
     */
    bool update()
    {
        return true;
    }

    /**
     * @brief Retrieves the current epoch time.
     *
     * Returns the host system time in seconds since UNIX epoch plus the
     * configured offset. This allows deterministic timestamp generation for
     * systems that rely on a time reference.
     *
     * @return Current time in seconds since UNIX epoch (UTC).
     */
    unsigned long getEpochTime()
    {
        using namespace std::chrono;
        const auto now   = system_clock::now();
        const auto secs  = duration_cast<seconds>(now.time_since_epoch()).count();
        const auto epoch = static_cast<long long>(secs) + static_cast<long long>(m_timeOffsetSec);
        return static_cast<unsigned long>(epoch);
    }

private:
    WiFiUDP&      m_udp;               /**< Reference to UDP interface (unused in simulation). */
    const char*   m_server;            /**< Simulated NTP pool server name. */
    long          m_timeOffsetSec;     /**< Offset applied to host system time. */
    unsigned long m_updateIntervalMs;  /**< Update interval (unused in simulation). */
};

#endif /* NTPCLIENT_H */
/** @} */
