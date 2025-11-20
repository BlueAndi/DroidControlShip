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
 * @brief  Time synchronization
 * @author Tobias Haeckel <tobias.haeckel@gmx.net>
 *
 * @addtogroup Application
 * @{
 */

#ifndef TIMESYNC_H
#define TIMESYNC_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <SimpleTimer.hpp>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include "SerMuxChannelProvider.h"
#include "SerialMuxChannels.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * @brief  Time synchronization orchestrator (ESP32 side).
 *
 * Provides:
 * - Network time via SNTP (Host as NTP server)
 * - Serial ping-pong time sync with Zumo32u4 over SerialMux
 * - Linear time mapping helpers for future extensions
 */
class TimeSync
{
public:
    /**
     * Construct the time synchronization orchestrator.
     *
     * @param[in] serMuxProvider SerialMux provider to send/receive time sync frames.
     */
    TimeSync(SerMuxChannelProvider& serMuxProvider);

    /**
     * Log current time sync status (for debugging/testing).
     */
    void logStatus() const;

    /**
     * Initialize time synchronization.
     */
    void begin();

    /**
     * Process periodic activities (send pings, housekeeping).
     */
    void process();

    /**
     * Returns whether serial ping-pong has a valid offset estimate.
     */
    bool isZumoSynced() const
    {
        return m_zumoGoodSamples >= 3;
    }

    /**
     * Returns whether RTC (SNTP) is available.
     */
    bool isRtcSynced() const
    {
        return m_rtcSynced;
    }

    /**
     * Map Zumo timestamp [ms] to ESP32 local time [ms] using latest offset.
     *
     * @param[in] zumoTsMs  Zumo millis() timestamp [ms].
     * @return ESP32 local time [ms].
     */
    uint64_t mapZumoToLocalMs(uint32_t zumoTsMs) const;

    /**
     * Get current local time [ms].
     */
    uint64_t localNowMs() const;

    /**
     * Get current epoch time [ms] if RTC mapping is available, otherwise 0.
     */
    uint64_t nowEpochMs() const;

    /**
     * Get estimated Zumo->ESP offset [ms]. Positive means Zumo ahead of ESP.
     */
    int64_t getZumoToEspOffsetMs() const
    {
        return m_zumoToEspOffsetMs;
    }

    /**
     * Log detailed RTC/NTP status information.
     */
    void logRtcStatus() const;

    /**
     * Log detailed serial (Zumo) time sync status information.
     */
    void logZumoStatus() const;

private:
    SerMuxChannelProvider& m_serMuxProvider; /**< SerialMux provider. */

    // --- RTC (NTP) ---
    WiFiUDP     m_ntpUdp;               /**< UDP socket for NTP client. */
    NTPClient   m_ntpClient;            /**< NTP client instance. */
    bool        m_rtcSynced;            /**< RTC synchronized flag. */
    int64_t     m_epochToLocalOffsetMs; /**< Epoch time - local time [ms]. */
    uint32_t    m_rtcRefreshMs;         /**< RTC mapping refresh period. */
    SimpleTimer m_rtcTimer;             /**< Timer for refreshing RTC mapping. */

    // --- Serial ping-pong with Zumo ---
    SimpleTimer m_pingTimer;          /**< Ping timer. */
    uint32_t    m_pingPeriodMs;       /**< Ping period [ms]. */
    uint32_t    m_seq;                /**< Sequence counter. */
    bool        m_pending;            /**< Waiting for response. */
    uint32_t    m_pendingT1_32;       /**< Pending T1 (lower 32 bits) [ms]. */
    uint64_t    m_pendingT1_64;       /**< Pending T1 [ms]. */
    uint32_t    m_minRttMs;           /**< Best observed RTT [ms]. */
    int64_t     m_zumoToEspOffsetMs;  /**< Estimated offset Zumo->ESP [ms]. */
    uint8_t     m_zumoGoodSamples;    /**< Number of good samples collected. */
    uint32_t    m_pendingSeq;         /**< Sequence number of pending request. */
    uint64_t    m_lastStatusLogMs{0}; /**< Last status log time [ms]. */

    uint64_t m_lastRtcUpdateLocalMs{0}; /**< Local time of last RTC update [ms]. */
    int64_t  m_lastRtcCorrectionMs{0};  /**< Last RTC correction applied [ms]. */
    int64_t  m_maxRtcCorrectionMs{0};   /**< Maximum RTC correction applied [ms]. */
    uint32_t m_lastAcceptedRttMs{0};    /**< Last accepted RTT [ms]. */
    int64_t  m_lastOffsetEstMs{0};      /**< Last offset estimate [ms]. */
    int64_t  m_minOffsetMs{0};          /**< Minimum observed offset [ms]. */
    int64_t  m_maxOffsetMs{0};          /**< Maximum observed offset [ms]. */

    /**
     * Handle an incoming time sync response from Zumo.
     */
    void onTimeSyncResponse(const TimeSyncResponse& rsp);

    /**
     * Try to refresh epoch-to-local mapping using RTC (SNTP) if available.
     */
    void refreshRtcMapping();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* TIMESYNC_H */
/** @} */
