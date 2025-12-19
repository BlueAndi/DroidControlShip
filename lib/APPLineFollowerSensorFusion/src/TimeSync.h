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
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include <SimpleTimer.hpp>
#include "SerMuxChannelProvider.h"
#include "SerialMuxChannels.h"
#include <MqttClient.h>

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * @brief Time synchronization orchestrator (ESP32 side).
 *
 * Responsibilities:
 * - Serial ping-pong time sync with Zumo32u4 over SerialMux
 * - Mapping Zumo millis() timestamps to ESP32 local time (millis())
 * - Host (Laptop/PC) time sync over MQTT using NTP-style T1..T4 timestamps
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
     * Initialize time synchronization.
     */
    void begin();

    /**
     * Process periodic activities (send Zumo pings, housekeeping).
     */
    void process();

    /**
     * Log current time sync status (Zumo + host).
     */
    void logStatus() const;

    /**********************************************************************
     * Zumo <-> ESP (SerialMux)
     *********************************************************************/

    /**
     * Returns whether serial ping-pong has a valid offset estimate.
     *
     * @return true if Zumo<->ESP is considered synchronized; otherwise false.
     */
    bool isZumoSynced() const
    {
        static const uint8_t REQUIRED_GOOD_SAMPLES = 3U;
        return m_zumoGoodSamples >= REQUIRED_GOOD_SAMPLES;
    }

    /**
     * Map Zumo timestamp [ms] to ESP32 local time [ms] using the latest offset.
     *
     * @param[in] zumoTsMs  Zumo millis() timestamp [ms].
     *
     * @return ESP32 local time [ms] corresponding to the given Zumo timestamp.
     */
    uint64_t mapZumoToLocalMs(uint32_t zumoTsMs) const;

    /**
     * Get current ESP32 local time [ms] (wrapper around millis()).
     *
     * @return Local time [ms].
     */
    uint64_t localNowMs() const;

    /**
     * Get estimated Zumo->ESP offset [ms]. Positive means Zumo is ahead of ESP.
     *
     * @return Time offset Zumo->ESP [ms].
     */
    int64_t getZumoToEspOffsetMs() const
    {
        return m_zumoToEspOffsetMs;
    }

    /**
     * Log detailed serial (Zumo) time sync status information.
     */
    void logZumoStatus() const;

    /**********************************************************************
     * Host <-> ESP (MQTT, T1..T4)
     *
     * Protocol:
     *  - ESP sends request (T1_esp_ms, seq)
     *  - Host (Python) records T2_host_ms, T3_host_ms and sends a response
     *  - ESP records T4_esp_ms on reception and computes offset + RTT
     *********************************************************************/


    /**
     * @brief Send a host time sync request message via MQTT.
     *
     * The function:
     *  - increments the host sequence counter
     *  - measures T1 (ESP localNowMs())
     *  - builds a JSON payload { "seq", "t1_esp_ms" }
     *  - calls the provided publish callback
     *
     * @param[in] mqttClient  MQTT instance.
     * @param[in] topic      MQTT topic for the request message.
     */
    void sendHostTimeSyncRequest(MqttClient& mqttClient,
                                 const char* topic);

    /**
     * @brief Handle a host time sync response received via MQTT.
     *
     * Expected payload (generated by the host):
     *  - seq         : Sequence number (must match the sent request).
     *  - t1_esp_ms   : T1 (as originally sent by the ESP).
     *  - t2_host_ms  : T2 (host receive time of the request).
     *  - t3_host_ms  : T3 (host send time of the response).
     *
     * The function:
     *  - measures T4 (ESP localNowMs())
     *  - computes offset and RTT using NTP-style formulas
     *  - stores offset and RTT for later use
     *
     * @param[in] seq         Sequence number.
     * @param[in] t1EspMs     T1 on ESP side [ms].
     * @param[in] t2HostMs    T2 on host side [ms].
     * @param[in] t3HostMs    T3 on host side [ms].
     * @param[in] t4EspMs     T4 on ESP side [ms].
     */
    void onHostTimeSyncResponse(uint32_t seq,
                                uint64_t t1EspMs,
                                uint64_t t2HostMs,
                                uint64_t t3HostMs,
                                uint64_t t4EspMs);

    /**
     * @brief Returns whether a valid host synchronization offset is available.
     *
     * @return true if a valid host<->ESP offset is available; otherwise false.
     */
    bool isHostSynced() const
    {
        return m_hostSyncValid;
    }

    /**
     * @brief Get the estimated Host->ESP offset [ms].
     *
     * Interpretation:
     *   host_time_ms ≈ esp_local_ms + offsetHostMinusEsp
     *
     * @return Estimated Host->ESP offset [ms].
     */
    int64_t getHostOffsetMs() const
    {
        return m_hostOffsetMs;
    }

    /**
     * @brief Convert a host timestamp [ms] to ESP local time [ms].
     *
     * Uses the estimated host offset:
     *   host_time_ms ≈ esp_local_ms + offset
     *   => esp_local_ms ≈ host_time_ms - offset
     *
     * If no valid host sync is available, localNowMs() is returned.
     *
     * @param[in] hostMs  Host time [ms].
     *
     * @return Approximate ESP local time [ms] corresponding to the host time.
     */
    uint64_t hostToEspLocalMs(uint64_t hostMs) const;

private:
    SerMuxChannelProvider& m_serMuxProvider; /**< SerialMux provider. */

    /* --- Serial ping-pong with Zumo --- */
    SimpleTimer m_pingTimer;         /**< Ping timer. */
    uint32_t    m_pingPeriodMs;      /**< Ping period [ms]. */
    uint32_t    m_seq;               /**< Sequence counter. */
    uint32_t    m_pendingT1_32;      /**< T1 of pending request [ms]. */
    uint32_t    m_minRttMs;          /**< Best observed RTT [ms]. */
    int64_t     m_zumoToEspOffsetMs; /**< Estimated offset Zumo->ESP [ms]. */
    uint8_t     m_zumoGoodSamples;   /**< Number of good samples collected. */
    uint32_t    m_pendingSeq;        /**< Sequence number of pending request. */
    uint64_t    m_lastStatusLogMs;   /**< Last status log time [ms]. */

    uint32_t m_lastAcceptedRttMs; /**< Last accepted RTT [ms]. */
    int64_t  m_lastOffsetEstMs;   /**< Last offset estimate [ms]. */
    int64_t  m_minOffsetMs;       /**< Minimum observed offset [ms]. */
    int64_t  m_maxOffsetMs;       /**< Maximum observed offset [ms]. */

    /* --- Host time sync over MQTT --- */
    uint32_t m_hostSeq;       /**< Host sync sequence counter. */
    bool     m_hostSyncValid; /**< Host sync offset valid flag. */
    int64_t  m_hostOffsetMs;  /**< Estimated offset Host->ESP [ms]. */
    uint32_t m_lastHostRttMs; /**< Last RTT to host [ms]. */

    /**
     * @brief Handle an incoming time sync response from the Zumo.
     *
     * @param[in] rsp Time sync response frame received via SerialMux.
     */
    void onTimeSyncResponse(const TimeSyncResponse& rsp);
};

#endif /* TIMESYNC_H */
/** @} */
