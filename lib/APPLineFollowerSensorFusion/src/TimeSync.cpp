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
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "TimeSync.h"
#include <Arduino.h>
#include <Logging.h>
#include <ArduinoJson.h>


/******************************************************************************
 * Local Variables / Constants
 *****************************************************************************/
namespace
{
    /** Default ping period for time synchronization (ms). */
    constexpr uint32_t TSYNC_DEFAULT_PING_PERIOD_MS = 1000U;

    /** Initial value for the minimum RTT (max uint32_t). */
    constexpr uint32_t TSYNC_MIN_RTT_INITIAL = 0xFFFFFFFFUL;

    /** Maximum number of accepted good RTT samples. */
    constexpr uint8_t TSYNC_MAX_GOOD_SAMPLES = 255U;

    /** RTT margin: accept RTTs up to 20% above the best recorded RTT.*/
    constexpr uint32_t TSYNC_RTT_MARGIN_NUM = 6U;
    constexpr uint32_t TSYNC_RTT_MARGIN_DEN = 5U;
} // namespace

/******************************************************************************
 * Public Methods
 *****************************************************************************/

TimeSync::TimeSync(SerMuxChannelProvider& serMuxProvider) :
    m_serMuxProvider(serMuxProvider),
    m_pingTimer(),
    m_pingPeriodMs(TSYNC_DEFAULT_PING_PERIOD_MS),
    m_seq(0U),
    m_pendingT1_32(0U),
    m_minRttMs(TSYNC_MIN_RTT_INITIAL),
    m_zumoToEspOffsetMs(0),
    m_zumoGoodSamples(0U),
    m_pendingSeq(0U),
    m_lastStatusLogMs(0ULL),
    m_lastAcceptedRttMs(0U),
    m_lastOffsetEstMs(0),
    m_minOffsetMs(0),
    m_maxOffsetMs(0),
    m_hostSeq(0U),
    m_hostSyncValid(false),
    m_hostOffsetMs(0),
    m_lastHostRttMs(0U)
{
}

void TimeSync::begin()
{
    m_pingTimer.start(m_pingPeriodMs);

    /* Register time sync response callback. */
    m_serMuxProvider.registerTimeSyncResponseCallback(
        [this](const TimeSyncResponse& rsp) { onTimeSyncResponse(rsp); });

    m_lastStatusLogMs = localNowMs();
}

void TimeSync::process()
{
    /* Periodically send ping to Zumo for time sync. */
    if ((true == m_serMuxProvider.isInSync()) && (true == m_pingTimer.isTimeout()))
    {
        const uint64_t now64 = localNowMs();
        const uint32_t now32 = static_cast<uint32_t>(now64);

        if (false == m_serMuxProvider.sendTimeSyncRequest(m_seq, now32))
        {
            LOG_WARNING("TimeSync: Failed to send TIME_SYNC_REQ.");
        }
        else
        {
            m_pendingSeq   = m_seq;
            m_pendingT1_32 = now32;
            ++m_seq;
        }
        m_pingTimer.restart();
    }

    /* Periodically log status. */
    const uint64_t nowMs = localNowMs();
    if ((nowMs - m_lastStatusLogMs) >= 10000ULL)
    {
        logZumoStatus();
        if (m_hostSyncValid)
        {
            LOG_INFO("Host sync: offset=%ld ms, last RTT=%lu ms",
                     static_cast<long>(m_hostOffsetMs),
                     static_cast<unsigned long>(m_lastHostRttMs));
        }
        else
        {
            LOG_INFO("Host sync: not yet valid");
        }
        m_lastStatusLogMs = nowMs;
    }
}

uint64_t TimeSync::mapZumoToLocalMs(uint32_t zumoTsMs) const
{
    /* Apply last known offset (no skew correction yet). */
    int64_t local32 = static_cast<int64_t>(zumoTsMs) - m_zumoToEspOffsetMs;

    /* Normalize to 64-bit local by referencing current time high word. */
    const uint64_t now64 = localNowMs();
    const uint32_t now32 = static_cast<uint32_t>(now64);

    /* Reconstruct 64-bit around now: choose the 32-bit epoch closest to now32. */
    const int32_t  diff   = static_cast<int32_t>(local32 - static_cast<int64_t>(now32));
    const uint64_t base64 = now64 - static_cast<int64_t>(now32);

    return base64 + static_cast<uint32_t>(static_cast<int32_t>(now32) + diff);
}

uint64_t TimeSync::localNowMs() const
{
    return static_cast<uint64_t>(millis());
}

uint64_t TimeSync::hostToEspLocalMs(uint64_t hostMs) const
{
    if (false == m_hostSyncValid)
    {
        /* Fallback: keine Host-Sync-Info, einfach aktuelle Local-Time. */
        return localNowMs();
    }

    /* host_time_ms ≈ esp_local_ms + offsetHostMinusEsp
     * => esp_local_ms ≈ host_time_ms - offsetHostMinusEsp
     */
    const int64_t localMs = static_cast<int64_t>(hostMs) - m_hostOffsetMs;
    return (localMs >= 0) ? static_cast<uint64_t>(localMs) : 0ULL;
}

/******************************************************************************
 * Host <-> ESP MQTT TimeSync
 *****************************************************************************/

void TimeSync::sendHostTimeSyncRequest(MqttClient& mqttClient,
                                       const char* topic)
{
    if (nullptr == topic)
    {
        return;
    }

    const uint32_t seq = m_hostSeq++;
    const uint64_t t1  = millis();

    JsonDocument doc;
    doc["seq"]       = seq;
    doc["t1_esp_ms"] = t1;

    String payload;
    serializeJson(doc, payload);

    if (!mqttClient.publish(topic, true, payload))
    {
        LOG_WARNING("HostTimeSync: Failed to publish sync request to topic %s.", topic);
    }
}


void TimeSync::onHostTimeSyncResponse(uint32_t seq,
                                      uint64_t t1EspMs,
                                      uint64_t t2HostMs,
                                      uint64_t t3HostMs,
                                      uint64_t t4EspMs)
{
    /* NTP-style formulas: */
    /* offset = ((T2 - T1) + (T3 - T4)) / 2 */
    /* rtt    = (T4 - T1) - (T3 - T2) */
    /* T1 = t1EspMs   (ESP) */
    /* T2 = t2HostMs  (Host) */
    /* T3 = t3HostMs  (Host) */
    /* T4 = t4EspMs   (ESP) */
    LOG_INFO("Host sync: seq=%u", static_cast<unsigned>(seq));
    LOG_INFO("t1=%llu ms", t1EspMs);
    LOG_INFO("t2=%llu ms", t2HostMs);
    LOG_INFO("t3=%llu ms", t3HostMs);
    LOG_INFO("t4=%llu ms", t4EspMs);
    

    const int64_t d1 = static_cast<int64_t>(t2HostMs) - static_cast<int64_t>(t1EspMs);
    const int64_t d2 = static_cast<int64_t>(t3HostMs) - static_cast<int64_t>(t4EspMs);

    const int64_t offset = (d1 + d2) / 2;

    const int64_t rtt = (static_cast<int64_t>(t4EspMs) - static_cast<int64_t>(t1EspMs))
                      - (static_cast<int64_t>(t3HostMs) - static_cast<int64_t>(t2HostMs));

    m_hostOffsetMs  = offset;
    m_lastHostRttMs = (rtt >= 0) ? static_cast<uint32_t>(rtt) : 0U;
    m_hostSyncValid = true;

    const float offset_s = static_cast<float>(m_hostOffsetMs) / 1000.0F;

    LOG_INFO("Host sync: seq=%u offset=%.3f s rtt=%lu ms",
            static_cast<unsigned>(seq),
            offset_s,
            static_cast<unsigned long>(m_lastHostRttMs));
}

/******************************************************************************
 * Zumo <-> ESP SerialMux TimeSync
 *****************************************************************************/

void TimeSync::onTimeSyncResponse(const TimeSyncResponse& rsp)
{
    if (rsp.sequenceNumber != m_pendingSeq)
    {
        LOG_WARNING("TimeSync: seq mismatch (expected=%u, got=%u) – using last T1",
                    static_cast<unsigned>(m_pendingSeq),
                    rsp.sequenceNumber);
    }

    const uint64_t t4_64 = localNowMs();
    const uint32_t t4_32 = static_cast<uint32_t>(t4_64);

    /* 32-bit safe differences (modulo wrap). */
    const uint32_t delta_local = static_cast<uint32_t>(t4_32 - m_pendingT1_32);
    const uint32_t delta_zumo  = static_cast<uint32_t>(rsp.t3_ms - rsp.t2_ms);

    if (delta_local < delta_zumo)
    {
        LOG_WARNING("TimeSync: invalid deltas (delta_local=%u, delta_zumo=%u)",
                    delta_local, delta_zumo);
        return;
    }

    /* RTT estimation. */
    const uint32_t rtt_ms = static_cast<uint32_t>(delta_local - delta_zumo);

    /* Offset Zumo->ESP: ((t2 - t1) + (t3 - t4)) / 2 */
    const int64_t d1         = static_cast<int32_t>(rsp.t2_ms - m_pendingT1_32);
    const int64_t d2         = static_cast<int32_t>(rsp.t3_ms - t4_32);
    const int64_t offset_est = (d1 + d2) / 2;

    m_lastAcceptedRttMs = rtt_ms;
    m_lastOffsetEstMs   = offset_est;

    /* Keep the best samples (min RTT heuristic). */
    bool accept = false;
    if (rtt_ms <= m_minRttMs)
    {
        m_minRttMs = rtt_ms;
        accept     = true;
    }
    else if (rtt_ms <= (m_minRttMs * TSYNC_RTT_MARGIN_NUM / TSYNC_RTT_MARGIN_DEN))
    {
        accept = true;
    }

    if (true == accept)
    {
        m_zumoToEspOffsetMs = offset_est;

        if (m_zumoGoodSamples < TSYNC_MAX_GOOD_SAMPLES)
        {
            ++m_zumoGoodSamples;
        }

        if (1U == m_zumoGoodSamples)
        {
            m_minOffsetMs = offset_est;
            m_maxOffsetMs = offset_est;
        }
        else
        {
            if (offset_est < m_minOffsetMs)
            {
                m_minOffsetMs = offset_est;
            }
            if (offset_est > m_maxOffsetMs)
            {
                m_maxOffsetMs = offset_est;
            }
        }
    }
}

/******************************************************************************
 * Diagnostic Logging
 *****************************************************************************/

void TimeSync::logZumoStatus() const
{
    const uint32_t bestRttMs = (TSYNC_MIN_RTT_INITIAL == m_minRttMs) ? 0U : m_minRttMs;
    const uint32_t estAccuMs = bestRttMs / 2U;

    LOG_INFO("Zumo sync: lastSeq=%u goodSamples=%u",
             static_cast<unsigned>(m_pendingSeq),
             static_cast<unsigned>(m_zumoGoodSamples));

    LOG_INFO("Zumo sync: lastRTT=%lu ms minRTT=%lu ms",
             static_cast<unsigned long>(m_lastAcceptedRttMs),
             static_cast<unsigned long>(bestRttMs));

    if (m_zumoGoodSamples > 0U)
    {
        const long offset32    = static_cast<long>(m_zumoToEspOffsetMs);
        const long minOffset32 = static_cast<long>(m_minOffsetMs);
        const long maxOffset32 = static_cast<long>(m_maxOffsetMs);
        const long span32      = maxOffset32 - minOffset32;
        const long absSpan32   = (span32 >= 0) ? span32 : -span32;

        LOG_INFO("Zumo sync: offsetZ2E=%ld ms span=[%ld .. %ld] ms",
                 offset32, minOffset32, maxOffset32);
        LOG_INFO("Zumo sync: est. accuracy=+/- %lu ms (from minRTT/2, span=%ld ms)",
                 static_cast<unsigned long>(estAccuMs), absSpan32);
    }
    else
    {
        LOG_INFO("Zumo sync: no valid samples yet.");
    }
}

void TimeSync::logStatus() const
{
    LOG_INFO("================= TimeSync Detailed Status =================");
    logZumoStatus();
    if (m_hostSyncValid)
    {
        LOG_INFO("Host sync: offset=%ld ms, last RTT=%lu ms",
                 static_cast<long>(m_hostOffsetMs),
                 static_cast<unsigned long>(m_lastHostRttMs));
    }
    else
    {
        LOG_INFO("Host sync: not yet valid");
    }
    LOG_INFO("============================================================");
}