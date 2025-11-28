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
#include <Logging.h>

/******************************************************************************
 * Local Variables / Constants
 *****************************************************************************/
namespace
{
    /** Default ping period for time synchronization (ms). */
    constexpr uint32_t TSYNC_DEFAULT_PING_PERIOD_MS = 10000U;

    /** Initial value for the minimum RTT (max uint32_t). */
    constexpr uint32_t TSYNC_MIN_RTT_INITIAL = 0xFFFFFFFFUL;

    /** Maximum number of accepted good RTT samples. */
    constexpr uint8_t TSYNC_MAX_GOOD_SAMPLES = 255U;

    /** RTT margin: accept RTTs up to 20% above the best recorded RTT.*/
    constexpr uint32_t TSYNC_RTT_MARGIN_NUM = 6U;
    constexpr uint32_t TSYNC_RTT_MARGIN_DEN = 5U;

    /** Default NTP client configuration. */
    constexpr const char*   TSYNC_DEFAULT_NTP_SERVER     = "pool.ntp.org";
    constexpr unsigned long TSYNC_NTP_UPDATE_INTERVAL_MS = 60000UL;

    /** Timezone config for NTP client (example: GMT+1). */
    constexpr long TSYNC_GMT_OFFSET_SEC = 3600L;
} // namespace

/******************************************************************************
 * Public Methods
 *****************************************************************************/

TimeSync::TimeSync(SerMuxChannelProvider& serMuxProvider) :
    m_serMuxProvider(serMuxProvider),
    m_ntpUdp(),
    m_ntpClient(m_ntpUdp, TSYNC_DEFAULT_NTP_SERVER, TSYNC_GMT_OFFSET_SEC, TSYNC_NTP_UPDATE_INTERVAL_MS),
    m_rtcSynced(false),
    m_epochToLocalOffsetMs(0),
    m_rtcRefreshMs(TSYNC_NTP_UPDATE_INTERVAL_MS),
    m_rtcTimer(),
    m_pingTimer(),
    m_pingPeriodMs(TSYNC_DEFAULT_PING_PERIOD_MS),
    m_seq(0U),
    m_pendingSeq(0U),
    m_pendingT1_32(0U),
    m_minRttMs(TSYNC_MIN_RTT_INITIAL),
    m_zumoToEspOffsetMs(0),
    m_zumoGoodSamples(0U),
    m_lastStatusLogMs(0),
    m_lastRtcUpdateLocalMs(0),
    m_lastRtcCorrectionMs(0),
    m_maxRtcCorrectionMs(0),
    m_lastAcceptedRttMs(0),
    m_lastOffsetEstMs(0),
    m_minOffsetMs(0),
    m_maxOffsetMs(0)
{
}

void TimeSync::begin()
{
    m_pingTimer.start(m_pingPeriodMs);

    /* Register time sync response callback. */
    m_serMuxProvider.registerTimeSyncResponseCallback([this](const TimeSyncResponse& rsp) { onTimeSyncResponse(rsp); });

    m_ntpClient.begin();

    /* Establish initial epoch-to-local mapping if possible. */
    refreshRtcMapping();
}

void TimeSync::process()
{
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
    refreshRtcMapping();
}

uint64_t TimeSync::mapZumoToLocalMs(uint32_t zumoTsMs) const
{
    /* Apply last known offset (no skew correction yet). */
    int64_t local = static_cast<int64_t>(zumoTsMs) - m_zumoToEspOffsetMs;

    /* Normalize to 64-bit local by referencing current time high word. */
    uint64_t now64 = localNowMs();
    uint32_t now32 = static_cast<uint32_t>(now64);

    /* Reconstruct 64-bit around now: choose the 32-bit epoch closest to now32. */
    int32_t  diff   = static_cast<int32_t>(local - static_cast<int64_t>(now32));
    uint64_t base64 = now64 - static_cast<int64_t>(now32);
    return base64 + static_cast<uint32_t>(static_cast<int32_t>(now32) + diff);
}

uint64_t TimeSync::localNowMs() const
{
    return static_cast<uint64_t>(millis());
}

uint64_t TimeSync::nowEpochMs() const
{
    if (false == m_rtcSynced)
    {
        return 0ULL;
    }

    const int64_t epochMs = static_cast<int64_t>(localNowMs()) + m_epochToLocalOffsetMs;
    return (epochMs >= 0) ? static_cast<uint64_t>(epochMs) : 0ULL;
}

void TimeSync::onTimeSyncResponse(const TimeSyncResponse& rsp)
{
    if (rsp.sequenceNumber != m_pendingSeq)
    {
        LOG_WARNING("TimeSync: seq mismatch (expected=%u, got=%u) – using last T1", static_cast<unsigned>(m_pendingSeq),
                    rsp.sequenceNumber);
    }

    const uint64_t t4_64 = localNowMs();
    const uint32_t t4_32 = static_cast<uint32_t>(t4_64);

    /* 32-bit safe differences (modulo wrap). */
    const uint32_t delta_local = static_cast<uint32_t>(t4_32 - m_pendingT1_32);
    const uint32_t delta_zumo  = static_cast<uint32_t>(rsp.t3_ms - rsp.t2_ms);

    if (delta_local < delta_zumo)
    {
        LOG_WARNING("TimeSync: invalid deltas (delta_local=%u, delta_zumo=%u)", delta_local, delta_zumo);
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
 * Private Methods
 *****************************************************************************/

void TimeSync::refreshRtcMapping()
{
    /* Try to update NTP time; if it fails, mark RTC as not synced. */
    if (false == m_ntpClient.update())
    {
        if (true == m_rtcSynced)
        {
            LOG_WARNING("TimeSync: NTP update failed; RTC mapping unavailable");
        }
        m_rtcSynced = false;
        return;
    }

    /* On success, compute epoch-to-local mapping in milliseconds. */
    const unsigned long epochSec = m_ntpClient.getEpochTime();
    const uint64_t      epochMs  = static_cast<uint64_t>(epochSec) * 1000ULL;
    const uint64_t      localMs  = localNowMs();

    /* Log correction info for diagnostics. */
    if (true == m_rtcSynced)
    {
        const int64_t  oldOffset        = m_epochToLocalOffsetMs;
        const uint64_t predictedEpoch   = static_cast<uint64_t>(static_cast<int64_t>(localMs) + oldOffset);
        const int64_t  correction       = static_cast<int64_t>(epochMs) - static_cast<int64_t>(predictedEpoch);
        const int64_t  absCorrection    = (correction >= 0) ? correction : -correction;
        const int64_t  absMaxCorrection = (m_maxRtcCorrectionMs >= 0) ? m_maxRtcCorrectionMs : -m_maxRtcCorrectionMs;

        m_lastRtcCorrectionMs = correction;
        if (absCorrection > absMaxCorrection)
        {
            m_maxRtcCorrectionMs = correction;
        }
    }

    m_epochToLocalOffsetMs = static_cast<int64_t>(epochMs) - static_cast<int64_t>(localMs);
    m_lastRtcUpdateLocalMs = localMs;
    m_rtcSynced            = true;
}

/******************************************************************************
 * Diagnostic Logging
 *****************************************************************************/

void TimeSync::logRtcStatus() const
{
    LOG_INFO("RTC / NTP Status");
    LOG_INFO("RTC Synced:            %s", m_rtcSynced ? "yes" : "no");
    LOG_INFO("Epoch to Local Offset: %lld ms", static_cast<long long>(m_epochToLocalOffsetMs));
    LOG_INFO("Local/Epoch time now:   %llu / %llu ms", static_cast<unsigned long long>(localNowMs()),
             static_cast<unsigned long long>(nowEpochMs()));
}

void TimeSync::logZumoStatus() const
{
    const uint32_t bestRttMs = (TSYNC_MIN_RTT_INITIAL == m_minRttMs) ? 0U : m_minRttMs;
    const uint32_t estAccuMs = bestRttMs / 2U;

    LOG_INFO("Zumo sync: lastSeq=%u goodSamples=%u", static_cast<unsigned>(m_pendingSeq),
             static_cast<unsigned>(m_zumoGoodSamples));

    LOG_INFO("Zumo sync: lastRTT=%lu ms minRTT=%lu ms", static_cast<unsigned long>(m_lastAcceptedRttMs),
             static_cast<unsigned long>(bestRttMs));

    if (m_zumoGoodSamples > 0U)
    {
        const long offset32    = static_cast<long>(m_zumoToEspOffsetMs);
        const long minOffset32 = static_cast<long>(m_minOffsetMs);
        const long maxOffset32 = static_cast<long>(m_maxOffsetMs);
        const long span32      = maxOffset32 - minOffset32;
        const long absSpan32   = (span32 >= 0) ? span32 : -span32;

        LOG_INFO("Zumo sync: offsetZ2E=%ld ms span=[%ld .. %ld] ms", offset32, minOffset32, maxOffset32);
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
    logRtcStatus();
    logZumoStatus();
    LOG_INFO("============================================================");
}
