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
#include <cstring>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/
namespace
{
    /** Periodic RTC refresh */
    constexpr uint32_t TSYNC_RTC_REFRESH_MS = 5000U;

    /** Default ping period for time synchronization (ms). */
    constexpr uint32_t TSYNC_DEFAULT_PING_PERIOD_MS = 200U;

    /** Timeout for a pending time-sync request (ms). */
    constexpr uint64_t TSYNC_REQUEST_TIMEOUT_MS = 1000ULL;

    /** Initial value for the minimum RTT (max uint32_t). */
    constexpr uint32_t TSYNC_MIN_RTT_INITIAL = 0xFFFFFFFFUL;

    /** Maximum number of accepted good RTT samples. */
    constexpr uint8_t TSYNC_MAX_GOOD_SAMPLES = 255U;

    /** RTT margin: accept RTTs up to 20% above the best recorded RTT.*/
    constexpr uint32_t TSYNC_RTT_MARGIN_NUM = 6U;
    constexpr uint32_t TSYNC_RTT_MARGIN_DEN = 5U;

    /** Default NTP client configuration. */
    constexpr const char*   TSYNC_DEFAULT_NTP_SERVER     = "pool.ntp.org";
    constexpr unsigned long TSYNC_NTP_UPDATE_INTERVAL_MS = 60000UL; // 60 s

    /** Timezone config for NTP client (example: GMT+1). */
    constexpr long TSYNC_GMT_OFFSET_SEC = 3600L; // CET base offset (no DST handling here)
} // namespace

/******************************************************************************
 * Public Methods
 *****************************************************************************/

TimeSync::TimeSync(SerMuxChannelProvider& serMuxProvider) :
    m_serMuxProvider(serMuxProvider),
    m_ntpUdp(),
    m_ntpClient(m_ntpUdp, TSYNC_DEFAULT_NTP_SERVER, 0L, TSYNC_NTP_UPDATE_INTERVAL_MS),
    m_ntpServer{0},
    m_ntpTimeOffsetSec(TSYNC_GMT_OFFSET_SEC),
    m_ntpUpdateIntervalMs(TSYNC_NTP_UPDATE_INTERVAL_MS),
    m_rtcSynced(false),
    m_epochToLocalOffsetMs(0),
    m_rtcRefreshMs(TSYNC_RTC_REFRESH_MS),
    m_rtcTimer(),
    m_pingTimer(),
    m_pingPeriodMs(TSYNC_DEFAULT_PING_PERIOD_MS),
    m_seq(0U),
    m_pending(false),
    m_pendingSeq(0U),
    m_pendingT1_32(0U),
    m_pendingT1_64(0U),
    m_minRttMs(TSYNC_MIN_RTT_INITIAL),
    m_zumoToEspOffsetMs(0),
    m_zumoGoodSamples(0U)
{
    std::strncpy(m_ntpServer, TSYNC_DEFAULT_NTP_SERVER, sizeof(m_ntpServer) - 1U);
}

void TimeSync::begin(const char* ntpServerAddr, uint32_t pingPeriodMs, uint32_t rtcRefreshMs)
{
    m_pingPeriodMs = pingPeriodMs;
    m_rtcRefreshMs = rtcRefreshMs;
    m_pingTimer.start(m_pingPeriodMs);
    m_rtcTimer.start(m_rtcRefreshMs);

    /* Register time sync response callback. */
    m_serMuxProvider.registerTimeSyncResponseCallback([this](const TimeSyncResponse& rsp) { onTimeSyncResponse(rsp); });

    /* Configure and start NTP client (WiFi connectivity is assumed to be handled externally). */
    if (nullptr != ntpServerAddr)
    {
        m_ntpClient.setPoolServerName(ntpServerAddr);
        std::strncpy(m_ntpServer, ntpServerAddr, sizeof(m_ntpServer) - 1U);
        m_ntpServer[sizeof(m_ntpServer) - 1U] = '\0';
    }
    m_ntpTimeOffsetSec = TSYNC_GMT_OFFSET_SEC;
    m_ntpClient.setTimeOffset(m_ntpTimeOffsetSec);
    m_ntpClient.begin();
    (void)m_ntpClient.update(); /* Best-effort initial sync. */
    refreshRtcMapping();        /* Establish initial epoch-to-local mapping if possible. */
}

void TimeSync::process()
{
    if (m_pending && (localNowMs() - m_pendingT1_64 > TSYNC_REQUEST_TIMEOUT_MS)) // 1 s
    {
        LOG_WARNING("TimeSync: request timeout (no response)");
        m_pending = false;
    }

    /* Send ping if SerialMux is synced and no ping pending. */
    if ((true == m_serMuxProvider.isInSync()) && (true == m_pingTimer.isTimeout()) && (false == m_pending))
    {
        const uint64_t now64 = localNowMs();
        const uint32_t now32 = static_cast<uint32_t>(now64);

        if (false == m_serMuxProvider.sendTimeSyncRequest(m_seq, now32))
        {
            LOG_WARNING("Failed to send TIME_SYNC_REQ.");
        }
        else
        {
            m_pending      = true;
            m_pendingSeq   = m_seq;
            m_pendingT1_32 = now32;
            m_pendingT1_64 = now64;
            m_seq++;
        }

        m_pingTimer.restart();
    }

    /* Periodically refresh RTC mapping using NTP time. */
    if (true == m_rtcTimer.isTimeout())
    {
        refreshRtcMapping();
        m_rtcTimer.restart();
    }
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
    if (!m_pending)
    {
        LOG_WARNING("TimeSync: Unexpected response (no pending request). seq=%u", rsp.seq);
        return;
    }
    if (rsp.seq != m_pendingSeq)
    {
        LOG_WARNING("TimeSync: seq mismatch (expected=%u, got=%u)", m_pendingSeq, rsp.seq);
        m_pending = false;
        return;
    }

    const uint64_t t4_64 = localNowMs();
    const uint32_t t4_32 = static_cast<uint32_t>(t4_64);

    /* 32-bit safe differences (modulo wrap). */
    const uint32_t delta_local = static_cast<uint32_t>(t4_32 - m_pendingT1_32);
    const uint32_t delta_zumo  = static_cast<uint32_t>(rsp.t3_ms - rsp.t2_ms);

    if (delta_local < delta_zumo)
    {
        LOG_WARNING("TimeSync: invalid deltas (delta_local=%u, delta_zumo=%u)", delta_local, delta_zumo);
        m_pending = false;
        return;
    }

    /* RTT estimation. */
    const uint32_t rtt_ms = static_cast<uint32_t>(delta_local - delta_zumo);

    /* Offset Zumo->ESP: ((t2 - t1) + (t3 - t4)) / 2 */
    const int64_t d1         = static_cast<int32_t>(rsp.t2_ms - m_pendingT1_32);
    const int64_t d2         = static_cast<int32_t>(rsp.t3_ms - t4_32);
    const int64_t offset_est = (d1 + d2) / 2;

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
    }

    m_pending = false;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

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

    m_epochToLocalOffsetMs = static_cast<int64_t>(epochMs) - static_cast<int64_t>(localMs);
    m_rtcSynced            = true;
}

/******************************************************************************
 * Diagnostic Logging
 *****************************************************************************/

void TimeSync::logRtcStatus() const
{
    const uint64_t localMs = localNowMs();
    const uint64_t epochMs = nowEpochMs();

    LOG_INFO("════════ RTC / NTP Status ════════════════════════════════");
    LOG_INFO("RTC synced:           %s", m_rtcSynced ? "YES" : "NO");
    LOG_INFO("NTP server:           %s", m_ntpServer);
    LOG_INFO("NTP time offset:      %ld s", m_ntpTimeOffsetSec);
    LOG_INFO("NTP update interval:  %lu ms", (unsigned long)m_ntpUpdateIntervalMs);
    LOG_INFO("RTC refresh period:   %lu ms", (unsigned long)m_rtcRefreshMs);
    LOG_INFO("Local now:            %llu ms", (unsigned long long)localMs);
    LOG_INFO("Epoch now:            %llu ms", (unsigned long long)epochMs);
    LOG_INFO("Epoch-Local offset:   %+lld ms", (long long)m_epochToLocalOffsetMs);
    LOG_INFO("═══════════════════════════════════════════════════════════");
}

void TimeSync::logZumoStatus() const
{
    const bool     inSync       = m_serMuxProvider.isInSync();
    const bool     pending      = m_pending;
    const uint64_t nowMs        = localNowMs();
    const uint64_t pendingAgeMs = pending ? (nowMs - m_pendingT1_64) : 0ULL;

    LOG_INFO("════════ Zumo Serial Time Sync ════════════════════════════");
    LOG_INFO("SerialMux in sync:     %s", inSync ? "YES" : "NO");
    LOG_INFO("Ping period:           %lu ms", (unsigned long)m_pingPeriodMs);
    LOG_INFO("Next seq:              %lu", (unsigned long)m_seq);
    LOG_INFO("Pending request:       %s", pending ? "YES" : "NO");
    if (pending)
    {
        LOG_INFO("  Pending seq:         %lu", (unsigned long)m_pendingSeq);
        LOG_INFO("  Pending t1_32:       %lu ms", (unsigned long)m_pendingT1_32);
        LOG_INFO("  Pending t1_64 age:   %llu ms", (unsigned long long)pendingAgeMs);
    }
    LOG_INFO("Best RTT (min):        %lu ms", (unsigned long)m_minRttMs);
    LOG_INFO("Clock offset Zumo->ESP:%+lld ms", (long long)m_zumoToEspOffsetMs);
    LOG_INFO("Accepted samples:      %u", (unsigned)m_zumoGoodSamples);
    LOG_INFO("Zumo synced (>=3):     %s", (m_zumoGoodSamples >= 3U) ? "YES" : "NO");
    LOG_INFO("═══════════════════════════════════════════════════════════");
}

void TimeSync::logStatus() const
{
    LOG_INFO("================= TimeSync Detailed Status =================");
    logRtcStatus();
    logZumoStatus();
    LOG_INFO("============================================================");
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
