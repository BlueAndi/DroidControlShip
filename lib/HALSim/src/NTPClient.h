/*
 * Lightweight NTPClient replacement for native simulation builds.
 * Mimics the subset of the Arduino NTPClient API used by TimeSync.
 */
#ifndef SIM_NTPCLIENT_H
#define SIM_NTPCLIENT_H

#include <chrono>
#include <cstdint>
#include "WiFiUdp.h"

class NTPClient
{
public:
    NTPClient(WiFiUDP& udp, const char* poolServerName, long timeOffset, unsigned long updateInterval) :
        m_udp(udp),
        m_server(poolServerName),
        m_timeOffsetSec(timeOffset),
        m_updateIntervalMs(updateInterval)
    {
        (void)m_updateIntervalMs;
    }

    void begin()
    {
        /* No-op in simulation. */
    }

    void setTimeOffset(long seconds)
    {
        m_timeOffsetSec = seconds;
    }

    void setPoolServerName(const char* server)
    {
        m_server = server;
    }

    bool update()
    {
        /* Always succeeds in simulation. */
        return true;
    }

    unsigned long getEpochTime()
    {
        using namespace std::chrono;
        const auto now   = system_clock::now();
        const auto secs  = duration_cast<seconds>(now.time_since_epoch()).count();
        const auto epoch = static_cast<long long>(secs) + static_cast<long long>(m_timeOffsetSec);
        return static_cast<unsigned long>(epoch);
    }

private:
    WiFiUDP&      m_udp;
    const char*   m_server;
    long          m_timeOffsetSec;
    unsigned long m_updateIntervalMs;
};

#endif /* SIM_NTPCLIENT_H */
