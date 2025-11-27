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
 */

#include "NTPClient.h"
#include <chrono>

/******************************************************************************
 * Public Methods
 *****************************************************************************/

NTPClient::NTPClient(WiFiUDP& udp, const char* poolServerName, int32_t timeOffset, uint32_t updateInterval) :
    m_udp(udp),
    m_server(poolServerName),
    m_timeOffsetSec(timeOffset),
    m_updateIntervalMs(updateInterval)
{
}

void NTPClient::begin()
{
    /* No-op in simulation mode */
}

void NTPClient::setTimeOffset(int32_t seconds)
{
    m_timeOffsetSec = seconds;
}

void NTPClient::setPoolServerName(const char* server)
{
    m_server = server;
}

bool NTPClient::update()
{
    return true; /* Always succeeds */
}

uint32_t NTPClient::getEpochTime()
{
    using namespace std::chrono;
    const system_clock::time_point now   = system_clock::now();
    const seconds::rep             secs  = duration_cast<seconds>(now.time_since_epoch()).count();
    const int64_t                  epoch = static_cast<int64_t>(secs) + static_cast<int64_t>(m_timeOffsetSec);
    return static_cast<uint32_t>(epoch);
}