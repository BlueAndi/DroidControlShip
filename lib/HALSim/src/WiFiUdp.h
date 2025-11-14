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
 * @brief Lightweight WiFiUDP stub used in native simulation builds.
 *
 * This class provides a minimal placeholder implementation of Arduino's
 * WiFiUDP API. It is used to satisfy dependencies in components such as
 * TimeSync or NTPClient during simulation builds where no actual UDP
 * communication is required or performed.
 *
 * The purpose of this stub is solely to ensure interface compatibility;
 * all networking functionality is intentionally omitted.
 *
 * @author Tobias Haeckel <tobias.haeckel@gmx.net>
 *
 * @addtogroup HALSim
 * @{
 */

#ifndef SIM_WIFIUDP_H
#define SIM_WIFIUDP_H

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * @brief Minimal stub of the Arduino WiFiUDP class.
 *
 * This class provides only the constructor and destructor and no real
 * networking operations. It exists purely to allow code depending on
 * WiFiUDP to compile in a native (non-embedded) simulation context.
 */
class WiFiUDP
{
public:
    /**
     * @brief Default constructor.
     *
     * Creates an empty WiFiUDP stub instance.
     */
    WiFiUDP() = default;

    /**
     * @brief Default destructor.
     */
    ~WiFiUDP() = default;
};

#endif /* SIM_WIFIUDP_H */
/** @} */
