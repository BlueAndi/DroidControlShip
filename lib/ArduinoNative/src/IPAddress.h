/* MIT License
 *
 * Copyright (c) 2023 - 2024 Andreas Merkle <web@blue-andi.de>
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
 *  @brief  Minimal IPAddress API
 *  @author Norbert Schulz <schulz.norbert@gmail.com>
 *
 * @addtogroup Arduino
 *
 * @{
 */

#ifndef IPADDRESS_H
#define IPADDRESS_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>

#include <WString.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Provide IPAddress class compatible to Arduino.
 *
 * Only subsets of the API are implemented based on actual application needs.
 */
class IPAddress
{
public:
    /**
     * Default contstruction
     *
     * @param[in] addr The raw 32bit IP V4 address (default: 0U).
     */
    IPAddress(uint32_t addr = 0U) : m_addr(addr)
    {
    }

    /**
     *  Construction from octets.
     *
     * @param[in] o1 Address bits 24..31.
     * @param[in] o2 Address bits 16..23.
     * @param[in] o3 Address bits  8..15.
     * @param[in] 04 Address vits  0.. 7.
     */
    IPAddress(uint8_t o1, uint8_t o2, uint8_t o3, uint8_t o4) : m_addr(toUint32(o1, o2, o3, o4))
    {
    }

    /**
     * Define copy constructor
     *
     * @param[in] other The other IPAddress instance to clone.
     */
    IPAddress(const IPAddress& other) : m_addr(other.m_addr)
    {
    }

    /**
     * class destruction
     */
    virtual ~IPAddress()
    {
    }

    /**
     * Convert to IPV4 String
     * 
     * @return string representation of address.
     */
    String toString() const;

    /**
     * Initialize from a "x.x.x.x" string.
     *
     * @param[in] str IPV4 numeric string
     *
     * @return bool true or false
     */
    bool fromString(const String& str);

    /**
     * Define class comparison for equal.
     *
     * @param[in] other The other instance to compare with.
     */
    bool operator==(const IPAddress& other) const
    {
        return m_addr == other.m_addr;
    }

    /**
     * Define class comparison for not equal.
     *
     * @param[in] other The Other instance to compare with.
     */
    bool operator!=(const IPAddress& other) const
    {
        return m_addr != other.m_addr;
    }

    /**
     * Define class assignment.
     *
     * @param[in] other The other instance to assign from.
     */
    const IPAddress& operator=(const IPAddress& other)
    {
        if (this != &other)
        {
            m_addr = other.m_addr;
        }

        return *this;
    }

    /**
     * Get raw address value
     *
     * @return address value
     */
    uint32_t raw(void) const
    {
        return m_addr;
    }

private:
    /**
     * Combine octets into 32bit IP V4 address value
     */
    uint32_t toUint32(uint8_t o1, uint8_t o2, uint8_t o3, uint8_t o4);

    uint32_t m_addr; /**< IP V4 address value. */
};

inline uint32_t IPAddress::toUint32(uint8_t o1, uint8_t o2, uint8_t o3, uint8_t o4)
{
    return ((uint32_t)o1 << 24) | ((uint32_t)o2 << 16) | ((uint32_t)o3 << 8) | ((uint32_t)o4 << 0);
}

#endif /* IPADDRESS_H */

/** @} */