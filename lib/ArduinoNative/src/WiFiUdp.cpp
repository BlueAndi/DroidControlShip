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
 * @brief  UDP Client for Native platform. Emulation of WiFiUDP.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "WiFiUdp.h"
#include <Util.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

uint8_t WiFiUDP::begin(uint16_t port)
{
    UTIL_NOT_USED(port);
    return 0;
}

void WiFiUDP::stop()
{
}

int WiFiUDP::beginPacket(IPAddress ip, uint16_t port) // NOLINT(performance-unnecessary-value-param)
{
    UTIL_NOT_USED(ip);
    UTIL_NOT_USED(port);
    return 0;
}

size_t WiFiUDP::write(const uint8_t* buffer, size_t length)
{
    UTIL_NOT_USED(buffer);
    UTIL_NOT_USED(length);
    return 0U;
}

int WiFiUDP::endPacket()
{
    return 0;
}

int WiFiUDP::getWriteError()
{
    return 0;
}

void WiFiUDP::flush()
{
}

int WiFiUDP::parsePacket()
{
    return 0;
}

int WiFiUDP::available() const
{
    return 0;
}

int WiFiUDP::read(uint8_t* buffer, size_t size)
{
    return readBytes(buffer, size);
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void WiFiUDP::print(const char str[])
{
    /* Not implemented. */
    UTIL_NOT_USED(str);
}

void WiFiUDP::print(uint8_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::print(uint16_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::print(uint32_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::print(int8_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::print(int16_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::print(int32_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(const char str[])
{
    /* Not implemented. */
    UTIL_NOT_USED(str);
}

void WiFiUDP::println(uint8_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(uint16_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(uint32_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(int8_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(int16_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

void WiFiUDP::println(int32_t value)
{
    /* Not implemented. */
    UTIL_NOT_USED(value);
}

size_t WiFiUDP::readBytes(uint8_t* buffer, size_t length)
{
    UTIL_NOT_USED(buffer);
    UTIL_NOT_USED(length);
    return 0;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
