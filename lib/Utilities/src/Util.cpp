/* MIT License
 *
 * Copyright (c) 2019 - 2023 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Utility
 * @author Yann Le Glaz <yann_le@web.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Util.h"
#include <stdio.h>

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

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

extern bool Util::strToUInt8(const String& str, uint8_t& value)
{
    bool          success = false;
    char*         endPtr  = nullptr;
    unsigned long tmp     = 0UL;

    errno = 0;
    tmp   = strtoul(str.c_str(), &endPtr, 0);

    if ((0 == errno) && (nullptr != endPtr) && ('\0' == *endPtr) && (str.c_str() != endPtr) && (UINT8_MAX >= tmp))
    {
        value   = static_cast<uint8_t>(tmp);
        success = true;
    }

    return success;
}

extern bool Util::strToUInt16(const String& str, uint16_t& value)
{
    bool          success = false;
    char*         endPtr  = nullptr;
    unsigned long tmp     = 0UL;

    errno = 0;
    tmp   = strtoul(str.c_str(), &endPtr, 0);

    if ((0 == errno) && (nullptr != endPtr) && ('\0' == *endPtr) && (str.c_str() != endPtr) && (UINT16_MAX >= tmp))
    {
        value   = static_cast<uint16_t>(tmp);
        success = true;
    }

    return success;
}

extern bool Util::strToInt32(const String& str, int32_t& value)
{
    bool  success = false;
    char* endPtr  = nullptr;
    long  tmp     = 0L;

    errno = 0;
    tmp   = strtol(str.c_str(), &endPtr, 0);

    if ((0 == errno) && (nullptr != endPtr) && ('\0' == *endPtr) && (str.c_str() != endPtr) && (INT32_MAX >= tmp))
    {
        value   = static_cast<int32_t>(tmp);
        success = true;
    }

    return success;
}

extern bool Util::strToUInt32(const String& str, uint32_t& value)
{
    bool          success = false;
    char*         endPtr  = nullptr;
    unsigned long tmp     = 0UL;

    errno = 0;
    tmp   = strtoul(str.c_str(), &endPtr, 0);

    if ((0 == errno) && (nullptr != endPtr) && ('\0' == *endPtr) && (str.c_str() != endPtr) && (UINT32_MAX >= tmp))
    {
        value   = static_cast<uint32_t>(tmp);
        success = true;
    }

    return success;
}

extern String Util::uint32ToHex(uint32_t value)
{
    char buffer[9]; /* Contains a 32-bit value in hex */

    (void)snprintf(buffer, UTIL_ARRAY_NUM(buffer), "%x", value);

    return String(buffer);
}

extern uint32_t Util::hexToUInt32(const String& str)
{
    uint32_t value   = 0U;
    uint32_t idx     = 0U;
    bool     isError = false;

    if ((0U != str.startsWith("0x")) || (0U != str.startsWith("0X")))
    {
        idx = 2U;
    }

    while ((str.length() > idx) && (false == isError))
    {
        value *= 16U;

        if (('0' <= str[idx]) && ('9' >= str[idx]))
        {
            value += static_cast<uint32_t>(str[idx] - '0');
        }
        else if (('a' <= str[idx]) && ('f' >= str[idx]))
        {
            value += static_cast<uint32_t>(str[idx] - 'a') + 10U;
        }
        else if (('A' <= str[idx]) && ('F' >= str[idx]))
        {
            value += static_cast<uint32_t>(str[idx] - 'A') + 10U;
        }
        else
        {
            value   = 0U;
            isError = true;
        }

        ++idx;
    }

    return value;
}

extern void Util::int16ToByteArray(uint8_t* buffer, size_t size, int16_t value)
{
    if ((nullptr != buffer) && (sizeof(int16_t) <= size))
    {
        buffer[0U] = ((value >> 8U) & 0xFF);
        buffer[1U] = ((value >> 0U) & 0xFF);
    }
}

extern void Util::uint16ToByteArray(uint8_t* buffer, size_t size, uint16_t value)
{
    if ((nullptr != buffer) && (sizeof(uint16_t) <= size))
    {
        buffer[0U] = ((value >> 8U) & 0xFF);
        buffer[1U] = ((value >> 0U) & 0xFF);
    }
}

extern void Util::int32ToByteArray(uint8_t* buffer, size_t size, int32_t value)
{
    if ((nullptr != buffer) && (sizeof(int32_t) <= size))
    {
        uint16_t hiBytes  = ((value >> 16U) & 0xFFFF);
        uint16_t lowBytes = ((value >> 0U) & 0xFFFF);

        buffer[0U] = ((hiBytes >> 8U) & 0xFF);
        buffer[1U] = ((hiBytes >> 0U) & 0xFF);
        buffer[2U] = ((lowBytes >> 8U) & 0xFF);
        buffer[3U] = ((lowBytes >> 0U) & 0xFF);
    }
}

extern void Util::uint32ToByteArray(uint8_t* buffer, size_t size, uint32_t value)
{
    if ((nullptr != buffer) && (sizeof(uint32_t) <= size))
    {
        uint16_t hiBytes  = ((value >> 16U) & 0xFFFF);
        uint16_t lowBytes = ((value >> 0U) & 0xFFFF);

        buffer[0U] = ((hiBytes >> 8U) & 0xFF);
        buffer[1U] = ((hiBytes >> 0U) & 0xFF);
        buffer[2U] = ((lowBytes >> 8U) & 0xFF);
        buffer[3U] = ((lowBytes >> 0U) & 0xFF);
    }
}

extern bool Util::byteArrayToInt16(const uint8_t* buffer, size_t size, int16_t& value)
{
    bool isSuccess = false;

    if ((nullptr != buffer) && (sizeof(int16_t) <= size))
    {
        value =
            ((static_cast<int16_t>(buffer[0U]) << 8U) & 0xFF00) | ((static_cast<int16_t>(buffer[1U]) << 0U) & 0x00FF);

        isSuccess = true;
    }

    return isSuccess;
}

extern bool Util::byteArrayToUint16(const uint8_t* buffer, size_t size, uint16_t& value)
{
    bool isSuccess = false;

    if ((nullptr != buffer) && (sizeof(uint16_t) <= size))
    {
        value =
            ((static_cast<uint16_t>(buffer[0U]) << 8U) & 0xFF00) | ((static_cast<uint16_t>(buffer[1U]) << 0U) & 0x00FF);

        isSuccess = true;
    }

    return isSuccess;
}

extern bool Util::byteArrayToUint32(const uint8_t* buffer, size_t size, uint32_t& value)
{
    bool isSuccess = false;

    if ((nullptr != buffer) && (sizeof(uint32_t) <= size))
    {
        value = ((static_cast<uint32_t>(buffer[0U]) << 24U) & 0xFF000000) |
                ((static_cast<uint32_t>(buffer[1U]) << 16U) & 0x00FF0000) |
                ((static_cast<uint32_t>(buffer[2U]) << 8U) & 0x0000FF00) |
                ((static_cast<uint32_t>(buffer[3U]) << 0U) & 0x000000FF);

        isSuccess = true;
    }

    return isSuccess;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
