/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
 *  @brief  Implementation of IPAddress
 *  @author Norbert Schulz <schulz.norbert@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdio.h>

#include "IPAddress.h"

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

String IPAddress::toString() const
{
    const uint8_t* p(reinterpret_cast<const uint8_t*>(&m_addr));
    char           buf[20];

    sprintf(buf, "%d.%d.%d.%d", p[3], p[2], p[1], p[0]);
    return String(buf);
}

bool IPAddress::fromString(const String& str)
{
    uint8_t octet[4];
    bool    result(false);

    if (4 == sscanf(str.c_str(), "%hhu.%hhu.%hhu.%hhu", &octet[0], &octet[1], &octet[2], &octet[3]))
    {
        m_addr = toUint32(octet[0], octet[1], octet[2], octet[3]);
        result = true;
    }

    return result;
}