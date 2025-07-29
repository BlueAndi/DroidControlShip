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
 *  @brief  File Handler
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "FileHandler.h"
#include <stdio.h>
#include <Logging.h>
#include <Util.h>

/******************************************************************************
 * Macros
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

FileHandler::FileHandler()
{
}

FileHandler::~FileHandler()
{
}

size_t FileHandler::readFile(const String& fileName, char* outBuffer, const uint32_t maxBufferSize) const
{
    UTIL_NOT_USED(fileName);
    UTIL_NOT_USED(outBuffer);
    UTIL_NOT_USED(maxBufferSize);
    LOG_WARNING("FileHandler::readFile not implemented for test environment.");
    return 0U;
}

size_t FileHandler::writeFile(const String& fileName, const char* buffer, const uint32_t bufferSize)
{
    UTIL_NOT_USED(fileName);
    UTIL_NOT_USED(buffer);
    UTIL_NOT_USED(bufferSize);
    LOG_WARNING("FileHandler::writeFile not implemented for test environment.");
    return 0U;
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
