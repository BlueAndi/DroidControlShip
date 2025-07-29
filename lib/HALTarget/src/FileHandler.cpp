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
 *  @brief  File Handler
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "FileHandler.h"
#include <FS.h>
#include <LittleFS.h>
#include <Logging.h>

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
    if (false == LittleFS.begin(true))
    {
        LOG_ERROR("Failed to mount file system.");
    }
}

FileHandler::~FileHandler()
{
    LittleFS.end();
}

size_t FileHandler::readFile(const String& fileName, char* outBuffer, const uint32_t maxBufferSize) const
{
    size_t bytesRead = 0U;
    File   file      = LittleFS.open(fileName, "r");

    if ((false == file) || (true == file.isDirectory()))
    {
        LOG_ERROR("Failed to open file \"%s\".", fileName.c_str());
    }
    else
    {
        bytesRead = file.readBytes(outBuffer, maxBufferSize);

        if (true == file.available())
        {
            LOG_ERROR("File \"%s\" is too big for the buffer.", fileName.c_str());
            bytesRead = 0;
        }
        else
        {
            /* File read successfully. */
        }
        file.close();
    }

    return bytesRead;
}

size_t FileHandler::writeFile(const String& fileName, const char* buffer, const uint32_t bufferSize)
{
    size_t bytesWritten = 0U;
    File   file         = LittleFS.open(fileName, "w");

    if ((false == file) || (true == file.isDirectory()))
    {
        LOG_ERROR("Failed to open file \"%s\".", fileName.c_str());
    }
    else
    {
        const void*    vBuffer = buffer;
        const uint8_t* uBuffer = static_cast<const uint8_t*>(vBuffer);

        bytesWritten = file.write(uBuffer, bufferSize);

        if (bytesWritten != bufferSize)
        {
            LOG_ERROR("Failed to write file \"%s\".", fileName.c_str());
            bytesWritten = 0U;
        }
        else
        {
            /* File written successfully. */
        }
        file.close();
    }

    return bytesWritten;
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
