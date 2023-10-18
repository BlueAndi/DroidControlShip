/* MIT License

Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 *  @brief  File Reader
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "FileReader.h"
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

FileReader::FileReader()
{
    if (false == LittleFS.begin(true))
    {
        LOG_ERROR("Failed to mount file system.");
    }
}

FileReader::~FileReader()
{
    LittleFS.end();
}

size_t FileReader::readFile(const String& fileName, char* outBuffer, const uint32_t maxBufferSize)
{
    size_t bytesRead = 0;
    File   file      = LittleFS.open(fileName, "r");

    if ((false == file) || (file.isDirectory()))
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

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
