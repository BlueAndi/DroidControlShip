/* MIT License
 *
 * Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>
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
 *  @brief  File Reader
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "FileReader.h"
#include <stdio.h>
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
}

FileReader::~FileReader()
{
}

size_t FileReader::readFile(const String& fileName, char* outBuffer, const uint32_t maxBufferSize)
{
    size_t readBytes = 0;
    FILE*  file      = fopen(fileName.c_str(), "r");

    if (nullptr == file)
    {
        LOG_ERROR("Failed to open file \"%s\".", fileName.c_str());
    }
    else
    {
        readBytes = fread(outBuffer, sizeof(char), maxBufferSize, file);

        if (ferror(file) != 0)
        {
            LOG_ERROR("Error ocurred while reading file \"%s\".", fileName.c_str());
            readBytes = 0;
        }
        else if (feof(file) == 0)
        {
            LOG_ERROR("File \"%s\" is too big for the buffer.", fileName.c_str());
            readBytes = 0;
        }
        else
        {
            /* File read successfully. */
        }
        fclose(file);
    }

    return readBytes;
}

size_t FileReader::writeFile(const String& fileName, const char* buffer, const uint32_t bufferSize)
{
    size_t writtenBytes = 0;
    FILE*  file         = fopen(fileName.c_str(), "w");

    if (nullptr == file)
    {
        LOG_ERROR("Failed to open file \"%s\".", fileName.c_str());
    }
    else
    {
        writtenBytes = fwrite(buffer, sizeof(char), bufferSize, file);

        if (ferror(file) != 0)
        {
            LOG_ERROR("Error ocurred while writing file \"%s\".", fileName.c_str());
            writtenBytes = 0;
        }
        else
        {
            /* File written successfully. */
            LOG_DEBUG("File \"%s\" written successfully.", fileName.c_str());
        }
        fclose(file);
    }

    return writtenBytes;
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
