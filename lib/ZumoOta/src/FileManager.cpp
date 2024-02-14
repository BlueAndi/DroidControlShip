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
 * @brief  FileManager realization
 *
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "FileManager.h"
#include <LittleFS.h>
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <FS.h>
/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/
#ifndef CONFIG_LOG_SEVERITY
#define CONFIG_LOG_SEVERITY (Logging::LOG_LEVEL_INFO)
#endif /* CONFIG_LOG_SEVERITY */
/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/
uint8_t FileManager::m_filePosition = 0;
/******************************************************************************
 * Public Methods
 *****************************************************************************/

FileManager::FileManager()
{
}

FileManager::~FileManager()
{
}

bool FileManager::init()
{
    if (!LittleFS.begin(true))
    {
        LOG_ERROR("Failed to mount file system.");
        return false;
    }
    return true;
}

size_t FileManager::readBytes(const char* firmwareName, uint8_t* buffer)
{
    /*Number of bytes to read.*/
    const size_t BYTES_TO_READ = 128;
    int16_t readBytes = 0;

    /*Open the File with the given firmware name in LittleFS.*/
    File firmwareFile = LittleFS.open(firmwareName, "r");

    if (firmwareFile)
    {
        /*Set the file's read position to the current position.*/
        firmwareFile.seek(m_filePosition);
        /*Read the specified number of bytes from the file.*/
        readBytes = firmwareFile.read(buffer, BYTES_TO_READ);
        /*Update the current file position for the next call.*/
        m_filePosition = firmwareFile.position();
        /*Close the file. */
        firmwareFile.close();
    }

    /*Perform actions if readBytes is less than 128 bytes.*/
    if ((BYTES_TO_READ > readBytes) && (0 != readBytes))
    {
        /*If the number of bytes read is less than 128, fill the remaining space with a pattern (e.g., 0xFF).*/
        for (size_t i = readBytes; i < BYTES_TO_READ; ++i)
        {
            buffer[i] = 0xFF; /**<Use any suitable pattern here, like 0xFF or 0x00.*/
        }
        m_filePosition = 0; /**<Reset the file position if the file is completely read.*/
    }

    return readBytes;
}
