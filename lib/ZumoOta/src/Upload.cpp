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
 * @brief  Upload realization
 *
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Upload.h"
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <Logging.h>
#include <map>
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
/* Maximale Dateigröße. */
const size_t MAX_FILE_SIZE = 10 * 1024 * 1024;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

Upload::Upload()
{
    expectedfileSize = 0U;
}

Upload::~Upload()
{
}

void Upload::handleFileUpload(AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data,
                              size_t len, bool final)
{
    String updatedFilename = filename;
    size_t flashfileSize   = 0U;

    /* Initialize fileSize. */
    if (false == filename.startsWith("/"))
    {
        updatedFilename = "/" + filename;
    }

    if (0 == index)
    {
        /* Get file size from special header if available. */
        AsyncWebHeader* headerXFileSizeFirmware = request->getHeader("X-File-Size-Firmware");
        /* Firmware file size available? */
        if (nullptr != headerXFileSizeFirmware)
        {
            /* If conversion fails, it will contain UPDATE_SIZE_UNKNOWN. */
            (void)Util::strToUInt32(headerXFileSizeFirmware->value(), expectedfileSize);
            LOG_DEBUG("File size: %d", expectedfileSize);
            if (expectedfileSize > MAX_FILE_SIZE)
            {
                /* If file size is too large, abort upload. */
                request->send(413, "text/plain",
                              "File size exceeds the limit."); // HTTP-Fehlercode 413 (Payload Too Large)
                return;
                LOG_ERROR("File size exceeds the limit.");
            }
        }
        else
        {
            LOG_DEBUG("No file size given.");
        }

        /* Save file in the request object. */
        request->_tempFile = LittleFS.open(updatedFilename, "w");
        LOG_DEBUG("Upload Start: " + String(updatedFilename));
    }

    if (len)
    {
        /* Write data to the file. */
        request->_tempFile.write(data, len);
    }

    /* If this is the last data block, close the file. */
    if (true == final)
    {
        LOG_DEBUG("Last block received - final: true");
        request->_tempFile.close();

        /* Check if the file exists in FileSystem. */
        if (LittleFS.exists(updatedFilename))
        {
            /* Open the file in read mode. */
            File file = LittleFS.open(updatedFilename, "r");

            if (file)
            {
                /* Get the size of the file. */
                flashfileSize = file.size();
                LOG_DEBUG("Size of " + updatedFilename + " in FileSystem: " + String(flashfileSize));

                /* Close the file. */
                file.close();
            }
            else
            {
                LOG_ERROR("Failed to open " + updatedFilename);
            }
        }
        else
        {
            LOG_DEBUG(updatedFilename + " is not in FileSystem.");
        }

        if (expectedfileSize == flashfileSize)
        {
            LOG_DEBUG("Received file size matches Content-Length header (" + String(expectedfileSize) + ")");
        }
        else
        {
            LOG_ERROR("Received file size (" + String(flashfileSize) + ") does not match Content-Length header (" +
                      String(expectedfileSize) + ")!");
        }
        request->redirect("/filelist");
    }

    else
    {
        LOG_ERROR("Please keep trying this is not the last data block!");
    }
}
