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
#include<Logging.h>
#include <map> 
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

Upload::Upload()
{   
}

Upload::~Upload()
{    
}

void Upload::handleFileUpload(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final)
{
    String updatedFilename = filename;
    

    if (!filename.startsWith("/"))
    {
        updatedFilename = "/" + filename;
    }

    if (index==0)
    {   
        
        /*Save file in the request object*/
        request->_tempFile = LittleFS.open(updatedFilename, "w");
        LOG_DEBUG("Upload Start: " + String(updatedFilename));
    }
    else
    {
        LOG_ERROR("Problem to save the request object!");
       
    }
    if(len)
    {
        /* Write data to the file*/
        request->_tempFile.write(data, len);
    }

    /* If this is the last data block, close the file*/
    if (final)
    {
        request->_tempFile.close();
       /* Check if the file exits in FileSystem */
        if (LittleFS.exists(updatedFilename))
        {
            /* Open the file in read mode */
            File file = LittleFS.open(updatedFilename, "r");

            if (file)
            {
                /* Move to the end of the file */
                file.seek(0, SeekEnd);

                /* Get the current position in the file content (the size of the file) */
                size_t fileSize = file.position();

                /* Move back to the beginning of the file */
                file.seek(0, SeekSet);

                LOG_DEBUG("Size of " + updatedFilename + ": " + String(fileSize));

                /* Close the file */
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
            request->redirect("/filelist");
    
        }
    else
    {
        LOG_ERROR("Please keep trying this is not the last datablock!");
       
    }
}









