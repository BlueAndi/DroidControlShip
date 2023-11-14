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

void Upload::handleUploadButtonPress(AsyncWebServerRequest *request)
{
    request->send(200, "text/plain", "Upload Button gedrueckt");
 
}

void Upload::handleFileUpload(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final)
{
    static File file;
    String updatedFilename = filename;

    if (!filename.startsWith("/"))
    {
        updatedFilename = "/" + filename;
    }

    if (!index)
    {
        // Dies ist der erste Datenblock, Datei öffnen oder erstellen
        file = LittleFS.open(updatedFilename, "w");

        if (!file)
        {
            LOG_DEBUG("Failed to open file for writing");
            file.close();
            return;
        }
    }
    if (file)
    {
      // Datei im Request-Objekt speichern
      request->_tempFile = file;
      LOG_DEBUG("Upload Start: " + String(filename));
    }
    else
    {
      return;
    }
    if(len)
    {
        // Daten in die Datei schreiben
        request->_tempFile.write(data, len);
    }

    // Wenn dies der letzte Datenblock ist, Datei schließen
    if (final)
    {
        file.close();
        request->redirect("/filelist");
    }

    // Überprüfe, ob die Datei im Dateisystem liegt
    if (isFileUploaded(request, updatedFilename))
    {
        LOG_DEBUG("Die hochgeladene Datei liegt im Dateisystem.");
    }
    else
    {
        LOG_DEBUG("Die hochgeladene Datei liegt nicht im Dateisystem.");
    }
}

bool Upload::isFileUploaded(AsyncWebServerRequest *request, String filename)
{
    if(LittleFS.exists(filename))
    {
        return true;
    } 
    else
    {
        return false;
    }
}









