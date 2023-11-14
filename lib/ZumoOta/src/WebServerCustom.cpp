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
 * @brief  WebServer realization
 * 
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "WebServerCustom.h"
#include <LittleFS.h>
#include <FS.h>
#include <Arduino.h>
#include <Logging.h>
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
Upload upload;
/******************************************************************************
 * Public Methods
 *****************************************************************************/

WebServerCustom::WebServerCustom()
{
    
}

WebServerCustom::~WebServerCustom()
{
}

String WebServerCustom::listFiles(bool ishtml)
{
  String returnText = "";
  LOG_DEBUG("Listing files stored on LittleFS");
  File root = LittleFS.open("/");
  File foundfile = root.openNextFile();
  if (ishtml) {
    returnText += "<table><tr><th align='left'>Name</th><th align='left'>Size</th></tr>";
  }
  while (foundfile) {
    if (ishtml) {
      returnText += "<tr align='left'><td>" + String(foundfile.name()) + "</td><td>" + humanReadableSize(foundfile.size()) + "</td></tr>";
    } else {
      returnText += "File: " + String(foundfile.name()) + "\n";
    }
    foundfile = root.openNextFile();
  }
  if (ishtml) {
    returnText += "</table>";
  }
  root.close();
  foundfile.close();
  return returnText;
}

String WebServerCustom:: humanReadableSize(const size_t bytes)
{
  if (bytes < 1024) return String(bytes) + " B";
  else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
  else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}

void WebServerCustom::init()
{
   server.on("/filelist", HTTP_GET, [this](AsyncWebServerRequest *request)
    {
        String fileList = listFiles(true);
        request->send(200,"text/html", fileList);
    });

    // ... Weitere Routen und Konfigurationen ...
   
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        File file = LittleFS.open("/upload.html", "r");
    if(file)
    {
        request->send(LittleFS, "/upload.html","text/html");
        file.close();
    }else
    {
        request->send(404, "text/plain", "File not found");
    }
    });

    server.begin();
}

void WebServerCustom::handleUploadRequest()
{
    server.on("/upload", HTTP_POST, [this](AsyncWebServerRequest *request) {
    }, [this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
        // Verarbeite den Dateiupload
        upload.handleFileUpload(request, filename, index, data, len, final);
    });
}


/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/


