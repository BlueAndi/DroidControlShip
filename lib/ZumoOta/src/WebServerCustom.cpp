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
#include <SettingsHandler.h>

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
    File root      = LittleFS.open("/");
    File foundfile = root.openNextFile();
    if (ishtml)
    {
        returnText += "<style>.filename { color: black; }</style>";
        returnText +=
            "<table style='background-color: yellow;'><tr><th align='left'>Name</th><th align='right'>Size</th></tr>";
    }
    while (foundfile)
    {
        if (ishtml)
        {
            returnText += "<tr align='left'><td class='filename'>" + String(foundfile.name()) + "</td></tr>";
            returnText += "<tr align='right'><td>" + humanReadableSize(foundfile.size()) + "</td></tr>";
        }
        else
        {
            returnText += "File: " + String(foundfile.name()) + "\n";
        }
        foundfile = root.openNextFile();
    }
    if (ishtml)
    {
        returnText += "</table>";
    }
    root.close();
    foundfile.close();
    return returnText;
}

String WebServerCustom::humanReadableSize(const size_t bytes)
{
    const char* sizes[] = {"B", "KB", "MB", "GB"};
    int         i       = 0;
    double      value   = static_cast<double>(bytes);

    while (value >= 1024 && i < 3)
    {
        value /= 1024;
        i++;
    }
    /* Adjust the buffer size as needed.*/
    char buffer[16];

    if (i == 0)
    {
        snprintf(buffer, sizeof(buffer), "%.0f %s", value, sizes[i]);
    }
    else
    {
        snprintf(buffer, sizeof(buffer), "%.2f %s", value, sizes[i]);
    }

    return String(buffer);
}

void WebServerCustom::init()
{
    server.on("/filelist", HTTP_GET,
              [this](AsyncWebServerRequest* request)
              {
                  String fileList = listFiles(true);
                  request->send(200, "text/html", fileList);
              });

    /*...  Additional routes and configurations ...*/

    server.on("/upload", HTTP_GET,
              [](AsyncWebServerRequest* request) { request->send(LittleFS, "/upload.html", "text/html"); });

    server.on(
        "/", HTTP_GET,
        [](AsyncWebServerRequest* request)
        {
            SettingsHandler& settings = SettingsHandler::getInstance();
            if (!request->authenticate(settings.getWebServerUser().c_str(), settings.getWebServerPassword().c_str()))
            {
                return request->requestAuthentication();
            }
            else
            {
                request->send(LittleFS, "/login.html", "text/html");
            }
        });

    server.begin();
}

void WebServerCustom::handleUploadRequest()
{
    server.on(
        "/upload", HTTP_POST, [this](AsyncWebServerRequest* request) {},
        [this](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final)
        {
            /* Process the file upload.*/
            upload.handleFileUpload(request, filename, index, data, len, final);
        });
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
