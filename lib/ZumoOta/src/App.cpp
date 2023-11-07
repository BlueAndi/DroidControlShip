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
 * @brief  ZumoOta application
 * @author Decareme Pauline Ngangnou <ngandeca@yahoo.fr>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "App.h"
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>  
#include "FileManager.h"
#include "WebServerCustom.h"

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
WebServerCustom webserver;
FileManager fileManager;
Upload upload;

/** Serial interface baudrate. */
static const uint32_t SERIAL_BAUDRATE = 115200U;

/*defines the WiFi Credentials*/
const char* ssid = "your_ssid";
const char* password = "WiFipassword";
/******************************************************************************
 * Public Methods
 *****************************************************************************/

void App::setup()
{
  Serial.begin(SERIAL_BAUDRATE);

    // Access Point Modus start
  /*WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP-Adresse: ");
  Serial.println(IP);*/

  //Station Mode start
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi Failed!\n");
        return;
    }

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    webserver.begin();
    fileManager.createUploadPage();
    
    String pageContent = fileManager.readUploadPage();
    
    if (!pageContent.isEmpty()) { 
      
        webserver.on("/", HTTP_GET, [pageContent](AsyncWebServerRequest *request) {
          request->send(200, "text/html", pageContent); 
        });
    }

  webserver.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
   upload.handleUploadButtonPress();
    request->send(200, "text/plain", "Upload Button gedrueckt");
});
}

void App::loop()
{
    
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

