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
#include "NetworkCustom.h"
#include <WiFi.h>
#include <Logging.h>
#include "MySettings.h"

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
MySettings settings;
/******************************************************************************
 * Public Methods
 *****************************************************************************/
NetworkCustom::NetworkCustom()
{
    connected = false;
    connectAttempts = 0;
}

NetworkCustom::~NetworkCustom()
{
    
}

void NetworkCustom::connectToWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(settings.getWiFiSSID(), settings.getWiFiPassword());
    delay(10000);

    if (WiFi.status() == WL_CONNECTED)
    {
        connected = true;
        LOG_DEBUG("Connected to WiFi. IP Address: %s", WiFi.localIP().toString().c_str());
    }
    else
    {
        connected = false;
        checkConnection();
    }
}

void NetworkCustom::switchToAPMode()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(settings.getapSSID(), settings.getapPassword());
    LOG_DEBUG("IP Address: %s", WiFi.softAPIP().toString().c_str());
    connected = true;
    
}

void NetworkCustom::checkConnection()
{
    while(!connected)
    {
        LOG_ERROR("Connection failed. Retry a connection...");
        connectAttempts++;
        
        if (connectAttempts >= 3)
        {
            switchToAPMode();
            connectAttempts = 0;
        }
        delay(30000);
    }
}



/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/