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
#include "NetworkConnection.h"
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
/* Maximum number of connection attempts. */
const int MAX_CONNECTION_ATTEMPTS = 3; 

/* Delay between connection retries in milliseconds. */
const int RETRY_DELAY_MS = 10000; 

/* Delay between reconnect attempts in milliseconds. */
const int RECONNECT_DELAY_MS = 30000;  
/******************************************************************************
 * Public Methods
 *****************************************************************************/
NetworkConnection::NetworkConnection()
{
    m_connected = false;
    m_connectAttempts = 0;
}

NetworkConnection::~NetworkConnection()
{
    
}

void NetworkConnection::connectToWiFi()
{
    int connectAttempts = 0;
    /* Loop to attempt multiple connections with provided credentials. */
    while(!m_connected && connectAttempts < MAX_CONNECTION_ATTEMPTS)
    {
        WiFi.begin(settings.getWiFiSSID(), settings.getWiFiPassword());
        LOG_DEBUG("Connecting...");
        delay(5000);
        /* Check if the connection to the Wi-Fi network is successful. */
        if(WiFi.status() == WL_CONNECTED)
        {
            m_connected = true;
            LOG_DEBUG("Connected to WiFi. IP Address: %s", WiFi.localIP().toString().c_str());
        } else
        {
            m_connected = false;
            connectAttempts++;
            LOG_DEBUG("No Wifi available!Retrying....");
            delay(RETRY_DELAY_MS);
        }
    }
    /* If connection fails after maximum attempts, create a local Wi-Fi network. */
    if (!m_connected && connectAttempts >= MAX_CONNECTION_ATTEMPTS)
    {
        switchToAPMode();
    }
}
void NetworkConnection::switchToAPMode()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(settings.getapSSID(), settings.getapPassword());
    LOG_DEBUG("IP Address: %s", WiFi.softAPIP().toString().c_str());
    LOG_DEBUG("Local Wifi ready!");
    m_connected = true;
    
}

void NetworkConnection::checkConnection()
{
    while (true)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            m_connected = false;
            LOG_ERROR("Connection lost. Reconnecting...");

            /* Attempt to reconnect. */
            connectToWiFi();  

            if (WiFi.status() == WL_CONNECTED)
            {
                m_connected = true;
                LOG_DEBUG("Reconnected to WiFi. IP Address: %s", WiFi.localIP().toString().c_str());
            }
        }

        delay(RECONNECT_DELAY_MS);
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/