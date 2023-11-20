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
#include <Logging.h>
#include <LogSinkPrinter.h>


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

/** Serial interface baudrate. */
static const uint32_t SERIAL_BAUDRATE = 115200U;

/*defines the WiFi Credentials */
const char* ssid = "your_ssid";
const char* password = "your_password";

/** Serial log sink */
static LogSinkPrinter gLogSinkSerial("Serial", &Serial);
/******************************************************************************
 * Public Methods
 *****************************************************************************/

App::App()
{
}

App::~App()
{
}

bool App::loginit()
{
    /* Initialize Serial communication */
    Serial.begin(SERIAL_BAUDRATE);

    /* Register serial log sink and select it per default.*/
    if (true == Logging::getInstance().registerSink(&gLogSinkSerial))
    {
        (void)Logging::getInstance().selectSink("Serial");

        /* Set severity of logging system. */
        Logging::getInstance().setLogLevel(CONFIG_LOG_SEVERITY);

        LOG_DEBUG("LOGGER READY");
    }
    return true;
}

void App::start()
{
    if (m_fileManager.init())
    {
        LOG_DEBUG("LittleFS initialization successful");
        m_webServer.init();
    }
    else
    {
        LOG_FATAL("LittleFS initialization failed. The application will not start.");
        halt(); // Call a function to stop the application
    }
}

void App::halt()
{
    LOG_ERROR("Application halted due to critical error.");
    while (true)
    {
        // Stop the application in an endless loop
    }
}

void App::setup()
{
    if (false == loginit())
    {
        /* Halt the application or take appropriate action for failed logging initialization */
        halt();
    }

    // Access Point Modus start
    /*WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    /*LOG_DEBUG("IP Address: %s", WiFi.softAPIP().toString().c_str());
    */

    //Station Mode start
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        LOG_ERROR("WiFi Failed!\n");
        return;
    }

    LOG_DEBUG("IP Address: %s", WiFi.localIP().toString().c_str());
   
    start();
    m_webServer.handleUploadRequest();
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