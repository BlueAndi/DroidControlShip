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
 *  @brief  Settings Service
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Settings.h"
#include <ArduinoJson.h>
#include <Logging.h>

/******************************************************************************
 * Macros
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

bool Settings::loadConfigurationFile(const String& filename)
{
    bool                              isSuccessful  = false;
    const uint32_t                    maxBufferSize = 1024;
    StaticJsonDocument<maxBufferSize> doc;
    char                              buffer[maxBufferSize];

    if (0U == m_fileReader.readFile(filename, buffer, maxBufferSize))
    {
        LOG_ERROR("Unable to load configuration file \"%s\".", filename.c_str());
    }
    else
    {
        DeserializationError error = deserializeJson(doc, buffer);

        if (error != DeserializationError::Ok)
        {
            LOG_ERROR("Unable to deserialize configuration file.");
        }
        else
        {
            JsonVariantConst jsonRobotName = doc["robotName"];
            JsonVariantConst jsonWifiSsid  = doc["WIFI"]["SSID"];
            JsonVariantConst jsonWifiPswd  = doc["WIFI"]["PSWD"];
            JsonVariantConst jsonMqttHost  = doc["MQTT"]["HOST"];
            JsonVariantConst jsonMqttPort  = doc["MQTT"]["PORT"];

            if (false == jsonRobotName.isNull())
            {
                m_robotName = jsonRobotName.as<const char*>();
            }

            if (false == jsonWifiSsid.isNull())
            {
                m_wifiSSID = jsonWifiSsid.as<const char*>();
            }

            if (false == jsonWifiPswd.isNull())
            {
                m_wifiPassword = jsonWifiPswd.as<const char*>();
            }

            if (false == jsonMqttHost.isNull())
            {
                m_mqttBrokerAddress = jsonMqttHost.as<const char*>();
            }

            if (false == jsonMqttPort.isNull())
            {
                m_mqttPort = jsonMqttPort.as<uint16_t>();
            }

            isSuccessful = true;
        }
    }

    return isSuccessful;
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

Settings::Settings() :
    m_robotName(),
    m_wifiSSID(),
    m_wifiPassword(),
    m_mqttBrokerAddress(),
    m_mqttPort(0U),
    m_fileReader()
{
}

Settings::~Settings()
{
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
