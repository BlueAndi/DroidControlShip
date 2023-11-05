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

bool Settings::isConfigLoaded() const
{
    return m_configLoaded;
}

bool Settings::loadConfigurationFile(const String& filename, const String& robotName)
{
    const uint32_t                    maxBufferSize = 1024;
    StaticJsonDocument<maxBufferSize> doc;
    char                              buffer[maxBufferSize];

    /* Ignore previously saved configuration. */
    m_configLoaded = false;

    if (true == robotName.isEmpty())
    {
        LOG_ERROR("Robot name is required.");
    }
    else if (0U == m_fileReader.readFile(filename, buffer, maxBufferSize))
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
            JsonVariantConst    jsonWifiSsid    = doc["WIFI"]["SSID"];
            JsonVariantConst    jsonWifiPswd    = doc["WIFI"]["PSWD"];
            JsonVariantConst    jsonMqttHost    = doc["MQTT"]["HOST"];
            JsonVariantConst    jsonMqttPort    = doc["MQTT"]["PORT"];

            m_robotName = robotName;

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

            m_configLoaded = true;
        }
    }

    return m_configLoaded;
}

bool Settings::setConfiguration(const String& robotName, const String& networkSSID, const String& networkPassword,
                                const String& mqttBrokerAddress, uint16_t mqttPort)
{
    if (true == robotName.isEmpty())
    {
        LOG_ERROR("Robot name is not allowed to be empty.");
    }
    else
    {
        m_robotName         = robotName;
        m_wifiSSID          = networkSSID;
        m_wifiPassword      = networkPassword;
        m_mqttBrokerAddress = mqttBrokerAddress;
        m_mqttPort          = mqttPort;
        m_configLoaded      = true;
    }

    return m_configLoaded;
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

Settings::Settings() :
    m_configLoaded(false),
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
