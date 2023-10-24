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
#include <WiFi.h>

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

bool Settings::loadConfigurationFile(const String& filename)
{
    const uint32_t                    maxBufferSize = 1024;
    StaticJsonDocument<maxBufferSize> doc;
    char                              buffer[maxBufferSize];

    /* Ignore previously saved configuration. */
    m_configLoaded = false;

    /* Generate name based on MAC Address. */
    String macAddress = WiFi.macAddress();

    if (true == macAddress.isEmpty())
    {
        LOG_ERROR("Unable to get device MAC Address");
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
            /* Remove separators. */
            macAddress.remove(14, 1);
            macAddress.remove(11, 1);
            macAddress.remove(8, 1);
            macAddress.remove(5, 1);
            macAddress.remove(2, 1);

            /* Set name. */
            m_robotName         = macAddress;
            m_wifiSSID          = doc["WIFI"]["SSID"].as<const char*>();
            m_wifiPassword      = doc["WIFI"]["PSWD"].as<const char*>();
            m_mqttBrokerAddress = doc["MQTT"]["HOST"].as<const char*>();
            m_mqttPort          = doc["MQTT"]["PORT"].as<uint16_t>();
            m_configLoaded      = true;
        }
    }

    return m_configLoaded;
}

bool Settings::setConfiguration(const String& instanceName, const String& networkSSID, const String& networkPassword,
                                const String& mqttBrokerAddress, uint16_t mqttPort)
{
    if (true == instanceName.isEmpty())
    {
        LOG_ERROR("Instance name is not allowed to be empty.");
    }
    else
    {
        m_robotName         = instanceName;
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
    m_robotName(""),
    m_wifiSSID(""),
    m_wifiPassword(""),
    m_mqttBrokerAddress(""),
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
