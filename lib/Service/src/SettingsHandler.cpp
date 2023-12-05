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
 * @brief  Settings Handler for loading and managing configuration.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "SettingsHandler.h"
#include <ArduinoJson.h>
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

/******************************************************************************
 * Public Methods
 *****************************************************************************/

bool SettingsHandler::loadConfigurationFile(const String& filename)
{
    bool                              isSuccessful  = false;
    const uint32_t                    maxBufferSize = 1024U;
    StaticJsonDocument<maxBufferSize> doc;
    char                              buffer[maxBufferSize];

    if (0U == m_fileHandler.readFile(filename, buffer, maxBufferSize))
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
            JsonVariantConst jsonRobotName        = doc["robotName"];
            JsonVariantConst jsonWifiSsid         = doc["WIFI"]["SSID"];
            JsonVariantConst jsonWifiPswd         = doc["WIFI"]["PSWD"];
            JsonVariantConst jsonMqttHost         = doc["MQTT"]["HOST"];
            JsonVariantConst jsonMqttPort         = doc["MQTT"]["PORT"];
            JsonVariantConst jsonApSSID           = doc["AP"]["SSID"];
            JsonVariantConst jsonApPswd           = doc["AP"]["PSWD"];
            JsonVariantConst jsonWebServerUser    = doc["WEBSERVER"]["USER"];
            JsonVariantConst jsonWebServerPswd    = doc["WEBSERVER"]["PSWD"];
            JsonVariantConst jsonPlatoonPlatoonId = doc["PLATOON"]["PLATOON_ID"];
            JsonVariantConst jsonPlatoonVehicleId = doc["PLATOON"]["VEHICLE_ID"];

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

            if (false == jsonApSSID.isNull())
            {
                m_apSSID = jsonApSSID.as<const char*>();
            }

            if (false == jsonApPswd.isNull())
            {
                m_apPassword = jsonApPswd.as<const char*>();
            }

            if (false == jsonWebServerUser.isNull())
            {
                m_webServerUser = jsonWebServerUser.as<const char*>();
            }

            if (false == jsonWebServerPswd.isNull())
            {
                m_webServerPassword = jsonWebServerPswd.as<const char*>();
            }

            if (false == jsonPlatoonPlatoonId.isNull())
            {
                m_platoonPlatoonId = jsonPlatoonPlatoonId.as<uint8_t>();
            }

            if (false == jsonPlatoonVehicleId.isNull())
            {
                m_platoonVehicleId = jsonPlatoonVehicleId.as<uint8_t>();
            }

            isSuccessful = true;
        }
    }

    return isSuccessful;
}

bool SettingsHandler::saveConfigurationFile(const String& filename)
{
    bool                           isSuccessful = false;
    const size_t                   maxDocSize   = 1024U;
    StaticJsonDocument<maxDocSize> doc;
    size_t                         jsonBufferSize = 0U;
    size_t                         bytesToWrite   = 0U;

    doc["robotName"]             = m_robotName.c_str();
    doc["WIFI"]["SSID"]          = m_wifiSSID.c_str();
    doc["WIFI"]["PSWD"]          = m_wifiPassword.c_str();
    doc["MQTT"]["HOST"]          = m_mqttBrokerAddress.c_str();
    doc["MQTT"]["PORT"]          = m_mqttPort;
    doc["AP"]["SSID"]            = m_apSSID.c_str();
    doc["AP"]["PSWD"]            = m_apPassword.c_str();
    doc["WEBSERVER"]["USER"]     = m_webServerUser.c_str();
    doc["WEBSERVER"]["PSWD"]     = m_webServerPassword.c_str();
    doc["PLATOON"]["PLATOON_ID"] = m_platoonPlatoonId;
    doc["PLATOON"]["VEHICLE_ID"] = m_platoonVehicleId;

    jsonBufferSize = measureJsonPretty(doc) + 1U;
    char jsonBuffer[jsonBufferSize];
    bytesToWrite = serializeJsonPretty(doc, jsonBuffer, jsonBufferSize);

    if (0U == bytesToWrite)
    {
        LOG_ERROR("Unable to serialize configuration file.");
    }
    else
    {
        if (0U == m_fileHandler.writeFile(filename, jsonBuffer, bytesToWrite))
        {
            LOG_ERROR("Unable to save configuration file \"%s\".", filename.c_str());
        }
        else
        {
            isSuccessful = true;
        }
    }

    return isSuccessful;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

SettingsHandler::SettingsHandler() :
    m_robotName(),
    m_wifiSSID(),
    m_wifiPassword(),
    m_mqttBrokerAddress(),
    m_mqttPort(0U),
    m_apSSID(),
    m_apPassword(),
    m_webServerUser(),
    m_webServerPassword(),
    m_platoonPlatoonId(0U),
    m_platoonVehicleId(0U),
    m_fileHandler()
{
}

SettingsHandler::~SettingsHandler()
{
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
