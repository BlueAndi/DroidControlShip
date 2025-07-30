/* MIT License
 *
 * Copyright (c) 2023 - 2025 Andreas Merkle <web@blue-andi.de>
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
#include <ConfigurationKeys.h>

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
    bool           isSuccessful  = false;
    const uint32_t maxBufferSize = 1024U;
    JsonDocument   jsonDoc;
    char           buffer[maxBufferSize];

    if (0U == m_fileHandler.readFile(filename, buffer, maxBufferSize))
    {
        LOG_ERROR("Unable to load configuration file \"%s\".", filename.c_str());
    }
    else
    {
        DeserializationError error = deserializeJson(jsonDoc, buffer);

        if (error != DeserializationError::Ok)
        {
            LOG_ERROR("Unable to deserialize configuration file.");
        }
        else
        {
            JsonVariantConst jsonRobotName        = jsonDoc[ConfigurationKeys::ROBOT_NAME];
            JsonVariantConst jsonWifiSsid         = jsonDoc[ConfigurationKeys::WIFI][ConfigurationKeys::SSID];
            JsonVariantConst jsonWifiPswd         = jsonDoc[ConfigurationKeys::WIFI][ConfigurationKeys::PASSWORD];
            JsonVariantConst jsonMqttHost         = jsonDoc[ConfigurationKeys::MQTT][ConfigurationKeys::HOST];
            JsonVariantConst jsonMqttPort         = jsonDoc[ConfigurationKeys::MQTT][ConfigurationKeys::PORT];
            JsonVariantConst jsonApSSID           = jsonDoc[ConfigurationKeys::AP][ConfigurationKeys::SSID];
            JsonVariantConst jsonApPswd           = jsonDoc[ConfigurationKeys::AP][ConfigurationKeys::PASSWORD];
            JsonVariantConst jsonWebServerUser    = jsonDoc[ConfigurationKeys::WEBSERVER][ConfigurationKeys::USER];
            JsonVariantConst jsonWebServerPswd    = jsonDoc[ConfigurationKeys::WEBSERVER][ConfigurationKeys::PASSWORD];
            JsonVariantConst jsonPlatoonPlatoonId = jsonDoc[ConfigurationKeys::PLATOON][ConfigurationKeys::PLATOON_ID];
            JsonVariantConst jsonPlatoonVehicleId = jsonDoc[ConfigurationKeys::PLATOON][ConfigurationKeys::VEHICLE_ID];
            JsonVariantConst jsonInitialXPosition =
                jsonDoc[ConfigurationKeys::INITIAL_POSITION][ConfigurationKeys::INITIAL_X_POSITION];
            JsonVariantConst jsonInitialYPosition =
                jsonDoc[ConfigurationKeys::INITIAL_POSITION][ConfigurationKeys::INITIAL_Y_POSITION];
            JsonVariantConst jsonInitialHeading =
                jsonDoc[ConfigurationKeys::INITIAL_POSITION][ConfigurationKeys::INITIAL_HEADING];
            JsonVariantConst jsonURosAgentHost = jsonDoc[ConfigurationKeys::MICROROS_AGENT][ConfigurationKeys::HOST];
            JsonVariantConst jsonURosAgentPort = jsonDoc[ConfigurationKeys::MICROROS_AGENT][ConfigurationKeys::PORT];

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

            if (false == jsonInitialXPosition.isNull())
            {
                m_initialXPosition = jsonInitialXPosition.as<int32_t>();
            }

            if (false == jsonInitialYPosition.isNull())
            {
                m_initialYPosition = jsonInitialYPosition.as<int32_t>();
            }

            if (false == jsonInitialHeading.isNull())
            {
                m_initialHeading = jsonInitialHeading.as<int32_t>();
            }

            if (false == jsonURosAgentHost.isNull())
            {
                m_microROSAgentAddress = jsonURosAgentHost.as<const char*>();
            }

            if (false == jsonURosAgentPort.isNull())
            {
                m_microROSAgentPort = jsonURosAgentPort.as<uint16_t>();
            }

            isSuccessful = true;
        }
    }

    return isSuccessful;
}

bool SettingsHandler::saveConfigurationFile(const String& filename)
{
    bool         isSuccessful = false;
    const size_t maxDocSize   = 1024U;
    JsonDocument jsonDoc;
    size_t       jsonBufferSize = 0U;
    size_t       bytesToWrite   = 0U;

    jsonDoc[ConfigurationKeys::ROBOT_NAME]                                              = m_robotName;
    jsonDoc[ConfigurationKeys::WIFI][ConfigurationKeys::SSID]                           = m_wifiSSID;
    jsonDoc[ConfigurationKeys::WIFI][ConfigurationKeys::PASSWORD]                       = m_wifiPassword;
    jsonDoc[ConfigurationKeys::MQTT][ConfigurationKeys::HOST]                           = m_mqttBrokerAddress;
    jsonDoc[ConfigurationKeys::MQTT][ConfigurationKeys::PORT]                           = m_mqttPort;
    jsonDoc[ConfigurationKeys::AP][ConfigurationKeys::SSID]                             = m_apSSID;
    jsonDoc[ConfigurationKeys::AP][ConfigurationKeys::PASSWORD]                         = m_apPassword;
    jsonDoc[ConfigurationKeys::WEBSERVER][ConfigurationKeys::USER]                      = m_webServerUser;
    jsonDoc[ConfigurationKeys::WEBSERVER][ConfigurationKeys::PASSWORD]                  = m_webServerPassword;
    jsonDoc[ConfigurationKeys::PLATOON][ConfigurationKeys::PLATOON_ID]                  = m_platoonPlatoonId;
    jsonDoc[ConfigurationKeys::PLATOON][ConfigurationKeys::VEHICLE_ID]                  = m_platoonVehicleId;
    jsonDoc[ConfigurationKeys::INITIAL_POSITION][ConfigurationKeys::INITIAL_X_POSITION] = m_initialXPosition;
    jsonDoc[ConfigurationKeys::INITIAL_POSITION][ConfigurationKeys::INITIAL_Y_POSITION] = m_initialYPosition;
    jsonDoc[ConfigurationKeys::INITIAL_POSITION][ConfigurationKeys::INITIAL_HEADING]    = m_initialHeading;
    jsonDoc[ConfigurationKeys::MICROROS_AGENT][ConfigurationKeys::HOST]                 = m_microROSAgentAddress;
    jsonDoc[ConfigurationKeys::MICROROS_AGENT][ConfigurationKeys::PORT]                 = m_microROSAgentPort;

    jsonBufferSize = measureJsonPretty(jsonDoc) + 1U;
    char jsonBuffer[jsonBufferSize];
    bytesToWrite = serializeJsonPretty(jsonDoc, jsonBuffer, jsonBufferSize);

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
    m_initialXPosition(0),
    m_initialYPosition(0),
    m_initialHeading(0),
    m_microROSAgentAddress(),
    m_microROSAgentPort(0U),
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
