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

static bool convertToJson(const String& str, JsonVariant variant);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/**
 * Configuration keys found in the config file.
 */
namespace configKeys
{
    /** Robot name */
    static const char ROBOT_NAME[] = "robotName";

    /** WiFi */
    static const char WIFI[] = "wifi";

    /** MQTT */
    static const char MQTT[] = "mqtt";

    /** Access point */
    static const char AP[] = "ap";

    /** WebServer */
    static const char WEBSERVER[] = "webServer";

    /** Platoon */
    static const char PLATOON[] = "platoon";

    /** SSID */
    static const char SSID[] = "ssid";

    /** Passphrase */
    static const char PASSWORD[] = "pswd";

    /** Host */
    static const char HOST[] = "host";

    /** Port */
    static const char PORT[] = "port";

    /** User */
    static const char USER[] = "user";

    /** Platoon ID */
    static const char PLATOON_ID[] = "platoonId";

    /** Vehicle ID */
    static const char VEHICLE_ID[] = "vehicleId";

} // namespace configKeys

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
            JsonVariantConst jsonRobotName        = doc[configKeys::ROBOT_NAME];
            JsonVariantConst jsonWifiSsid         = doc[configKeys::WIFI][configKeys::SSID];
            JsonVariantConst jsonWifiPswd         = doc[configKeys::WIFI][configKeys::PASSWORD];
            JsonVariantConst jsonMqttHost         = doc[configKeys::MQTT][configKeys::HOST];
            JsonVariantConst jsonMqttPort         = doc[configKeys::MQTT][configKeys::PORT];
            JsonVariantConst jsonApSSID           = doc[configKeys::AP][configKeys::SSID];
            JsonVariantConst jsonApPswd           = doc[configKeys::AP][configKeys::PASSWORD];
            JsonVariantConst jsonWebServerUser    = doc[configKeys::WEBSERVER][configKeys::USER];
            JsonVariantConst jsonWebServerPswd    = doc[configKeys::WEBSERVER][configKeys::PASSWORD];
            JsonVariantConst jsonPlatoonPlatoonId = doc[configKeys::PLATOON][configKeys::PLATOON_ID];
            JsonVariantConst jsonPlatoonVehicleId = doc[configKeys::PLATOON][configKeys::VEHICLE_ID];

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

    doc[configKeys::ROBOT_NAME]                      = m_robotName;
    doc[configKeys::WIFI][configKeys::SSID]          = m_wifiSSID;
    doc[configKeys::WIFI][configKeys::PASSWORD]      = m_wifiPassword;
    doc[configKeys::MQTT][configKeys::HOST]          = m_mqttBrokerAddress;
    doc[configKeys::MQTT][configKeys::PORT]          = m_mqttPort;
    doc[configKeys::AP][configKeys::SSID]            = m_apSSID;
    doc[configKeys::AP][configKeys::PASSWORD]        = m_apPassword;
    doc[configKeys::WEBSERVER][configKeys::USER]     = m_webServerUser;
    doc[configKeys::WEBSERVER][configKeys::PASSWORD] = m_webServerPassword;
    doc[configKeys::PLATOON][configKeys::PLATOON_ID] = m_platoonPlatoonId;
    doc[configKeys::PLATOON][configKeys::VEHICLE_ID] = m_platoonVehicleId;

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

/**
 * Convert a string to a json variant.
 *
 * @param[in]   str     String to convert.
 * @param[out]  variant Json variant.
 *
 * @return If successful, it will return true otherwise false.
 */
bool convertToJson(const String& str, JsonVariant variant)
{
    return variant.set(str.c_str());
}
