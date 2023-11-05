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
 *  @brief  Settings Service for loading configuration.
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <WString.h>
#include <stdint.h>
#include <FileReader.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/

/**
 * Settings Service.
 */
class Settings
{
public:
    /**
     * Get Settings instance.
     * @returns Settings instance.
     */
    static Settings& getInstance()
    {
        static Settings instance; /* Idiom. */
        return instance;
    }

    /**
     * Get status of configuration loaded.
     */
    bool isConfigLoaded() const;

    /**
     * Loads the configuration from a file.
     * @param[in] filename Name of file to read.
     * @returns true if configuration succesfully loaded. Otherwise, false.
     */
    bool loadConfigurationFile(const String& filename);

    /**
     * Set configuration manually.
     * @param[in] instanceName Instance name.
     * @param[in] wifiSSID WiFi SSID.
     * @param[in] wifiPassword WiFi password.
     * @param[in] mqttBrokerAddress MQTT Broker IP/HOST.
     * @param[in] mqttPort MQTT Broker Port.
     * @returns true if configuration succesfully loaded. Otherwise, false.
     */
    bool setConfiguration(const String& instanceName, const String& wifiSSID, const String& wifiPassword,
                          const String& mqttBrokerAddress, uint16_t mqttPort);

    /**
     * Get robot name.
     * @returns Robot name.
     */
    const String& getRobotName()
    {
        return m_robotName;
    }

    /**
     * Get network SSID.
     * @returns Network SSID.
     */
    const String& getWiFiSSID()
    {
        return m_wifiSSID;
    }

    /**
     * Get network password.
     * @returns Network password.
     */
    const String& getWiFiPassword()
    {
        return m_wifiPassword;
    }

    /**
     * Get MQTT Broker IP/HOST.
     * @returns MQTT Broker IP/HOST.
     */
    const String& getMqttBrokerAddress()
    {
        return m_mqttBrokerAddress;
    }

    /**
     * Get MQTT Broker Port.
     * @returns MQTT Broker Port.
     */
    uint16_t getMqttPort()
    {
        return m_mqttPort;
    }

private:
    /**
     * Configuration loaded flag.
     */
    bool m_configLoaded;

    /**
     * Instance Name.
     */
    String m_robotName;

    /**
     * Network SSID.
     */
    String m_wifiSSID;

    /**
     * Network Password.
     */
    String m_wifiPassword;

    /**
     * MQTT Broker IP/HOST.
     */
    String m_mqttBrokerAddress;

    /**
     * MQTT Broker Port.
     */
    uint16_t m_mqttPort;

    /* FileReader instance. */
    FileReader m_fileReader;

private:
    /**
     * Settings Constructor.
     */
    Settings();

    /**
     * Settings Default Destructor.
     */
    ~Settings();

private:
    Settings(const Settings& settings);
    Settings& operator=(const Settings& settings);
};

#endif /* SETTINGS_H_ */