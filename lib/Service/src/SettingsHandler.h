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
 *
 * @addtogroup Service
 *
 * @{
 */
#ifndef SETTINGS_HANDLER_H
#define SETTINGS_HANDLER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

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

/**
 * Settings Handler for loading and managing configuration.
 */
class SettingsHandler
{
public:
    /**
     * Get the settings handler instance.
     * @returns Settings handler instance.
     */
    static SettingsHandler& getInstance()
    {
        static SettingsHandler instance; /* Idiom. */
        return instance;
    }

    /**
     * Loads the configuration from a file.
     *
     * @param[in] filename Name of file to read.
     *
     * @returns true if configuration succesfully loaded. Otherwise, false.
     */
    bool loadConfigurationFile(const String& filename);

    /**
     * Get robot name.
     *
     * @returns Robot name.
     */
    const String& getRobotName()
    {
        return m_robotName;
    }

    /**
     * Set the robot name.
     *
     * @param[in] robotName The robot name.
     */
    void setRobotName(const String& robotName)
    {
        m_robotName = robotName;
    }

    /**
     * Get network SSID.
     *
     * @returns Network SSID.
     */
    const String& getWiFiSSID()
    {
        return m_wifiSSID;
    }

    /**
     * Get network password.
     *
     * @returns Network password.
     */
    const String& getWiFiPassword()
    {
        return m_wifiPassword;
    }

    /**
     * Get MQTT Broker IP/HOST.
     *
     * @returns MQTT Broker IP/HOST.
     */
    const String& getMqttBrokerAddress()
    {
        return m_mqttBrokerAddress;
    }

    /**
     * Get MQTT Broker Port.
     *
     * @returns MQTT Broker Port.
     */
    uint16_t getMqttPort()
    {
        return m_mqttPort;
    }

private:
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

    /**
     * FileReader instance.
     */
    FileReader m_fileReader;

    /**
     * Settings Handler Default Constructor.
     */
    SettingsHandler();

    /**
     * Settings Handler Default Destructor.
     */
    ~SettingsHandler();

private:
    /* Not allowed. */
    SettingsHandler(const SettingsHandler& handler);            /**< Copy construction of an instance. */
    SettingsHandler& operator=(const SettingsHandler& handler); /**< Assignment of an instance. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SETTINGS_HANDLER_H */
/** @} */
