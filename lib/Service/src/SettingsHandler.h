/* MIT License
 *
 * Copyright (c) 2023 - 2024 Andreas Merkle <web@blue-andi.de>
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
#include <FileHandler.h>

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
     * Saves the current configuration to a file.
     *
     * @param[in] filename Name of file to write.
     *
     * @returns true if configuration succesfully saved. Otherwise, false.
     */
    bool saveConfigurationFile(const String& filename);

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
     * Set WiFi SSID.
     *
     * @param[in] wifiSSID WiFi SSID.
     */
    void setWiFiSSID(const String& wifiSSID)
    {
        m_wifiSSID = wifiSSID;
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
     * Set WiFi password.
     *
     * @param[in] wifiPassword WiFi password.
     */
    void setWiFiPassword(const String& wifiPassword)
    {
        m_wifiPassword = wifiPassword;
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

    /**
     * Get Access Point SSID.
     *
     * @returns Access Point SSID.
     */
    const String& getApSSID()
    {
        return m_apSSID;
    }

    /**
     * Get Access Point Password.
     *
     * @returns Access Point Password.
     */
    const String& getApPassword()
    {
        return m_apPassword;
    }

    /**
     * Get Web Server User.
     *
     * @returns Web Server User.
     */
    const String& getWebServerUser()
    {
        return m_webServerUser;
    }

    /**
     * Get Web Server Password.
     *
     * @returns Web Server Password.
     */
    const String& getWebServerPassword()
    {
        return m_webServerPassword;
    }

    /**
     * Get Platoon ID.
     *
     * @returns Platoon ID.
     */
    uint8_t getPlatoonPlatoonId()
    {
        return m_platoonPlatoonId;
    }

    /**
     * Get Vehicle ID.
     *
     * @returns Vehicle ID.
     */
    uint8_t getPlatoonVehicleId()
    {
        return m_platoonVehicleId;
    }

    /**
     * Get Initial Position: X [mm]
     *
     * @returns Initial Position: X [mm]
     */
    int32_t getInitialXPosition()
    {
        return m_initialXPosition;
    }

    /**
     * Get Initial Position: Y [mm]
     *
     * @returns Initial Position: Y [mm]
     */
    int32_t getInitialYPosition()
    {
        return m_initialYPosition;
    }

    /**
     * Get Initial Position: Heading [mrad]
     *
     * @returns Initial Position: Heading [mrad]
     */
    int32_t getInitialHeading()
    {
        return m_initialHeading;
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
     * Access Point SSID.
     */
    String m_apSSID;

    /**
     * Access Point Password.
     */
    String m_apPassword;

    /**
     * Web Server User.
     */
    String m_webServerUser;

    /**
     * Web Server Password.
     */
    String m_webServerPassword;

    /**
     * Platoon: Platoon ID.
     */
    uint8_t m_platoonPlatoonId;

    /**
     * Platoon: Vehicle ID.
     */
    uint8_t m_platoonVehicleId;

    /**
     * Initial Position: X [mm]
     */
    int32_t m_initialXPosition;

    /**
     * Initial Position: Y [mm]
     */
    int32_t m_initialYPosition;

    /**
     * Initial Position: Heading [mrad]
     */
    int32_t m_initialHeading;

    /**
     * FileHandler instance.
     */
    FileHandler m_fileHandler;

    /**
     * Settings Handler Default Constructor.
     */
    SettingsHandler();

    /**
     * Settings Handler Default Destructor.
     */
    ~SettingsHandler();

private:
    /**
     * Copy construction of an instance.
     * Not allowed.
     *
     * @param[in] handler Source instance.
     */
    SettingsHandler(const SettingsHandler& handler);

    /**
     * Assignment of an instance.
     * Not allowed.
     *
     * @param[in] handler Source instance.
     *
     * @returns Reference to SettingsHandler instance.
     */
    SettingsHandler& operator=(const SettingsHandler& handler);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SETTINGS_HANDLER_H */
/** @} */
