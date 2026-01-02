/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
 * @file
 * @brief  Network realization
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALTargetCommon
 *
 * @{
 */

#ifndef NETWORK_H
#define NETWORK_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "INetwork.h"
#include <SimpleTimer.hpp>
#include <WiFi.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class provides access to the robot's network. */
class Network : public INetwork
{
public:
    /**
     * Constructs the network adapter.
     */
    Network();

    /**
     * Destroys the network adapter.
     */
    virtual ~Network();

    /**
     * Initialize network driver.
     *
     * @return If successfully initialized, returns true. Otherwise, false.
     */
    bool init() final;

    /**
     * Process network tasks according to current state.
     */
    void process() final;

    /**
     * Set client configuration.
     *
     * @param[in] settings NetworkSettings struct containing ssid and password.
     * @return If successfully set, returns true. Otherwise, false.
     */
    bool setConfig(const NetworkSettings& settings) final;

    /**
     * Is the network interface initialized?
     *
     * @return If network interface is initialized, returns true. Otherwise, false.
     */
    bool isUp() const final;

    /**
     * Get the IP address.
     *
     * @return IP address if available, otherwise IPAddr(), which is 0.0.0.0.
     */
    IPAddress getIp() const final;

private:
    /** Network Service States. */
    enum State
    {
        STATE_UNINITIALIZED = 0, /**< Uninitialized state. */
        STATE_SETUP,             /**< Setup state. */
        STATE_CONNECTING,        /**< Connecting state. */
        STATE_CONNECTED,         /**< Connected state. */
        STATE_DISCONNECTED,      /**< Disconnected state. */
        STATE_AP_SETUP,          /**< Access point setup state. */
        STATE_AP_UP,             /**< Access point up and running state.*/
    };

    /** Current network state */
    State m_state;

    /** Timeout time for WiFi connection. */
    static const uint32_t WIFI_TIMEOUT = 10000U;

    /** Configuration Set Flag. */
    bool m_configSet;

    /** WiFi SSID */
    String m_wiFiSSID;

    /** WiFi Password */
    String m_wiFiPassword;

    /** WiFi Access Point Mode SSID */
    String m_apSSID;

    /** WiFi Access Point Mode Password */
    String m_apPassword;

    /** WiFi Timeout Timer. */
    SimpleTimer m_wifiTimeoutTimer;

private:
    /**
     * Setup network connection
     */
    void handleStationSetup();

    /**
     * Check if the connection was established and change state if so.
     */
    void handleConnectingState();

    /**
     * Handle connection specific tasks.
     */
    void manageConnection();

    /**
     * Switch WiFi to Access Point Mode.
     */
    void switchToAPMode();

    /**
     * Setup WiFi Access Point.
     */
    void handleAPSetup();

    /**
     * Handle Access Point Management.
     */
    void handleAPState();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* NETWORK_H */
/** @} */
