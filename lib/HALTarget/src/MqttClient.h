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
 * @brief  MQTTClient realization
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALTarget
 *
 * @{
 */

#ifndef MQTTCLIENT_H
#define MQTTCLIENT_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "IMqttClient.h"
#include <PubSubClient.h>
#include <SimpleTimer.hpp>
#include <vector>
#include <WiFi.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class provides access to the robot's MQTTClient. */
class MqttClient : public IMqttClient
{
public:
    /**
     * Constructs the MQTTClient adapter.
     */
    MqttClient();

    /**
     * Destroys the MQTTClient adapter.
     */
    virtual ~MqttClient();

    /**
     * Initialize MQTTClient driver.
     *
     * @return If successfully initialized, returns true. Otherwise, false.
     */
    bool init() final;

    /**
     * Process communication with the MQTTClient.
     *
     * @return If communication is successful, returns true. Otherwise, false.
     */
    bool process() final;

    /**
     * Set client configuration.
     *
     * @param[in] settings Mqtt settings struct.
     * @return If successfully set, returns true. Otherwise, false.
     */
    bool setConfig(const MqttSettings& settings) final;

    /**
     * Start connection to the MQTTClient.
     * This method does not necessarily wait for the connection to be established, it just starts the connection
     * process. Check `isConnected()` for the current connection status.
     *
     * @return If connection has been succesfully started, returns true. Otherwise, false.
     */
    bool connect() final;

    /**
     * Disconnect from the MQTTClient.
     */
    void disconnect() final;

    /**
     * Is connected to the MQTTClient?
     *
     * @return If connected, it will return true otherwise false.
     */
    bool isConnected() const final;

    /**
     * Publishes a message to the MQTTClient.
     *
     * @param[in] topic     Topic to publish to.
     * @param[in] useClientBaseTopic   If true, the client ID is used as the base (prefix) of the topic.
     * @param[in] message   Message to publish.
     */
    bool publish(const String& topic, const bool useClientBaseTopic, const String& message) final;

    /**
     * Subscribes to a topic.
     *
     * @param[in] topic     Topic to subscribe to. The Client ID is used as base topic: `Client ID`/`topic`
     * @param[in] callback  Callback function, which is called on a new message.
     * @return If successfully subscribed, returns true. Otherwise, false.
     */
    bool subscribe(const String& topic, TopicCallback callback) final;

    /**
     * Unsubscribes from a topic.
     *
     * @param[in] topic     Topic to unsubscribe from. The Client ID is used as base topic: `Client ID`/`topic`
     */
    void unsubscribe(const String& topic) final;

private:
    /** MQTT Service States. */
    enum State
    {
        STATE_UNINITIALIZED = 0, /**< Uninitialized state. */
        STATE_SETUP,             /**< Setup state. */
        STATE_DISCONNECTED,      /**< Disconnecting state. */
        STATE_DISCONNECTING,     /**< Disconnecting state. */
        STATE_CONNECTED,         /**< Connected state. */
        STATE_CONNECTING,        /**< Connecting state. */
    };

    /**
     * Subscriber information
     */
    struct Subscriber
    {
        String        topic;    /**< The subscriber topic */
        TopicCallback callback; /**< The subscriber callback */
    };

    /**
     * This type defines a list of subscribers.
     */
    typedef std::vector<Subscriber*> SubscriberList;

    /** MQTT Keep Alive in seconds. */
    static const uint16_t MQTT_KEEP_ALIVE_S = 2U;

    /** Connecting Timeout. */
    static const uint32_t CONNECTING_TIMEOUT_MS = 1000;

    /** Reconnect Timeout. */
    static const uint32_t RECONNECT_TIMEOUT_MS = (2 * CONNECTING_TIMEOUT_MS);

    /**
     * Max. MQTT client buffer size in byte.
     * Received MQTT messages greather than this will be skipped.
     */
    static const size_t MAX_BUFFER_SIZE = 1024U;

    /** Connection state */
    State m_state;

    /** WiFi client */
    WiFiClient m_wifiClient;

    /** Mosquitto instance */
    PubSubClient m_mqttClient;

    /** Client ID. */
    String m_clientId;

    /** Broker address to connect to. */
    String m_brokerAddress;

    /** Broker port to connect to. */
    uint16_t m_brokerPort;

    /** Birth topic. */
    String m_birthTopic;

    /** Birth Message. */
    String m_birthMessage;

    /** Will topic. */
    String m_willTopic;

    /** Will message. */
    String m_willMessage;

    /** Reconnect Flag. */
    bool m_reconnect;

    /** Reconnect Timer. */
    SimpleTimer m_reconnectTimer;

    /** Connection Timer. */
    SimpleTimer m_connectionTimer;

    /** List of subscribers */
    SubscriberList m_subscriberList;

    /** Configuration Set Flag. */
    bool m_configSet;

    /** User connection request. */
    bool m_connectRequest;

    /** User disconnection request. */
    bool m_disconnectRequest;

private:
    /**
     * Process the Idle state.
     */
    void handleSetupState();

    /**
     * Process the Disconnected state.
     */
    void handleDisconnectedState();

    /**
     * Process the Disconnecting state.
     */
    void handleDisconnectingState();

    /**
     * Process the Connected state.
     */
    void handleConnectedState();

    /**
     * Process the Connected state.
     */
    void handleConnectingState();

    /**
     * Resubscribe to all topics.
     */
    void resubscribe();

    /**
     * Attempt to establish connection to the broker.
     */
    void attemptConnection();

    /**
     * Callback function, which is called on message reception.
     *
     * @param[in] topic     The topic name.
     * @param[in] payload   The payload of the topic.
     * @param[in] length    Payload length in byte.
     */
    void onMessageCallback(char* topic, uint8_t* payload, uint32_t length);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* MQTTCLIENT_H */
/** @} */
