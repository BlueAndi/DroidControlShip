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
 * @brief  Network realization
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Network.h"
#include <Logging.h>
#include <WiFi.h>
#include <SimpleTimer.hpp>

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

Network::Network() :
    INetwork(),
    m_state(STATE_UNINITIALIZED),
    m_wifiClient(),
    m_mqttClient(m_wifiClient),
    m_clientId(""),
    m_brokerAddress(""),
    m_brokerPort(0U),
    m_birthTopic(""),
    m_birthMessage(""),
    m_willTopic(""),
    m_willMessage(""),
    m_reconnect(true),
    m_reconnectTimer(),
    m_connectionTimer(),
    m_subscriberList(),
    m_configSet(false),
    m_connectRequest(false),
    m_disconnectRequest(false),
    m_wiFiSSID(""),
    m_wiFiPassword(""),
    m_isWiFiConfigured(false)
{
}

Network::~Network()
{
}

bool Network::init()
{
    m_state = STATE_SETUP;
    return true;
}

bool Network::process()
{
    bool isSuccess = manageWiFi();

    switch (m_state)
    {
    case STATE_UNINITIALIZED:
        /* Nothing to do. */
        break;

    case STATE_SETUP:
        setupState();
        break;

    case STATE_DISCONNECTED:
        disconnectedState();
        break;

    case STATE_DISCONNECTING:
        disconnectingState();
        break;

    case STATE_CONNECTED:
        connectedState();
        break;

    case STATE_CONNECTING:
        connectingState();
        break;

    default:
        break;
    }

    return isSuccess;
}

bool Network::setConfig(const String& clientId, const String& ssid, const String& password, const String& brokerAddress,
                        uint16_t brokerPort, const String& birthTopic, const String& birthMessage,
                        const String& willTopic, const String& willMessage, bool reconnect)
{
    if (true == clientId.isEmpty())
    {
        LOG_ERROR("Client ID is empty.");
    }
    else if (true == ssid.isEmpty())
    {
        /* Check only performed on target. */
        LOG_ERROR("WiFi SSID is empty.");
    }
    else if (true == brokerAddress.isEmpty())
    {
        LOG_ERROR("Broker address is empty.");
    }
    else if (0U == brokerPort)
    {
        LOG_ERROR("Broker port is zero.");
    }
    else
    {
        if (false == birthTopic.isEmpty())
        {
            m_birthTopic   = birthTopic;
            m_birthMessage = birthMessage;
        }

        if (false == willTopic.isEmpty())
        {
            m_willTopic   = willTopic;
            m_willMessage = willMessage;
        }

        m_clientId      = clientId;
        m_wiFiSSID      = ssid;
        m_wiFiPassword  = password;
        m_brokerAddress = brokerAddress;
        m_brokerPort    = brokerPort;
        m_reconnect     = reconnect;
        m_configSet     = true;
    }

    return m_configSet;
}

bool Network::connect()
{
    m_connectRequest = true;
    return m_connectRequest;
}

void Network::disconnect()
{
    m_disconnectRequest = true;
    m_state             = STATE_DISCONNECTING;

    /* No reconnection if user called for disconnect. */
    m_reconnect = false;
}

bool Network::isConnected() const
{
    return (STATE_CONNECTED == m_state);
}

bool Network::publish(const String& topic, const bool useClientBaseTopic, const String& message)
{
    bool isSuccess = false;
    LOG_DEBUG("Publishing message to topic %s", topic.c_str());

    if ((true == isConnected()) && (false == topic.isEmpty()))
    {
        String fullTopic = "";

        if ((true == useClientBaseTopic) && (false == m_clientId.isEmpty()))
        {
            fullTopic = m_clientId + "/" + topic;
        }
        else
        {
            fullTopic = topic;
        }

        isSuccess = m_mqttClient.publish(fullTopic.c_str(), message.c_str());
    }

    return isSuccess;
}

bool Network::subscribe(const String& topic, TopicCallback callback)
{
    bool isSuccess = false;

    if ((false == topic.isEmpty()) && (false == m_clientId.isEmpty()))
    {
        SubscriberList::const_iterator it;
        String                         fullTopic = m_clientId + "/" + topic;

        /* Register a topic only once! */
        for (it = m_subscriberList.begin(); it != m_subscriberList.end(); ++it)
        {
            if (nullptr != (*it))
            {
                if ((*it)->topic == fullTopic)
                {
                    break;
                }
            }
        }

        if (it == m_subscriberList.end())
        {
            Subscriber* subscriber = new (std::nothrow) Subscriber;

            if (nullptr != subscriber)
            {
                subscriber->topic    = fullTopic;
                subscriber->callback = callback;

                if (false == isConnected())
                {
                    m_subscriberList.push_back(subscriber);
                    isSuccess = true;
                }
                else
                {
                    m_mqttClient.subscribe(subscriber->topic.c_str());
                }

                if (false == isSuccess)
                {
                    delete subscriber;
                    subscriber = nullptr;
                }
            }
        }
    }

    return isSuccess;
}

void Network::unsubscribe(const String& topic)
{
    if ((false == topic.isEmpty()) && (false == m_clientId.isEmpty()))
    {
        String                   fullTopic = m_clientId + "/" + topic;
        SubscriberList::iterator it        = m_subscriberList.begin();

        while (m_subscriberList.end() != it)
        {
            if (nullptr != (*it))
            {
                if ((*it)->topic == fullTopic)
                {
                    Subscriber* subscriber = *it;

                    m_mqttClient.unsubscribe(subscriber->topic.c_str());

                    (void)m_subscriberList.erase(it);
                    delete subscriber;

                    break;
                }
            }

            ++it;
        }
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void Network::setupState()
{
    if (true == m_configSet)
    {
        if (false == m_mqttClient.setBufferSize(MAX_BUFFER_SIZE))
        {
            LOG_ERROR("Failed to set MQTT buffer size.");
        }
        else if (WL_CONNECT_FAILED == WiFi.begin(m_wiFiSSID.c_str(), m_wiFiPassword.c_str()))
        {
            LOG_ERROR("Failed to connect to WiFi.");
        }
        else
        {
            /* Functions return instance to allow chaining. */
            (void)m_mqttClient.setServer(m_brokerAddress.c_str(), m_brokerPort);
            (void)m_mqttClient.setKeepAlive(MQTT_KEEP_ALIVE_S);
            (void)m_mqttClient.setCallback([this](char* topic, uint8_t* payload, uint32_t length)
                                           { this->onMessageCallback(topic, payload, length); });

            m_isWiFiConfigured = true;
            m_state            = STATE_DISCONNECTED;
        }
    }
}

void Network::disconnectedState()
{
    bool connectNow = false;

    if (true == m_connectRequest)
    {
        /* User request. Connect now. */
        connectNow       = true;
        m_connectRequest = false;
    }
    else if (false == m_reconnect)
    {
        ; /* User set reconnect to false. Do nothing. */
    }
    else if (false == m_reconnectTimer.isTimerRunning())
    {
        /* Timer is not running. Connect now. */
        connectNow = true;

        /* Start reconnect timer. */
        m_reconnectTimer.start(RECONNECT_TIMEOUT_MS);
    }
    else if (true == m_reconnectTimer.isTimeout())
    {
        /* Timer timeout. Connect now. */
        connectNow = true;
    }
    else
    {
        ; /* Do nothing. */
    }

    if (true == connectNow)
    {
        attemptConnection();
        m_reconnectTimer.restart();
    }
}

void Network::disconnectingState()
{
    m_mqttClient.disconnect();
    m_state = STATE_DISCONNECTED;
}

void Network::connectedState()
{
    /* Connection state is checked on loop. */
    if (false == m_mqttClient.loop())
    {
        LOG_DEBUG("MQTT connection lost.");
        m_state = STATE_DISCONNECTING;
    }
}

void Network::connectingState()
{
    if (false == m_connectionTimer.isTimerRunning())
    {
        /* Start connecting timer. */
        m_connectionTimer.start(CONNECTING_TIMEOUT_MS);
    }
    else if (true == m_connectionTimer.isTimeout())
    {
        LOG_DEBUG("MQTT connection attempt timeout.");

        /* Connection failed. Return to disconnected state. */
        m_state = STATE_DISCONNECTED;

        /* Stop Timer. */
        m_connectionTimer.stop();
    }
    else
    {
        if (MQTT_CONNECTED != m_mqttClient.state())
        {
            bool isSuccess = false;
            LOG_DEBUG("MQTT client connected to broker");

            if (false == m_birthTopic.isEmpty())
            {
                /* Publish birth message. Should succesfully publish if connected to broker. */
                isSuccess = publish(m_birthTopic, false, m_birthMessage);
            }
            else
            {
                isSuccess = true;
            }

            if (true == isSuccess)
            {
                resubscribe();
                m_state = STATE_CONNECTED;

                /* Stop Timer. */
                m_connectionTimer.stop();
            }
        }
    }
}

void Network::resubscribe()
{
    SubscriberList::const_iterator it;

    for (it = m_subscriberList.begin(); it != m_subscriberList.end(); ++it)
    {
        if (nullptr != (*it))
        {
            Subscriber* subscriber = *it;

            if (false == m_mqttClient.subscribe(subscriber->topic.c_str()))
            {
                LOG_WARNING("MQTT topic subscription not possible: %s", subscriber->topic.c_str());
            }
            else
            {
                LOG_DEBUG("MQTT topic subscription successful: %s", subscriber->topic.c_str());
            }
        }
    }
}

void Network::attemptConnection()
{
    bool isSuccess = false;

    if ((true == m_brokerAddress.isEmpty()) || (0U == m_brokerPort))
    {
        LOG_ERROR("Broker address or port not set.");
        m_state = STATE_UNINITIALIZED;
    }
    else if (WL_CONNECTED != WiFi.status())
    {
        LOG_ERROR("WiFi not connected.");
    }
    else
    {
        if (true == m_willTopic.isEmpty())
        {
            isSuccess = m_mqttClient.connect(m_clientId.c_str());
        }
        else
        {
            isSuccess = m_mqttClient.connect(m_clientId.c_str(), nullptr, nullptr, m_willTopic.c_str(), 0, true,
                                             m_willMessage.c_str());
        }

        if (false == isSuccess)
        {
            LOG_ERROR("Failed to connect to MQTT broker at %s:%d", m_brokerAddress.c_str(), m_brokerPort);
        }
        else
        {
            m_state = STATE_CONNECTING;
        }
    }
}

void Network::onMessageCallback(char* topic, uint8_t* payload, uint32_t length)
{
    SubscriberList::const_iterator it;

    LOG_DEBUG("MQTT Rx in topic %s", topic);

    for (it = m_subscriberList.begin(); it != m_subscriberList.end(); ++it)
    {
        if (nullptr != (*it))
        {
            if (0 == strcmp((*it)->topic.c_str(), topic))
            {
                Subscriber* subscriber = *it;

                String payloadString = String((char*)payload, length);

                subscriber->callback(payloadString);
                break;
            }
        }
    }
}

bool Network::manageWiFi()
{
    bool isSuccess = true;

    if (true == m_isWiFiConfigured)
    {
        if (WL_CONNECTED != WiFi.status())
        {
            if (false == m_wifiTimeoutTimer.isTimerRunning())
            {
                /* Start timeout timer. */
                LOG_DEBUG("Connection to WiFi lost. Reconnecting...");
                m_wifiTimeoutTimer.start(WIFI_TIMEOUT);
            }
            else if (true == m_wifiTimeoutTimer.isTimeout())
            {
                LOG_ERROR("WiFi Connection Timed-out!");
                isSuccess = false;
            }
            else
            {
                /* WiFi is connecting. Do nothing. */
            }
        }
        else
        {
            if (true == m_wifiTimeoutTimer.isTimerRunning())
            {
                /* Stop timer. */
                m_wifiTimeoutTimer.stop();
            }
        }
    }

    return isSuccess;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
