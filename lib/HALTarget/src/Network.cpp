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
    m_willTopic(""),
    m_willMessage(""),
    m_reconnect(true),
    m_reconnectTimer(),
    m_subscriberList()
{
}

Network::~Network()
{
}

bool Network::init()
{
    m_state = STATE_IDLE;

    return true;
}

bool Network::process()
{
    bool isSuccess = true;

    switch (m_state)
    {
    case STATE_UNINITIALIZED:
        /* Nothing to do. */
        break;

    case STATE_IDLE:
        idleState();
        break;

    case STATE_DISCONNECTED:
        disconnectedState();
        break;

    case STATE_CONNECTED:
        connectedState();
        break;

    default:
        break;
    }

    return isSuccess;
}

bool Network::setConfig(const String& clientId, const String& brokerAddress, uint16_t brokerPort,
                        const String& willTopic, const String& willMessage, bool reconnect)
{
    bool isSuccess = false;

    if (true == clientId.isEmpty())
    {
        LOG_ERROR("Client ID is empty.");
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
        if (false == willTopic.isEmpty())
        {
            m_willTopic   = willTopic;
            m_willMessage = willMessage;
        }

        m_clientId      = clientId;
        m_brokerAddress = brokerAddress;
        m_brokerPort    = brokerPort;
        m_reconnect     = reconnect;
        isSuccess       = true;
    }

    return isSuccess;
}

bool Network::connect()
{
    bool isSuccess = false;

    if (STATE_CONNECTED == m_state)
    {
        LOG_INFO("Already connected to Broker.");
        isSuccess = true;
    }
    else if (STATE_DISCONNECTED != m_state)
    {
        LOG_ERROR("Invalid state. Current state: %d", m_state);
    }
    else if ((true == m_brokerAddress.isEmpty()) || (0U == m_brokerPort))
    {
        LOG_ERROR("Broker address or port not set.");
        m_state = STATE_IDLE;
    }
    else if (WiFi.status() != WL_CONNECTED)
    {
        LOG_ERROR("WiFi not connected.");
    }
    else
    {
        LOG_DEBUG("Connecting to MQTT broker at %s:%d", m_brokerAddress.c_str(), m_brokerPort);
        if (true == m_willTopic.isEmpty())
        {
            isSuccess = m_mqttClient.connect(m_clientId.c_str());
        }
        else
        {
            isSuccess = m_mqttClient.connect(m_clientId.c_str(), nullptr, nullptr, m_willTopic.c_str(), 0, true,
                                             m_willMessage.c_str());
        }
    }

    if (false == isSuccess)
    {
        LOG_ERROR("Failed to connect to MQTT broker at %s:%d", m_brokerAddress.c_str(), m_brokerPort);
        m_state = STATE_DISCONNECTED;
    }
    else
    {
        LOG_DEBUG("MQTT client connected to broker");
        resubscribe();
        m_state = STATE_CONNECTED;
    }

    return isSuccess;
}

void Network::disconnect()
{
    if (STATE_CONNECTED != m_state)
    {
        LOG_INFO("Already disconnected from Broker.");
    }
    else
    {
        m_mqttClient.disconnect();
        m_state = STATE_DISCONNECTED;
        /* No reconnection if user called for disconnect. */
        m_reconnect = false;
    }
}

bool Network::isConnected() const
{
    return (STATE_CONNECTED == m_state);
}

bool Network::publish(const String& topic, const String& message)
{
    LOG_DEBUG("Publishing message to topic %s", topic.c_str());
    return m_mqttClient.publish(topic.c_str(), message.c_str());
}

bool Network::subscribe(const String& topic, TopicCallback callback)
{
    bool isSuccess = false;

    if (false == topic.isEmpty())
    {
        SubscriberList::const_iterator it;

        /* Register a topic only once! */
        for (it = m_subscriberList.begin(); it != m_subscriberList.end(); ++it)
        {
            if (nullptr != (*it))
            {
                if ((*it)->topic == topic)
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
                subscriber->topic    = topic;
                subscriber->callback = callback;

                if (false == isConnected())
                {
                    m_subscriberList.push_back(subscriber);
                    isSuccess = true;
                }
                else
                {
                    m_mqttClient.unsubscribe(subscriber->topic.c_str());
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
    if (false == topic.isEmpty())
    {
        SubscriberList::iterator it = m_subscriberList.begin();

        while (m_subscriberList.end() != it)
        {
            if (nullptr != (*it))
            {
                if ((*it)->topic == topic)
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

void Network::idleState()
{
    if ((false == m_clientId.isEmpty()) && (false == m_brokerAddress.isEmpty()) && (0U != m_brokerPort))
    {
        (void)m_mqttClient.setServer(m_brokerAddress.c_str(), m_brokerPort);
        (void)m_mqttClient.setBufferSize(MAX_BUFFER_SIZE);
        (void)m_mqttClient.setKeepAlive(MQTT_KEEP_ALIVE_S);
        (void)m_mqttClient.setCallback([this](char* topic, uint8_t* payload, uint32_t length)
                                       { this->onMessageCallback(topic, payload, length); });

        m_state = STATE_DISCONNECTED;
    }
}

void Network::disconnectedState()
{
    bool connectNow = false;

    if (false == m_reconnect)
    {
        ; /* Do nothing. */
    }
    else if (false == m_reconnectTimer.isTimerRunning())
    {
        connectNow = true;

        /* Start reconnect timer. */
        m_reconnectTimer.start(RECONNECT_TIMEOUT_MS);
    }
    else if (true == m_reconnectTimer.isTimeout())
    {
        connectNow = true;
    }
    else
    {
        ; /* Do nothing. */
    }

    if (true == connectNow)
    {
        if (false == connect())
        {
            /* Reconnect at a later time. */
            m_reconnectTimer.restart();
        }
        else
        {
            /* Stop reconnect timer. */
            m_reconnectTimer.stop();
        }
    }
}

void Network::connectedState()
{
    /* Connection with broker lost? */
    if (false == m_mqttClient.connected())
    {
        LOG_DEBUG("MQTT connection lost.");
        m_state = STATE_DISCONNECTED;
    }
    /* Connection to broker still established. */
    else
    {
        (void)m_mqttClient.loop();
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

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
