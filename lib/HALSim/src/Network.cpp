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

static void onDisconnect(mosquitto* mosq, void* obj, int rc);
static void onMessage(mosquitto* mosq, void* obj, const mosquitto_message* msg);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

Network::Network() :
    INetwork(),
    m_state(STATE_UNINITIALIZED),
    m_mqttClient(nullptr),
    m_clientId(""),
    m_brokerAddress(""),
    m_brokerPort(0U),
    m_birthTopic(""),
    m_birthMessage(""),
    m_willTopic(""),
    m_willMessage(""),
    m_reconnect(true),
    m_reconnectTimer(),
    m_subscriberList()
{
}

Network::~Network()
{
    if (nullptr != m_mqttClient)
    {
        mosquitto_destroy(m_mqttClient);
        m_mqttClient = nullptr;
        m_state      = STATE_UNINITIALIZED;
    }

    /* Is always success. */
    (void)mosquitto_lib_cleanup();
}

bool Network::init()
{
    bool isSuccess = false;

    if (MOSQ_ERR_SUCCESS != mosquitto_lib_init())
    {
        LOG_ERROR("Failed to initialize mosquitto library.");
    }
    else
    {
        isSuccess = true;
        m_state   = STATE_IDLE;
    }

    return isSuccess;
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

bool Network::setConfig(const String& clientId, const String& ssid, const String& password, const String& brokerAddress,
                        uint16_t brokerPort, const String& birthTopic, const String& birthMessage,
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
        LOG_ERROR("Already connected to Broker.");
        isSuccess = true;
    }
    else if (STATE_DISCONNECTED != m_state)
    {
        LOG_ERROR("Invalid state. Current state: %d", m_state);
    }
    else if (nullptr == m_mqttClient)
    {
        LOG_ERROR("MQTT client not initialized.");
        m_state = STATE_IDLE;
    }
    else if ((true == m_brokerAddress.isEmpty()) || (0U == m_brokerPort))
    {
        LOG_ERROR("Broker address or port not set.");
        m_state = STATE_IDLE;
    }
    else if (MOSQ_ERR_SUCCESS != mosquitto_connect(m_mqttClient, m_brokerAddress.c_str(), m_brokerPort, 60))
    {
        LOG_ERROR("Failed to connect to MQTT broker at %s:%d", m_brokerAddress.c_str(), m_brokerPort);
        m_state = STATE_DISCONNECTED;
    }
    else
    {
        LOG_DEBUG("MQTT client connected to broker");
        resubscribe();
        m_state = STATE_CONNECTED;

        if (false == m_birthTopic.isEmpty())
        {
            /* Publish birth message. Should succesfully publish if connected to broker. */
            isSuccess = publish(m_birthTopic, false, m_birthMessage);
        }
        else
        {
            isSuccess = true;
        }
    }

    return isSuccess;
}

void Network::disconnect()
{
    if (STATE_CONNECTED != m_state)
    {
        LOG_ERROR("Already disconnected from Broker.");
    }
    else if (nullptr != m_mqttClient)
    {
        (void)mosquitto_disconnect(m_mqttClient);
        m_state = STATE_DISCONNECTED;
    }
}

bool Network::isConnected() const
{
    return (STATE_CONNECTED == m_state);
}

bool Network::publish(const String& topic, const bool useClientBaseTopic, const String& message)
{
    bool isSuccess = false;

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

        int ret =
            mosquitto_publish(m_mqttClient, nullptr, fullTopic.c_str(), message.length(), message.c_str(), 0, false);

        if (MOSQ_ERR_SUCCESS != ret)
        {
            LOG_ERROR("Failed to publish message to topic %s with Error %d", fullTopic.c_str(), ret);
        }
        else
        {
            isSuccess = true;
        }
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
                    if (MOSQ_ERR_SUCCESS != mosquitto_subscribe(m_mqttClient, nullptr, subscriber->topic.c_str(), 0))
                    {
                        LOG_WARNING("MQTT topic subscription not possible: %s", subscriber->topic.c_str());
                    }
                    else
                    {
                        m_subscriberList.push_back(subscriber);
                        isSuccess = true;
                    }
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

                    if (MOSQ_ERR_SUCCESS != mosquitto_unsubscribe(m_mqttClient, NULL, subscriber->topic.c_str()))
                    {
                        LOG_WARNING("MQTT topic unsubscription not possible: %s", subscriber->topic.c_str());
                    }
                    else
                    {
                        LOG_DEBUG("MQTT topic unsubscribed: %s", subscriber->topic.c_str());
                    }

                    (void)m_subscriberList.erase(it);
                    delete subscriber;

                    break;
                }
            }

            ++it;
        }
    }
}

void Network::onDisconnectCallback(int rc)
{
    LOG_DEBUG("MQTT client disconnected from broker with return code %d", rc);
    m_state = STATE_DISCONNECTED;

    if (0 == rc)
    {
        /* User has called disconnect(), so reconnect is not necessary. */
        m_reconnect = false;
    }
}

void Network::onMessageCallback(const mosquitto_message* msg)
{
    SubscriberList::const_iterator it;

    LOG_DEBUG("MQTT client received message on topic %s with payload %s", msg->topic, (char*)msg->payload);

    for (it = m_subscriberList.begin(); it != m_subscriberList.end(); ++it)
    {
        if (nullptr != (*it))
        {
            if (0 == strcmp((*it)->topic.c_str(), msg->topic))
            {
                Subscriber* subscriber = *it;

                String payload = (char*)msg->payload;
                subscriber->callback(payload);
                break;
            }
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
    if ((nullptr == m_mqttClient) && (false == m_clientId.isEmpty()) && (false == m_brokerAddress.isEmpty()) &&
        (0U != m_brokerPort))
    {
        /* Create MQTT client */
        m_mqttClient = mosquitto_new(m_clientId.c_str(), true, this);

        if (nullptr == m_mqttClient)
        {
            LOG_ERROR("Failed to create MQTT client: %s", mosquitto_strerror(errno));
            /* Uninitialize as something important may be wrong. */
            m_state = STATE_UNINITIALIZED;
        }
        else
        {
            m_state = STATE_DISCONNECTED;

            /* Set Callbacks. */
            mosquitto_disconnect_callback_set(m_mqttClient, onDisconnect);
            mosquitto_message_callback_set(m_mqttClient, onMessage);

            /* Set Will */
            if (false == m_willTopic.isEmpty())
            {
                if (MOSQ_ERR_SUCCESS != mosquitto_will_set(m_mqttClient, m_willTopic.c_str(), m_willMessage.length(),
                                                           m_willMessage.c_str(), 0, false))
                {
                    LOG_ERROR("Failed to set MQTT will.");
                }
            }
        }
    }
}

void Network::disconnectedState()
{
    bool connectNow = false;

    if (false == m_reconnectTimer.isTimerRunning())
    {
        connectNow = true;

        /* Start reconnect timer. */
        m_reconnectTimer.start(RECONNECT_TIMEOUT_MS);
    }
    else if ((true == m_reconnect) && (true == m_reconnectTimer.isTimeout()))
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
    int ret = mosquitto_loop(m_mqttClient, MQTT_LOOP_TIMEOUT_MS, 1);

    switch (ret)
    {
    case MOSQ_ERR_SUCCESS:
        /* Nothing to do. */
        break;

    case MOSQ_ERR_CONN_LOST:
        LOG_DEBUG("MQTT connection lost.");
        m_state = STATE_DISCONNECTED;
        break;

    default:
        LOG_ERROR("MQTT loop failed: %d", ret);
        disconnect();
        break;
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

            if (MOSQ_ERR_SUCCESS != mosquitto_subscribe(m_mqttClient, nullptr, subscriber->topic.c_str(), 0))
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

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * Callback function, which is called on disconnect.
 *
 * @param[in] mosq  Mosquitto instance.
 * @param[in] obj   Object passed on mosquitto_new.
 * @param[in] rc    Result code.
 */
void onDisconnect(mosquitto* mosq, void* obj, int rc)
{
    Network* network = static_cast<Network*>(obj);

    if (nullptr != network)
    {
        network->onDisconnectCallback(rc);
    }
}

/**
 * Callback function, which is called on message reception.
 *
 * @param[in] mosq  Mosquitto instance.
 * @param[in] obj   Object passed on mosquitto_new.
 * @param[in] msg   Message received.
 */
void onMessage(mosquitto* mosq, void* obj, const mosquitto_message* msg)
{
    Network* network = static_cast<Network*>(obj);

    if (nullptr != network)
    {
        network->onMessageCallback(msg);
    }
}
