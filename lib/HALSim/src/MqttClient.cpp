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
 * @brief  MQTTClient realization
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "MqttClient.h"
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

extern void onConnect(mosquitto* mosq, void* obj, int rc);
extern void onDisconnect(mosquitto* mosq, void* obj, int rc);
extern void onMessage(mosquitto* mosq, void* obj, const mosquitto_message* msg);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

MqttClient::MqttClient() :
    IMqttClient(),
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
    m_connectionTimer(),
    m_subscriberList(),
    m_configSet(false),
    m_connectRequest(false),
    m_disconnectRequest(false)
{
}

MqttClient::~MqttClient()
{
    if (nullptr != m_mqttClient)
    {
        (void)mosquitto_loop_stop(m_mqttClient, true);
        mosquitto_destroy(m_mqttClient);
        m_mqttClient = nullptr;
        m_state      = STATE_UNINITIALIZED;
    }

    /* Is always success. */
    (void)mosquitto_lib_cleanup();
}

bool MqttClient::init()
{
    bool isSuccess = false;

    if (MOSQ_ERR_SUCCESS != mosquitto_lib_init())
    {
        LOG_ERROR("Failed to initialize mosquitto library.");
    }
    else
    {
        isSuccess = true;
        m_state   = STATE_SETUP;
    }

    return isSuccess;
}

bool MqttClient::process()
{
    bool isSuccess = true;

    switch (m_state)
    {
    case STATE_UNINITIALIZED:
        /* Nothing to do. */
        break;

    case STATE_SETUP:
        handleSetupState();
        break;

    case STATE_DISCONNECTED:
        handleDisconnectedState();
        break;

    case STATE_DISCONNECTING:
        handleDisconnectingState();
        break;

    case STATE_CONNECTED:
        handleConnectedState();
        break;

    case STATE_CONNECTING:
        handleConnectingState();
        break;

    default:
        break;
    }

    return isSuccess;
}

bool MqttClient::setConfig(const MqttSettings& settings)
{
    if (true == settings.clientId.isEmpty())
    {
        LOG_ERROR("Client ID is empty.");
    }
    else if (true == settings.brokerAddress.isEmpty())
    {
        LOG_ERROR("Broker address is empty.");
    }
    else if (0U == settings.brokerPort)
    {
        LOG_ERROR("Broker port is zero.");
    }
    else
    {
        if (false == settings.birthTopic.isEmpty())
        {
            m_birthTopic   = settings.birthTopic;
            m_birthMessage = settings.birthMessage;
        }

        if (false == settings.willTopic.isEmpty())
        {
            m_willTopic   = settings.willTopic;
            m_willMessage = settings.willMessage;
        }

        m_clientId      = settings.clientId;
        m_brokerAddress = settings.brokerAddress;
        m_brokerPort    = settings.brokerPort;
        m_reconnect     = settings.reconnect;
        m_configSet     = true;
    }

    return m_configSet;
}

bool MqttClient::connect()
{
    m_connectRequest = true;
    return m_connectRequest;
}

void MqttClient::disconnect()
{
    m_disconnectRequest = true;
    m_state             = STATE_DISCONNECTING;
}

bool MqttClient::isConnected() const
{
    return (STATE_CONNECTED == m_state);
}

bool MqttClient::publish(const String& topic, const bool useClientIdAsBaseTopic, const String& message)
{
    bool isSuccess = false;

    if ((true == isConnected()) && (false == topic.isEmpty()))
    {
        String fullTopic = "";

        if ((true == useClientIdAsBaseTopic) && (false == m_clientId.isEmpty()))
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

bool MqttClient::subscribe(const String& topic, const bool useClientIdAsBaseTopic, TopicCallback callback)
{
    bool isSuccess = false;

    if ((false == topic.isEmpty()) && (false == m_clientId.isEmpty()))
    {
        SubscriberList::const_iterator it;
        String                         fullTopic = "";

        if ((true == useClientIdAsBaseTopic) && (false == m_clientId.isEmpty()))
        {
            fullTopic = m_clientId + "/" + topic;
        }
        else
        {
            fullTopic = topic;
        }

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

void MqttClient::unsubscribe(const String& topic, const bool useClientIdAsBaseTopic)
{
    if ((false == topic.isEmpty()) && (false == m_clientId.isEmpty()))
    {
        SubscriberList::iterator it        = m_subscriberList.begin();
        String                   fullTopic = "";

        if ((true == useClientIdAsBaseTopic) && (false == m_clientId.isEmpty()))
        {
            fullTopic = m_clientId + "/" + topic;
        }
        else
        {
            fullTopic = topic;
        }

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

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void MqttClient::handleSetupState()
{
    if ((nullptr == m_mqttClient) && (true == m_configSet))
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
            /* Set Callbacks. */
            mosquitto_connect_callback_set(m_mqttClient, onConnect);
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

            /* Start Mosquitto Thread. */
            if (MOSQ_ERR_SUCCESS != mosquitto_loop_start(m_mqttClient))
            {
                LOG_ERROR("Failed to start MQTT client thread.");
            }
            else
            {
                /* Set new state. */
                m_state = STATE_DISCONNECTED;
                LOG_DEBUG("MQTT client initialized.");
            }
        }
    }
}

void MqttClient::handleDisconnectedState()
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

void MqttClient::handleDisconnectingState()
{
    if ((nullptr != m_mqttClient) && (true == m_disconnectRequest))
    {
        m_disconnectRequest = false;
        int ret             = mosquitto_disconnect(m_mqttClient);

        switch (ret)
        {
        case MOSQ_ERR_SUCCESS:
            LOG_DEBUG("MQTT client disconnecting from broker.");
            /* STATE_DISCONNECTED is set in onDisconnectCallback() */
            break;

        case MOSQ_ERR_INVAL:
            LOG_ERROR("Invalid parameter for MQTT client disconnection.");
            break;

        case MOSQ_ERR_NO_CONN:
            LOG_WARNING("MQTT client not connected to a broker.");
            m_state = STATE_DISCONNECTED;
            break;

        default:
            break;
        }
    }
}

void MqttClient::handleConnectedState()
{
    /* Nothing to do. */
}

void MqttClient::handleConnectingState()
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
        ; /* Do nothing. */
    }
}

void MqttClient::resubscribe()
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

void MqttClient::attemptConnection()
{
    if (nullptr == m_mqttClient)
    {
        LOG_ERROR("MQTT client not initialized.");
        m_state = STATE_UNINITIALIZED;
    }
    else if ((true == m_brokerAddress.isEmpty()) || (0U == m_brokerPort))
    {
        LOG_ERROR("Broker address or port not set.");
        m_state = STATE_UNINITIALIZED;
    }
    else
    {
        int ret = mosquitto_connect_async(m_mqttClient, m_brokerAddress.c_str(), m_brokerPort, 60);

        switch (ret)
        {
        case MOSQ_ERR_SUCCESS:
            LOG_DEBUG("MQTT client connecting to broker at %s:%d", m_brokerAddress.c_str(), m_brokerPort);
            m_state = STATE_CONNECTING;
            break;

        case MOSQ_ERR_INVAL:
            LOG_ERROR("Invalid parameter for MQTT client connection.");
            break;

        case MOSQ_ERR_ERRNO:
            LOG_ERROR("MQTT client connection failed: %s", strerror(errno));

        default:
            break;
        }
    }
}

void MqttClient::onConnectCallback(int rc)
{
    LOG_DEBUG("MQTT client connected to broker. rc=%d", rc);
    resubscribe();
    m_state = STATE_CONNECTED;

    /* Publish birth message. */
    (void)publish(m_birthTopic, false, m_birthMessage);
}

void MqttClient::onDisconnectCallback(int rc)
{
    LOG_DEBUG("MQTT client disconnected from broker with return code %d", rc);
    m_state = STATE_DISCONNECTED;

    if (0 == rc)
    {
        /* User has called disconnect(), so reconnect is not necessary. */
        m_reconnect = false;
    }
}

void MqttClient::onMessageCallback(const mosquitto_message* msg)
{
    SubscriberList::const_iterator it;
    char*                          payloadCStr = static_cast<char*>(msg->payload);
    String                         payloadStr  = String(payloadCStr, msg->payloadlen);

    for (it = m_subscriberList.begin(); it != m_subscriberList.end(); ++it)
    {
        if (nullptr != (*it))
        {
            if (0 == strcmp((*it)->topic.c_str(), msg->topic))
            {
                Subscriber* subscriber = *it;

                subscriber->callback(payloadStr);
                break;
            }
        }
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/**
 * Callback function, which is called on connect.
 *
 * @param[in] mosq  Mosquitto instance.
 * @param[in] obj   Object passed on mosquitto_new.
 * @param[in] rc    Result code.
 */
extern void onConnect(mosquitto* mosq, void* obj, int rc)
{
    MqttClient* mqttClient = static_cast<MqttClient*>(obj);

    if (nullptr != mqttClient)
    {
        mqttClient->onConnectCallback(rc);
    }
}

/**
 * Callback function, which is called on disconnect.
 *
 * @param[in] mosq  Mosquitto instance.
 * @param[in] obj   Object passed on mosquitto_new.
 * @param[in] rc    Result code.
 */
extern void onDisconnect(mosquitto* mosq, void* obj, int rc)
{
    MqttClient* mqttClient = static_cast<MqttClient*>(obj);

    if (nullptr != mqttClient)
    {
        mqttClient->onDisconnectCallback(rc);
    }
}

/**
 * Callback function, which is called on message reception.
 *
 * @param[in] mosq  Mosquitto instance.
 * @param[in] obj   Object passed on mosquitto_new.
 * @param[in] msg   Message received.
 */
extern void onMessage(mosquitto* mosq, void* obj, const mosquitto_message* msg)
{
    MqttClient* mqttClient = static_cast<MqttClient*>(obj);

    if (nullptr != mqttClient)
    {
        mqttClient->onMessageCallback(msg);
    }
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
