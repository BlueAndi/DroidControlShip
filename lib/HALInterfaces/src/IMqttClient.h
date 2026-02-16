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
 * @brief  Abstract MQTT Client interface
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALInterfaces
 *
 * @{
 */

#ifndef IMQTTCLIENT_H
#define IMQTTCLIENT_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include <WString.h>
#include <functional>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** MQTT settings struct */
struct MqttSettings
{
    String   clientId;      /**< Client ID. */
    String   brokerAddress; /**< Broker address to connect to. */
    uint16_t brokerPort;    /**< Broker port to connect to. */
    String   birthTopic;    /**< Birth topic. If empty, no birth message is used. */
    String   birthMessage;  /**< Birth message. */
    String   willTopic;     /**< Last will topic. If empty, no last will is used. */
    String   willMessage;   /**< Last will message. */
    bool     reconnect;     /**< If true, the client will try to reconnect to the broker, if the connection is lost. */
};

/** The abstract MQTT Client interface. */
class IMqttClient
{
public:
    /**
     * Topic callback prototype.
     */
    typedef std::function<void(const String& payload)> TopicCallback;

    /**
     * Destroys the interface.
     */
    virtual ~IMqttClient()
    {
    }

    /**
     * Initialize client state.
     *
     * @return If successfully initialized, returns true. Otherwise, false.
     */
    virtual bool init() = 0;

    /**
     * Process communication.
     *
     * @return If communication is successful, returns true. Otherwise, false.
     */
    virtual bool process() = 0;

    /**
     * Set client configuration.
     *
     * @param[in] settings Mqtt settings struct.
     * @return If successfully set, returns true. Otherwise, false.
     */
    virtual bool setConfig(const MqttSettings& settings) = 0;

    /**
     * Start connection to the broker.
     * This method does not necessarily wait for the connection to be established, it just starts the connection
     * process. Check `isConnected()` for the current connection status.
     *
     * @return If connection has been successfully started, returns true. Otherwise, false.
     */
    virtual bool connect() = 0;

    /**
     * Disconnect from the broker.
     */
    virtual void disconnect() = 0;

    /**
     * Is connected to the broker?
     *
     * @return If connected, it will return true otherwise false.
     */
    virtual bool isConnected() const = 0;

    /**
     * Publishes a message to the broker.
     *
     * @param[in] topic                     Topic to publish to.
     * @param[in] useClientIdAsBaseTopic    If true, the client ID is used as the base (prefix) of the topic.
     * @param[in] message                   Message to publish.
     *
     * @return If successfully published, returns true. Otherwise, false.
     */
    virtual bool publish(const String& topic, const bool useClientIdAsBaseTopic, const String& message) = 0;

    /**
     * Subscribes to a topic.
     *
     * @param[in] topic                     Topic to subscribe to.
     * @param[in] useClientIdAsBaseTopic    If true, the client ID is used as the base (prefix) of the topic.
     * @param[in] callback                  Callback function, which is called on a new message.
     *
     * @return If successfully subscribed, returns true. Otherwise, false.
     */
    virtual bool subscribe(const String& topic, const bool useClientIdAsBaseTopic, TopicCallback callback) = 0;

    /**
     * Unsubscribes from a topic.
     *
     * @param[in] topic                     Topic to unsubscribe from.
     * @param[in] useClientIdAsBaseTopic    If true, the client ID is used as the base (prefix) of the topic.
     */
    virtual void unsubscribe(const String& topic, const bool useClientIdAsBaseTopic) = 0;

protected:
    /**
     * Constructs the interface.
     */
    IMqttClient()
    {
    }

private:
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* IMQTTCLIENT_H */
/** @} */
