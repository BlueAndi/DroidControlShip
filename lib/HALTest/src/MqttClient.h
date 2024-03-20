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
 *
 * @addtogroup HALSim
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

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class provides mqtt client functionality. */
class MqttClient : public IMqttClient
{
public:
    /**
     * Constructs the MQTTClient.
     */
    MqttClient();

    /**
     * Destroys the MQTTClient.
     */
    virtual ~MqttClient();

    /**
     * Initialize MQTTClient state.
     *
     * @return If successfully initialized, returns true. Otherwise, false.
     */
    bool init() final;

    /**
     * Process communication with the broker.
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
     * Start connection to the broker.
     * This method does not necessarily wait for the connection to be established, it just starts the connection
     * process. Check `isConnected()` for the current connection status.
     *
     * @return If connection has been succesfully started, returns true. Otherwise, false.
     */
    bool connect() final;

    /**
     * Disconnect from the broker.
     */
    void disconnect() final;

    /**
     * Is connected to the broker?
     *
     * @return If connected, it will return true otherwise false.
     */
    bool isConnected() const final;

    /**
     * Publishes a message to the broker.
     *
     * @param[in] topic                     Topic to publish to.
     * @param[in] useClientIdAsBaseTopic    If true, the client ID is used as the base (prefix) of the topic.
     * @param[in] message                   Message to publish.
     *
     * @return If successfully published, returns true. Otherwise, false.
     */
    bool publish(const String& topic, const bool useClientIdAsBaseTopic, const String& message) final;

    /**
     * Subscribes to a topic.
     *
     * @param[in] topic                     Topic to subscribe to.
     * @param[in] useClientIdAsBaseTopic    If true, the client ID is used as the base (prefix) of the topic.
     * @param[in] callback                  Callback function, which is called on a new message.
     *
     * @return If successfully subscribed, returns true. Otherwise, false.
     */
    bool subscribe(const String& topic, const bool useClientIdAsBaseTopic, TopicCallback callback) final;

    /**
     * Unsubscribes from a topic.
     *
     * @param[in] topic                     Topic to unsubscribe from.
     * @param[in] useClientIdAsBaseTopic    If true, the client ID is used as the base (prefix) of the topic.
     */
    void unsubscribe(const String& topic, const bool useClientIdAsBaseTopic) final;

private:
    /* Not allowed. */
    MqttClient(const MqttClient& src);            /**< Copy construction of an instance. */
    MqttClient& operator=(const MqttClient& rhs); /**< Assignment of an instance. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* MQTTCLIENT_H */
/** @} */
