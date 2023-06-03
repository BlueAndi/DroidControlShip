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
 *
 * @addtogroup HALTarget
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
     * Process communication with the network.
     *
     * @return If communication is successful, returns true. Otherwise, false.
     */
    bool process() final;

    /**
     * Set client configuration.
     *
     * @param[in] clientId      Client ID.
     * @param[in] brokerAddress Broker address to connect to.
     * @param[in] brokerPort    Broker port to connect to.
     * @param[in] willTopic     Last will topic. If empty, no last will is used.
     * @param[in] willMessage   Last will message.
     * @param[in] reconnect     If true, the client will try to reconnect to the broker, if the connection is lost.
     * @return If successfully set, returns true. Otherwise, false.
     */
    bool setConfig(const String& clientId, const String& brokerAddress, uint16_t brokerPort, const String& willTopic,
                   const String& willMessage, bool reconnect) final;

    /**
     * Connect to the network.
     *
     * @return If successfully connected, returns true. Otherwise, false.
     */
    bool connect() final;

    /**
     * Disconnect from the network.
     */
    void disconnect() final;

    /**
     * Is connected to the network?
     *
     * @return If connected, it will return true otherwise false.
     */
    bool isConnected() const final;

    /**
     * Publishes a message to the network.
     *
     * @param[in] topic     Topic to publish to.
     * @param[in] message   Message to publish.
     */
    bool publish(const String& topic, const String& message) final;

    /**
     * Subscribes to a topic.
     *
     * @param[in] topic     Topic to subscribe to.
     * @param[in] callback  Callback function, which is called on a new message.
     * @return If successfully subscribed, returns true. Otherwise, false.
     */
    bool subscribe(const String& topic, TopicCallback callback) final;

    /**
     * Unsubscribes from a topic.
     *
     * @param[in] topic     Topic to unsubscribe from.
     */
    void unsubscribe(const String& topic) final;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* NETWORK_H */
/** @} */
