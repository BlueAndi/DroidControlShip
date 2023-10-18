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
 * @brief  Abstract network interface
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALInterfaces
 *
 * @{
 */

#ifndef INETWORK_H
#define INETWORK_H

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

/** The abstract network interface. */
class INetwork
{
public:
    /**
     * Topic callback prototype.
     */
    typedef std::function<void(const String& payload)> TopicCallback;

    /**
     * Destroys the interface.
     */
    virtual ~INetwork()
    {
    }

    /**
     * Initialize network driver.
     *
     * @return If successfully initialized, returns true. Otherwise, false.
     */
    virtual bool init() = 0;

    /**
     * Process communication with the network.
     *
     * @return If communication is successful, returns true. Otherwise, false.
     */
    virtual bool process() = 0;

    /**
     * Set client configuration.
     *
     * @param[in] clientId      Client ID.
     * @param[in] ssid          SSID of the WiFi network.
     * @param[in] password      Password of the WiFi network.
     * @param[in] brokerAddress Broker address to connect to.
     * @param[in] brokerPort    Broker port to connect to.
     * @param[in] birthTopic    Birth topic. If empty, no birth message is used.
     * @param[in] birthMessage  Birth message.
     * @param[in] willTopic     Last will topic. If empty, no last will is used.
     * @param[in] willMessage   Last will message.
     * @param[in] reconnect     If true, the client will try to reconnect to the broker, if the connection is lost.
     * @return If successfully set, returns true. Otherwise, false.
     */
    virtual bool setConfig(const String& clientId, const String& ssid, const String& password,
                           const String& brokerAddress, uint16_t brokerPort, const String& birthTopic,
                           const String& birthMessage, const String& willTopic, const String& willMessage,
                           bool reconnect) = 0;

    /**
     * Start connection to the network.
     * This method does not necessarily wait for the connection to be established, it just starts the connection
     * process. Check `isConnected()` for the current connection status.
     *
     * @return If connection has been succesfully started, returns true. Otherwise, false.
     */
    virtual bool connect() = 0;

    /**
     * Disconnect from the network.
     */
    virtual void disconnect() = 0;

    /**
     * Is connected to the network?
     *
     * @return If connected, it will return true otherwise false.
     */
    virtual bool isConnected() const = 0;

    /**
     * Publishes a message to the network.
     *
     * @param[in] topic                Topic to publish to.
     * @param[in] useClientBaseTopic   If true, the client ID is used as the base (prefix) of the topic.
     * @param[in] message              Message to publish.
     */
    virtual bool publish(const String& topic, const bool useClientBaseTopic, const String& message) = 0;

    /**
     * Subscribes to a topic.
     *
     * @param[in] topic     Topic to subscribe to. The Client ID is used as base topic: <Client ID>/<topic>
     * @param[in] callback  Callback function, which is called on a new message.
     * @return If successfully subscribed, returns true. Otherwise, false.
     */
    virtual bool subscribe(const String& topic, TopicCallback callback) = 0;

    /**
     * Unsubscribes from a topic.
     *
     * @param[in] topic     Topic to unsubscribe from.  The Client ID is used as base topic: <Client ID>/<topic>
     */
    virtual void unsubscribe(const String& topic) = 0;

protected:
    /**
     * Constructs the interface.
     */
    INetwork()
    {
    }

private:
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* INETWORK_H */
/** @} */
