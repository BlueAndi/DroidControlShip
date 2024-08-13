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
 * @brief  Subscriber Class.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Logging.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

static void localCallback(const void* msgData, void* subscriberContext);

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Base subscriber class.
 */
class BaseSubscriber
{
public:
    /**
     * Default destructor.
     */
    virtual ~BaseSubscriber()
    {
    }

    /**
     * Initialize subscription.
     *
     * @param[in] node      Pointer to the RCL node.
     * @param[in] executor  Pointer to the RCLC executor.
     *
     * @returns If succesfully initialized, returns true. Otherwise, false.
     */
    virtual bool init(rcl_node_t* node, rclc_executor_t* executor) = 0;

    /**
     * Topic callback.
     *
     * @param[in] msgData   Incoming data from the topic message.
     */
    virtual void topicCallback(const void* msgData) = 0;

    /**
     * Topic Name
     */
    String m_topicName;

    /**
     * Type Support Structure from the ROS IDL.
     * Must be initialized using the ROSIDL_GET_MSG_TYPE_SUPPORT macro.
     */
    const rosidl_message_type_support_t* m_typeSupport;

    /**
     * RCL Subscription instance.
     */
    rcl_subscription_t m_subscriber;

protected:
    /**
     * Constructor of a BaseSubscriber.
     *
     * @param[in] topicName     Name of the topic to subscribe to.
     * @param[in] typeSupport   Pointer to support structure for the message type.
     */
    BaseSubscriber(const String& topicName, const rosidl_message_type_support_t* typeSupport) :
        m_topicName(topicName),
        m_typeSupport(typeSupport),
        m_subscriber()
    {
    }

private:
    /**
     * Default Constructor.
     * Not allowed.
     */
    BaseSubscriber();
};

/**
 * Subscriber class for an specific message type.
 * @tparam messageType ROS2 Topic message type
 */
template<typename messageType>
class Subscriber : public BaseSubscriber
{
public:
    /**
     * ROS Topic Prototype Callback.
     * Provides the received data in the respective topic to the application.
     *
     * @param[in] msgData       Received data.
     */
    typedef std::function<void(const messageType* msgData)> RosTopicCallback;

    /**
     * Construct a subscriber.
     *
     * @param[in] topicName     Name of the topic to subscribe to.
     * @param[in] typeSupport   Pointer to support structure for the message type.
     * @param[in] callback      Callback for the topic subscriber.
     */
    Subscriber(const String& topicName, const rosidl_message_type_support_t* typeSupport, RosTopicCallback callback) :
        BaseSubscriber(topicName, typeSupport),
        m_callback(callback),
        m_buffer()
    {
    }

    /**
     * Default destructor.
     */
    virtual ~Subscriber()
    {
    }

    /**
     * Initialize subscription.
     *
     * @param[in] node      Pointer to the RCL node.
     * @param[in] executor  Pointer to the RCLC executor.
     *
     * @returns If succesfully initialized, returns true. Otherwise, false.
     */
    bool init(rcl_node_t* node, rclc_executor_t* executor) final
    {
        bool isSuccessful = false;

        if ((nullptr == node) || (nullptr == executor))
        {
            LOG_ERROR("Node or executor are nullptr.");
        }
        else if (RCL_RET_OK != rclc_subscription_init_default(&m_subscriber, node, m_typeSupport, m_topicName.c_str()))
        {
            LOG_ERROR("Failed initialization of subscription.");
        }
        else if (RCL_RET_OK != rclc_executor_add_subscription_with_context(executor, &m_subscriber, &m_buffer,
                                                                           &localCallback, this, ON_NEW_DATA))
        {
            LOG_ERROR("Failed to add subscription to executor.");
        }
        else
        {
            isSuccessful = true;
        }

        return isSuccessful;
    }

    /**
     * Topic callback.
     *
     * @param[in] msgData   Incoming data from the topic message.
     */
    void topicCallback(const void* msgData) final
    {
        if (nullptr != msgData)
        {
            m_callback(reinterpret_cast<const messageType*>(msgData));
        }
    }

    /**
     * Callback for the subscription topic.
     */
    RosTopicCallback m_callback;

    /**
     * Buffer for the incomming messages.
     */
    messageType m_buffer;

private:
    /**
     * Default Constructor.
     * Not allowed.
     */
    Subscriber();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

/**
 * Local callback for all topics and subscribers.
 * Serves as a router for the correct callback.
 *
 * @param[in] msgData               Received data.
 * @param[in] subscriberContext     Pointer to the correct subscriber.
 */
static void localCallback(const void* msgData, void* subscriberContext)
{
    if ((nullptr != msgData) && (nullptr != subscriberContext))
    {
        BaseSubscriber* subscriber = reinterpret_cast<BaseSubscriber*>(subscriberContext);
        subscriber->topicCallback(msgData);
    }
}

#endif /* SUBSCRIBER_H */
/** @} */
