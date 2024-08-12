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

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

class BaseSubscriber
{
public:
    String                               m_topicName;
    const rosidl_message_type_support_t* m_typeSupport;
    rcl_subscription_t                   m_subscriber;

protected:
    BaseSubscriber(const String& topicName, const rosidl_message_type_support_t* typeSupport) :
        m_topicName(topicName),
        m_typeSupport(typeSupport),
        m_subscriber()
    {
    }

    ~BaseSubscriber()
    {
    }
};

template<typename T>
class Subscriber : public BaseSubscriber
{
public:
    typedef std::function<void(const T*)> RosTopicCallback;

    Subscriber(const String& topicName, const rosidl_message_type_support_t* typeSupport, RosTopicCallback callback) :
        BaseSubscriber(topicName, typeSupport),
        m_callback(callback),
        m_allocatedBuffer()
    {
    }

    virtual ~Subscriber()
    {
    }

    T getBuffer() final
    {
        return m_allocatedBuffer;
    }

    RosTopicCallback m_callback;
    T                m_allocatedBuffer;

private:
    Subscriber();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SUBSCRIBER_H */
/** @} */
