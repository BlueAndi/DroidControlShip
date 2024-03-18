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
#include <Util.h>

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

MqttClient::MqttClient() : IMqttClient()
{
}

MqttClient::~MqttClient()
{
}

bool MqttClient::init()
{
    LOG_WARNING("MQTT client not implemented for test environment.");
    return false;
}

bool MqttClient::process()
{
    LOG_WARNING("MQTT client not implemented for test environment.");
    return false;
}

bool MqttClient::setConfig(const MqttSettings& settings)
{
    UTIL_NOT_USED(settings);
    LOG_WARNING("MQTT client not implemented for test environment.");
    return false;
}

bool MqttClient::connect()
{
    LOG_WARNING("MQTT client not implemented for test environment.");
    return false;
}

void MqttClient::disconnect()
{
    LOG_WARNING("MQTT client not implemented for test environment.");
}

bool MqttClient::isConnected() const
{
    LOG_WARNING("MQTT client not implemented for test environment.");
    return false;
}

bool MqttClient::publish(const String& topic, const bool useClientIdAsBaseTopic, const String& message)
{
    UTIL_NOT_USED(topic);
    UTIL_NOT_USED(useClientIdAsBaseTopic);
    UTIL_NOT_USED(message);
    LOG_WARNING("MQTT client not implemented for test environment.");
    return false;
}

bool MqttClient::subscribe(const String& topic, const bool useClientIdAsBaseTopic, TopicCallback callback)
{
    LOG_WARNING("MQTT client not implemented for test environment.");
    return false;
}

void MqttClient::unsubscribe(const String& topic, const bool useClientIdAsBaseTopic)
{
    UTIL_NOT_USED(topic);
    UTIL_NOT_USED(useClientIdAsBaseTopic);
    LOG_WARNING("MQTT client not implemented for test environment.");
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
