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
 * @brief  Vehicle to Vehicle (V2V) communication client.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "V2VClient.h"
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

V2VClient::V2VClient(MqttClient& mqttClient) :
    m_mqttClient(mqttClient),
    m_waypointQueue(),
    m_inputTopic(),
    m_outputTopic(),
    m_isLeader(false)
{
}

V2VClient::~V2VClient()
{
}

bool V2VClient::init(uint8_t platoonId, uint8_t vehicleId)
{
    bool        isSuccessful = false;
    char        inputTopicBuffer[MAX_TOPIC_LENGTH];
    char        outputTopicBuffer[MAX_TOPIC_LENGTH];
    uint8_t     followerVehicleId = vehicleId + 1U; /* Output is published to next vehicle. */
    const char* outputSubtopic    = "targetWaypoint";

    if (PLATOON_LEADER_ID == vehicleId)
    {
        /* Its the leader. */
        m_isLeader = true;
    }
    else if (MAX_FOLLOWERS == vehicleId)
    {
        /* Last follower. Sends data to the leader. */
        followerVehicleId = 0U;
        outputSubtopic    = "feedback";
    }
    else
    {
        ; /* Its a normal follower. Nothing to do */
    }

    if (MAX_FOLLOWERS < vehicleId)
    {
        /* Invalid ID. */
        LOG_ERROR("Invalid vehicle ID: %d. Maximum followers: %d.", vehicleId, MAX_FOLLOWERS);
    }
    else if (0 >= snprintf(inputTopicBuffer, MAX_TOPIC_LENGTH, "platoons/%d/vehicles/%d/targetWaypoint", platoonId,
                           vehicleId))
    {
        LOG_ERROR("Failed to create input topic.");
    }
    else if (0 >= snprintf(outputTopicBuffer, MAX_TOPIC_LENGTH, "platoons/%d/vehicles/%d/%s", platoonId,
                           followerVehicleId, outputSubtopic))
    {
        LOG_ERROR("Failed to create output topic.");
    }
    else
    {
        /* Set topics. */
        m_inputTopic  = inputTopicBuffer;
        m_outputTopic = outputTopicBuffer;

        LOG_DEBUG("Input Topic: %s", m_inputTopic.c_str());
        LOG_DEBUG("Output Topic: %s", m_outputTopic.c_str());

        IMqttClient::TopicCallback lambdaTargetWaypointTopicCallback = [this](const String& payload)
        { targetWaypointTopicCallback(payload); };

        if ((true == m_inputTopic.isEmpty()) || (true == m_outputTopic.isEmpty()))
        {
            LOG_ERROR("Failed to create Platoon MQTT topics.");
        }
        /* Subscribe to Input Topic. */
        else if (false == m_mqttClient.subscribe(m_inputTopic, false, lambdaTargetWaypointTopicCallback))
        {
            LOG_ERROR("Could not subcribe to MQTT Topic: %s.", m_inputTopic.c_str());
        }
        else
        {

            isSuccessful = true;
        }
    }

    return isSuccessful;
}

void V2VClient::process()
{
    /* Nothing to do here yet. */
}

bool V2VClient::sendWaypoint(const Waypoint& waypoint)
{
    bool   isSuccessful = false;
    String payload;
    waypoint.serialize(payload);

    if (true == payload.isEmpty())
    {
        LOG_DEBUG("Failed to serialize waypoint.");
    }
    else if (false == m_mqttClient.publish(m_outputTopic, false, payload))
    {
        LOG_ERROR("Failed to publish MQTT message to %s.", m_outputTopic.c_str());
    }
    else
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

bool V2VClient::getNextWaypoint(Waypoint& waypoint)
{
    bool isSuccessful = false;

    if (false == m_waypointQueue.empty())
    {
        /* Retrieve next waypoint. */
        Waypoint* nextWaypoint = m_waypointQueue.front();
        m_waypointQueue.pop();

        /* Copy waypoint. */
        waypoint = *nextWaypoint;

        /* Delete queued waypoint. */
        delete nextWaypoint;

        isSuccessful = true;
    }

    return isSuccessful;
}

size_t V2VClient::getWaypointQueueSize() const
{
    return m_waypointQueue.size();
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void V2VClient::targetWaypointTopicCallback(const String& payload)
{
    Waypoint* waypoint = Waypoint::deserialize(payload);

    if (nullptr == waypoint)
    {
        LOG_ERROR("Failed to deserialize received waypoint.");
    }
    else
    {
        m_waypointQueue.push(waypoint);
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/