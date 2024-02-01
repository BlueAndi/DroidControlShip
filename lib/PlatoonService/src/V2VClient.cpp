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

/* MQTT subtopic name for waypoint reception. */
const char* V2VClient::TOPIC_NAME_WAYPOINT_RX = "inputWaypoint";

/* MQTT subtopic name for platoon heartbeat. */
const char* V2VClient::TOPIC_NAME_PLATOON_HEARTBEAT = "heartbeat";

/******************************************************************************
 * Public Methods
 *****************************************************************************/

V2VClient::V2VClient(MqttClient& mqttClient) :
    m_mqttClient(mqttClient),
    m_waypointQueue(),
    m_waypointInputTopic(),
    m_waypointOutputTopic(),
    m_platoonHeartbeatTopic(),
    m_vehicleHeartbeatTopic(),
    m_participantType(PARTICIPANT_TYPE_UNKNOWN)
{
}

V2VClient::~V2VClient()
{
}

bool V2VClient::init(uint8_t platoonId, uint8_t vehicleId)
{
    bool isSuccessful = false;

    if (NUMBER_OF_FOLLOWERS < vehicleId)
    {
        /* Invalid ID. */
        LOG_ERROR("Invalid vehicle ID: %d. Maximum followers: %d.", vehicleId, NUMBER_OF_FOLLOWERS);
    }
    else if (PLATOON_LEADER_ID == vehicleId)
    {
        /* Its the leader. */
        m_participantType = PARTICIPANT_TYPE_LEADER;
    }
    else
    {
        /* Its a follower. */
        m_participantType = PARTICIPANT_TYPE_FOLLOWER;
    }

    if (PARTICIPANT_TYPE_UNKNOWN == m_participantType)
    {
        LOG_ERROR("Failed to determine participant type.");
    }
    else if (false == setupWaypointTopics(platoonId, vehicleId))
    {
        LOG_ERROR("Failed to setup waypoint topics.");
    }
    else if (false == setupHeartbeatTopics(platoonId, vehicleId))
    {
        LOG_ERROR("Failed to setup heartbeat topics.");
    }
    else
    {
        isSuccessful = true;
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
    else if (false == m_mqttClient.publish(m_waypointOutputTopic, false, payload))
    {
        LOG_ERROR("Failed to publish MQTT message to %s.", m_waypointOutputTopic.c_str());
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

void V2VClient::platoonHeartbeatTopicCallback(const String& payload)
{
    /* TODO: Receive Platoon Heartbeat, and send own vehicle heartbeat. */
}

bool V2VClient::setupWaypointTopics(uint8_t platoonId, uint8_t vehicleId)
{
    bool    isSuccessful  = false;
    uint8_t nextVehicleId = vehicleId + 1U; /* Output is published to next vehicle. */
    char    inputTopicBuffer[MAX_TOPIC_LENGTH];
    char    outputTopicBuffer[MAX_TOPIC_LENGTH];

    /* Input Topic. */
    if (0 >= snprintf(inputTopicBuffer, MAX_TOPIC_LENGTH, "platoons/%d/vehicles/%d/%s", platoonId, vehicleId,
                      TOPIC_NAME_WAYPOINT_RX))
    {
        LOG_ERROR("Failed to create input topic.");
    }
    /* Output Topic. */
    else if (0 >= snprintf(outputTopicBuffer, MAX_TOPIC_LENGTH, "platoons/%d/vehicles/%d/%s", platoonId, nextVehicleId,
                           TOPIC_NAME_WAYPOINT_RX))
    {
        LOG_ERROR("Failed to create output topic.");
    }
    else
    {
        /* Set topics. */
        m_waypointInputTopic  = inputTopicBuffer;
        m_waypointOutputTopic = outputTopicBuffer;

        if ((true == m_waypointInputTopic.isEmpty()) || (true == m_waypointOutputTopic.isEmpty()))
        {
            LOG_ERROR("Failed to create Waypoint MQTT topics.");
        }
        else
        {
            /* Create lambda callback function for the waypoint input topic. */
            IMqttClient::TopicCallback lambdaWaypointInputTopicCallback = [this](const String& payload)
            { this->targetWaypointTopicCallback(payload); };

            /* Subscribe to Input Topic. */
            if (false == m_mqttClient.subscribe(m_waypointInputTopic, false, lambdaWaypointInputTopicCallback))
            {
                LOG_ERROR("Could not subcribe to MQTT Topic: %s.", m_waypointInputTopic.c_str());
            }
            else
            {
                LOG_DEBUG("Waypoint Input Topic: %s", m_waypointInputTopic.c_str());
                LOG_DEBUG("Waypoint Output Topic: %s", m_waypointOutputTopic.c_str());
                isSuccessful = true;
            }
        }
    }

    return isSuccessful;
}

bool V2VClient::setupHeartbeatTopics(uint8_t platoonId, uint8_t vehicleId)
{
    bool isSuccessful = false;
    char platoonHeartbeatTopicBuffer[MAX_TOPIC_LENGTH];
    char vehicleHeartbeatTopicBuffer[MAX_TOPIC_LENGTH];

    /* Platoon Heartbeat Topic. */
    if (0 >= snprintf(platoonHeartbeatTopicBuffer, MAX_TOPIC_LENGTH, "platoons/%d/%s", platoonId,
                      TOPIC_NAME_PLATOON_HEARTBEAT))
    {
        LOG_ERROR("Failed to create Platoon Heartbeat topic.");
    }
    /* Heartbeat Response Topic. */
    else if (0 >= snprintf(vehicleHeartbeatTopicBuffer, MAX_TOPIC_LENGTH, "platoons/%d/vehicles/%d/%s", platoonId,
                           vehicleId, TOPIC_NAME_PLATOON_HEARTBEAT))
    {
        LOG_ERROR("Failed to create Heartbeat Response topic.");
    }
    else
    {
        /* Set topics. */
        m_platoonHeartbeatTopic = platoonHeartbeatTopicBuffer;
        m_vehicleHeartbeatTopic = vehicleHeartbeatTopicBuffer;

        LOG_DEBUG("Platoon Heartbeat Topic: %s", m_platoonHeartbeatTopic.c_str());
        LOG_DEBUG("Vehicle Heartbeat Topic: %s", m_vehicleHeartbeatTopic.c_str());

        if ((true == m_platoonHeartbeatTopic.isEmpty()) || (true == m_vehicleHeartbeatTopic.isEmpty()))
        {
            LOG_ERROR("Failed to create Platoon Heartbeat MQTT topics.");
        }
        else
        {
            /* Create lambda callback function for the platoon heartbeat topic. */
            IMqttClient::TopicCallback lambdaHeartbeatInputTopicCallback = [this](const String& payload)
            { this->platoonHeartbeatTopicCallback(payload); };

            /* Subscribe to platoon heartbeat Topic. */
            if (false == m_mqttClient.subscribe(m_platoonHeartbeatTopic, false, lambdaHeartbeatInputTopicCallback))
            {
                LOG_ERROR("Could not subcribe to MQTT Topic: %s.", m_platoonHeartbeatTopic.c_str());
            }
            else
            {
                isSuccessful = true;
            }
        }
    }

    return isSuccessful;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/