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
 * @brief  Vehicle to Vehicle (V2V) communication manager.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "V2VCommManager.h"
#include <Logging.h>
#include <ArduinoJson.h>

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
const char* V2VCommManager::TOPIC_NAME_WAYPOINT_RX = "inputWaypoint";

/* MQTT subtopic name for platoon heartbeat. */
const char* V2VCommManager::TOPIC_NAME_PLATOON_HEARTBEAT = "heartbeat";

/* MQTT subtopic name for platoon heartbeat. */
const char* V2VCommManager::TOPIC_NAME_PLATOON_HEARTBEAT_RESPONSE = "heartbeatResponse";

/** Buffer size for JSON serialization of heartbeat messages. */
static const uint32_t JSON_HEARTBEAT_MAX_SIZE = 128U;

/** Default size of the JSON Document for parsing. */
static const uint32_t JSON_DOC_DEFAULT_SIZE = 512U;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

V2VCommManager::V2VCommManager(MqttClient& mqttClient) :
    m_mqttClient(mqttClient),
    m_eventQueue(),
    m_waypointInputTopic(),
    m_waypointOutputTopic(),
    m_platoonHeartbeatTopic(),
    m_heartbeatResponseTopic(),
    m_feedbackTopic(),
    m_participantType(PARTICIPANT_TYPE_UNKNOWN),
    m_platoonId(0U),
    m_vehicleId(0U),
    m_lastPlatoonHeartbeatTimestamp(0U),
    m_followerResponseCounter(0U),
    m_platoonHeartbeatTimer(),
    m_vehicleHeartbeatTimeoutTimer(),
    m_v2vStatus(V2V_STATUS_NOT_INIT),
    m_vehicleStatus(),
    m_followers{},
    m_lastWaypointTimestamp(0U)
{
}

V2VCommManager::~V2VCommManager()
{
    /* Empty the queue. */
    while (0U != m_eventQueue.size())
    {
        V2VEvent event = m_eventQueue.front();
        m_eventQueue.pop();

        if (nullptr != event.data)
        {
            switch (event.type)
            {
            case V2V_EVENT_WAYPOINT:
                /* Fallthrough */
            case V2V_EVENT_FEEDBACK:
            {
                Waypoint* waypoint = static_cast<Waypoint*>(event.data);
                delete waypoint;
                break;
            }

            default:
                /* Should never be called!. */
                LOG_FATAL("Danger! Unknown pointer cannot be deleted!");
                break;
            }
        }
    }
}

bool V2VCommManager::init(uint8_t platoonId, uint8_t vehicleId)
{
    bool isSuccessful = false;

    m_vehicleId = vehicleId;

    if (NUMBER_OF_FOLLOWERS < m_vehicleId)
    {
        /* Invalid ID. */
        LOG_ERROR("Invalid vehicle ID: %d. Maximum followers: %d.", m_vehicleId, NUMBER_OF_FOLLOWERS);
    }
    else if (PLATOON_LEADER_ID == m_vehicleId)
    {
        /* Its the leader. */
        m_participantType = PARTICIPANT_TYPE_LEADER;
    }
    else if (NUMBER_OF_FOLLOWERS == m_vehicleId)
    {
        /* Its the last follower. */
        m_participantType = PARTICIPANT_TYPE_LAST_FOLLOWER;
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
    else if (false == setupWaypointTopics(platoonId, m_vehicleId))
    {
        LOG_ERROR("Failed to setup waypoint topics.");
    }
    else if (false == setupHeartbeatTopics(platoonId, m_vehicleId))
    {
        LOG_ERROR("Failed to setup heartbeat topics.");
    }
    else
    {
        isSuccessful = true;
    }

    if ((true == isSuccessful) && (PARTICIPANT_TYPE_LEADER == m_participantType))
    {
        isSuccessful = setupLeaderTopics();
        m_platoonHeartbeatTimer.start(PLATOON_HEARTBEAT_TIMER_INTERVAL);
    }

    if (true == isSuccessful)
    {
        m_v2vStatus = V2V_STATUS_OK;
    }

    return isSuccessful;
}

V2VCommManager::V2VStatus V2VCommManager::process(VehicleStatus status)
{
    m_vehicleStatus = status;

    /* Is MQTT client connected? */
    if ((true == m_mqttClient.isConnected()) && (V2V_STATUS_OK == m_v2vStatus))
    {
        /* Check participants heartbeats. Only active as leader. */
        if (true == m_vehicleHeartbeatTimeoutTimer.isTimeout())
        {
            if (NUMBER_OF_FOLLOWERS != m_followerResponseCounter)
            {
                LOG_ERROR("Not all participants responded to heartbeat.");
                m_v2vStatus = V2V_STATUS_LOST_FOLLOWER;
            }
            else
            {
                LOG_DEBUG("All participants responded to heartbeat.");
                m_v2vStatus = V2V_STATUS_OK;
            }

            /* Stop timer. */
            m_vehicleHeartbeatTimeoutTimer.stop();
        }

        /* Send Platoon Heartbeat. Only active as leader. */
        if (true == m_platoonHeartbeatTimer.isTimeout())
        {
            /* Send Platoon Heartbeat */
            if (false == sendPlatoonHeartbeat())
            {
                LOG_ERROR("Failed to send platoon heartbeat.");
                m_v2vStatus = V2V_STATUS_GENERAL_ERROR;
            }
            else
            {
                /* Start timeout timer. */
                m_vehicleHeartbeatTimeoutTimer.start(VEHICLE_HEARTBEAT_TIMEOUT_TIMER_INTERVAL);

                /* Reset follower response counter. */
                m_followerResponseCounter = 0U;
            }

            /* Reset timer. */
            m_platoonHeartbeatTimer.restart();
        }
    }

    return m_v2vStatus;
}

bool V2VCommManager::sendWaypoint(const Waypoint& waypoint)
{
    bool         isSuccessful = false;
    V2VEventType type         = V2V_EVENT_WAYPOINT;
    String       payload;
    waypoint.serialize(payload);

    if (PARTICIPANT_TYPE_LAST_FOLLOWER == m_participantType)
    {
        type = V2V_EVENT_FEEDBACK;
    }

    if (true == payload.isEmpty())
    {
        LOG_DEBUG("Failed to serialize waypoint.");
    }
    else if (false == publishEvent(m_waypointOutputTopic, type, payload))
    {
        LOG_ERROR("Failed to publish Waypoint Event");
    }
    else
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

bool V2VCommManager::getEvent(V2VEvent& event)
{
    bool isSuccessful = false;

    if (false == m_eventQueue.empty())
    {
        /* Retrieve next event. */
        event = m_eventQueue.front();
        m_eventQueue.pop();

        isSuccessful = true;
    }

    return isSuccessful;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void V2VCommManager::eventCallback(const String& payload)
{
    /* Deserialize payload. */
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> jsonPayload;
    DeserializationError                      error = deserializeJson(jsonPayload, payload.c_str());

    if (DeserializationError::Ok != error)
    {
        LOG_ERROR("JSON Deserialization Error %d.", error);
    }
    else
    {
        JsonVariant jsonEventVehicleId = jsonPayload["id"];        /* Vehicle ID. */
        JsonVariant jsonEventType      = jsonPayload["type"];      /* Event type. */
        JsonVariant jsonEventTimestamp = jsonPayload["timestamp"]; /* Timestamp [ms]. */
        JsonVariant jsonEventData      = jsonPayload["data"];      /* Event data. */

        if ((false == jsonEventVehicleId.isNull()) && (false == jsonEventType.isNull()) &&
            (false == jsonEventTimestamp.isNull()) && (false == jsonEventData.isNull()))
        {
            bool         pushEventToQueue = false;
            uint8_t      eventVehicleId   = jsonEventVehicleId.as<uint8_t>();
            V2VEventType eventType        = jsonEventType.as<V2VEventType>();
            uint32_t     eventTimestamp   = jsonEventTimestamp.as<uint32_t>();
            JsonObject   eventDataAsJson  = jsonEventData.as<JsonObject>();
            void*        eventData        = nullptr;

            switch (eventType)
            {
            case V2V_EVENT_WAYPOINT:
                /* Fallthrough */
            case V2V_EVENT_FEEDBACK:
                eventData = Waypoint::fromJsonObject(eventDataAsJson);
                if (nullptr == eventData)
                {
                    LOG_ERROR("Failed to create Waypoint from JSON.");
                    m_v2vStatus = V2V_STATUS_GENERAL_ERROR;
                }
                else if (eventTimestamp == m_lastWaypointTimestamp)
                {
                    LOG_ERROR("Received waypoint with same timestamp: %d.", eventTimestamp);
                    m_v2vStatus = V2V_STATUS_OLD_WAYPOINT;
                }
                else
                {
                    pushEventToQueue        = true;
                    m_lastWaypointTimestamp = eventTimestamp;
                }

                break;

            case V2V_EVENT_EMERGENCY:
                /* TODO: Implement Emergency */
                break;

            case V2V_EVENT_VEHICLE_HEARTBEAT:
                if (PLATOON_LEADER_ID == eventVehicleId)
                {
                    /* This is me, the leader! */
                }
                else if (eventVehicleId > NUMBER_OF_FOLLOWERS)
                {
                    LOG_ERROR("Received heartbeat from unknown vehicle %d.", eventVehicleId);
                }
                else
                {
                    JsonVariant jsonEventDataStatus    = eventDataAsJson["status"];    /* Vehicle status. */
                    JsonVariant jsonEventDataTimestamp = eventDataAsJson["timestamp"]; /* Timestamp [ms]. */

                    if ((false == jsonEventDataStatus.isNull()) && (false == jsonEventDataTimestamp.isNull()))
                    {
                        uint8_t  eventDataStatus    = jsonEventDataStatus.as<uint8_t>();
                        uint32_t eventDataTimestamp = jsonEventDataTimestamp.as<uint32_t>();

                        processFollowerHeartbeat(eventVehicleId, eventDataTimestamp, eventDataStatus);
                    }
                    else
                    {
                        LOG_ERROR("Null value in heartbeat from vehicle %d.", eventVehicleId);
                    }
                }
                break;

            case V2V_EVENT_PLATOON_HEARTBEAT:
                if (PLATOON_LEADER_ID != m_vehicleId)
                {
                    JsonVariant jsonEventDataTimestamp = eventDataAsJson["timestamp"]; /* Timestamp [ms]. */

                    if (false == jsonEventDataTimestamp.isNull())
                    {
                        /* Timestamp is sent back to acknowledge synchronization. */
                        uint32_t eventDataTimestamp = jsonEventDataTimestamp.as<uint32_t>();
                        StaticJsonDocument<JSON_HEARTBEAT_MAX_SIZE> heartbeatDoc;
                        heartbeatDoc["timestamp"] = eventDataTimestamp;
                        heartbeatDoc["status"]    = m_vehicleStatus;

                        if (false == publishEvent(m_heartbeatResponseTopic, V2V_EVENT_VEHICLE_HEARTBEAT,
                                                  heartbeatDoc.as<JsonObject>()))
                        {
                            LOG_ERROR("Failed to publish MQTT message to %s.", m_heartbeatResponseTopic.c_str());
                        }
                        else
                        {
                            LOG_DEBUG("Sent vehicle heartbeat: %d", eventDataTimestamp);
                        }
                    }
                }
                break;

            case V2V_EVENT_TYPE_UNKNOWN:
            default:
                LOG_ERROR("Unknown event type: %d.", eventType);
                break;
            }

            if (false == pushEventToQueue)
            {
                /* Do nothing. */
            }
            else if (MAX_EVENT_QUEUE_SIZE <= m_eventQueue.size())
            {
                LOG_ERROR("Event queue is full.");
            }
            else
            {
                /* Create and push event. */
                V2VEvent event(eventVehicleId, eventType, eventTimestamp, eventData);
                m_eventQueue.push(event);
            }
        }
    }
}

bool V2VCommManager::setupWaypointTopics(uint8_t platoonId, uint8_t vehicleId)
{
    bool    isSuccessful         = false;
    uint8_t nextVehicleId        = vehicleId + 1U;           /* Output is published to next vehicle. */
    uint8_t lastFollowerOutputId = NUMBER_OF_FOLLOWERS + 1U; /* Output is published to next vehicle. */
    char    inputTopicBuffer[MAX_TOPIC_LENGTH];
    char    outputTopicBuffer[MAX_TOPIC_LENGTH];
    char    feedbackTopicBuffer[MAX_TOPIC_LENGTH];

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
    /* Feedback Topic. */
    if (0 >= snprintf(feedbackTopicBuffer, MAX_TOPIC_LENGTH, "platoons/%d/vehicles/%d/%s", m_platoonId,
                      lastFollowerOutputId, TOPIC_NAME_WAYPOINT_RX))
    {
        LOG_ERROR("Failed to create feedback topic.");
    }
    else
    {
        /* Set topics. */
        m_waypointInputTopic  = inputTopicBuffer;
        m_waypointOutputTopic = outputTopicBuffer;
        m_feedbackTopic       = feedbackTopicBuffer;

        if ((true == m_waypointInputTopic.isEmpty()) || (true == m_waypointOutputTopic.isEmpty()) ||
            (true == m_feedbackTopic.isEmpty()))
        {
            LOG_ERROR("Failed to create Waypoint MQTT topics.");
        }
        else
        {
            /* Create lambda callback function for the waypoint input topic. */
            IMqttClient::TopicCallback lambdaWaypointInputTopicCallback = [this](const String& payload)
            { this->eventCallback(payload); };

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

bool V2VCommManager::setupHeartbeatTopics(uint8_t platoonId, uint8_t vehicleId)
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
    else if (0 >= snprintf(vehicleHeartbeatTopicBuffer, MAX_TOPIC_LENGTH, "platoons/%d/%s", platoonId,
                           TOPIC_NAME_PLATOON_HEARTBEAT_RESPONSE))
    {
        LOG_ERROR("Failed to create Heartbeat Response topic.");
    }
    else
    {
        /* Set topics. */
        m_platoonHeartbeatTopic  = platoonHeartbeatTopicBuffer;
        m_heartbeatResponseTopic = vehicleHeartbeatTopicBuffer;

        LOG_DEBUG("Platoon Heartbeat Topic: %s", m_platoonHeartbeatTopic.c_str());
        LOG_DEBUG("Vehicle Heartbeat Topic: %s", m_heartbeatResponseTopic.c_str());

        if ((true == m_platoonHeartbeatTopic.isEmpty()) || (true == m_heartbeatResponseTopic.isEmpty()))
        {
            LOG_ERROR("Failed to create Platoon Heartbeat MQTT topics.");
        }
        else
        {
            /* Create lambda callback function for the platoon heartbeat topic. */
            IMqttClient::TopicCallback lambdaHeartbeatInputTopicCallback = [this](const String& payload)
            { this->eventCallback(payload); };

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

bool V2VCommManager::setupLeaderTopics()
{
    bool isSuccessful = false;

    /* Participant is the leader. */
    IMqttClient::TopicCallback lambdaEventCallback = [this](const String& payload) { this->eventCallback(payload); };

    if (true == m_heartbeatResponseTopic.isEmpty())
    {
        LOG_ERROR("Failed to create Leader Heartbeat Response MQTT topic.");
    }
    else if (true == m_feedbackTopic.isEmpty())
    {
        LOG_ERROR("Failed to create Leader Feedback MQTT topic.");
    }
    /* Subscribe to Vehicles Heartbeat Topics. */
    else if (false == m_mqttClient.subscribe(m_heartbeatResponseTopic, false, lambdaEventCallback))
    {
        LOG_ERROR("Could not subcribe to MQTT Topic: %s.", m_platoonHeartbeatTopic.c_str());
    }
    /* Subscribe to Last Vehicle Feedback Topic. */
    else if (false == m_mqttClient.subscribe(m_feedbackTopic, false, lambdaEventCallback))
    {
        LOG_ERROR("Could not subcribe to MQTT Topic: %s.", m_feedbackTopic.c_str());
    }
    else
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

bool V2VCommManager::sendPlatoonHeartbeat()
{
    bool isSuccessful = false;

    /* Send platoon heartbeat. */
    StaticJsonDocument<JSON_HEARTBEAT_MAX_SIZE> heartbeatDoc;
    String                                      heartbeatPayload;
    uint32_t                                    timestamp = millis();

    heartbeatDoc["timestamp"] = timestamp;

    if (0U == serializeJson(heartbeatDoc, heartbeatPayload))
    {
        LOG_ERROR("Failed to serialize heartbeat.");
    }
    else if (false == publishEvent(m_platoonHeartbeatTopic, V2V_EVENT_PLATOON_HEARTBEAT, heartbeatPayload))
    // else if (false == m_mqttClient.publish(m_platoonHeartbeatTopic, false, heartbeatPayload))
    {
        LOG_ERROR("Failed to publish MQTT message to %s.", m_platoonHeartbeatTopic.c_str());
    }
    else
    {
        /* Save last timestamp in which the follower heartbeat was present. May be used for debugging in the future. */
        m_lastPlatoonHeartbeatTimestamp = timestamp;
        isSuccessful                    = true;
        LOG_DEBUG("Sent platoon heartbeat: %d", m_lastPlatoonHeartbeatTimestamp);
    }

    return isSuccessful;
}

bool V2VCommManager::publishEvent(const String& topic, V2VEventType type, const String& data)
{
    bool isSuccessful = false;

    /* Create JSON document. */
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> jsonPayload;
    String                                    payload;

    jsonPayload["id"]        = m_vehicleId;
    jsonPayload["type"]      = type;
    jsonPayload["timestamp"] = millis();
    jsonPayload["data"]      = serialized(data);

    if (0U == serializeJson(jsonPayload, payload))
    {
        LOG_ERROR("Failed to serialize event %d for topic %s.", type, topic.c_str());
    }
    else if (false == m_mqttClient.publish(topic, false, payload))
    {
        LOG_ERROR("Failed to publish V2V Event to %s.", topic.c_str());
    }
    else
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

bool V2VCommManager::publishEvent(const String& topic, V2VEventType type, const JsonObject& data)
{
    bool   isSuccessful = false;
    String payload;

    serializeJson(data, payload);

    if (0U == payload.length())
    {
        LOG_ERROR("Failed to serialize event %d for topic %s.", type, topic.c_str());
    }
    else
    {
        isSuccessful = publishEvent(topic, type, payload);
    }

    return isSuccessful;
}

void V2VCommManager::processFollowerHeartbeat(uint8_t eventVehicleId, uint32_t eventDataTimestamp,
                                              uint8_t eventDataStatus)
{
    if (eventDataTimestamp != m_lastPlatoonHeartbeatTimestamp)
    {
        LOG_ERROR("Received heartbeat from vehicle %d with timestamp %d, expected %d.", eventVehicleId,
                  eventDataTimestamp, m_lastPlatoonHeartbeatTimestamp);
    }
    else
    {
        uint8_t   followerArrayIndex = eventVehicleId - 1U; /*Already checked id != 0U */
        Follower& follower           = m_followers[followerArrayIndex];

        /* Update follower. */
        follower.setLastHeartbeatTimestamp(eventDataTimestamp);
        follower.setStatus(eventDataStatus);

        /* Increment counter regardless of the status. */
        ++m_followerResponseCounter;

        /* Check follower status. */
        if (VEHICLE_STATUS_OK != eventDataStatus)
        {
            LOG_ERROR("Follower %d status: %d.", eventVehicleId, eventDataStatus);
            m_v2vStatus = V2V_STATUS_FOLLOWER_ERROR;
        }
        else
        {
            LOG_DEBUG("Follower %d Ok", eventVehicleId);
        }
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/