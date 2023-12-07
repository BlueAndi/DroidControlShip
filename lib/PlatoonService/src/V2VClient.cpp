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
 * @brief  Vehicle to Vehicle (V2V) communication client.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "V2VClient.h"
#include <SettingsHandler.h>
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

/* MQTT topic name for birth messages. */
const char* V2VClient::TOPIC_NAME_BIRTH = "birth";

/* MQTT topic name for will messages. */
const char* V2VClient::TOPIC_NAME_WILL = "will";

/** Default size of the JSON Document for parsing. */
static const uint32_t JSON_DOC_DEFAULT_SIZE = 1024U;

/** Platoon leader vehicle ID. */
static const uint8_t PLATOON_LEADER_ID = 0U;

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
    bool    isSuccessful = false;
    char    inputTopicBuffer[MAX_TOPIC_LENGTH];
    char    outputTopicBuffer[MAX_TOPIC_LENGTH];
    uint8_t followerVehicleId = vehicleId + 1U; /* Output is published to next vehicle. */

    if (PLATOON_LEADER_ID == vehicleId)
    {
        /* Its the leader. */
        m_isLeader = true;
    }
    else if (MAX_FOLLOWERS == vehicleId)
    {
        /* Last follower. Sends data to the leader. */
        followerVehicleId = 0U;
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
    else if (0 >= snprintf(outputTopicBuffer, MAX_TOPIC_LENGTH, "platoons/%d/vehicles/%d/targetWaypoint", platoonId,
                           followerVehicleId))
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
    bool                                      isSuccessful = false;
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> jsonPayload;

    jsonPayload["X"]           = waypoint.xPos;        /**< X position [mm]. */
    jsonPayload["Y"]           = waypoint.yPos;        /**< Y position [mm]. */
    jsonPayload["Orientation"] = waypoint.orientation; /**< Orientation [mrad]. */
    jsonPayload["Left"]        = waypoint.left;        /**< Left motor speed [steps/s]. */
    jsonPayload["Right"]       = waypoint.right;       /**< Right motor speed [steps/s]. */
    jsonPayload["Center"]      = waypoint.center;      /**< Center speed [steps/s]. */

    size_t jsonBufferSize = measureJson(jsonPayload);
    char   jsonBuffer[jsonBufferSize];

    if (jsonBufferSize != serializeJson(jsonPayload, jsonBuffer, jsonBufferSize))
    {
        LOG_ERROR("JSON serialization failed.");
    }
    else if (false == m_mqttClient.publish(m_outputTopic, false, String(jsonBuffer)))
    {
        LOG_ERROR("Failed to publish MQTT message to %s.", m_outputTopic);
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

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void V2VClient::targetWaypointTopicCallback(const String& payload)
{
    StaticJsonDocument<JSON_DOC_DEFAULT_SIZE> jsonPayload;
    DeserializationError                      error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON Deserialization Error %d.", error);
    }
    else
    {
        JsonVariant jsonXPos        = jsonPayload["X"];           /**< X position [mm]. */
        JsonVariant jsonYPos        = jsonPayload["Y"];           /**< Y position [mm]. */
        JsonVariant jsonOrientation = jsonPayload["Orientation"]; /**< Orientation [mrad]. */
        JsonVariant jsonLeft        = jsonPayload["Left"];        /**< Left motor speed [steps/s]. */
        JsonVariant jsonRight       = jsonPayload["Right"];       /**< Right motor speed [steps/s]. */
        JsonVariant jsonCenter      = jsonPayload["Center"];      /**< Center speed [steps/s]. */

        if ((false == jsonXPos.isNull()) && (false == jsonYPos.isNull()) && (false == jsonOrientation.isNull()) &&
            (false == jsonLeft.isNull()) && (false == jsonRight.isNull()) && (false == jsonCenter.isNull()))
        {
            Waypoint* waypoint = new (std::nothrow) Waypoint();

            if (nullptr != waypoint)
            {
                waypoint->xPos        = jsonXPos.as<int32_t>();
                waypoint->yPos        = jsonYPos.as<int32_t>();
                waypoint->orientation = jsonOrientation.as<int32_t>();
                waypoint->left        = jsonLeft.as<int16_t>();
                waypoint->right       = jsonRight.as<int16_t>();
                waypoint->center      = jsonCenter.as<int16_t>();

                m_waypointQueue.push(waypoint);
            }
            else
            {
                LOG_ERROR("Failed to allocate memory for received waypoint.");
            }
        }
        else
        {
            LOG_WARNING("Received invalid waypoint.");
        }
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/