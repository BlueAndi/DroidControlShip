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
 *
 * @addtogroup PlatoonService
 *
 * @{
 */
#ifndef V2V_CLIENT_H
#define V2V_CLIENT_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <MqttClient.h>
#include <Waypoint.h>
#include <queue>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** V2V Client for external communication in the platooning context. */
class V2VClient
{
public:
    /** Platoon leader vehicle ID. */
    static const uint8_t PLATOON_LEADER_ID = 0U;

    /** Type of platoon participant. */
    enum ParticipantType : uint8_t
    {
        PARTICIPANT_TYPE_UNKNOWN = 0U, /**< Unknown participant type */
        PARTICIPANT_TYPE_LEADER,       /**< Platoon leader */
        PARTICIPANT_TYPE_FOLLOWER      /**< Platoon follower */
    };

    /**
     * Constructs a V2V client.
     *
     * @param[in] mqttClient    MQTT client instance.
     */
    V2VClient(MqttClient&);

    /**
     * Default destructor.
     */
    ~V2VClient();

    /**
     * Initialize the V2V client.
     *
     * @param[in] platoonId     ID of the platoon.
     * @param[in] vehicleId     ID of the vehicle inside the platoon.
     *
     * @return If the V2V client was initialized successfully, returns true. Otherwise, false.
     */
    bool init(uint8_t platoonId, uint8_t vehicleId);

    /**
     * Process the V2V client.
     */
    void process();

    /**
     * Send a Waypoint to the next vehicle in the platoon.
     *
     * @param[in] waypoint  Waypoint to send.
     *
     * @return If the waypoint was sent successfully, returns true. Otherwise, false.
     */
    bool sendWaypoint(const Waypoint& waypoint);

    /**
     * Get the next recevied Waypoint from the V2V client.
     *
     * @param[out] waypoint  Next waypoint to receive.
     *
     * @return If a waypoint was successfully received, returns true. Otherwise, false.
     */
    bool getNextWaypoint(Waypoint& waypoint);

    /**
     * Get the number of waypoints in the queue.
     *
     * @return Number of waypoints in the queue.
     */
    size_t getWaypointQueueSize() const;

private:
    /** Max topic length */
    static const uint8_t MAX_TOPIC_LENGTH = 64U;

    /** Number of followers. */
    static const uint8_t NUMBER_OF_FOLLOWERS = 1U;

    /** MQTT subtopic name for waypoint reception. */
    static const char* TOPIC_NAME_WAYPOINT_RX;

    /** MQTT subtopic name for platoon heartbeat. */
    static const char* TOPIC_NAME_PLATOON_HEARTBEAT;

    /** MQTT subtopic name for platoon heartbeat reponse. */
    static const char* TOPIC_NAME_PLATOON_HEARTBEAT_RESPONSE;

    /** MQTTClient Instance. */
    MqttClient& m_mqttClient;

    /**
     * Queue for the received waypoints.
     * Stores pointers to the waypoints in the queue when received in the callback.
     * The queue is emptied by the getNextWaypoint() method.
     * The queue is filled by the targetWaypointTopicCallback() method.
     *
     * @tparam Waypoint*    Pointer to a Waypoint.
     */
    std::queue<Waypoint*> m_waypointQueue;

    /** Topic to receive target Waypoints. */
    String m_waypointInputTopic;

    /** Topic to send target Waypoints. */
    String m_waypointOutputTopic;

    /** Topic to receive platoon heartbeat messages. */
    String m_platoonHeartbeatTopic;

    /** Topic to send vehicle heartbeat messages. */
    String m_heartbeatResponseTopic;

    /** Type of Platoon Participant.*/
    ParticipantType m_participantType;

    /** Platoon ID. */
    uint8_t m_platoonId;

    /** Vehicle ID. */
    uint8_t m_vehicleId;

private:
    /**
     * Callback for Position Setpoint MQTT Topic.
     *
     * @param[in] payload   Payload of the MQTT message.
     */
    void targetWaypointTopicCallback(const String& payload);

    /**
     * Callback for Platoon Heartbeat MQTT Topic.
     *
     * @param[in] payload   Payload of the MQTT message.
     */
    void platoonHeartbeatTopicCallback(const String& payload);

    /**
     * Callback for Vehicle Heartbeat MQTT Topic. Only used by the leader.
     *
     * @param[in] payload   Payload of the MQTT message.
     */
    void vehicleHeartbeatTopicCallback(const String& payload);

    /**
     * Setup waypoint input and output topics.
     *
     * @param[in] platoonId     ID of the platoon.
     * @param[in] vehicleId     ID of the vehicle inside the platoon.
     *
     * @return If the topics were setup successfully, returns true. Otherwise, false.
     */
    bool setupWaypointTopics(uint8_t platoonId, uint8_t vehicleId);

    /**
     * Setup heartbeat input and output topics.
     *
     * @param[in] platoonId     ID of the platoon.
     * @param[in] vehicleId     ID of the vehicle inside the platoon.
     *
     * @return If the topics were setup successfully, returns true. Otherwise, false.
     */
    bool setupHeartbeatTopics(uint8_t platoonId, uint8_t vehicleId);

    /**
     * Leader-specific setup. Sets up the topics for the leader, which must take care of its followers.
     *
     * @return If the topics were setup successfully, returns true. Otherwise, false.
     */
    bool setupLeaderTopics();

    /**
     * Default constructor.
     */
    V2VClient();

    /**
     * Copy constructor.
     */
    V2VClient(const V2VClient&);

    /**
     * Assignment operator.
     */
    V2VClient& operator=(const V2VClient&);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* V2V_CLIENT_H */
/** @} */
