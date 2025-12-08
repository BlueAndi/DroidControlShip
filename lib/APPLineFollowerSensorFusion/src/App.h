/* MIT License
 *
 * Copyright (c) 2023 - 2025 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Line follower Sensor fusion application
 * @author Tobias Haeckel <tobias.haeckel@gmx.net>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef APP_H
#define APP_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <Board.h>
#include <SerialMuxProtServer.hpp>
#include <SimpleTimer.hpp>
#include <StateMachine.h>
#include "SerMuxChannelProvider.h"
#include "LineSensors.h"
#include "Motors.h"
#include <MqttClient.h>
#include "TimeSync.h"
#include "EKF.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * @brief Space Ship Radar pose in global (SSR) coordinate frame.
 */
struct SpaceShipRadarPose
{
    uint32_t timestamp; /**< Local timestamp in ms (ESP time base). */
    float    x;         /**< X position in mm. */
    float    y;         /**< Y position in mm. */
    float    theta;     /**< Heading in mrad. */
    float    v_x;       /**< Velocity in X direction in mm/s. */
    float    v_y;       /**< Velocity in Y direction in mm/s. */
};

/**
 * @brief Line follower Sensor Fusion application.
 */
class App
{
public:
    /**
     * @brief Constructs the application.
     */
    App();

    /**
     * @brief Destroys the application.
     */
    ~App()
    {
    }

    /**
     * @brief Setup the application.
     */
    void setup();

    /**
     * @brief Process the application periodically.
     */
    void loop();

private:
    /** MQTT topic name for birth messages. */
    static const char* TOPIC_NAME_BIRTH;

    /** MQTT topic name for will messages. */
    static const char* TOPIC_NAME_WILL;

    /** MQTT topic name for status messages. */
    static const char* TOPIC_NAME_STATUS;

    /** MQTT topic name for fusion pose. */
    static const char* TOPIC_NAME_FUSION_POSE;

    /** MQTT topic name for raw sensor data. */
    static const char* TOPIC_NAME_RAW_SENSORS;

    /** MQTT topic name for receiving Space Ship Radar pose. */
    static const char* TOPIC_NAME_RADAR_POSE;

    /** MQTT topic name for host time sync request. */
    static const char* TOPIC_NAME_HOST_TIMESYNC_REQ;

    /** MQTT topic name for host time sync response. */
    static const char* TOPIC_NAME_HOST_TIMESYNC_RSP;

    /**
     * @brief Flag for setting initial data through SMP.
     */
    bool m_initialDataSent;

    /**
     * @brief Timer for sending system status to Radon Ulzer.
     */
    SimpleTimer m_statusTimer;

    /**
     * @brief Timer for host time synchronization requests.
     */
    SimpleTimer m_hostTimeSyncTimer;

    /**
     * @brief SerialMux channel provider handler.
     */
    SerMuxChannelProvider m_serMuxChannelProvider;

    /**
     * @brief Time synchronization handler.
     */
    TimeSync m_timeSync;

    /**
     * @brief Line sensors handler.
     */
    LineSensors m_lineSensors;

    /**
     * @brief Motors handler.
     */
    Motors m_motors;

    /**
     * @brief State machine for the application.
     */
    StateMachine m_stateMachine;

    /**
     * @brief MQTT client handler.
     */
    MqttClient m_mqttClient;

    /**
     * @brief Extended Kalman Filter handler (5D state).
     */
    ExtendedKalmanFilter5D m_ekf;

    /**
     * @brief Last time the EKF was updated [ms] (local time base).
     */
    uint32_t m_lastEkfUpdateMs;

    /**
     * @brief Last received vehicle data (odometry + IMU) from Zumo.
     */
    VehicleData m_lastVehicleData;

    /**
     * @brief Flag indicating if at least one valid vehicle data has been received.
     */
    bool m_hasVehicleData;

    /**
     * @brief Last received Space Ship Radar pose.
     */
    SpaceShipRadarPose m_lastSsrPose;

    /**
     * @brief Flag indicating if at least one valid SSR pose has been received.
     */
    bool m_hasSsrPose;

    /**
     * @brief Flag indicating if the odometry origin in the global frame is initialized.
     */
    bool  m_odoOriginInitialized;

    /**
     * @brief Odometry origin X coordinate in mm in global (SSR) frame.
     */
    float m_odoOriginX_mm;

    /**
     * @brief Odometry origin Y coordinate in mm in global (SSR) frame.
     */
    float m_odoOriginY_mm;

    /**
     * @brief Flag indicating if the EKF has been initialized from SSR.
     */
    bool m_ekfInitializedFromSSR;

    /**
     * @brief Setup the MQTT connection.
     *
     * @param[in] clientId   The MQTT client id.
     * @param[in] brokerAddr The address of the MQTT broker.
     * @param[in] brokerPort The port of the MQTT broker.
     *
     * @return true if successful, otherwise false.
     */
    bool setupMqtt(const String& clientId, const String& brokerAddr, uint16_t brokerPort);

    /**
     * @brief Callback for receiving Space Ship Radar pose over MQTT topic.
     *
     * @param[in] payload The topic payload as JSON string.
     */
    void ssrTopicCallback(const String& payload);

    /**
     * @brief Callback for receiving host time sync response over MQTT topic.
     *
     * @param[in] payload The topic payload as JSON string.
     */
    void hostTimeSyncResponseCallback(const String& payload);

    /**
     * @brief Publish a combined snapshot of vehicle and line sensor data via MQTT.
     *
     * @param[in] data Vehicle data received via SerialMux.
     */
    void publishVehicleAndSensorSnapshot(const VehicleData& data);

    /**
     * @brief Run sensor fusion for vehicle data and Space Ship Radar pose.
     *
     * This function:
     * - selects the newest data source (vehicle or SSR),
     * - performs the EKF prediction step,
     * - applies the corresponding measurement update(s),
     * - and publishes the fused pose via MQTT.
     *
     * @param[in] data    Vehicle data received via SerialMux.
     * @param[in] ssrPose Latest Space Ship Radar pose (global frame).
     */
    void filterLocationData(const VehicleData& data, const SpaceShipRadarPose& ssrPose);

    /**
     * @brief Transform odometry from local Zumo frame into global (SSR) frame.
     *
     * The transformation:
     * - applies a configurable origin offset (initialized from SSR),
     * - flips the Y axis to match the SSR convention,
     * - flips the heading sign to match the SSR rotation direction.
     *
     * @param[in]  vehicleData     Raw odometry from Zumo.
     * @param[out] xGlob_mm        Global X position in mm.
     * @param[out] yGlob_mm        Global Y position in mm.
     * @param[out] thetaGlob_mrad  Global heading in mrad.
     */
    void transformOdometryToGlobal(const VehicleData& vehicleData,
                                   float&             xGlob_mm,
                                   float&             yGlob_mm,
                                   float&             thetaGlob_mrad) const;

    /**
     * @brief Publish the current EKF fusion pose via MQTT.
     *
     * @param[in] tsMs Timestamp in ms (local time base) associated with the EKF state.
     */
    void publishFusionPose(uint32_t tsMs);

    /**
     * @brief Copy construction of an instance (not allowed).
     *
     * @param[in] app Source instance.
     */
    App(const App& app);

    /**
     * @brief Assignment of an instance (not allowed).
     *
     * @param[in] app Source instance.
     *
     * @returns Reference to App instance.
     */
    App& operator=(const App& app);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* APP_H */
/** @} */
