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
 * @brief  Line follower Sensor Fusion application
 * @author Tobias Haeckel <tobias.haeckel@gmx.net>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "App.h"
#include <cmath>
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <Util.h>
#include <SettingsHandler.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "RemoteControl.h"
#include "States/StartupState.h"
#include "States/LineSensorsCalibrationState.h"
#include "States/ErrorState.h"
#include "States/ReadyState.h"
#include "States/DrivingState.h"

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

#ifndef CONFIG_LOG_SEVERITY
#define CONFIG_LOG_SEVERITY (Logging::LOG_LEVEL_INFO)
#endif /* CONFIG_LOG_SEVERITY */

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Serial interface baudrate. */
static const uint32_t SERIAL_BAUDRATE = 115200U;

/** Serial log sink. */
static LogSinkPrinter gLogSinkSerial("Serial", &Serial);

/* MQTT topic name for birth messages. */
const char* App::TOPIC_NAME_BIRTH = "birth";

/* MQTT topic name for will messages. */
const char* App::TOPIC_NAME_WILL = "will";

/** MQTT topic name for status messages. */
const char* App::TOPIC_NAME_STATUS = "zumo/status";

/** MQTT topic name for fusion pose. */
const char* App::TOPIC_NAME_FUSION_POSE = "zumo/fusion";

/** MQTT topic name for raw sensor data. */
const char* App::TOPIC_NAME_RAW_SENSORS = "zumo/sensors";

/** MQTT topic name for receiving Space Ship Radar pose. */
const char* App::TOPIC_NAME_RADAR_POSE = "ssr";

/** MQTT topic name for host time sync request. */
const char* App::TOPIC_NAME_HOST_TIMESYNC_REQ = "zumo/time_sync/request";

/** MQTT topic name for host time sync response. */
const char* App::TOPIC_NAME_HOST_TIMESYNC_RSP = "zumo/time_sync/response";

/** Buffer size for JSON serialization of birth / will message. */
static const uint32_t JSON_BIRTHMESSAGE_MAX_SIZE = 64U;

/** Buffer size for JSON serialization of combined sensor snapshot. */
static const uint32_t JSON_SENSOR_SNAPSHOT_MAX_SIZE = 256U;

/** Buffer size for JSON serialization of fusion pose including nested NIS fields. */
static const uint32_t JSON_FUSION_POSE_MAX_SIZE = 320U;

/** Status send interval in ms. */
const uint16_t STATUS_SEND_INTERVAL_MS = 1000U;

/** Host time sync interval in ms. */
const uint16_t HOST_TIMESYNC_INTERVAL_MS = 10000U;

/* Convenience aliases for EKF types. */
using OdoMeasurementVector = ExtendedKalmanFilter4D::OdoMeasurementVector;
using CamMeasurementVector = ExtendedKalmanFilter4D::CamMeasurementVector;
using StateVector          = ExtendedKalmanFilter4D::StateVector;
using StateMatrix          = ExtendedKalmanFilter4D::StateMatrix;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

App::App() :
    m_initialDataSent(false),
    m_statusTimer(),
    m_hostTimeSyncTimer(),
    m_serMuxChannelProvider(Board::getInstance().getRobot().getStream()),
    m_timeSync(m_serMuxChannelProvider),
    m_lineSensors(m_serMuxChannelProvider),
    m_motors(m_serMuxChannelProvider),
    m_stateMachine(),
    m_mqttClient(),
    m_ekf(),
    m_lastEkfUpdateMs(0U),
    m_lastVehicleData{},
    m_hasVehicleData(false),
    m_lastSsrPose{},
    m_hasSsrPose(false),
    m_ekfInitializedFromSSR(false)
{
    /* Inject dependencies into states. */
    StartupState::getInstance().injectDependencies(m_serMuxChannelProvider, m_motors);
    LineSensorsCalibrationState::getInstance().injectDependencies(m_serMuxChannelProvider);
    ReadyState::getInstance().injectDependencies(m_lineSensors);
    DrivingState::getInstance().injectDependencies(m_lineSensors, m_motors);
}

void App::setup()
{
    bool             isSuccessful = false;
    SettingsHandler& settings     = SettingsHandler::getInstance();
    Board&           board        = Board::getInstance();

    Serial.begin(SERIAL_BAUDRATE);

    /* Register serial log sink and select it per default. */
    if (true == Logging::getInstance().registerSink(&gLogSinkSerial))
    {
        (void)Logging::getInstance().selectSink("Serial");

        /* Set severity of logging system. */
        Logging::getInstance().setLogLevel(CONFIG_LOG_SEVERITY);
    }

    /* Initialize HAL. */
    if (false == board.init())
    {
        LOG_FATAL("HAL init failed.");
    }
    /* Settings shall be loaded from configuration file. */
    else if (false == settings.loadConfigurationFile(board.getConfigFilePath()))
    {
        LOG_FATAL("Settings could not be loaded from %s.", board.getConfigFilePath());
    }
    else
    {
        NetworkSettings networkSettings = {settings.getWiFiSSID(), settings.getWiFiPassword(), settings.getRobotName(),
                                           ""};

        /* If the robot name is empty, use the WiFi MAC address as robot name. */
        if (true == settings.getRobotName().isEmpty())
        {
            String robotName = WiFi.macAddress();

            /* Remove MAC separators from robot name. */
            robotName.replace(":", "");

            settings.setRobotName(robotName);
        }

        if (false == board.getNetwork().setConfig(networkSettings))
        {
            LOG_FATAL("Network configuration could not be set.");
        }
        else if (false == m_serMuxChannelProvider.init())
        {
            LOG_FATAL("SerialMuxChannelProvider init failed.");
        }
        else if (false == setupMqtt(settings.getRobotName(), settings.getMqttBrokerAddress(), settings.getMqttPort()))
        {
            LOG_FATAL("MQTT connection could not be setup.");
        }
        else
        {
            /* Log incoming vehicle data and corresponding time sync information. */
            m_serMuxChannelProvider.registerVehicleDataCallback([this](const VehicleData& data)
                                                                { onVehicleData(data); });

            /* Start network time (NTP) against host and Zumo serial ping-pong. */
            m_ekf.init();
            m_timeSync.begin();
            m_statusTimer.start(STATUS_SEND_INTERVAL_MS);
            m_hostTimeSyncTimer.start(HOST_TIMESYNC_INTERVAL_MS);
            isSuccessful = true;
        }
    }

    if (false == isSuccessful)
    {
        LOG_FATAL("Initialization failed.");
        m_stateMachine.setState(ErrorState::getInstance());
    }
    else
    {
        LOG_INFO("Line follower Sensor Fusion application is ready.");

        /* Set initial state of the state machine. */
        m_stateMachine.setState(StartupState::getInstance());
    }
}

void App::loop()
{
    /* Process battery, device and network. */
    Board::getInstance().process();

    /* Process MQTT communication. */
    m_mqttClient.process();

    /* Process serial multiplexer. */
    m_serMuxChannelProvider.process();

    /* Process time synchronization (serial ping-pong). */
    m_timeSync.process();

    if (m_hostTimeSyncTimer.isTimeout())
    {
        m_timeSync.sendHostTimeSyncRequest(m_mqttClient, TOPIC_NAME_HOST_TIMESYNC_REQ);
        m_hostTimeSyncTimer.restart();
    }

    /* Process state machine. */
    m_stateMachine.process();

    /* Send heartbeat status to Radon Ulzer controller periodically. */
    if ((true == m_statusTimer.isTimeout()) && (true == m_serMuxChannelProvider.isInSync()))
    {
        Status dcsStatus = {SMPChannelPayload::Status::STATUS_FLAG_OK};

        if (&ErrorState::getInstance() == m_stateMachine.getState())
        {
            dcsStatus.status = SMPChannelPayload::Status::STATUS_FLAG_ERROR;
        }

        if (false == m_serMuxChannelProvider.sendStatus(dcsStatus))
        {
            LOG_WARNING("Failed to send status to RU.");
        }
        m_statusTimer.restart();
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool App::setupMqtt(const String& clientId, const String& brokerAddr, uint16_t brokerPort)
{
    bool         isSuccessful = false;
    JsonDocument jsonBirthDoc;
    char         birthMsgArray[JSON_BIRTHMESSAGE_MAX_SIZE];
    String       birthMessage;
    const String ssrTopic = String(TOPIC_NAME_RADAR_POSE) + "/" + clientId;

    jsonBirthDoc["name"] = clientId.c_str();
    (void)serializeJson(jsonBirthDoc, birthMsgArray);
    birthMessage = birthMsgArray;

    if (false == m_mqttClient.init())
    {
        LOG_FATAL("Failed to initialize MQTT client.");
    }
    else
    {
        MqttSettings mqttSettings = {clientId,     brokerAddr,      brokerPort,   TOPIC_NAME_BIRTH,
                                     birthMessage, TOPIC_NAME_WILL, birthMessage, true};

        if (false == m_mqttClient.setConfig(mqttSettings))
        {
            LOG_FATAL("MQTT configuration could not be set.");
        }
        else if (false == m_mqttClient.subscribe(ssrTopic, false, [this](const String& payload)
                                                 { ssrTopicCallback(payload); }))
        {
            LOG_FATAL("Could not subscribe to MQTT topic: %s.", TOPIC_NAME_RADAR_POSE);
        }
        else if (false == m_mqttClient.subscribe(TOPIC_NAME_HOST_TIMESYNC_RSP, true, [this](const String& payload)
                                                 { hostTimeSyncResponseCallback(payload); }))
        {
            LOG_FATAL("Could not subscribe to MQTT topic: %s.", TOPIC_NAME_HOST_TIMESYNC_RSP);
        }
        else
        {
            isSuccessful = true;
            LOG_INFO("Subscribed to MQTT topic: %s.", ssrTopic.c_str());
            LOG_INFO("Subscribed to MQTT topic: %s.", TOPIC_NAME_HOST_TIMESYNC_RSP);
        }
    }

    LOG_INFO("MQTT setup %s.", (true == isSuccessful) ? "successful" : "failed");
    return isSuccessful;
}

void App::ssrTopicCallback(const String& payload)
{
    JsonDocument         jsonPayload;
    DeserializationError error = deserializeJson(jsonPayload, payload.c_str());

    if (error != DeserializationError::Ok)
    {
        LOG_ERROR("JSON deserialization error %d.", error);
    }
    else
    {
        JsonVariantConst xPos_mm      = jsonPayload["positionX"];    /* int : in mm */
        JsonVariantConst yPos_mm      = jsonPayload["positionY"];    /* int : in mm */
        JsonVariantConst xVel_mms     = jsonPayload["speedX"];       /* int : in mm/s */
        JsonVariantConst yVel_mms     = jsonPayload["speedY"];       /* int : in mm/s */
        JsonVariantConst angle_mrad   = jsonPayload["angle"];        /* int : in mrad */
        JsonVariantConst timestamp_ms = jsonPayload["timestamp_ms"]; /* int : host epoch in ms */

        const int      x_mm_i      = xPos_mm.as<int>();
        const int      y_mm_i      = yPos_mm.as<int>();
        const int      vx_mms_i    = xVel_mms.as<int>();
        const int      vy_mms_i    = yVel_mms.as<int>();
        const int      ang_mrad_i  = angle_mrad.as<int>();
        const uint64_t hostEpochMs = timestamp_ms.as<uint64_t>();

        const bool     hostSynced   = m_timeSync.isHostSynced();
        const uint64_t ssrLocalTsMs = hostSynced ? m_timeSync.hostToEspLocalMs(hostEpochMs)
                                                 : m_timeSync.localNowMs();

        LOG_INFO("SSR pose: ts_host_ms=%llu (hostSync=%s)", hostEpochMs, hostSynced ? "true" : "false");

        SpaceShipRadarPose ssrPose;
        ssrPose.x         = static_cast<float>(x_mm_i);
        ssrPose.y         = static_cast<float>(y_mm_i);
        ssrPose.theta     = static_cast<float>(ang_mrad_i);
        ssrPose.timestamp = static_cast<uint32_t>(ssrLocalTsMs);

        if (false == m_ekfInitializedFromSSR)
        {
            const float vxMms = static_cast<float>(vx_mms_i);
            const float vyMms = static_cast<float>(vy_mms_i);
            StateVector x0;
            x0.setZero();

            x0(0) = ssrPose.x;
            x0(1) = ssrPose.y;
            x0(2) = ssrPose.theta;
            x0(3) = std::sqrt((vxMms * vxMms) + (vyMms * vyMms));

            StateMatrix P0 = StateMatrix::Identity();
            P0(0, 0)       = 50.0F * 50.0F;
            P0(1, 1)       = 50.0F * 50.0F;
            P0(2, 2)       = 200.0F * 200.0F;
            P0(3, 3)       = 200.0F * 200.0F;

            m_ekf.init(x0, P0);
            m_lastEkfUpdateMs       = ssrPose.timestamp;
            m_ekfInitializedFromSSR = true;

            LOG_INFO("EKF initialized from SSR: x=%.1fmm y=%.1fmm theta=%.1fmrad v=%.1fmm/s", x0(0), x0(1), x0(2),
                     x0(3));
        }

        m_lastSsrPose = ssrPose;
        m_hasSsrPose  = true;

        if (true == m_hasVehicleData)
        {
            filterLocationData(m_lastVehicleData, m_lastSsrPose);
        }
    }
}

void App::publishVehicleAndSensorSnapshot(const VehicleData& data)
{
    const uint32_t zumoTs32      = static_cast<uint32_t>(data.timestamp);
    const uint64_t mappedLocalMs = m_timeSync.mapZumoToLocalMs(zumoTs32);
    const int64_t  offsetMs      = m_timeSync.getZumoToEspOffsetMs();
    const bool     zumoSynced    = m_timeSync.isZumoSynced();
    const bool     hostSynced    = m_timeSync.isHostSynced();

    (void)offsetMs; /* keep for future debug if not used directly */

    const uint16_t* lineSensorValues = m_lineSensors.getSensorValues();
    JsonDocument    payloadJson;
    char            payloadArray[JSON_SENSOR_SNAPSHOT_MAX_SIZE];

    payloadJson["ts_local_ms"]  = mappedLocalMs;
    payloadJson["zumo_sync_ok"] = zumoSynced;
    payloadJson["host_sync_ok"] = hostSynced;

    JsonObject vehicleObj          = payloadJson["vehicle"].to<JsonObject>();
    vehicleObj["ts_zumo_ms"]       = static_cast<int64_t>(data.timestamp);
    vehicleObj["x_mm"]             = static_cast<int32_t>(data.xPos);
    vehicleObj["y_mm"]             = static_cast<int32_t>(data.yPos);
    vehicleObj["orientation_mrad"] = static_cast<int32_t>(data.orientation);
    vehicleObj["vL_mms"]           = static_cast<int32_t>(data.left);
    vehicleObj["vR_mms"]           = static_cast<int32_t>(data.right);
    vehicleObj["vC_mms"]           = static_cast<int32_t>(data.center);
    vehicleObj["accelX_digit"]     = static_cast<int16_t>(data.accelerationX);
    vehicleObj["turnRateZ_digit"]  = static_cast<int16_t>(data.turnRateZ);

    JsonObject lineObj = payloadJson["line"].to<JsonObject>();
    JsonArray  values  = lineObj["values"].to<JsonArray>();
    for (uint32_t idx = 0U; idx < LineSensors::getNumLineSensors(); ++idx)
    {
        values.add(static_cast<int32_t>(lineSensorValues[idx]));
    }

    (void)serializeJson(payloadJson, payloadArray);
    String payloadStr(payloadArray);
    if (false == m_mqttClient.publish(TOPIC_NAME_RAW_SENSORS, true, payloadStr))
    {
        LOG_WARNING("Publishing vehicle + sensor snapshot via MQTT failed.");
    }
}

void App::filterLocationData(const VehicleData& vehicleData, const SpaceShipRadarPose& ssrPose)
{
    uint32_t zumoTs32      = 0U;
    uint32_t zumoLocalMs32 = 0U;
    uint32_t ssrLocalMs32  = 0U;
    uint32_t newestLocalTs = 0U;
    uint32_t dtMs          = 0U;
    float    dt            = 0.0F;
    Source   newestSource  = Source::None;
    bool     hasTimestamp  = false;

    /* Do not run fusion until EKF has been initialized from SSR. */
    if (false == m_ekfInitializedFromSSR)
    {
        return;
    }

    /* Timestamp conversion. */
    zumoTs32      = static_cast<uint32_t>(vehicleData.timestamp);
    zumoLocalMs32 = static_cast<uint32_t>(m_timeSync.mapZumoToLocalMs(zumoTs32));
    ssrLocalMs32  = static_cast<uint32_t>(ssrPose.timestamp);

    LOG_INFO("Filtering location data: Zumo=%u ms, SSR=%u ms", zumoLocalMs32, ssrLocalMs32);

    /* Initialize EKF time on first data. */
    hasTimestamp = initializeEkfTimestamp(zumoLocalMs32, ssrLocalMs32);

    if (false == hasTimestamp)
    {
        return;
    }

    newestLocalTs = m_lastEkfUpdateMs;

    /* Determine which sensor has the newest update. */
    newestSource = determineNewestSource(zumoLocalMs32, ssrLocalMs32, m_lastEkfUpdateMs, newestLocalTs);

    if (newestSource == Source::None)
    {
        return;
    }

    /* Time delta for prediction step. */
    dtMs = newestLocalTs - m_lastEkfUpdateMs;
    dt   = static_cast<float>(dtMs) / 1000.0F;

    /* Use the latest available gyro yaw rate as control input for the process model. */
    const float omegaMradPerSec =
        ExtendedKalmanFilter4D::gyroDigitsToMradPerSec(static_cast<int16_t>(vehicleData.turnRateZ));

    /* EKF prediction. */
    m_ekf.predict(omegaMradPerSec, dt);

    /* EKF correction step based on sensor source. */
    if (newestSource == Source::Vehicle)
    {
        updateFromVehicle(vehicleData);
    }
    else if (newestSource == Source::SSR)
    {
        updateFromSsr(ssrPose);
    }
    else if (newestSource == Source::VehicleAndSSR)
    {
        updateFromVehicle(vehicleData);
        updateFromSsr(ssrPose);
    }

    /* Update last EKF timestamp. */
    m_lastEkfUpdateMs = newestLocalTs;

    /* Publish fused pose. */
    publishFusionPose(newestLocalTs);
}

void App::publishFusionPose(uint32_t tsMs)
{
    const StateVector&                          state       = m_ekf.getState();
    const ExtendedKalmanFilter4D::NisData&     odometryNis = m_ekf.getLastOdometryNis();
    const ExtendedKalmanFilter4D::NisData&     cameraNis   = m_ekf.getLastCameraNis();
    const float                                 omegaMradPerSec =
        (true == m_hasVehicleData)
            ? ExtendedKalmanFilter4D::gyroDigitsToMradPerSec(static_cast<int16_t>(m_lastVehicleData.turnRateZ))
            : 0.0F;

    JsonDocument payloadJson;
    char         payloadArray[JSON_FUSION_POSE_MAX_SIZE];

    payloadJson["ts_ms"]      = static_cast<int64_t>(tsMs);
    payloadJson["x_mm"]       = static_cast<int32_t>(state(0));
    payloadJson["y_mm"]       = static_cast<int32_t>(state(1));
    payloadJson["theta_mrad"] = static_cast<int32_t>(state(2));
    payloadJson["v_mms"]      = static_cast<int32_t>(state(3));

    /* Keep the latest gyro control input in the payload for existing consumers. */
    payloadJson["omega_mradps"] = static_cast<int32_t>(omegaMradPerSec);

    JsonObject nisObj = payloadJson["nis"].to<JsonObject>();

    JsonObject camObj = nisObj["cam"].to<JsonObject>();
    camObj["sensor"]  = "cam";
    camObj["valid"]   = cameraNis.isValid;
    if (true == cameraNis.isValid)
    {
        camObj["value"] = cameraNis.value;
        camObj["ts_ms"] = static_cast<int64_t>(cameraNis.timestampMs);
    }

    JsonObject odoObj = nisObj["odo"].to<JsonObject>();
    odoObj["sensor"]  = "odo";
    odoObj["valid"]   = odometryNis.isValid;
    if (true == odometryNis.isValid)
    {
        odoObj["value"] = odometryNis.value;
        odoObj["ts_ms"] = static_cast<int64_t>(odometryNis.timestampMs);
    }

    (void)serializeJson(payloadJson, payloadArray);
    String payloadStr(payloadArray);

    if (false == m_mqttClient.publish(TOPIC_NAME_FUSION_POSE, true, payloadStr))
    {
        LOG_WARNING("Publishing fusion pose via MQTT failed.");
    }
}

void App::hostTimeSyncResponseCallback(const String& payload)
{
    uint64_t             t4_ts = millis();
    JsonDocument         doc;
    DeserializationError err = deserializeJson(doc, payload);

    if (err != DeserializationError::Ok)
    {
        LOG_WARNING("HostTimeSync: JSON parse failed: %d", err.code());
        return;
    }

    if ((doc["seq"].is<uint32_t>() == false) || (doc["t1_esp_ms"].is<uint64_t>() == false) ||
        (doc["t2_host_ms"].is<uint64_t>() == false) || (doc["t3_host_ms"].is<uint64_t>() == false))
    {
        LOG_WARNING("HostTimeSync: Missing required fields in response JSON.");
        return;
    }

    uint32_t seq      = doc["seq"].as<uint32_t>();
    uint64_t t1EspMs  = doc["t1_esp_ms"].as<uint64_t>();
    uint64_t t2HostMs = doc["t2_host_ms"].as<uint64_t>();
    uint64_t t3HostMs = doc["t3_host_ms"].as<uint64_t>();

    m_timeSync.onHostTimeSyncResponse(seq, t1EspMs, t2HostMs, t3HostMs, t4_ts);
}

void App::onVehicleData(const VehicleData& data)
{
    publishVehicleAndSensorSnapshot(data);

    m_lastVehicleData = data;
    m_hasVehicleData  = true;

    /* Run sensor fusion whenever new vehicle data arrives.
     * SSR data may or may not be available yet; fusion will
     * only start after SSR-based EKF initialization.
     */
    filterLocationData(data, m_lastSsrPose);
}

bool App::initializeEkfTimestamp(uint32_t zumoLocalMs32, uint32_t ssrLocalMs32)
{
    bool initialized = true;

    /* If EKF has no reference timestamp yet, initialize from first valid source. */
    if (m_lastEkfUpdateMs == 0U)
    {
        if (zumoLocalMs32 != 0U)
        {
            m_lastEkfUpdateMs = zumoLocalMs32;
        }
        else if (ssrLocalMs32 != 0U)
        {
            m_lastEkfUpdateMs = ssrLocalMs32;
        }
        else
        {
            /* No valid timestamp available. */
            initialized = false;
        }
    }

    return initialized;
}

Source App::determineNewestSource(uint32_t zumoLocalMs32, uint32_t ssrLocalMs32, uint32_t lastEkfUpdateMs,
                                  uint32_t& newestLocalTs) const
{
    Source   source      = Source::None;
    uint32_t candidateTs = lastEkfUpdateMs;

    const bool hasVehicle = (zumoLocalMs32 > lastEkfUpdateMs);
    const bool hasSSR     = (ssrLocalMs32 > lastEkfUpdateMs);

    if (hasVehicle && hasSSR)
    {
        source      = Source::VehicleAndSSR;
        candidateTs = (ssrLocalMs32 > zumoLocalMs32) ? ssrLocalMs32 : zumoLocalMs32;
    }
    else if (hasVehicle)
    {
        source      = Source::Vehicle;
        candidateTs = zumoLocalMs32;
    }
    else if (hasSSR)
    {
        source      = Source::SSR;
        candidateTs = ssrLocalMs32;
    }

    newestLocalTs = candidateTs;
    return source;
}

void App::updateFromVehicle(const VehicleData& vehicleData)
{
    const uint32_t         zumoTs32      = static_cast<uint32_t>(vehicleData.timestamp);
    const uint32_t         zumoLocalMs32 = static_cast<uint32_t>(m_timeSync.mapZumoToLocalMs(zumoTs32));
    OdoMeasurementVector   z_odo;

    LOG_INFO("EKF update from Vehicle.");

    /* Odometry measurement: z_odo(0) = v */
    z_odo(0) = static_cast<float>(vehicleData.center);

    m_ekf.updateOdometry(z_odo, zumoLocalMs32);
}

void App::updateFromSsr(const SpaceShipRadarPose& ssrPose)
{
    CamMeasurementVector z_cam;

    LOG_INFO("EKF update from SSR.");

    /* Camera measurement: z_cam(0..2) = [x, y, theta] */
    z_cam(0) = ssrPose.x;
    z_cam(1) = ssrPose.y;
    z_cam(2) = ssrPose.theta;

    m_ekf.updateCamera(z_cam, ssrPose.timestamp);
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
