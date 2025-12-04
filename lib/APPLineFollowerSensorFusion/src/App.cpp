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

/** Buffer size for JSON serialization of birth / will message. */
static const uint32_t JSON_BIRTHMESSAGE_MAX_SIZE = 64U;

/** Buffer size for JSON serialization of combined sensor snapshot. */
static const uint32_t JSON_SENSOR_SNAPSHOT_MAX_SIZE = 256U;

/** Buffer size for JSON serialization of fusion pose. */
static const uint32_t JSON_FUSION_POSE_MAX_SIZE = 128U;

/* Convenience aliases for EKF types. */
using ImuMeasVector = ExtendedKalmanFilter5D::ImuMeasVector;
using OdoMeasVector = ExtendedKalmanFilter5D::OdoMeasVector;
using CamMeasVector = ExtendedKalmanFilter5D::CamMeasVector;
using StateVector   = ExtendedKalmanFilter5D::StateVector;
using StateMatrix   = ExtendedKalmanFilter5D::StateMatrix;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

App::App() :
    m_initialDataSent(false),
    m_statusTimer(),
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
    m_odoOriginInitialized(false),
    m_odoOriginX_mm(0.0F),
    m_odoOriginY_mm(0.0F),
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
        else if (false == m_ekf.init())
        {
            LOG_FATAL("Extended Kalman Filter could not be initialized.");
        }
        else
        {
            /* Log incoming vehicle data and corresponding time sync information. */
            m_serMuxChannelProvider.registerVehicleDataCallback(
                [this](const VehicleData& data)
                {
                    publishVehicleAndSensorSnapshot(data);

                    m_lastVehicleData = data;
                    m_hasVehicleData  = true;

                    /* Run sensor fusion whenever new vehicle data arrives.
                     * SSR data may or may not be available yet; fusion will
                     * only start after SSR-based EKF initialization.
                     */
                    filterLocationData(data, m_lastSsrPose);
                });

            /* Start network time (NTP) against host and Zumo serial ping-pong. */
            m_timeSync.begin();
            m_statusTimer.start(1000U);
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

    /* Process time synchronization (NTP refresh + serial ping-pong). */
    m_timeSync.process();

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
        /* Subscribe to Space Ship Radar topic. */
        else if (false ==
                 m_mqttClient.subscribe(ssrTopic, false, [this](const String& payload) { ssrTopicCallback(payload); }))
        {
            LOG_FATAL("Could not subscribe to MQTT topic: %s.", TOPIC_NAME_RADAR_POSE);
        }
        else
        {
            isSuccessful = true;
            LOG_INFO("Subscribed to MQTT topic: %s.", ssrTopic.c_str());
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
        JsonVariantConst xPos_mm    = jsonPayload["positionX"];  /* int : in mm */
        JsonVariantConst yPos_mm    = jsonPayload["positionY"];  /* int : in mm */
        JsonVariantConst xVel_mms   = jsonPayload["speedX"];     /* int : in mm/s */
        JsonVariantConst yVel_mms   = jsonPayload["speedY"];     /* int : in mm/s */
        JsonVariantConst angle_mrad = jsonPayload["angle"];      /* int : in mrad */
        JsonVariantConst id         = jsonPayload["identifier"]; /* int : unique id of the target */

        const int x_mm_i     = xPos_mm.as<int>();
        const int y_mm_i     = yPos_mm.as<int>();
        const int vx_mms_i   = xVel_mms.as<int>();
        const int vy_mms_i   = yVel_mms.as<int>();
        const int ang_mrad_i = angle_mrad.as<int>();

        LOG_INFO("SSR pose: xPos=%dmm yPos=%dmm angle=%dmrad vx=%dmm/s vy=%dmm/s",
                 x_mm_i, y_mm_i, ang_mrad_i, vx_mms_i, vy_mms_i);

        /* 1) Copy pose into struct. */
        SpaceShipRadarPose ssrPose;
        ssrPose.timestamp = static_cast<uint32_t>(m_timeSync.localNowMs());
        ssrPose.x         = static_cast<float>(x_mm_i);
        ssrPose.y         = static_cast<float>(y_mm_i);
        ssrPose.theta     = static_cast<float>(ang_mrad_i);
        ssrPose.v_x       = static_cast<float>(vx_mms_i);
        ssrPose.v_y       = static_cast<float>(vy_mms_i);

        /* 2) Initialize odometry origin from first SSR pose. */
        if (false == m_odoOriginInitialized)
        {
            m_odoOriginX_mm        = ssrPose.x;
            m_odoOriginY_mm        = ssrPose.y;
            m_odoOriginInitialized = true;

            LOG_INFO("Odometry origin set from SSR: x=%dmm y=%dmm", x_mm_i, y_mm_i);
        }

        /* 3) Initialize EKF from first SSR pose. */
        if (false == m_ekfInitializedFromSSR)
        {
            StateVector x0;
            x0.setZero();

            /* Position and heading from SSR. */
            x0(0) = ssrPose.x;     /* p_x [mm] */
            x0(1) = ssrPose.y;     /* p_y [mm] */
            x0(2) = ssrPose.theta; /* theta [mrad] */

            /* Velocity magnitude from vx, vy. */
            const float v_mms = std::sqrt(ssrPose.v_x * ssrPose.v_x + ssrPose.v_y * ssrPose.v_y);
            x0(3)             = v_mms; /* v [mm/s] */
            x0(4)             = 0.0F;  /* omega [mrad/s], unknown -> 0 */

            StateMatrix P0 = StateMatrix::Identity();
            /* Position uncertainty ~ 50 mm. */
            P0(0,0) = 50.0F * 50.0F;
            P0(1,1) = 50.0F * 50.0F;
            /* Heading uncertainty ~ 200 mrad. */
            P0(2,2) = 200.0F * 200.0F;
            /* Velocity and omega moderately uncertain. */
            P0(3,3) = 200.0F * 200.0F;
            P0(4,4) = 200.0F * 200.0F;

            (void)m_ekf.init(x0, P0);
            m_lastEkfUpdateMs       = ssrPose.timestamp;
            m_ekfInitializedFromSSR = true;

            LOG_INFO("EKF initialized from SSR: x=%.1fmm y=%.1fmm theta=%.1fmrad v=%.1fmm/s",
                     x0(0), x0(1), x0(2), x0(3));
        }

        /* 4) Store last SSR pose. */
        m_lastSsrPose = ssrPose;
        m_hasSsrPose  = true;

        /* 5) Run fusion if we already have at least one vehicle data sample. */
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
    const uint64_t localNowMs    = m_timeSync.localNowMs();
    const uint64_t epochNowMs    = m_timeSync.nowEpochMs();
    const int64_t  offsetMs      = m_timeSync.getZumoToEspOffsetMs();
    const bool     zumoSynced    = m_timeSync.isZumoSynced();
    const bool     rtcSynced     = m_timeSync.isRtcSynced();

    (void)localNowMs;
    (void)offsetMs;
    (void)zumoSynced;
    (void)rtcSynced;

    /* Publish snapshot of vehicle + line sensor data. */
    const uint16_t* lineSensorValues = m_lineSensors.getSensorValues();
    JsonDocument    payloadJson;
    char            payloadArray[JSON_SENSOR_SNAPSHOT_MAX_SIZE];

    payloadJson["ts_epoch"]    = epochNowMs;
    payloadJson["ts_local_ms"] = mappedLocalMs;

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
    /* Do not run fusion until EKF has been initialized from SSR. */
    if (false == m_ekfInitializedFromSSR)
    {
        return;
    }

    const uint32_t zumoTs32 = static_cast<uint32_t>(vehicleData.timestamp);
    const uint32_t ssrTs32  = static_cast<uint32_t>(ssrPose.timestamp);

    /* If EKF has no valid timestamp yet, initialize from the first available one. */
    if (0U == m_lastEkfUpdateMs)
    {
        if (0U != zumoTs32)
        {
            m_lastEkfUpdateMs = zumoTs32;
        }
        else if (0U != ssrTs32)
        {
            m_lastEkfUpdateMs = ssrTs32;
        }
        else
        {
            return;
        }
    }

    /* Determine which sensor provides the newest information. */
    enum class Source
    {
        None,
        Vehicle,
        SSR
    };

    Source   newestSource = Source::None;
    uint32_t newestTs     = m_lastEkfUpdateMs;

    if (zumoTs32 > newestTs)
    {
        newestTs     = zumoTs32;
        newestSource = Source::Vehicle;
    }

    if (ssrTs32 > newestTs)
    {
        newestTs     = ssrTs32;
        newestSource = Source::SSR;
    }

    /* No newer data available -> nothing to do. */
    if (Source::None == newestSource)
    {
        return;
    }

    /* Time difference in seconds for prediction. */
    const uint32_t dtMs = newestTs - m_lastEkfUpdateMs;
    const float    dt   = static_cast<float>(dtMs) / 1000.0F;

    /* Longitudinal acceleration as input to the process model (mm/s^2). */
    const float a_x = static_cast<float>(vehicleData.accelerationX);

    /* EKF prediction step. */
    m_ekf.predict(a_x, dt);

    /* Measurement update depending on source. */
    if (Source::Vehicle == newestSource)
    {
        /* IMU update: only yaw rate (turnRateZ). */
        {
            ImuMeasVector z_imu;
            z_imu(0) = static_cast<float>(vehicleData.turnRateZ); /* [mrad/s] */
            m_ekf.updateImu(z_imu);
        }

        /* Odometry update in global frame.
         *
         * EKF odometryModel(x):
         *   z_odo(0) = p_x
         *   z_odo(1) = p_y
         *   z_odo(2) = v
         *   z_odo(3) = theta
         */
        {
            float xGlob_mm      = 0.0F;
            float yGlob_mm      = 0.0F;
            float thetaGlob_mrad = 0.0F;

            transformOdometryToGlobal(vehicleData, xGlob_mm, yGlob_mm, thetaGlob_mrad);

            OdoMeasVector z_odo;
            z_odo(0) = xGlob_mm;                               /* global p_x [mm] */
            z_odo(1) = yGlob_mm;                               /* global p_y [mm] */
            z_odo(2) = static_cast<float>(vehicleData.center); /* speed [mm/s] (v_center) */
            z_odo(3) = thetaGlob_mrad;                         /* global theta [mrad] */

            m_ekf.updateOdometry(z_odo);
        }
    }
    else if (Source::SSR == newestSource)
    {
        /* Camera / SpaceShipRadar update.
         *
         * EKF cameraModel(x):
         *   z_cam(0) = p_x
         *   z_cam(1) = p_y
         *   z_cam(2) = theta [mrad]
         *   z_cam(3) = v_x
         *   z_cam(4) = v_y
         */
        CamMeasVector z_cam;
        z_cam(0) = ssrPose.x;     /* p_x [mm] */
        z_cam(1) = ssrPose.y;     /* p_y [mm] */
        z_cam(2) = ssrPose.theta; /* theta [mrad] */
        z_cam(3) = ssrPose.v_x;   /* v_x [mm/s] */
        z_cam(4) = ssrPose.v_y;   /* v_y [mm/s] */

        m_ekf.updateCamera(z_cam);
    }

    /* Update timestamp of last EKF update. */
    m_lastEkfUpdateMs = newestTs;

    /* Publish fused pose. */
    publishFusionPose(newestTs);
}

void App::transformOdometryToGlobal(const VehicleData& vehicleData,
                                    float&             xGlob_mm,
                                    float&             yGlob_mm,
                                    float&             thetaGlob_mrad) const
{
    /* Y axis and heading sign differ between local odometry frame and SSR frame. */
    constexpr float Y_SIGN     = -1.0F;
    constexpr float THETA_SIGN = -1.0F;

    const float xLocal_mm       = static_cast<float>(vehicleData.xPos);
    const float yLocal_mm       = static_cast<float>(vehicleData.yPos);
    const float thetaLocal_mrad = static_cast<float>(vehicleData.orientation);

    const float originX = (true == m_odoOriginInitialized) ? m_odoOriginX_mm : 0.0F;
    const float originY = (true == m_odoOriginInitialized) ? m_odoOriginY_mm : 0.0F;

    xGlob_mm       = originX + xLocal_mm;
    yGlob_mm       = originY + (Y_SIGN * yLocal_mm);
    thetaGlob_mrad = THETA_SIGN * thetaLocal_mrad;
}

void App::publishFusionPose(uint32_t tsMs)
{
    /* EKF state: [p_x, p_y, theta, v, omega]. */
    const StateVector& state = m_ekf.getState();

    JsonDocument payloadJson;
    char         payloadArray[JSON_FUSION_POSE_MAX_SIZE];

    payloadJson["ts_ms"]        = static_cast<int64_t>(tsMs);
    payloadJson["x_mm"]         = static_cast<int32_t>(state(0));
    payloadJson["y_mm"]         = static_cast<int32_t>(state(1));
    payloadJson["theta_mrad"]   = static_cast<int32_t>(state(2));
    payloadJson["v_mms"]        = static_cast<int32_t>(state(3));
    payloadJson["omega_mradps"] = static_cast<int32_t>(state(4));

    (void)serializeJson(payloadJson, payloadArray);
    String payloadStr(payloadArray);

    if (false == m_mqttClient.publish(TOPIC_NAME_FUSION_POSE, true, payloadStr))
    {
        LOG_WARNING("Publishing fusion pose via MQTT failed.");
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
