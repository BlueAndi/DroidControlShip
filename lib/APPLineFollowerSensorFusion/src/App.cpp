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

/** MQTT topic name for host time sync request. */
const char* App::TOPIC_NAME_HOST_TIMESYNC_REQ = "zumo/time_sync/request";

/** MQTT topic name for host time sync response. */
const char* App::TOPIC_NAME_HOST_TIMESYNC_RSP = "zumo/time_sync/response";

/** Buffer size for JSON serialization of birth / will message. */
static const uint32_t JSON_BIRTHMESSAGE_MAX_SIZE = 64U;

/** Buffer size for JSON serialization of combined sensor snapshot. */
static const uint32_t JSON_SENSOR_SNAPSHOT_MAX_SIZE = 256U;

/** Buffer size for JSON serialization of fusion pose. */
static const uint32_t JSON_FUSION_POSE_MAX_SIZE = 128U;

/** Status send interval in ms. */
const uint16_t STATUS_SEND_INTERVAL_MS = 1000U;

/** Host time sync interval in ms. */
const uint16_t HOST_TIMESYNC_INTERVAL_MS = 10000U;

/* Convenience aliases for EKF types. */
using ImuMeasurementVector = ExtendedKalmanFilter5D::ImuMeasurementVector;
using OdoMeasurementVector = ExtendedKalmanFilter5D::OdoMeasurementVector;
using CamMeasurementVector = ExtendedKalmanFilter5D::CamMeasurementVector;
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
        else
        {
            /* Log incoming vehicle data and corresponding time sync information. */
            m_serMuxChannelProvider.registerVehicleDataCallback(
                [this](const VehicleData& data) { onVehicleData(data); });

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
#if defined(CONFIG_ENABLE_SIM_GPS)
    publishGps(m_mqttClient, m_timeSync.localNowMs());
#endif /* CONFIG_ENABLE_SIM_GPS */

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
    const String ssrTopic      = String(TOPIC_NAME_RADAR_POSE) + "/" + clientId;

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
        else if (false ==
                 m_mqttClient.subscribe(ssrTopic, false, [this](const String& payload) { ssrTopicCallback(payload); }))
        {
            LOG_FATAL("Could not subscribe to MQTT topic: %s.", TOPIC_NAME_RADAR_POSE);
        }
        else if (false ==
                 m_mqttClient.subscribe(TOPIC_NAME_HOST_TIMESYNC_RSP, true,
                                        [this](const String& payload) { hostTimeSyncResponseCallback(payload); }))
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
        JsonVariantConst xPos_mm       = jsonPayload["positionX"];   /* int : in mm */
        JsonVariantConst yPos_mm       = jsonPayload["positionY"];   /* int : in mm */
        JsonVariantConst xVel_mms      = jsonPayload["speedX"];      /* int : in mm/s */
        JsonVariantConst yVel_mms      = jsonPayload["speedY"];      /* int : in mm/s */
        JsonVariantConst angle_mrad    = jsonPayload["angle"];       /* int : in mrad */
        JsonVariantConst timestamp_ms  = jsonPayload["timestamp_ms"];/* int : host epoch in ms */

        const int      x_mm_i      = xPos_mm.as<int>();
        const int      y_mm_i      = yPos_mm.as<int>();
        const int      vx_mms_i    = xVel_mms.as<int>();
        const int      vy_mms_i    = yVel_mms.as<int>();
        const int      ang_mrad_i  = angle_mrad.as<int>();
        const uint64_t hostEpochMs = timestamp_ms.as<uint64_t>();

        const bool hostSynced = m_timeSync.isHostSynced();
        const uint64_t ssrLocalTsMs =
            hostSynced ? m_timeSync.hostToEspLocalMs(hostEpochMs)
                       : m_timeSync.localNowMs();  // fallback if no host sync yet

        LOG_INFO("SSR pose: ts_host_ms=%llu (hostSync=%s)",
                 hostEpochMs,
                 hostSynced ? "true" : "false");

        SpaceShipRadarPose ssrPose;
        ssrPose.x         = static_cast<float>(x_mm_i);
        ssrPose.y         = static_cast<float>(y_mm_i);
        ssrPose.theta     = static_cast<float>(ang_mrad_i);
        ssrPose.v_x       = static_cast<float>(vx_mms_i);
        ssrPose.v_y       = static_cast<float>(vy_mms_i);
        ssrPose.timestamp = static_cast<uint32_t>(ssrLocalTsMs);

        if (false == m_odoOriginInitialized)
        {
            m_odoOriginX_mm        = ssrPose.x;
            m_odoOriginY_mm        = ssrPose.y;
            m_odoOriginInitialized = true;

            LOG_INFO("Odometry origin set from SSR: x=%dmm y=%dmm", x_mm_i, y_mm_i);
        }

        if (false == m_ekfInitializedFromSSR)
        {
            StateVector x0;
            x0.setZero();

            x0(0) = ssrPose.x;
            x0(1) = ssrPose.y;
            x0(2) = ssrPose.theta;

            const float v_mms = std::sqrt(ssrPose.v_x * ssrPose.v_x + ssrPose.v_y * ssrPose.v_y);
            x0(3)             = v_mms;
            x0(4)             = 0.0F;

            StateMatrix P0 = StateMatrix::Identity();
            P0(0,0) = 50.0F * 50.0F;
            P0(1,1) = 50.0F * 50.0F;
            P0(2,2) = 200.0F * 200.0F;
            P0(3,3) = 200.0F * 200.0F;
            P0(4,4) = 200.0F * 200.0F;

            (void)m_ekf.init(x0, P0);
            m_lastEkfUpdateMs       = ssrPose.timestamp;
            m_ekfInitializedFromSSR = true;

            LOG_INFO("EKF initialized from SSR: x=%.1fmm y=%.1fmm theta=%.1fmrad v=%.1fmm/s",
                     x0(0), x0(1), x0(2), x0(3));
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

    payloadJson["ts_local_ms"]    = mappedLocalMs;
    payloadJson["zumo_sync_ok"]   = zumoSynced;
    payloadJson["host_sync_ok"]   = hostSynced;

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


void App::filterLocationData(const VehicleData& vehicleData,
                             const SpaceShipRadarPose& ssrPose)
{
    /* Local variables (all declared at top as requested). */
    uint32_t zumoTs32        = 0U;
    uint32_t zumoLocalMs32   = 0U;
    uint32_t ssrLocalMs32    = 0U;
    uint32_t newestLocalTs   = 0U;
    uint32_t dtMs            = 0U;
    float    dt              = 0.0F;
    float    a_x             = 0.0F;
    Source   newestSource    = Source::None;
    bool     ekfReady        = false;
    bool     hasTimestamp    = false;
    bool     doProcessing    = false;

    /* Do not run fusion until EKF has been initialized from SSR. */
    ekfReady = m_ekfInitializedFromSSR;

    if (ekfReady)
    {
        doProcessing = true;
    }

    if (doProcessing)
    {
        /* Timestamp conversion. */
        zumoTs32      = static_cast<uint32_t>(vehicleData.timestamp);
        zumoLocalMs32 = static_cast<uint32_t>(m_timeSync.mapZumoToLocalMs(zumoTs32));
        ssrLocalMs32  = static_cast<uint32_t>(ssrPose.timestamp);

        LOG_INFO("Filtering location data: Zumo=%u ms, SSR=%u ms",
                 zumoLocalMs32, ssrLocalMs32);

        /* Initialize EKF time on first data. */
        hasTimestamp = initializeEkfTimestamp(zumoLocalMs32, ssrLocalMs32);

        if (hasTimestamp)
        {
            newestLocalTs = m_lastEkfUpdateMs;

            /* Determine which sensor has the newest update. */
            newestSource = determineNewestSource(zumoLocalMs32,
                                                 ssrLocalMs32,
                                                 m_lastEkfUpdateMs,
                                                 newestLocalTs);

            if (newestSource != Source::None)
            {
                /* Time delta for prediction step. */
                dtMs = newestLocalTs - m_lastEkfUpdateMs;
                dt   = static_cast<float>(dtMs) / 1000.0F;

                /* Longitudinal acceleration input. */
                a_x = static_cast<float>(vehicleData.accelerationX);

                /* EKF prediction. */
                m_ekf.predict(a_x, dt);

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
        }
    }
}


void App::transformOdometryToGlobal(const VehicleData& vehicleData,
                                    float&             xGlob_mm,
                                    float&             yGlob_mm,
                                    float&             thetaGlob_mrad) const
{
    /* Y axis and heading sign differ between local odometry frame and SSR frame. */
    constexpr float Y_SIGN     = -1.0F;
    constexpr float THETA_SIGN = 1.0F;

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

void App::hostTimeSyncResponseCallback(const String& payload)
{
    uint64_t t4_ts = millis();
    JsonDocument         doc;
    DeserializationError err = deserializeJson(doc, payload);

    if (err != DeserializationError::Ok)
    {
        LOG_WARNING("HostTimeSync: JSON parse failed: %d", err.code());
        return;
    }


    if ( (doc["seq"].is<uint32_t>() == false) ||
        (doc["t1_esp_ms"].is<uint64_t>() == false) ||
        (doc["t2_host_ms"].is<uint64_t>() == false) ||
        (doc["t3_host_ms"].is<uint64_t>() == false) )
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

#if defined(CONFIG_ENABLE_SIM_GPS)
void App::publishGps(MqttClient& mqttClient, uint32_t tsMs)
{
    IGps* gps = Board::getInstance().getGps();

    if ((nullptr == gps) || (false == mqttClient.isConnected()))
    {
        if (nullptr == gps)
        {
            LOG_DEBUG("No GPS available, skipping GPS publish.");
        }
        
        return;
    }

    int32_t xPosMm      = 0;
    int32_t yPosMm      = 0;
    int32_t headingMrad = 0;

    if (!gps->getPosition(xPosMm, yPosMm))
    {
        LOG_DEBUG("No GPS position available, skipping GPS publish.");
        return;
    }

    bool hasOrientation = gps->getOrientation(headingMrad);
    
    static bool     havePrev = false;
    static int32_t  prevX    = 0;
    static int32_t  prevY    = 0;
    static uint32_t prevTs   = 0;

    float speedMmPs = 0.0F;

    if (havePrev)
    {
        uint32_t dtMs = tsMs - prevTs;

        if (dtMs > 0U)
        {
            float dx = static_cast<float>(xPosMm - prevX);
            float dy = static_cast<float>(yPosMm - prevY);
            float dist = sqrtf(dx * dx + dy * dy);

            speedMmPs = dist * 1000.0F / static_cast<float>(dtMs);
        }
    }

    prevX    = xPosMm;
    prevY    = yPosMm;
    prevTs   = tsMs;
    havePrev = true;
    LOG_INFO("GPS: x=%dmm y=%dmm heading=%dmrad speed=%.1fmm/s",
             xPosMm, yPosMm,
             hasOrientation ? headingMrad : 0,
             speedMmPs);


    JsonDocument payloadJson;
    char         payloadArray[JSON_FUSION_POSE_MAX_SIZE];

    payloadJson["ts_ms"]            = static_cast<int64_t>(tsMs);
    payloadJson["x_mm"]             = xPosMm;
    payloadJson["y_mm"]             = yPosMm;
    payloadJson["orientation_mrad"] = hasOrientation ? headingMrad : 0;
    payloadJson["speed_mms"]        = static_cast<float>(speedMmPs);

    (void)serializeJson(payloadJson, payloadArray);
    String payloadStr(payloadArray);

    if (false == mqttClient.publish("zumo/gps", true, payloadStr))
    {
        LOG_WARNING("Publishing GPS data via MQTT failed.");
    }
}
#endif



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

Source App::determineNewestSource(uint32_t zumoLocalMs32,
                                  uint32_t ssrLocalMs32,
                                  uint32_t lastEkfUpdateMs,
                                  uint32_t& newestLocalTs) const
{
    Source source = Source::None;
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
    const int16_t rawGyroZ = static_cast<int16_t>(vehicleData.turnRateZ);
    float xGlob_mm = 0.0F;
    float yGlob_mm = 0.0F;
    float thetaGlob_mrad = 0.0F;
    OdoMeasurementVector z_odo;

    LOG_INFO("EKF update from Vehicle.");

    /* IMU update using yaw-rate only. */
    m_ekf.updateImuFromDigits(rawGyroZ);

    /* Convert odometry into global coordinates. */
    transformOdometryToGlobal(vehicleData, xGlob_mm, yGlob_mm, thetaGlob_mrad);

    /* Update odometry measurement:
     * z_odo(0) = v
     * z_odo(1) = theta
     */
    z_odo(0) = static_cast<float>(vehicleData.center);
    z_odo(1) = thetaGlob_mrad;

    m_ekf.updateOdometry(z_odo);
}

void App::updateFromSsr(const SpaceShipRadarPose& ssrPose)
{
    CamMeasurementVector z_cam;

    LOG_INFO("EKF update from SSR.");

    /* Camera measurement:
     * z_cam(0..4) = [x, y, theta, v_x, v_y]
     */
    z_cam(0) = ssrPose.x;
    z_cam(1) = ssrPose.y;
    z_cam(2) = ssrPose.theta;
    z_cam(3) = ssrPose.v_x;
    z_cam(4) = ssrPose.v_y;

    m_ekf.updateCamera(z_cam);
}



/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
