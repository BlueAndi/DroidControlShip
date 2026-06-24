/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
 * @file
 * @brief  SLAM application
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "App.h"
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <Util.h>
#include <SettingsHandler.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/** Configuration of the logging severity if not previously defined. */
#ifndef CONFIG_LOG_SEVERITY
#define CONFIG_LOG_SEVERITY (Logging::LOG_LEVEL_INFO)
#endif /* CONFIG_LOG_SEVERITY */

/** TCP server IP address if not defined by the build configuration. */
#define APPSLAM_TCP_SERVER_IP "127.0.0.1"

/** TCP server port if not defined by the build configuration. */
#define APPSLAM_TCP_SERVER_PORT 8888U

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

static void App_cmdRspChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData);
static void App_lineSensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData);
static void App_currentVehicleChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Serial interface baudrate. */
static const uint32_t SERIAL_BAUDRATE = 115200U;

/** Serial log sink */
static LogSinkPrinter gLogSinkSerial("Serial", &Serial);

/** TCP reconnect interval in milliseconds. */
static const uint32_t TCP_RECONNECT_INTERVAL = 2000U;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void App::setup()
{
    bool             isSuccessful = false;
    SettingsHandler& settings     = SettingsHandler::getInstance();
    Board&           board        = Board::getInstance();

    Serial.begin(SERIAL_BAUDRATE);

    /* Register serial log sink and select it per default. */
    if (true == Logging::getInstance().registerSink(&gLogSinkSerial))
    {
        (void)Logging::getInstance().selectSink(gLogSinkSerial.getName());

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

        /* If the robot name is empty, use the wifi MAC address as robot name. */
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
        else if (false == setupSerialMuxProtServer())
        {
            LOG_FATAL("SerialMuxProt server could not be setup.");
        }
        else
        {
            (void)connectToServer();
            m_statusTimer.start(1000U);
            isSuccessful = true;
        }
    }

    if (false == isSuccessful)
    {
        LOG_FATAL("Initialization failed.");
        fatalErrorHandler();
    }
}

void App::loop()
{
    if (false == m_isFatalError)
    {
        /* Process Battery, Device and Network. */
        Board::getInstance().process();

        /* Handle TCP communication. */
        if (true == m_tcpClient.connected())
        {
            receivePackets();
        }
        else
        {
            uint32_t now = millis();

            if ((now - m_lastReconnectAttempt) >= TCP_RECONNECT_INTERVAL)
            {
                m_lastReconnectAttempt = now;

                LOG_INFO("TCP server disconnected.");
                m_tcpClient.stop();

                if (false == connectToServer())
                {
                    LOG_WARNING("TCP reconnect attempt failed.");
                }
            }
        }

        /* Process SerialMuxProt. */
        m_smpServer.process(millis());

        if ((false == m_initialDataSent) && (true == m_smpServer.isSynced()))
        {
            SettingsHandler& settings = SettingsHandler::getInstance();
            Command          cmd      = {SMPChannelPayload::CMD_ID_SET_INIT_POS, settings.getInitialXPosition(),
                                         settings.getInitialYPosition(), settings.getInitialHeading()};

            if (true == m_smpServer.sendData(m_serialMuxProtChannelIdRemoteCtrl, &cmd, sizeof(cmd)))
            {
                LOG_DEBUG("Initial vehicle data sent.");
                m_initialDataSent = true;
            }
            else
            {
                LOG_WARNING("Failed to send initial vehicle data.");
            }
        }

        if ((true == m_statusTimer.isTimeout()) && (true == m_smpServer.isSynced()))
        {
            Status payload = {SMPChannelPayload::Status::STATUS_FLAG_OK};

            if (false == m_smpServer.sendData(m_serialMuxProtChannelIdStatus, &payload, sizeof(payload)))
            {
                LOG_WARNING("Failed to send current status to RU.");
            }

            m_statusTimer.restart();
        }
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void App::fatalErrorHandler()
{
    if (false == m_isFatalError)
    {
        /* Turn on Red LED to signal fatal error. */
        Board::getInstance().getRedLed().enable(true);
    }

    m_isFatalError = true;
}

bool App::setupSerialMuxProtServer()
{
    bool isSuccessful = false;

    m_serialMuxProtChannelIdRemoteCtrl = m_smpServer.createChannel(COMMAND_CHANNEL_NAME, COMMAND_CHANNEL_DLC);
    m_serialMuxProtChannelIdMotorSpeeds =
        m_smpServer.createChannel(MOTOR_SPEED_SETPOINT_CHANNEL_NAME, MOTOR_SPEED_SETPOINT_CHANNEL_DLC);
    m_serialMuxProtChannelIdRobotSpeeds =
        m_smpServer.createChannel(ROBOT_SPEED_SETPOINT_CHANNEL_NAME, ROBOT_SPEED_SETPOINT_CHANNEL_DLC);
    m_serialMuxProtChannelIdStatus = m_smpServer.createChannel(STATUS_CHANNEL_NAME, STATUS_CHANNEL_DLC);

    m_smpServer.subscribeToChannel(COMMAND_RESPONSE_CHANNEL_NAME, App_cmdRspChannelCallback);
    m_smpServer.subscribeToChannel(LINE_SENSOR_CHANNEL_NAME, App_lineSensorChannelCallback);
    m_smpServer.subscribeToChannel(CURRENT_VEHICLE_DATA_CHANNEL_NAME, App_currentVehicleChannelCallback);

    if ((0U == m_serialMuxProtChannelIdRemoteCtrl) || (0U == m_serialMuxProtChannelIdMotorSpeeds) ||
        (0U == m_serialMuxProtChannelIdRobotSpeeds) || (0U == m_serialMuxProtChannelIdStatus))
    {
        LOG_ERROR("Failed to create SerialMuxProt channels.");
    }
    else
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

bool App::connectToServer()
{
    bool      isSuccessful = false;
    IPAddress serverIp;

    if (false == serverIp.fromString(APPSLAM_TCP_SERVER_IP))
    {
        LOG_ERROR("Invalid APPSlam TCP server IP: %s", APPSLAM_TCP_SERVER_IP);
    }
    else if (true == m_tcpClient.connect(serverIp, APPSLAM_TCP_SERVER_PORT))
    {
        LOG_INFO("TCP server connected to %s:%u", APPSLAM_TCP_SERVER_IP, APPSLAM_TCP_SERVER_PORT);
        isSuccessful = true;
    }
    else
    {
        LOG_WARNING("Failed to connect to TCP server %s:%u", APPSLAM_TCP_SERVER_IP, APPSLAM_TCP_SERVER_PORT);
    }

    return isSuccessful;
}

void App::sendPacket(const String& payload)
{
    if (true == m_tcpClient.connected())
    {
        String packet = payload + '\n';
        (void)m_tcpClient.write(reinterpret_cast<const uint8_t*>(packet.c_str()), packet.length());

        LOG_DEBUG("TX: %s", payload.c_str());
    }
}

void App::receivePackets()
{
    uint8_t buffer[128U];
    int     bytesRead = 0;

    do
    {
        bytesRead = m_tcpClient.read(buffer, sizeof(buffer));

        if (bytesRead > 0)
        {
            for (int i = 0; i < bytesRead; ++i)
            {
                char c = static_cast<char>(buffer[i]);

                if ('\n' == c)
                {
                    const int newlineIndex = m_tcpRxBuffer.lastIndexOf(String("\n"));

                    if (newlineIndex >= 0)
                    {
                        m_tcpRxBuffer = m_tcpRxBuffer.substring(0U, static_cast<unsigned int>(newlineIndex));
                    }

                    if (0 != m_tcpRxBuffer.length())
                    {
                        processIncomingLine(m_tcpRxBuffer);
                    }

                    m_tcpRxBuffer = String();
                }
                else
                {
                    m_tcpRxBuffer += c;
                }
            }
        }
    } while (bytesRead > 0);

    if (bytesRead < 0)
    {
        /* Error reading from TCP client. Close connection. */
        LOG_WARNING("TCP read error: %d", bytesRead);
        m_tcpClient.stop();
    }
}

void App::processIncomingLine(const String& line)
{
    LOG_DEBUG("RX: %s", line.c_str());

    /* Handle velocity command via ArduinoJson */
    {
        JsonDocument         doc;
        DeserializationError err = deserializeJson(doc, line.c_str());

        if (DeserializationError::Ok == err)
        {
            if ((!doc["linear"].isNull()) && (!doc["angular"].isNull()))
            {
                int32_t linearCenter = doc["linear"].as<int32_t>();
                int32_t angular      = doc["angular"].as<int32_t>();

                RobotSpeed robotSpeed = {linearCenter, angular};
                LOG_INFO("Received velocity command - Linear: %d mm/s, Angular: %d mrad/s", linearCenter, angular);
                if (false == m_smpServer.sendData(m_serialMuxProtChannelIdRobotSpeeds, &robotSpeed, sizeof(robotSpeed)))
                {
                    LOG_WARNING("Failed to send velocity command to RU.");
                }
                else
                {
                    LOG_DEBUG("Velocity command sent - Linear: %d mm/s, Angular: %d mrad/s", linearCenter, angular);
                }
            }
            else
            {
                LOG_WARNING("JSON does not contain expected fields 'linear' and 'angular'.");
            }
        }
        else
        {
            LOG_WARNING("Failed to parse JSON: %s", err.c_str());
        }
    }
}

void App::handleVehicleData(const VehicleData* vehicleData)
{
    if ((nullptr != vehicleData) && (true == m_tcpClient.connected()))
    {
        /* Format vehicle data as JSON using ArduinoJson and send via TCP */
        JsonDocument doc;
        doc["type"] = "vehicle_data";
        doc["t"]    = vehicleData->timestamp;
        doc["x"]    = vehicleData->xPos;
        doc["y"]    = vehicleData->yPos;
        doc["h"]    = vehicleData->orientation;
        doc["l"]    = vehicleData->left;
        doc["r"]    = vehicleData->right;
        doc["c"]    = vehicleData->center;
        doc["ax"]   = vehicleData->accelerationX;
        doc["tz"]   = vehicleData->turnRateZ;

        String out;
        serializeJson(doc, out);
        sendPacket(out);
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * Receives remote control command responses over SerialMuxProt channel.
 *
 * @param[in] payload       Command id
 * @param[in] payloadSize   Size of command id
 * @param[in] userData      User data
 */
void App_cmdRspChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    UTIL_NOT_USED(userData);
    if ((nullptr != payload) && (COMMAND_RESPONSE_CHANNEL_DLC == payloadSize))
    {
        const CommandResponse* cmdRsp = reinterpret_cast<const CommandResponse*>(payload);
        LOG_DEBUG("CMD_RSP: ID: 0x%02X , RSP: 0x%02X", cmdRsp->commandId, cmdRsp->responseId);

        if (SMPChannelPayload::CmdId::CMD_ID_GET_MAX_SPEED == cmdRsp->commandId)
        {
            LOG_DEBUG("Max Speed: %d", cmdRsp->maxMotorSpeed);
        }
    }
    else
    {
        LOG_WARNING("CMD_RSP: Invalid payload size. Expected: %u Received: %u", COMMAND_RESPONSE_CHANNEL_DLC,
                    payloadSize);
    }
}

/**
 * Receives line sensor data over SerialMuxProt channel.
 * @param[in]   payload         Line sensor data
 * @param[in]   payloadSize     Size of 5 line sensor data
 * @param[in]   userData        User data
 */
void App_lineSensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    UTIL_NOT_USED(payload);
    UTIL_NOT_USED(payloadSize);
    UTIL_NOT_USED(userData);
}

/**
 * Receives current position, heading and IMU of the robot over SerialMuxProt channel.
 *
 * @param[in] payload       Current vehicle data: Timestamp, two coordinates, one orientation,
 *                          three motor speeds and raw IMU data (X acceleration, Z axis turn rate).
 * @param[in] payloadSize   The size of the received payload data.
 * @param[in] userData      Instance of App class.
 */
void App_currentVehicleChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    if ((nullptr != payload) && (CURRENT_VEHICLE_DATA_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        const VehicleData* currentVehicleData = reinterpret_cast<const VehicleData*>(payload);
        App*               appInstance        = reinterpret_cast<App*>(userData);

        LOG_DEBUG("Timestamp: %d X: %d Y: %d Heading: %d Left: %d Right: %d Center: %d AccX: %d TurnZ: %d ",
                  currentVehicleData->timestamp, currentVehicleData->xPos, currentVehicleData->yPos,
                  currentVehicleData->orientation, currentVehicleData->left, currentVehicleData->right,
                  currentVehicleData->center, currentVehicleData->accelerationX, currentVehicleData->turnRateZ);

        /* Forward vehicle data to TCP connection */
        appInstance->handleVehicleData(currentVehicleData);
    }
    else
    {
        LOG_WARNING("%s: Invalid payload size. Expected: %u Received: %u", CURRENT_VEHICLE_DATA_CHANNEL_NAME,
                    CURRENT_VEHICLE_DATA_CHANNEL_DLC, payloadSize);
    }
}
