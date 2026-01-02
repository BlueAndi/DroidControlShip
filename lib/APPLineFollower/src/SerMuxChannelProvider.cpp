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
 * @brief  Serial multiplexer channel provider
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "SerMuxChannelProvider.h"
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

extern void SerMuxChannelProvider_cmdRspChannelCallback(const uint8_t* payload, const uint8_t payloadSize,
                                                        void* userData);

extern void SerMuxChannelProvider_currentVehicleChannelCallback(const uint8_t* payload, const uint8_t payloadSize,
                                                                void* userData);

extern void SerMuxChannelProvider_lineSensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize,
                                                            void* userData);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

const SerMuxChannelProvider::ChannelCfg SerMuxChannelProvider::CHANNEL_CFG[MAX_CHANNELS] = {
    /* Tx: Command channel */
    {COMMAND_CHANNEL_NAME, COMMAND_CHANNEL_DLC, nullptr},

    /* Rx: Command response channel */
    {COMMAND_RESPONSE_CHANNEL_NAME, COMMAND_RESPONSE_CHANNEL_DLC, SerMuxChannelProvider_cmdRspChannelCallback},

    /* Tx: Motor speed channel */
    {MOTOR_SPEED_SETPOINT_CHANNEL_NAME, MOTOR_SPEED_SETPOINT_CHANNEL_DLC, nullptr},

    /* Tx:Robot speed channel */
    {ROBOT_SPEED_SETPOINT_CHANNEL_NAME, ROBOT_SPEED_SETPOINT_CHANNEL_DLC, nullptr},

    /* Rx: Current vehicle data channel */
    {CURRENT_VEHICLE_DATA_CHANNEL_NAME, CURRENT_VEHICLE_DATA_CHANNEL_DLC,
     SerMuxChannelProvider_currentVehicleChannelCallback},

    /* Tx: Status channel */
    {STATUS_CHANNEL_NAME, STATUS_CHANNEL_DLC, nullptr},

    /* Rx: Line sensor channel */
    {LINE_SENSOR_CHANNEL_NAME, LINE_SENSOR_CHANNEL_DLC, SerMuxChannelProvider_lineSensorChannelCallback},

    /* Not used. */
    {nullptr, 0U, nullptr},
    {nullptr, 0U, nullptr},
    {nullptr, 0U, nullptr}};

/******************************************************************************
 * Public Methods
 *****************************************************************************/

bool SerMuxChannelProvider::init()
{
    bool   isSuccessful = true;
    size_t idx;

    /* Create the tx channels */
    for (idx = 0U; idx < MAX_CHANNELS; ++idx)
    {
        ChannelCfg channelCfg = CHANNEL_CFG[idx];

        if (nullptr != channelCfg.name)
        {
            /* Is it a transmit channel?*/
            if (nullptr == channelCfg.callback)
            {
                m_channelIds[idx] = m_smpServer.createChannel(channelCfg.name, channelCfg.dlc);

                if (0U == m_channelIds[idx])
                {
                    LOG_ERROR("Failed to create SerialMuxProt channel %s.", channelCfg.name);
                    isSuccessful = false;
                }
            }
            /* Its a receive channel. */
            else
            {
                m_channelIds[idx] = 0U; /* No channel ID for receive channels. */

                m_smpServer.subscribeToChannel(channelCfg.name, channelCfg.callback);
            }
        }
    }

    return isSuccessful;
}

bool SerMuxChannelProvider::sendStatus(const Status& status) const
{
    return m_smpServer.sendData(m_channelIds[CHANNEL_CFG_ID_STATUS], &status, sizeof(status));
}

bool SerMuxChannelProvider::sendMotorSpeed(const MotorSpeed& motorSpeed) const
{
    return m_smpServer.sendData(m_channelIds[CHANNEL_CFG_ID_MOTOR_SPEED_SETPOINT], &motorSpeed, sizeof(motorSpeed));
}

bool SerMuxChannelProvider::sendRobotSpeed(const RobotSpeed& robotSpeed) const
{
    return m_smpServer.sendData(m_channelIds[CHANNEL_CFG_ID_ROBOT_SPEED_SETPOINT], &robotSpeed, sizeof(robotSpeed));
}

bool SerMuxChannelProvider::requestMaxMotorSpeed(MaxMotorSpeedFunc cb)
{
    Command cmd = {SMPChannelPayload::CmdId::CMD_ID_GET_MAX_SPEED};

    m_maxMotorSpeedCallback = cb;

    return m_smpServer.sendData(m_channelIds[CHANNEL_CFG_ID_COMMAND], &cmd, sizeof(cmd));
}

bool SerMuxChannelProvider::requestLineSensorCalibration(LineSensorCalibFunc cb)
{
    Command cmd = {SMPChannelPayload::CmdId::CMD_ID_START_LINE_SENSOR_CALIB};

    m_lineSensorCalibCallback = cb;

    return m_smpServer.sendData(m_channelIds[CHANNEL_CFG_ID_COMMAND], &cmd, sizeof(cmd));
}

bool SerMuxChannelProvider::requestMotorSpeedCalibration(MotorSpeedCalibFunc cb)
{
    Command cmd = {SMPChannelPayload::CmdId::CMD_ID_START_MOTOR_SPEED_CALIB};

    m_motorSpeedCalibCallback = cb;

    return m_smpServer.sendData(m_channelIds[CHANNEL_CFG_ID_COMMAND], &cmd, sizeof(cmd));
}

bool SerMuxChannelProvider::requestReinitBoard(ReinitBoardFunc cb)
{
    Command cmd = {SMPChannelPayload::CmdId::CMD_ID_REINIT_BOARD};

    m_reinitBoardCallback = cb;

    return m_smpServer.sendData(m_channelIds[CHANNEL_CFG_ID_COMMAND], &cmd, sizeof(cmd));
}

void SerMuxChannelProvider::process()
{
    m_smpServer.process(millis());
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void SerMuxChannelProvider::serMuxCdRspChannelCallback(const CommandResponse* cmdRsp)
{
    if (nullptr != cmdRsp)
    {
        switch (cmdRsp->commandId)
        {
        case SMPChannelPayload::CmdId::CMD_ID_IDLE:
            LOG_DEBUG("Idle command response received.");
            break;

        case SMPChannelPayload::CmdId::CMD_ID_GET_MAX_SPEED:
            LOG_DEBUG("Max motor speed: %d", cmdRsp->maxMotorSpeed);

            if (nullptr != m_maxMotorSpeedCallback)
            {
                m_maxMotorSpeedCallback(cmdRsp->responseId, cmdRsp->maxMotorSpeed);
            }
            break;

        case SMPChannelPayload::CmdId::CMD_ID_START_LINE_SENSOR_CALIB:
            LOG_DEBUG("Line sensor calibration started.");

            if (nullptr != m_lineSensorCalibCallback)
            {
                m_lineSensorCalibCallback(cmdRsp->responseId);
            }
            break;

        case SMPChannelPayload::CmdId::CMD_ID_START_MOTOR_SPEED_CALIB:
            LOG_DEBUG("Motor speed calibration started.");

            if (nullptr != m_motorSpeedCalibCallback)
            {
                m_motorSpeedCalibCallback(cmdRsp->responseId);
            }
            break;

        case SMPChannelPayload::CmdId::CMD_ID_REINIT_BOARD:
            LOG_DEBUG("Reinitializing board.");

            if (nullptr != m_reinitBoardCallback)
            {
                m_reinitBoardCallback(cmdRsp->responseId);
            }
            break;

        case SMPChannelPayload::CmdId::CMD_ID_START_DRIVING:
            LOG_DEBUG("Start driving command received.");
            break;

        case SMPChannelPayload::CmdId::CMD_ID_SET_INIT_POS:
            LOG_DEBUG("Set initial position command received.");
            break;

        default:
            LOG_WARNING("Received unknown command response %d.", cmdRsp->commandId);
            break;
        }
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/**
 * Receives remote control command responses over SerialMuxProt channel.
 *
 * @param[in] payload       Command id.
 * @param[in] payloadSize   Size of command id.
 * @param[in] userData      Instance of App class.
 */
void SerMuxChannelProvider_cmdRspChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    if ((nullptr != payload) && (COMMAND_RESPONSE_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        SerMuxChannelProvider* provider = reinterpret_cast<SerMuxChannelProvider*>(userData);
        const CommandResponse* cmdRsp   = reinterpret_cast<const CommandResponse*>(payload);

        provider->serMuxCdRspChannelCallback(cmdRsp);
    }
    else
    {
        LOG_WARNING("CMD_RSP: Invalid payload size. Expected: %u Received: %u", COMMAND_RESPONSE_CHANNEL_DLC,
                    payloadSize);
    }
}

/**
 * Receives current position and heading of the robot over SerialMuxProt channel.
 *
 * @param[in] payload       Current vehicle data.
 * @param[in] payloadSize   Size of current vehicle data.
 * @param[in] userData      Instance of App class.
 */
void SerMuxChannelProvider_currentVehicleChannelCallback(const uint8_t* payload, const uint8_t payloadSize,
                                                         void* userData)
{
    if ((nullptr != payload) && (CURRENT_VEHICLE_DATA_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        SerMuxChannelProvider* provider           = static_cast<SerMuxChannelProvider*>(userData);
        const VehicleData*     currentVehicleData = reinterpret_cast<const VehicleData*>(payload);

        if (nullptr != provider->m_vehicleDataCallback)
        {
            provider->m_vehicleDataCallback(*currentVehicleData);
        }
    }
    else
    {
        LOG_WARNING("%s: Invalid payload size. Expected: %u Received: %u", CURRENT_VEHICLE_DATA_CHANNEL_NAME,
                    CURRENT_VEHICLE_DATA_CHANNEL_DLC, payloadSize);
    }
}

/**
 * Receives line sensor data over SerialMuxProt channel.
 *
 * @param[in]   payload         Line sensor data.
 * @param[in]   payloadSize     Size of 5 line sensor data.
 * @param[in]   userData        Instance of App class.
 */
void SerMuxChannelProvider_lineSensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize, void* userData)
{
    if ((nullptr != payload) && (LINE_SENSOR_CHANNEL_DLC == payloadSize) && (nullptr != userData))
    {
        SerMuxChannelProvider* provider       = static_cast<SerMuxChannelProvider*>(userData);
        const LineSensorData*  lineSensorData = reinterpret_cast<const LineSensorData*>(payload);

        if (nullptr != provider->m_lineSensorCallback)
        {
            provider->m_lineSensorCallback(*lineSensorData);
        }
    }
    else
    {
        LOG_WARNING("%s: Invalid payload size. Expected: %u Received: %u", LINE_SENSOR_CHANNEL_NAME,
                    LINE_SENSOR_CHANNEL_DLC, payloadSize);
    }
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
