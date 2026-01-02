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
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef SER_MUX_CHANNEL_PROVIDER_H
#define SER_MUX_CHANNEL_PROVIDER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include "SerialMuxChannels.h"
#include <functional>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The serial multiplexer protocol channel provider wraps the handling with
 * the serial multiplexer protocol server. It solves the problem of having
 * several sinks for different channels, but only one source context for all
 * channels.
 */
class SerMuxChannelProvider
{
public:
    /**
     * This type defines the callback for line sensor data.
     */
    typedef std::function<void(const LineSensorData& data)> LineSensorCallback;

    /**
     * This type defines the callback for vehicle data.
     */
    typedef std::function<void(const VehicleData& data)> VehicleDataCallback;

    /**
     * This type defines the callback function for handling maximum motor speed requests.
     */
    typedef std::function<void(SMPChannelPayload::RspId status, int16_t maxMotorSpeed)> MaxMotorSpeedFunc;

    /**
     * This type defines the callback function for handling line sensor calibration requests.
     */
    typedef std::function<void(SMPChannelPayload::RspId status)> LineSensorCalibFunc;

    /**
     * This type defines the callback function for handling motor speed calibration requests.
     */
    typedef std::function<void(SMPChannelPayload::RspId status)> MotorSpeedCalibFunc;

    /**
     * This type defines the callback function for handling board reinitialization requests.
     */
    typedef std::function<void(SMPChannelPayload::RspId status)> ReinitBoardFunc;

    /**
     * Construct the serial multiplexer protocol channel provider.
     *
     * @param[in] stream Stream for sending and receiving data.
     */
    SerMuxChannelProvider(Stream& stream) :
        m_smpServer(stream),
        m_channelIds{0U},
        m_lineSensorCallback(nullptr),
        m_vehicleDataCallback(nullptr),
        m_maxMotorSpeedCallback(nullptr),
        m_lineSensorCalibCallback(nullptr),
        m_motorSpeedCalibCallback(nullptr),
        m_reinitBoardCallback(nullptr)
    {
        m_smpServer.setUserData(this);
    }

    /**
     * Destroy the serial multiplexer protocol channel provider.
     */
    ~SerMuxChannelProvider()
    {
        /* Nothing to do. */
    }

    /**
     * Initialize the serial multiplexer protocol channels.
     *
     * @return If channels are successfully initialized, it will return true otherwise false.
     * @note This method must be called before using the channels.
     */
    bool init();

    /**
     * Returns current synchronization state with the Radon Ulzer controller.
     *
     * @return If the client is synced with the server, it will return true otherwise false.
     */
    bool isInSync()
    {
        return m_smpServer.isSynced();
    }

    /**
     * Register a callback for line sensor data.
     *
     * @param[in] callback Callback to be called when line sensor data is received.
     */
    void registerLineSensorCallback(LineSensorCallback callback)
    {
        m_lineSensorCallback = callback;
    }

    /**
     * Register a callback for vehicle data.
     *
     * @param[in] callback Callback to be called when vehicle data is received.
     */
    void registerVehicleDataCallback(VehicleDataCallback callback)
    {
        m_vehicleDataCallback = callback;
    }

    /**
     * Send a status frame with the given status.
     *
     * @param[in] status Status to be sent.
     *
     * @returns If the status was successfully sent, it will return true. Otherwise, false.
     */
    bool sendStatus(const Status& status) const;

    /**
     * Send a motor speed setpoint to the Radon Ulzer controller.
     *
     * @param[in] motorSpeed Motor speed setpoint to be sent.
     *
     * @returns If the motor speed was successfully sent, it will return true. Otherwise, false.
     */
    bool sendMotorSpeed(const MotorSpeed& motorSpeed) const;

    /**
     * Send a robot speed setpoint to the Radon Ulzer controller.
     *
     * @param[in] robotSpeed Robot speed setpoint to be sent.
     *
     * @returns If the robot speed was successfully sent, it will return true. Otherwise, false.
     */
    bool sendRobotSpeed(const RobotSpeed& robotSpeed) const;

    /**
     * Request maximum motor speed from the Radon Ulzer controller.
     *
     * @param[in] cb Callback function to handle the maximum motor speed response.
     *
     * @return If the request was successfully sent, it will return true. Otherwise, false.
     */
    bool requestMaxMotorSpeed(MaxMotorSpeedFunc cb);

    /**
     * Request line sensor calibration from the Radon Ulzer controller.
     *
     * @param[in] cb Callback function to handle the line sensor calibration response.
     *
     * @return If the request was successfully sent, it will return true. Otherwise, false.
     */
    bool requestLineSensorCalibration(LineSensorCalibFunc cb);

    /**
     * Request motor speed calibration from the Radon Ulzer controller.
     *
     * @param[in] cb Callback function to handle the motor speed calibration response.
     *
     * @return If the request was successfully sent, it will return true. Otherwise, false.
     */
    bool requestMotorSpeedCalibration(MotorSpeedCalibFunc cb);

    /**
     * Request board reinitialization from the Radon Ulzer controller.
     *
     * @param[in] cb Callback function to handle the board reinitialization response.
     *
     * @return If the request was successfully sent, it will return true. Otherwise, false.
     */
    bool requestReinitBoard(ReinitBoardFunc cb);

    /**
     * Process the serial multiplexer protocol server.
     * This method must be called cyclic to process incoming data and manage the server.
     */
    void process();

private:
    /**
     * This type defines the channel configuration identifiers.
     * They are used to identify the channels in the channel configuration array.
     */
    typedef enum
    {
        CHANNEL_CFG_ID_COMMAND = 0,          /**< Command channel ID */
        CHANNEL_CFG_ID_COMMAND_RESPONSE,     /**< Command response channel ID */
        CHANNEL_CFG_ID_MOTOR_SPEED_SETPOINT, /**< Motor speed setpoint channel ID */
        CHANNEL_CFG_ID_ROBOT_SPEED_SETPOINT, /**< Robot speed setpoint channel ID */
        CHANNEL_CFG_ID_CURRENT_VEHICLE_DATA, /**< Current vehicle data channel ID */
        CHANNEL_CFG_ID_STATUS,               /**< Status channel ID */
        CHANNEL_CFG_ID_LINE_SENSOR           /**< Line sensor channel ID */

    } ChannelCfgId;

    /**
     * This type defines the channel configuration.
     * It contains the name, DLC and callback for the channel.
     */
    typedef struct
    {
        const char*     name;     /**< Name of the channel */
        uint8_t         dlc;      /**< DLC of the channel */
        ChannelCallback callback; /**< Callback for the channel */

    } ChannelCfg;

    /**
     * Channel configuration array.
     */
    static const ChannelCfg CHANNEL_CFG[];

    /**
     * Serial multiplexer protocol server instance.
     */
    SMPServer m_smpServer;

    /**
     * Channel IDs for the Serial multiplexer protocol channels.
     */
    uint8_t m_channelIds[MAX_CHANNELS];

    /**
     * Callback for line sensor data.
     */
    LineSensorCallback m_lineSensorCallback;

    /**
     * Callback for vehicle data.
     */
    VehicleDataCallback m_vehicleDataCallback;

    /**
     * Callback for handling maximum motor speed requests.
     */
    MaxMotorSpeedFunc m_maxMotorSpeedCallback;

    /**
     * Callback for handling line sensor calibration requests.
     */
    LineSensorCalibFunc m_lineSensorCalibCallback;

    /**
     * Callback for handling motor speed calibration requests.
     */
    MotorSpeedCalibFunc m_motorSpeedCalibCallback;

    /**
     * Callback for handling board reinitialization requests.
     */
    ReinitBoardFunc m_reinitBoardCallback;

    /**
     * Receives remote control command responses over SerialMuxProt channel.
     *
     * @param[in] cmdRsp Command response
     */
    void serMuxCdRspChannelCallback(const CommandResponse* cmdRsp);

    /**
     * Default constructor not allowed.
     */
    SerMuxChannelProvider();

    friend void SerMuxChannelProvider_cmdRspChannelCallback(const uint8_t* payload, const uint8_t payloadSize,
                                                            void* userData);

    friend void SerMuxChannelProvider_currentVehicleChannelCallback(const uint8_t* payload, const uint8_t payloadSize,
                                                                    void* userData);

    friend void SerMuxChannelProvider_lineSensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize,
                                                                void* userData);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SER_MUX_CHANNEL_PROVIDER_H */
/** @} */
