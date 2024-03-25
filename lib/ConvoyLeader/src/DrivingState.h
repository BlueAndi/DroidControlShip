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
 * @brief  Driving state.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef DRIVING_STATE_H
#define DRIVING_STATE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>
#include <IState.h>
#include <StateMachine.h>
#include "SerialMuxChannels.h"
#include <CollisionAvoidance.h>
#include <V2VCommManager.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The system driving state. */
class DrivingState : public IState
{
public:
    /**
     * Get state instance.
     *
     * @return State instance.
     */
    static DrivingState& getInstance()
    {
        static DrivingState instance;

        /* Singleton idiom to force initialization during first usage. */

        return instance;
    }

    /**
     * If the state is entered, this method will called once.
     */
    void entry() final;

    /**
     * Processing the state.
     *
     * @param[in] sm State machine, which is calling this state.
     */
    void process(StateMachine& sm) final;

    /**
     * If the state is left, this method will be called once.
     */
    void exit() final;

    /**
     * Set maximum motor speed.
     * This also sets the range factor as is dependent on the maximum motor speed.
     *
     * @param[in] maxSpeed Maximum motor speed.
     */
    void setMaxMotorSpeed(int16_t maxSpeed);

    /**
     * Get the calculated top motor speed.
     *
     * @param[out] topMotorSpeed The calculated top motor speed.
     *
     * @returns true if the top motor speed is valid. Otherwise, false.
     */
    bool getTopMotorSpeed(int16_t& topMotorSpeed) const;

    /**
     * Set latest vehicle data.
     *
     * @param[in] data   Latest vehicle data.
     */
    void setVehicleData(const Telemetry& data);

    /**
     * Set the platoon length.
     *
     * @param[in] platoonLength  Platoon Length.
     */
    void setPlatoonLength(const int32_t platoonLength);

    /**
     * Is state active?
     *
     * @return If state is active, it will return true otherwise false.
     */
    bool isActive() const
    {
        return m_isActive;
    }

protected:
private:
    /** Vehicle length in mm. */
    static const uint8_t VEHICLE_LENGTH = 100U;

    /** Maximum inter vehicle space in mm. */
    static const uint16_t DEFAULT_IVS = 200;

    /** Maximum platoon length allowed. */
    static const uint16_t MAX_PLATOON_LENGTH = (VEHICLE_LENGTH + (V2VCommManager::NUMBER_OF_FOLLOWERS * DEFAULT_IVS));

    /** Flag: State is active. */
    bool m_isActive;

    /** Maximum motor speed. */
    int16_t m_maxMotorSpeed;

    /** Current linear speed setpoint to apply to the vehicle. */
    int16_t m_currentSpeedSetpoint;

    /** Latest vehicle data. */
    Telemetry m_vehicleData;

    /** Length of the platoon from start of leader to the end of the last follower. */
    int32_t m_platoonLength;

    /** Collision Avoidance instance. */
    CollisionAvoidance m_collisionAvoidance;

    /**
     * Limit the speed setpoint based on the platoon length.
     *
     * @param[out] speedSetpoint The speed setpoint.
     */
    void platoonLengthController(int16_t& speedSetpoint);

    /**
     * Default constructor.
     */
    DrivingState() :
        IState(),
        m_isActive(false),
        m_maxMotorSpeed(0),
        m_currentSpeedSetpoint(0),
        m_vehicleData(),
        m_platoonLength(),
        m_collisionAvoidance(SMPChannelPayload::RANGE_0_5, SMPChannelPayload::RANGE_20_25)
    {
    }

    /**
     * Default destructor.
     */
    virtual ~DrivingState()
    {
    }

    /* Not allowed. */
    DrivingState(const DrivingState& state);            /**< Copy construction of an instance. */
    DrivingState& operator=(const DrivingState& state); /**< Assignment of an instance. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* DRIVING_STATE_H */
/** @} */
