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
     * @param[in] vehicleData   Latest vehicle data.
     */
    void setVehicleData(const VehicleData& vehicleData);

    /**
     * Set last follower feedback.
     *
     * @param[in] feedback  Last follower feedback.
     */
    void setLastFollowerFeedback(const VehicleData& feedback);

protected:
private:
    /** Number of proximity Sensor ranges. */
    static const uint8_t NUM_PROXIMITY_SENSOR_RANGES = SMPChannelPayload::RANGE_0_5;

    /** Flag: State is active. */
    bool m_isActive;

    /** Maximum motor speed. */
    int16_t m_maxMotorSpeed;

    /** Calculated top motor speed. */
    int16_t m_topMotorSpeed;

    /** Latest vehicle data. */
    VehicleData m_vehicleData;

    /** Last follower feedback. */
    VehicleData m_followerFeedback;

    /**
     * Factor of the max motor speed substracted per proximity sensor range.
     * The closer an object is, the slower the vehicle should drive.
     * Factor is multiplied with the proximity sensor range.
     */
    int16_t m_rangeFactor;

    /**
     * Default constructor.
     */
    DrivingState() :
        IState(),
        m_isActive(false),
        m_maxMotorSpeed(0),
        m_topMotorSpeed(0),
        m_vehicleData{0},
        m_followerFeedback{0},
        m_rangeFactor(0)
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
