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
#include <PlatoonController.h>
#include <queue>
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
     *
     * @param[in] maxSpeed Maximum motor speed.
     */
    void setMaxMotorSpeed(int16_t maxSpeed);

    /**
     * Get the calculated motor speed setpoints.
     *
     * @param[out] leftMotorSpeed The calculated left motor speed.
     * @param[out] rightMotorSpeed The calculated right motor speed.
     *
     * @returns true if the speed setpoints are valid. Otherwise, false.
     */
    bool getMotorSpeedSetpoints(int16_t& leftMotorSpeed, int16_t& rightMotorSpeed) const;

    /**
     * Set the calculated motor speed setpoints. Shall only be called internally and not by the user.
     *
     * @param[in] leftMotorSpeed The calculated left motor speed.
     * @param[in] rightMotorSpeed The calculated right motor speed.
     *
     * @returns true, as the assignment cannot fail.
     */
    bool setMotorSpeedSetpoints(const int16_t leftMotorSpeed, const int16_t rightMotorSpeed);

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

    /**
     * Push the next waypoint into the queue.
     *
     * @param[in] waypoint  Pointer to next waypoint.
     *
     * @return If successful returns true, otherwise false.
     */
    bool pushWaypoint(Waypoint* waypoint);

    /**
     * Get latest waypoint to be sent to follower.
     *
     * @param[out] waypoint  Latest waypoint.
     *
     * @return If successful returns true, otherwise false.
     */
    bool getWaypoint(Waypoint& waypoint);

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
    /** Maximum invalid waypoints allowed before going into error state. */
    static const uint8_t MAX_INVALID_WAYPOINTS = 3U;

    /** Flag: State is active. */
    bool m_isActive;

    /** Flag: initialization is successful. */
    bool m_isInitSuccessful;

    /** Maximum motor speed. */
    int16_t m_maxMotorSpeed;

    /** Calculated left motor speed. */
    int16_t m_leftMotorSpeed;

    /** Calculated left motor speed. */
    int16_t m_rightMotorSpeed;

    /** Latest vehicle data. */
    VehicleData m_vehicleData;

    /** Last follower feedback. */
    VehicleData m_followerFeedback;

    /** Platoon controller. */
    PlatoonController m_platoonController;

    /** Outgoing Waypoint. */
    Waypoint m_outputWaypoint;

    /**
     * Queue for the received waypoints.
     * Stores pointers to the waypoints in the queue when received in the callback.
     * The queue is emptied by the getNextWaypoint() method.
     * The queue is filled by the targetWaypointTopicCallback() method.
     *
     * @tparam Waypoint*    Pointer to a Waypoint.
     */
    std::queue<Waypoint*> m_inputWaypointQueue;

    /**
     * Setup the platoon controller.
     *
     * @return If successful returns true, otherwise false.
     */
    bool setupPlatoonController();

    /**
     * Default constructor.
     */
    DrivingState() :
        IState(),
        m_isActive(false),
        m_isInitSuccessful(false),
        m_maxMotorSpeed(0),
        m_leftMotorSpeed(0),
        m_rightMotorSpeed(0),
        m_vehicleData{0},
        m_followerFeedback{0},
        m_platoonController(),
        m_outputWaypoint(),
        m_inputWaypointQueue()
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
