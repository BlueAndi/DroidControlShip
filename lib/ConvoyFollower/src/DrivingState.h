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
#include <queue>
#include <StateMachine.h>
#include "SerialMuxChannels.h"
#include <CollisionAvoidance.h>
#include <PIDController.h>
#include <SimpleTimer.hpp>
#include <HeadingFinder.h>

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
     * Set latest vehicle data.
     *
     * @param[in] vehicleData   Latest vehicle data.
     */
    void setVehicleData(const Telemetry& vehicleData);

    /**
     * Push the next waypoint into the queue.
     *
     * @param[in] waypoint  Pointer to next waypoint.
     *
     * @return If successful returns true, otherwise false.
     */
    bool pushWaypoint(Waypoint* waypoint);

    /**
     * Get the last reached waypoint.
     *
     * @return Last reached waypoint.
     */
    Waypoint getLastReachedWaypoint() const
    {
        return m_lastReachedWaypoint;
    }

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

    /** Aperture angle of the forward cone in mrad. */
    static const int32_t FORWARD_CONE_APERTURE = 1300; /* Aprox. 75 degrees */

    /** Period in ms for PID processing. */
    static const uint32_t IVS_PID_PROCESS_PERIOD = 50U;

    /** Default Inter Vehicle Space in mm. */
    static const int32_t DEFAULT_IVS = 200;

    /**
     * Error margin in mm for target waypoint.
     * Used to determine if target waypoint has been reached.
     */
    static const int32_t TARGET_WAYPOINT_ERROR_MARGIN = 30;

    /** PID factors for the Inter Vehicle Space Controller. */
    struct IVS_PID_FACTORS
    {
        /** The PID proportional factor numerator for the Inter Vehicle Space Controller. */
        static const int32_t PID_P_NUMERATOR = 3;

        /** The PID proportional factor denominator for the Inter Vehicle Space Controller.*/
        static const int32_t PID_P_DENOMINATOR = 4;

        /** The PID integral factor numerator for the Inter Vehicle Space Controller. */
        static const int32_t PID_I_NUMERATOR = 1;

        /** The PID integral factor denominator for the Inter Vehicle Space Controller. */
        static const int32_t PID_I_DENOMINATOR = 10;

        /** The PID derivative factor numerator for the Inter Vehicle Space Controller. */
        static const int32_t PID_D_NUMERATOR = 1;

        /** The PID derivative factor denominator for the Inter Vehicle Space Controller. */
        static const int32_t PID_D_DENOMINATOR = 10;
    };

    /** PID factors for the Heading. */
    struct HEADING_FINDER_PID_FACTORS
    {
        /** The PID proportional factor numerator for the Heading Finder. */
        static const int32_t PID_P_NUMERATOR = 2;

        /** The PID proportional factor denominator for the Heading Finder.*/
        static const int32_t PID_P_DENOMINATOR = 1;

        /** The PID integral factor numerator for the Heading Finder. */
        static const int32_t PID_I_NUMERATOR = 0;

        /** The PID integral factor denominator for the Heading Finder. */
        static const int32_t PID_I_DENOMINATOR = 1;

        /** The PID derivative factor numerator for the Heading Finder. */
        static const int32_t PID_D_NUMERATOR = 30;

        /** The PID derivative factor denominator for the Heading Finder. */
        static const int32_t PID_D_DENOMINATOR = 1;
    };

    /** Flag: State is active. */
    bool m_isActive;

    /** Maximum motor speed. */
    int16_t m_maxMotorSpeed;

    /** Calculated left motor speed. */
    int16_t m_leftMotorSpeed;

    /** Calculated left motor speed. */
    int16_t m_rightMotorSpeed;

    /** Latest vehicle data. */
    Telemetry m_vehicleData;

    /**
     * Queue for the received waypoints.
     * Stores pointers to the waypoints in the queue when received in the callback.
     * The queue is emptied by the processNextWaypoint() method.
     * The queue is filled by the pushWaypoint() method.
     *
     * @tparam Waypoint*    Pointer to a Waypoint.
     */
    std::queue<Waypoint*> m_inputWaypointQueue;

    /** Collision Avoidance instance. */
    CollisionAvoidance m_collisionAvoidance;

    /** Target waypoint. */
    Waypoint m_targetWaypoint;

    /** Last waypoint. */
    Waypoint m_lastReachedWaypoint;

    /** Counter of invalid waypoints. */
    uint8_t m_invalidWaypointCounter;

    /** PID controller, used for maintaining IVS. */
    PIDController<int32_t> m_ivsPidController;

    /** Timer used for periodically PID processing. */
    SimpleTimer m_ivsPidProcessTimer;

    /** Inter Vehicle Space (IVS) in mm. */
    int32_t m_ivs;

    /** Heading finder. */
    HeadingFinder m_headingFinder;

    /** Cumulative distance of waypoints in queue in mm */
    int32_t m_cumulativeQueueDistance;

    /** Distance to the predecessor in mm */
    int32_t m_distanceToPredecessor;

    /**
     * Get latest waypoint from the queue, validate it and set it to as the current target.
     */
    void processNextWaypoint();

    /**
     * Default constructor.
     */
    DrivingState();

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
