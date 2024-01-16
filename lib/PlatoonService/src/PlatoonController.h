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
 * @brief  Platoon controller class for calculating each step inside a platoon context.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup PlatoonService
 *
 * @{
 */
#ifndef PLATOON_CONTROLLER_H
#define PLATOON_CONTROLLER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include <functional>
#include <SimpleTimer.hpp>
#include "Waypoint.h"
#include "ProcessingChain.h"
#include "ProcessingChainFactory.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Platoon controller class for calculating each step inside a platoon context.
 * Follows a waypoint by calculating the speed setpoints for each motor.
 */
class PlatoonController
{
public:
    /**
     * Input waypoint callback.
     * Called in order to get the next waypoint into the platoon controller.
     */
    typedef std::function<bool(Waypoint& waypoint)> InputWaypointCallback;

    /**
     * Output waypoint callback.
     * Called in order to send the last waypoint to the next platoon participant.
     */
    typedef std::function<bool(const Waypoint& waypoint)> OutputWaypointCallback;

    /**
     * Motor setpoint callback.
     * Called in order to set the motor speeds.
     */
    typedef std::function<bool(const int16_t left, const int16_t right)> MotorSetpointCallback;

    /**
     * PlatoonController default constructor.
     */
    PlatoonController();

    /**
     * PlatoonController default destructor.
     */
    ~PlatoonController();

    /**
     * Initialize the platoon controller with the application callbacks and processing chain configuration.
     *
     * @param[in] inputWaypointCallback   Input waypoint callback.
     * @param[in] outputWaypointCallback  Output waypoint callback.
     * @param[in] motorSetpointCallback   Motor setpoint callback.
     *
     * @return If successfully initialized, returns true. Otherwise, false.
     */
    bool init(const InputWaypointCallback& inputWaypointCallback, const OutputWaypointCallback& outputWaypointCallback,
              const MotorSetpointCallback& motorSetpointCallback);

    /**
     * Process the PlatoonController.
     *
     * @param[in] numberOfAvailableWaypoints  Number of available waypoints.
     */
    void process(size_t numberOfAvailableWaypoints);

    /**
     * Set lastest vehicle data.
     *
     * @param[in] vehicleData  Lastest vehicle data in the form of a waypoint.
     */
    void setLatestVehicleData(const Waypoint& vehicleData);

private:
    /**
     * Period in ms for processing chain.
     */
    static const uint32_t PROCESSING_CHAIN_PERIOD = 50U;

    /**
     * Error margin in mm for target waypoint.
     * Used to determine if target waypoint has been reached.
     */
    static const int32_t TARGET_WAYPOINT_ERROR_MARGIN = 10;

    /**
     * Aperture angle of the forward cone in mrad.
     */
    static const int32_t FORWARD_CONE_APERTURE = 1300; /* Aprox. 75 degrees */

    /**
     * Distance interval between waypoints in mm.
     */
    static const int32_t WAYPOINT_DISTANCE_INTERVAL = 50;

    /**
     * Minimum number of available waypoints for release of processing chain.
     */
    static const size_t MIN_AVAILABLE_WAYPOINTS = 2U;

    /**
     * Input waypoint callback.
     */
    InputWaypointCallback m_inputWaypointCallback;

    /**
     * Output waypoint callback.
     */
    OutputWaypointCallback m_outputWaypointCallback;

    /**
     * Motor setpoint callback.
     */
    MotorSetpointCallback m_motorSetpointCallback;

    /**
     * Current waypoint to follow.
     */
    Waypoint m_currentWaypoint;

    /**
     * Next waypoint to follow.
     */
    Waypoint m_nextWaypoint;

    /**
     * Current vehicle data in the form of a waypoint.
     */
    Waypoint m_currentVehicleData;

    /**
     * Last sent waypoint to the next platoon participant.
     */
    Waypoint m_lastSentWaypoint;

    /**
     * Processing chain timer.
     */
    SimpleTimer m_processingChainTimer;

    /**
     * Processing chain.
     */
    ProcessingChain* m_processingChain;

    /**
     * Flag to indicate that the first vehicle data has been received.
     * This means, that the position of the vehicle is known.
     */
    bool m_isPositionKnown;

    /**
     * Flag to indicate the release of the processing chain.
     */
    bool m_processingChainRelease;

private:
    /**
     * Check if the target waypoint has been reached.
     *
     * @return If target waypoint has been reached, returns true. Otherwise, false.
     */
    bool targetWaypointReached() const;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* PLATOON_CONTROLLER_H */
/** @} */
