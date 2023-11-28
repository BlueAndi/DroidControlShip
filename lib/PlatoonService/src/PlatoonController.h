/* MIT License
 *
 * Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>
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
 * @addtogroup Service
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
 * Input waypoint callback.
 * Called in order to get the next waypoint into the platoon controller.
 */
typedef std::function<void(Waypoint& waypoint)> InputWaypointCallback;

/**
 * Output waypoint callback.
 * Called in order to send the last waypoint to the next platoon participant.
 */
typedef std::function<void(const Waypoint& waypoint)> OutputWaypointCallback;

/**
 * Motor setpoint callback.
 * Called in order to set the motor speeds.
 */
typedef std::function<void(const int16_t left, const int16_t right)> MotorSetpointCallback;

/**
 * Platoon controller class for calculating each step inside a platoon context.
 * Follows a waypoint by calculating the speed setpoints for each motor.
 */
class PlatoonController
{
public:
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
     */
    void process();

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
    static const int32_t TARGET_WAYPOINT_ERROR_MARGIN = 5U;

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
     * Current vehicle data in the form of a waypoint.
     */
    Waypoint m_currentVehicleData;

    /**
     * Processing chain timer.
     */
    SimpleTimer m_processingChainTimer;

    /**
     * Processing chain.
     */
    ProcessingChain* m_processingChain;

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
