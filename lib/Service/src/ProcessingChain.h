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
 * @brief  Processing Chain composed of a longitudinal and a lateral component.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Service
 *
 * @{
 */
#ifndef PROCESSING_CHAIN_H
#define PROCESSING_CHAIN_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "ILongitudinalController.h"
#include "ILongitudinalSafetyPolicy.h"
#include "ILateralController.h"
#include "ILateralSafetyPolicy.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Processing Chain composed of a longitudinal and a lateral component. */
class ProcessingChain
{
public:
    /**
     * Constructs the processing chain.
     */
    ProcessingChain(ILongitudinalController&   longitudinalController,
                    ILongitudinalSafetyPolicy& longitudinalSafetyPolicy, ILateralController& lateralController,
                    ILateralSafetyPolicy& lateralSafetyPolicy);

    /**
     * Destroys the processing chain.
     */
    ~ProcessingChain();

    /**
     * Calculates the motor speeds for the next step.
     *
     * @param[in]   currentWaypoint         Current waypoint where the vehicle is found.
     * @param[in]   targetWaypoint          Target waypoint to drive to.
     * @param[out]  leftMotorSpeedSetpoint  Left motor speed setpoint [steps/s].
     * @param[out]  rightMotorSpeedSetpoint Right motor speed setpoint [steps/s].
     *
     * @return If successful, returns true. Otherwise false.
     */
    bool calculateMotorSetpoints(const Waypoint& currentWaypoint, const Waypoint& targetWaypoint,
                                 int16_t& leftMotorSpeedSetpoint, int16_t& rightMotorSpeedSetpoint);

private:
    /** Longitudinal controller. */
    ILongitudinalController& m_longitudinalController;

    /** Longitudinal safety policy. */
    ILongitudinalSafetyPolicy& m_longitudinalSafetyPolicy;

    /** Lateral controller. */
    ILateralController& m_lateralController;

    /** Lateral safety policy. */
    ILateralSafetyPolicy& m_lateralSafetyPolicy;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* PROCESSING_CHAIN_H */
/** @} */
