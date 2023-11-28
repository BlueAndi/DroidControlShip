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
 * @brief  Interface for a Lateral Controller.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Service
 *
 * @{
 */
#ifndef I_LATERAL_CONTROLLER_H
#define I_LATERAL_CONTROLLER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "Waypoint.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Abstract lateral controller interface. */
class ILateralController
{
public:
    /**
     * ILateralController creation function, used by the ProcessingChainFactory to create a ILateralController
     * instance.
     */
    typedef ILateralController* (*CreateFunc)(void);

    /**
     * Destroys the interface.
     */
    virtual ~ILateralController()
    {
    }

    /**
     * Calculates the motor speeds for the next step.
     *
     * @param[in]   currentWaypoint         Current waypoint where the vehicle is found.
     * @param[in]   targetWaypoint          Target waypoint to drive to.
     * @param[in]   centerSpeedSetpoint     Center speed setpoint [steps/s] calculated by longitudinal controller.
     * @param[out]  leftMotorSpeedSetpoint  Left motor speed setpoint [steps/s].
     * @param[out]  rightMotorSpeedSetpoint Right motor speed setpoint [steps/s].
     *
     * @return If successful, returns true otherwise false.
     */
    virtual bool calculateLateralMovement(const Waypoint& currentWaypoint, const Waypoint& targetWaypoint,
                                          const int16_t centerSpeedSetpoint, int16_t& leftMotorSpeedSetpoint,
                                          int16_t& rightMotorSpeedSetpoint) = 0;

private:
    /**
     * Constructs the interface.
     */
    ILateralController()
    {
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* I_LATERAL_CONTROLLER_H */
/** @} */
