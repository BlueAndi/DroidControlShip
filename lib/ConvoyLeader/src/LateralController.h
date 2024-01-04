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
 * @brief  Concrete LateralController.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef LATERAL_CONTROLLER_H
#define LATERAL_CONTROLLER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <ILateralController.h>
#include <new>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Concrete LateralController. */
class LateralController : public ILateralController
{
public:
    /**
     * Lateral Controller constructor.
     */
    LateralController();

    /**
     * Lateral Controller destructor.
     */
    virtual ~LateralController();

    /**
     * Creates a LateralController instance, for registering in the ProcessingChainFactory.
     *
     * @return If successful, returns a pointer to the LateralController instance. Otherwise nullptr.
     */
    static ILateralController* create()
    {
        return new (std::nothrow) LateralController();
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
    bool calculateLateralMovement(const Waypoint& currentWaypoint, const Waypoint& targetWaypoint,
                                  const int16_t centerSpeedSetpoint, int16_t& leftMotorSpeedSetpoint,
                                  int16_t& rightMotorSpeedSetpoint) final;

private:
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* LATERAL_CONTROLLER_H */
/** @} */
