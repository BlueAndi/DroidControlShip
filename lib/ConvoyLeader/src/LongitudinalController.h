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
 * @brief  Concrete Longitudinal Controller.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef LONGITUDINAL_CONTROLLER_H
#define LONGITUDINAL_CONTROLLER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <ILongitudinalController.h>
#include <new>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Concrete Longitudinal Controller */
class LongitudinalController : public ILongitudinalController
{
public:
    /**
     * Longitudinal Controller constructor.
     */
    LongitudinalController();

    /**
     * Longitudinal Controller destructor.
     */
    virtual ~LongitudinalController();

    /**
     * Creates a LongitudinalController instance, for registering in the ProcessingChainFactory.
     *
     * @return If successful, returns a pointer to the LongitudinalController instance. Otherwise nullptr.
     */
    static ILongitudinalController* create()
    {
        return new (std::nothrow) LongitudinalController();
    }

    /**
     * Calculates the motor speeds for the next step.
     *
     * @param[in]   currentWaypoint         Current waypoint where the vehicle is found.
     * @param[in]   targetWaypoint          Target waypoint to drive to.
     * @param[out]  centerSpeedSetpoint     Center speed setpoint [steps/s].
     *
     * @return If successful, returns true otherwise false.
     */
    bool calculateLongitudinalMovement(const Waypoint& currentWaypoint, const Waypoint& targetWaypoint,
                                       int16_t& centerSpeedSetpoint) final;

private:
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* LONGITUDINAL_CONTROLLER_H */
/** @} */