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
#include <PlatoonController/ILongitudinalController.h>
#include <new>
#include "SerialMuxChannels.h"
#include <PIDController.h>
#include <SimpleTimer.hpp>

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
     * @param[in]   currentVehicleData      Current vehicle data.
     * @param[in]   targetWaypoint          Target waypoint to drive to.
     * @param[out]  centerSpeedSetpoint     Center speed setpoint [steps/s].
     *
     * @return If successful, returns true otherwise false.
     */
    bool calculateLongitudinalMovement(const Telemetry& currentVehicleData, const Waypoint& targetWaypoint,
                                       int16_t& centerSpeedSetpoint) final;

private:
    /**
     * Maximum motor speed in encoder steps/s
     * Speed determined experimentally using the motor calibration of the RadonUlzer.
     */
    static const int16_t MAX_MOTOR_SPEED = 3000;

    /** Period in ms for PID processing. */
    static const uint32_t PID_PROCESS_PERIOD = 50U;

    /** The PID proportional factor numerator for the heading controller. */
    static const int32_t PID_P_NUMERATOR = 3;

    /** The PID proportional factor denominator for the heading controller.*/
    static const int32_t PID_P_DENOMINATOR = 4;

    /** The PID integral factor numerator for the heading controller. */
    static const int32_t PID_I_NUMERATOR = 1;

    /** The PID integral factor denominator for the heading controller. */
    static const int32_t PID_I_DENOMINATOR = 10;

    /** The PID derivative factor numerator for the heading controller. */
    static const int32_t PID_D_NUMERATOR = 1;

    /** The PID derivative factor denominator for the heading controller. */
    static const int32_t PID_D_DENOMINATOR = 10;

    /** PID controller, used for maintaining IVS. */
    PIDController<int32_t> m_pidCtrl;

    /** Timer used for periodically PID processing. */
    SimpleTimer m_pidProcessTime;

    /** Inter Vehicle Space (IVS) in mm. */
    int32_t m_ivs;

    /**
     * Set the factors for the PID controller.
     *
     * @param[in] pNumerator Numerator of the proportional factor.
     * @param[in] pDenominator Denominator of the proportional factor.
     * @param[in] iNumerator Numerator of the integral factor.
     * @param[in] iDenominator Denominator of the integral factor.
     * @param[in] dNumerator Numerator of the derivative factor.
     * @param[in] dDenominator Denominator of the derivative factor.
     */
    void setPIDFactors(int32_t pNumerator, int32_t pDenominator, int32_t iNumerator, int32_t iDenominator,
                       int32_t dNumerator, int32_t dDenominator);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* LONGITUDINAL_CONTROLLER_H */
/** @} */
