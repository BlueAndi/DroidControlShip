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
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "LongitudinalController.h"
#include "PlatoonUtils.h"
#include <Arduino.h>
#include <Logging.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

LongitudinalController::LongitudinalController() :
    ILongitudinalController(),
    m_pidCtrl(),
    m_pidProcessTime(),
    m_ivs(350)
{
    /* Configure PID. */
    m_pidCtrl.clear();
    setPIDFactors(PID_P_NUMERATOR, PID_P_DENOMINATOR, PID_I_NUMERATOR, PID_I_DENOMINATOR, PID_D_NUMERATOR,
                  PID_D_DENOMINATOR);
    m_pidCtrl.setSampleTime(PID_PROCESS_PERIOD);
    m_pidCtrl.setLimits(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    m_pidCtrl.setDerivativeOnMeasurement(true);

    m_pidProcessTime.start(0); /* Immediate */
}

LongitudinalController::~LongitudinalController()
{
}

bool LongitudinalController::calculateLongitudinalMovement(const Telemetry& currentVehicleData,
                                                           const Waypoint& targetWaypoint, int16_t& centerSpeedSetpoint)
{
    bool    isSuccessful = true;
    int32_t pidDelta     = 0;
    int32_t distance     = PlatoonUtils::calculateAbsoluteDistance(targetWaypoint, currentVehicleData.asWaypoint());

    if (true == m_pidProcessTime.isTimeout())
    {
        /* Calculate PID delta. */
        pidDelta = m_pidCtrl.calculate(m_ivs, distance);
        m_pidProcessTime.start(PID_PROCESS_PERIOD);
    }

    centerSpeedSetpoint = constrain(targetWaypoint.center - pidDelta, 0, MAX_MOTOR_SPEED * 2);

    return isSuccessful;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void LongitudinalController::setPIDFactors(int32_t pNumerator, int32_t pDenominator, int32_t iNumerator,
                                           int32_t iDenominator, int32_t dNumerator, int32_t dDenominator)
{
    m_pidCtrl.setPFactor(pNumerator, pDenominator);
    m_pidCtrl.setIFactor(iNumerator, iDenominator);
    m_pidCtrl.setDFactor(dNumerator, dDenominator);
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
