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
 * @brief  Processing Chain composed of a longitudinal and a lateral component.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "ProcessingChain.h"
#include "Logging.h"

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

ProcessingChain::ProcessingChain(ILongitudinalController&   longitudinalController,
                                 ILongitudinalSafetyPolicy& longitudinalSafetyPolicy,
                                 ILateralController& lateralController, ILateralSafetyPolicy& lateralSafetyPolicy) :
    m_longitudinalController(longitudinalController),
    m_longitudinalSafetyPolicy(longitudinalSafetyPolicy),
    m_lateralController(lateralController),
    m_lateralSafetyPolicy(lateralSafetyPolicy)
{
}

ProcessingChain::~ProcessingChain()
{
    delete &m_longitudinalController;
    delete &m_longitudinalSafetyPolicy;
    delete &m_lateralController;
    delete &m_lateralSafetyPolicy;
}

bool ProcessingChain::calculateMotorSetpoints(const Telemetry& currentVehicleData, const Waypoint& targetWaypoint,
                                              int16_t& leftMotorSpeedSetpoint, int16_t& rightMotorSpeedSetpoint)
{
    bool    isSuccessful        = false;
    int16_t centerSpeedSetpoint = 0;

    /* Calculate longitudinal movement. */
    if (false ==
        m_longitudinalController.calculateLongitudinalMovement(currentVehicleData, targetWaypoint, centerSpeedSetpoint))
    {
        LOG_ERROR("Failed to calculate longitudinal movement.");
    }
    else if (false == m_longitudinalSafetyPolicy.check(centerSpeedSetpoint))
    {
        LOG_ERROR("Failed to check longitudinal safety policy.");
    }
    else if (false == m_lateralController.calculateLateralMovement(currentVehicleData, targetWaypoint,
                                                                   centerSpeedSetpoint, leftMotorSpeedSetpoint,
                                                                   rightMotorSpeedSetpoint))
    {
        LOG_ERROR("Failed to calculate lateral movement.");
    }
    else if (false == m_lateralSafetyPolicy.check(leftMotorSpeedSetpoint, rightMotorSpeedSetpoint))
    {
        LOG_ERROR("Failed to check lateral safety policy.");
    }
    else
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
