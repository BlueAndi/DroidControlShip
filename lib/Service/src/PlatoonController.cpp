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
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "PlatoonController.h"
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

PlatoonController::PlatoonController() :
    m_inputWaypointCallback(nullptr),
    m_outputWaypointCallback(nullptr),
    m_motorSetpointCallback(nullptr),
    m_currentWaypoint(),
    m_currentVehicleData()
{
}

PlatoonController::~PlatoonController()
{
}

bool PlatoonController::init(const ProcessingChainConfig&  chainConfig,
                             const InputWaypointCallback&  inputWaypointCallback,
                             const OutputWaypointCallback& outputWaypointCallback,
                             const MotorSetpointCallback&  motorSetpointCallback)
{
    bool isSuccessful = false;

    if ((nullptr != inputWaypointCallback) && (nullptr != outputWaypointCallback) && (nullptr != motorSetpointCallback))
    {
        m_inputWaypointCallback  = inputWaypointCallback;
        m_outputWaypointCallback = outputWaypointCallback;
        m_motorSetpointCallback  = motorSetpointCallback;

        LOG_DEBUG("PlatoonController configuration: %d %d %d %d", chainConfig.LogitudinalControllerId,
                  chainConfig.LongitdinalSafetyPolicyId, chainConfig.LateralControllerId,
                  chainConfig.LateralSafetyPolicyId);

        isSuccessful = true;
    }

    return isSuccessful;
}

void PlatoonController::process()
{
}

void PlatoonController::setLatestVehicleData(const Waypoint& vehicleData)
{
    m_currentVehicleData = vehicleData;
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
