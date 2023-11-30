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
    m_currentVehicleData(),
    m_processingChainTimer(),
    m_processingChain(nullptr)
{
}

PlatoonController::~PlatoonController()
{
    if (nullptr != m_processingChain)
    {
        delete m_processingChain;
        m_processingChain = nullptr;
    }
}

bool PlatoonController::init(const InputWaypointCallback&  inputWaypointCallback,
                             const OutputWaypointCallback& outputWaypointCallback,
                             const MotorSetpointCallback&  motorSetpointCallback)
{
    bool isSuccessful = false;

    if ((nullptr != inputWaypointCallback) && (nullptr != outputWaypointCallback) && (nullptr != motorSetpointCallback))
    {
        m_processingChain = ProcessingChainFactory::getInstance().create();

        if (nullptr == m_processingChain)
        {
            LOG_ERROR("Failed to create processing chain.");
        }
        else
        {

            m_inputWaypointCallback  = inputWaypointCallback;
            m_outputWaypointCallback = outputWaypointCallback;
            m_motorSetpointCallback  = motorSetpointCallback;

            m_processingChainTimer.start(PROCESSING_CHAIN_PERIOD);

            isSuccessful = true;
        }
    }

    return isSuccessful;
}

void PlatoonController::process()
{
    /* Check if target waypoint has been reached. */
    if ((true == targetWaypointReached()) && (nullptr != m_inputWaypointCallback) &&
        (nullptr != m_outputWaypointCallback))
    {
        /* Get next waypoint. */
        if (false == m_inputWaypointCallback(m_nextWaypoint))
        {
            ; /* Nothing to do here. Have to wait for a waypoint. */
        }
        /* Send current waypoint to the next vehicle only when a new one has been received. */
        else if (false == m_outputWaypointCallback(m_currentWaypoint))
        {
            LOG_ERROR("Failed to send waypoint to next vehicle.");
        }
        else
        {
            m_currentWaypoint = m_nextWaypoint;
        }
    }

    /* Process chain on timeout. */
    if ((true == m_processingChainTimer.isTimeout()) && (nullptr != m_motorSetpointCallback))
    {
        if (nullptr != m_processingChain)
        {
            int16_t leftMotorSpeedSetpoint  = 0;
            int16_t rightMotorSpeedSetpoint = 0;

            if (false == m_processingChain->calculateMotorSetpoints(m_currentVehicleData, m_currentWaypoint,
                                                                    leftMotorSpeedSetpoint, rightMotorSpeedSetpoint))
            {
                LOG_ERROR("Failed to calculate motor setpoints.");
            }
            else
            {
                m_motorSetpointCallback(leftMotorSpeedSetpoint, rightMotorSpeedSetpoint);
            }
        }
        else
        {
            LOG_ERROR("Processing chain is not initialized.");
        }

        m_processingChainTimer.restart();
    }
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

bool PlatoonController::targetWaypointReached() const
{
    bool isReached = false;

    int32_t differenceX = abs(m_currentWaypoint.xPos - m_currentVehicleData.xPos);
    int32_t differenceY = abs(m_currentWaypoint.yPos - m_currentVehicleData.yPos);

    if ((TARGET_WAYPOINT_ERROR_MARGIN >= differenceX) && (TARGET_WAYPOINT_ERROR_MARGIN >= differenceY))
    {
        isReached = true;
    }

    return isReached;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/