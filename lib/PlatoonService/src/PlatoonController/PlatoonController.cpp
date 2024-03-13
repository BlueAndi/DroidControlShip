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
 * @brief  Platoon controller class for calculating each step inside a platoon context.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "PlatoonController.h"
#include "PlatoonUtils.h"
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
    m_nextWaypoint(),
    m_currentVehicleData(),
    m_lastSentWaypoint(),
    m_processingChainTimer(),
    m_processingChain(nullptr),
    m_isPositionKnown(false),
    m_processingChainRelease(false),
    m_invalidWaypointCounter(0U)
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
    bool             isSuccessful    = false;
    SettingsHandler& settingsHandler = SettingsHandler::getInstance();

    if ((nullptr != inputWaypointCallback) && (nullptr != outputWaypointCallback) &&
        (nullptr != motorSetpointCallback) && (nullptr == m_processingChain))
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

            m_currentVehicleData.xPos        = settingsHandler.getInitialXPosition();
            m_currentVehicleData.yPos        = settingsHandler.getInitialYPosition();
            m_currentVehicleData.orientation = settingsHandler.getInitialHeading();

            m_currentWaypoint = m_currentVehicleData.asWaypoint();

            m_processingChainTimer.start(PROCESSING_CHAIN_PERIOD);

            isSuccessful = true;
        }
    }

    return isSuccessful;
}

void PlatoonController::process(size_t numberOfAvailableWaypoints)
{
    if (false == m_isPositionKnown)
    {
        /* Do nothing if the current vehicle position is not known. */
        return;
    }

    /* Check if target waypoint has been reached. */
    if ((numberOfAvailableWaypoints > MIN_AVAILABLE_WAYPOINTS) && (nullptr != m_inputWaypointCallback) &&
        (nullptr != m_outputWaypointCallback))
    {
        /* Get next waypoint. */
        if (false == m_inputWaypointCallback(m_nextWaypoint))
        {
            ; /* Nothing to do here. Have to wait for a waypoint. */
        }
        else
        {
            /* Sanitize the Waypoint. */
            int32_t headingDelta = 0;
            if (false ==
                PlatoonUtils::calculateRelativeHeading(m_nextWaypoint, m_currentVehicleData.asWaypoint(), headingDelta))
            {
                LOG_ERROR("Failed to calculate relative heading for (%d, %d)", m_nextWaypoint.xPos,
                          m_nextWaypoint.yPos);
            }
            /* Target is in the forward cone. */
            else if ((headingDelta > -FORWARD_CONE_APERTURE) && (headingDelta < FORWARD_CONE_APERTURE))
            {
                /* Update current waypoint. */
                m_currentWaypoint = m_nextWaypoint;
                LOG_DEBUG("New Waypoint: (%d, %d) Vc=%d", m_currentWaypoint.xPos, m_currentWaypoint.yPos,
                          m_currentWaypoint.center);

                /* Reset counter once a valid waypoint is received. */
                m_invalidWaypointCounter = 0U;
            }
            else
            {
                LOG_ERROR("Invalid target waypoint (%d, %d)", m_nextWaypoint.xPos, m_nextWaypoint.yPos);

                /* Prevent wrap-around. */
                if (UINT8_MAX > m_invalidWaypointCounter)
                {
                    /* Increase counter. */
                    ++m_invalidWaypointCounter;
                }
            }
        }
    }

    /* Send current position as waypoint in a constant distance interval. */
    if (WAYPOINT_DISTANCE_INTERVAL <
        PlatoonUtils::calculateAbsoluteDistance(m_lastSentWaypoint, m_currentVehicleData.asWaypoint()))
    {
        if (false == m_outputWaypointCallback(m_currentVehicleData.asWaypoint()))
        {
            LOG_ERROR("Failed to send waypoint to next vehicle.");
        }
        else
        {
            m_lastSentWaypoint = m_currentVehicleData.asWaypoint();
        }
    }

    /* Are there enough waypoints available at the start of the drive? */
    if ((false == m_processingChainRelease) && (MIN_AVAILABLE_WAYPOINTS < numberOfAvailableWaypoints))
    {
        m_processingChainRelease = true;
    }
    /* Process chain on timeout. */
    else if (true == m_processingChainTimer.isTimeout())
    {
        if (nullptr != m_processingChain)
        {
            int16_t leftMotorSpeedSetpoint  = 0;
            int16_t rightMotorSpeedSetpoint = 0;
            bool    calculationSuccessful   = true;

            if (false == m_processingChain->calculateMotorSetpoints(m_currentVehicleData, m_currentWaypoint,
                                                                    leftMotorSpeedSetpoint, rightMotorSpeedSetpoint))
            {
                LOG_ERROR("Failed to calculate motor setpoints.");
                calculationSuccessful = false;
            }
            else if (true == targetWaypointReached())
            {
                leftMotorSpeedSetpoint  = 0;
                rightMotorSpeedSetpoint = 0;
            }

            if ((true == calculationSuccessful) && (nullptr != m_motorSetpointCallback))
            {
                /* Send motor setpoints. */
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

void PlatoonController::setLatestVehicleData(const Telemetry& vehicleData)
{
    m_currentVehicleData = vehicleData;
    m_isPositionKnown    = true;
}

uint8_t PlatoonController::getInvalidWaypointCounter() const
{
    return m_invalidWaypointCounter;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool PlatoonController::targetWaypointReached() const
{
    return PlatoonUtils::areWaypointsEqual(m_currentWaypoint, m_currentVehicleData.asWaypoint(),
                                           TARGET_WAYPOINT_ERROR_MARGIN);
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
