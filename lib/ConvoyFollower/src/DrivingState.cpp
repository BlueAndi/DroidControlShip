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
 * @brief  Driving state.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "DrivingState.h"
#include "ErrorState.h"
#include <Logging.h>
#include <PlatoonUtils.h>

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

void DrivingState::entry()
{
    m_isActive = true;
}

void DrivingState::process(StateMachine& sm)
{
    /* Check if the state is active. */
    if (false == m_isActive)
    {
        /* Stop motors. */
        m_leftMotorSpeed  = 0;
        m_rightMotorSpeed = 0;
    }
    else
    {
        /* Get next waypoint from the queue. */
        processNextWaypoint();

        /* Check invalid waypoint count. */
        if (MAX_INVALID_WAYPOINTS <= m_invalidWaypointCounter)
        {
            /* Go to Error state. Too many invalid waypoints received. Are we going in the right direction? */
            LOG_ERROR("Too many invalid waypoints received. Going into error state.");
            sm.setState(&ErrorState::getInstance());
        }
        else
        {
            int32_t pidDelta            = 0;
            int32_t distance            = 0;
            int16_t centerSpeedSetpoint = 0;

            /* Longitudinal controller. */
            /* Calculate distance to target waypoint. */
            distance = PlatoonUtils::calculateAbsoluteDistance(m_targetWaypoint, m_vehicleData.asWaypoint());

            if (true == m_ivsPidProcessTimer.isTimeout())
            {
                /* Calculate PID delta. Objective is for distance to reach and maintain the IVS. */
                pidDelta = m_ivsPidController.calculate(m_ivs, distance);

                /* Restart timer. */
                m_ivsPidProcessTimer.start(IVS_PID_PROCESS_PERIOD);
            }

            /* Center speedpoint is relative to the center speed of the target waypoint. */
            centerSpeedSetpoint = m_targetWaypoint.center - pidDelta;

            /* Lateral controller. */
            /* Set current input values. */
            m_headingFinder.setOdometryData(m_vehicleData.xPos, m_vehicleData.yPos, m_vehicleData.orientation);
            m_headingFinder.setMotorSpeedData(centerSpeedSetpoint, centerSpeedSetpoint);
            m_headingFinder.setTargetHeading(m_targetWaypoint.xPos, m_targetWaypoint.yPos);

            /* Calculate differential motor speed setpoints to reach target. */
            m_headingFinder.process(m_leftMotorSpeed, m_rightMotorSpeed);

            /* Collision Avoidance. */
            m_collisionAvoidance.limitSpeedToAvoidCollision(m_leftMotorSpeed, m_rightMotorSpeed, m_vehicleData);
        }
    }
}

void DrivingState::exit()
{
    m_isActive = false;
}

void DrivingState::setMaxMotorSpeed(int16_t maxSpeed)
{
    m_maxMotorSpeed = maxSpeed;
}

bool DrivingState::getMotorSpeedSetpoints(int16_t& leftMotorSpeed, int16_t& rightMotorSpeed) const
{
    leftMotorSpeed  = m_leftMotorSpeed;
    rightMotorSpeed = m_rightMotorSpeed;

    /* Only valid if the state is active. */
    return m_isActive;
}

void DrivingState::setVehicleData(const Telemetry& vehicleData)
{
    m_vehicleData = vehicleData;
}

bool DrivingState::pushWaypoint(Waypoint* waypoint)
{
    bool isSuccessful = false;

    /* Check if the state is active. */
    if (true == m_isActive)
    {
        if (nullptr == waypoint)
        {
            LOG_ERROR("Waypoint is nullptr.");
        }
        else
        {
            m_inputWaypointQueue.push(waypoint);
            isSuccessful = true;
        }
    }

    return isSuccessful;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void DrivingState::processNextWaypoint()
{
    /* Get latest waypoint. */
    if (false == m_inputWaypointQueue.empty())
    {
        /* Retrieve next waypoint. */
        Waypoint* nextWaypoint = m_inputWaypointQueue.front();
        m_inputWaypointQueue.pop();

        /* Check for invalid waypoint. */
        int32_t headingDelta = 0;
        if (false == PlatoonUtils::calculateRelativeHeading(*nextWaypoint, m_vehicleData.asWaypoint(), headingDelta))
        {
            LOG_ERROR("Failed to calculate relative heading for (%d, %d)", nextWaypoint->xPos, nextWaypoint->yPos);
        }
        /* Target is in the forward cone. */
        else if ((headingDelta > -FORWARD_CONE_APERTURE) && (headingDelta < FORWARD_CONE_APERTURE))
        {
            /* Copy waypoint. */
            m_lastReachedWaypoint = m_targetWaypoint;
            m_targetWaypoint      = *nextWaypoint;

            LOG_DEBUG("New Waypoint: (%d, %d) Vc=%d", m_targetWaypoint.xPos, m_targetWaypoint.yPos,
                      m_targetWaypoint.center);

            /* Reset counter once a valid waypoint is received. */
            m_invalidWaypointCounter = 0U;
        }
        else
        {
            LOG_ERROR("Invalid target waypoint (%d, %d)", nextWaypoint->xPos, nextWaypoint->yPos);

            /* Prevent wrap-around. */
            if (UINT8_MAX > m_invalidWaypointCounter)
            {
                /* Increase counter. */
                ++m_invalidWaypointCounter;
            }
        }

        /* Delete queued waypoint. */
        delete nextWaypoint;
    }
}

DrivingState::DrivingState() :
    IState(),
    m_isActive(false),
    m_maxMotorSpeed(0),
    m_leftMotorSpeed(0),
    m_rightMotorSpeed(0),
    m_vehicleData(),
    m_inputWaypointQueue(),
    m_collisionAvoidance(SMPChannelPayload::RANGE_0_5, SMPChannelPayload::RANGE_10_15),
    m_targetWaypoint(),
    m_lastReachedWaypoint(),
    m_invalidWaypointCounter(0U),
    m_ivsPidController(),
    m_ivsPidProcessTimer(),
    m_ivs(350),
    m_headingFinder()
{
    /* Configure PID. */
    m_ivsPidController.clear();
    m_ivsPidController.setPFactor(IVS_PID_FACTORS::PID_P_NUMERATOR, IVS_PID_FACTORS::PID_P_DENOMINATOR);
    m_ivsPidController.setIFactor(IVS_PID_FACTORS::PID_I_NUMERATOR, IVS_PID_FACTORS::PID_I_DENOMINATOR);
    m_ivsPidController.setDFactor(IVS_PID_FACTORS::PID_D_NUMERATOR, IVS_PID_FACTORS::PID_D_DENOMINATOR);
    m_ivsPidController.setSampleTime(IVS_PID_PROCESS_PERIOD);
    m_ivsPidController.setLimits(-m_maxMotorSpeed, m_maxMotorSpeed);
    m_ivsPidController.setDerivativeOnMeasurement(true);
    m_ivsPidProcessTimer.start(0); /* Immediate */

    /* Configure heading finder. */
    m_headingFinder.setPIDFactors(
        HEADING_FINDER_PID_FACTORS::PID_P_NUMERATOR, HEADING_FINDER_PID_FACTORS::PID_P_DENOMINATOR,
        HEADING_FINDER_PID_FACTORS::PID_I_NUMERATOR, HEADING_FINDER_PID_FACTORS::PID_I_DENOMINATOR,
        HEADING_FINDER_PID_FACTORS::PID_D_NUMERATOR, HEADING_FINDER_PID_FACTORS::PID_D_DENOMINATOR);
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
