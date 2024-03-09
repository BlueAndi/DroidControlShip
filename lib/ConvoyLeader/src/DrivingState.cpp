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
#include <Logging.h>
#include <V2VCommManager.h>

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
        m_currentSpeedSetpoint = 0;
    }
    else
    {
        /* Compact implementation of the "Platoon Application Controller". */

        /* Drive as fast as possible */
        int16_t linearSpeedSetpoint = m_maxMotorSpeed;

        /* Constrain the setpoint based on platoon length. */
        platoonLengthController(linearSpeedSetpoint);

        /* Constrain the setpoint using collision avoidance module. */
        m_collisionAvoidance.limitSpeedToAvoidCollision(linearSpeedSetpoint, m_vehicleData);

        /* Constrain setpoint to max and min motor speeds. */
        m_currentSpeedSetpoint = constrain(linearSpeedSetpoint, 0, m_maxMotorSpeed);
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

bool DrivingState::getTopMotorSpeed(int16_t& topMotorSpeed) const
{
    topMotorSpeed = m_currentSpeedSetpoint;

    /* Only valid if the state is active. */
    return m_isActive;
}

void DrivingState::setVehicleData(const Telemetry& data)
{
    m_vehicleData = data;
}

void DrivingState::setLastFollowerFeedback(const Telemetry& feedback)
{
    m_followerFeedback = feedback;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void DrivingState::platoonLengthController(int16_t& linearCenterSpeedSetpoint)
{
    int32_t distanceToLast           = 0;
    int16_t maxPossiblePlatoonLength = 0;

    /* Calculate current platoon length based on distance to last follower. */
    distanceToLast =
        (abs((m_vehicleData.xPos - m_followerFeedback.xPos)) + abs(m_vehicleData.yPos - m_followerFeedback.yPos));

    /* Calculate max. possible platoon length. */
    maxPossiblePlatoonLength = (V2VCommManager::NUMBER_OF_FOLLOWERS * (VEHICLE_LENGTH + MAX_INTER_VEHICLE_SPACE));

    /* Limit the speed setpoint if platoon length greater than maximum allowed. */
    if (distanceToLast > maxPossiblePlatoonLength)
    {
        int16_t maxPossibleSpeed = linearCenterSpeedSetpoint;

        maxPossibleSpeed = maxPossibleSpeed - (distanceToLast * maxPossibleSpeed / (maxPossiblePlatoonLength * 2));

        /* Make sure the speed is constraint. */
        if (maxPossibleSpeed < linearCenterSpeedSetpoint)
        {
            linearCenterSpeedSetpoint = maxPossibleSpeed;
        }
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
