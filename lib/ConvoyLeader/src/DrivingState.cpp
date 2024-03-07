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
        m_topMotorSpeed = 0;
    }
    else
    {
        int16_t maxPossibleSpeed         = m_maxMotorSpeed;
        int32_t distanceToLast           = 0;
        int16_t maxPossiblePlatoonLength = 0;

        /* Check follower feedback. */
        distanceToLast =
            (abs((m_vehicleData.xPos - m_followerFeedback.xPos)) + abs(m_vehicleData.yPos - m_followerFeedback.yPos));

        /* Calculate inter vehicle space. */
        m_interVehicleSpace = constrain(m_interVehicleSpace, MIN_INTER_VEHICLE_SPACE, m_interVehicleSpace);

        /* Calculate platoon length and react accordingly. */
        maxPossiblePlatoonLength = (V2VCommManager::NUMBER_OF_FOLLOWERS * (VEHICLE_LENGTH + m_interVehicleSpace));

        if (distanceToLast > maxPossiblePlatoonLength)
        {
            maxPossibleSpeed = maxPossibleSpeed - (distanceToLast * maxPossibleSpeed / (maxPossiblePlatoonLength * 2));
        }

        /* Calculate top motor speed. */
        m_topMotorSpeed = constrain(maxPossibleSpeed, 0, m_maxMotorSpeed);

        /* Setpoint limiting by collision avoidance module. */
        m_collisionAvoidance.limitSpeedToAvoidCollision(m_topMotorSpeed, m_vehicleData);
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
    topMotorSpeed = m_topMotorSpeed;

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

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
