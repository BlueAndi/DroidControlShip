/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
    LOG_DEBUG("DEFAULT_IVS: %d", DEFAULT_IVS);
    LOG_DEBUG("MAX_PLATOON_LENGTH: %d", MAX_PLATOON_LENGTH);
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
        int32_t linearSpeedSetpoint = m_maxMotorSpeed;
        int32_t controlSpeed        = m_maxMotorSpeed;

        /* Constrain the setpoint based on platoon length. */
        platoonLengthController(linearSpeedSetpoint);

        if (controlSpeed > linearSpeedSetpoint)
        {
            LOG_DEBUG("Speed limited by Platoon Length Controller. Length: %d", m_platoonLength);
            controlSpeed = linearSpeedSetpoint;
        }

        /* Constrain the setpoint using collision avoidance module. */
        m_collisionAvoidance.limitSpeedToAvoidCollision(linearSpeedSetpoint, m_vehicleData);

        if (controlSpeed > linearSpeedSetpoint)
        {
            LOG_DEBUG("Speed limited by Collision Avoidance.");
            controlSpeed = linearSpeedSetpoint;
        }

        /* Constrain setpoint to max and min motor speeds. */
        m_currentSpeedSetpoint = constrain(linearSpeedSetpoint, 0, m_maxMotorSpeed);

        if (controlSpeed > m_currentSpeedSetpoint)
        {
            LOG_DEBUG("Speed limited by constrain.");
        }
    }
}

void DrivingState::exit()
{
    m_isActive = false;
}

void DrivingState::setMaxMotorSpeed(int32_t maxSpeed)
{
    m_maxMotorSpeed = maxSpeed;
    LOG_DEBUG("DrivingState: Max motor speed: %d", m_maxMotorSpeed);
}

bool DrivingState::getTopMotorSpeed(int32_t& topMotorSpeed) const
{
    topMotorSpeed = m_currentSpeedSetpoint;

    /* Only valid if the state is active. */
    return m_isActive;
}

void DrivingState::setVehicleData(const Telemetry& data)
{
    m_vehicleData = data;
}

void DrivingState::setPlatoonLength(const int32_t platoonLength)
{
    /*
     * Platoon length was calculated using the center points, not the vehicle edges.
     * Vehicle length is added to take front and back of first and last vehicles into account.
     */
    m_platoonLength = platoonLength + VEHICLE_LENGTH;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void DrivingState::platoonLengthController(int32_t& linearCenterSpeedSetpoint)
{
    /* Limit the speed setpoint if platoon length greater than maximum allowed. */
    if (m_platoonLength > MAX_PLATOON_LENGTH)
    {
        int32_t maxPossibleSpeed = linearCenterSpeedSetpoint;

        maxPossibleSpeed = maxPossibleSpeed - (m_platoonLength * maxPossibleSpeed / (MAX_PLATOON_LENGTH * 10));

        if (m_platoonLength > 2 * MAX_PLATOON_LENGTH)
        {
            maxPossibleSpeed = 0;
        }

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
