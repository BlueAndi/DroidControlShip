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
#include <ProcessingChainFactory.h>
#include "LongitudinalController.h"
#include "LongitudinalSafetyPolicy.h"
#include "LateralController.h"
#include "LateralSafetyPolicy.h"

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

    if (false == m_isInitSuccessful)
    {
        m_isInitSuccessful = setupPlatoonController();
    }

    if (false == m_isInitSuccessful)
    {
        LOG_FATAL("Could not initialize Platoon Controller.");
    }
}

void DrivingState::process(StateMachine& sm)
{
    /* Check initialization is successful. */
    if (false == m_isInitSuccessful)
    {
        sm.setState(&ErrorState::getInstance());
    }
    else
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
            /* Set latest vehicle data. */
            Waypoint vehicleDataAsWaypoint(m_vehicleData.xPos, m_vehicleData.yPos, m_vehicleData.orientation,
                                           m_vehicleData.left, m_vehicleData.right, m_vehicleData.center);

            m_platoonController.setLatestVehicleData(vehicleDataAsWaypoint);

            /* Process PlatoonController. */
            m_platoonController.process(m_inputWaypointQueue.size());

            /* Get invalid waypoint count. */
            if (MAX_INVALID_WAYPOINTS <= m_platoonController.getInvalidWaypointCounter())
            {
                /* Go to Error state. Too many invalid waypoints received. Are we going in the right direction? */
                LOG_ERROR("Too many invalid waypoints received. Going into error state.");
                sm.setState(&ErrorState::getInstance());
            }
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

bool DrivingState::setMotorSpeedSetpoints(const int16_t leftMotorSpeed, const int16_t rightMotorSpeed)
{
    m_leftMotorSpeed  = leftMotorSpeed;
    m_rightMotorSpeed = rightMotorSpeed;
    return true;
}

void DrivingState::setVehicleData(const VehicleData& vehicleData)
{
    m_vehicleData = vehicleData;
}

void DrivingState::setLastFollowerFeedback(const VehicleData& feedback)
{
    m_followerFeedback = feedback;
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

bool DrivingState::getWaypoint(Waypoint& waypoint)
{
    waypoint = m_outputWaypoint;

    /* Only valid if the state is active. */
    return m_isActive;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool DrivingState::setupPlatoonController()
{
    bool isSuccessful = false;

    ProcessingChainFactory& processingChainFactory = ProcessingChainFactory::getInstance();

    PlatoonController::InputWaypointCallback lambdaInputWaypointCallback = [this](Waypoint& waypoint)
    {
        bool isSuccessful = false;

        if (false == m_inputWaypointQueue.empty())
        {
            /* Retrieve next waypoint. */
            Waypoint* nextWaypoint = m_inputWaypointQueue.front();
            m_inputWaypointQueue.pop();

            /* Copy waypoint. */
            waypoint = *nextWaypoint;

            /* Delete queued waypoint. */
            delete nextWaypoint;

            isSuccessful = true;
        }

        return isSuccessful;
    };
    PlatoonController::OutputWaypointCallback lambdaOutputWaypointCallback = [this](const Waypoint& waypoint)
    {
        m_outputWaypoint = waypoint;
        return true;
    };
    PlatoonController::MotorSetpointCallback lambdaMotorSetpointCallback =
        [this](const int16_t left, const int16_t right) { return this->setMotorSpeedSetpoints(left, right); };

    processingChainFactory.registerLongitudinalControllerCreateFunc(LongitudinalController::create);
    processingChainFactory.registerLongitudinalSafetyPolicyCreateFunc(LongitudinalSafetyPolicy::create);
    processingChainFactory.registerLateralControllerCreateFunc(LateralController::create);
    processingChainFactory.registerLateralSafetyPolicyCreateFunc(LateralSafetyPolicy::create);

    if (false == m_platoonController.init(lambdaInputWaypointCallback, lambdaOutputWaypointCallback,
                                          lambdaMotorSetpointCallback))
    {
        LOG_FATAL("Could not initialize Platoon Controller.");
    }
    else
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
