/* MIT License
 *
 * Copyright (c) 2024 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Leader Longitudinal Controller.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "LongitudinalController.h"
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
    m_maxMotorSpeed(0),
    m_state(STATE_STARTUP),
    m_lastFollowerFeedback(),
    m_motorSetpointCallback(nullptr)
{
}

LongitudinalController::~LongitudinalController()
{
}

void LongitudinalController::process()
{
    switch (m_state)
    {
    case STATE_STARTUP:
        /* Max speed must be positive. */
        if (0 < m_maxMotorSpeed)
        {
            m_state = STATE_READY;
        }

        break;

    case STATE_READY:
        /* Wait for external release. */
        break;

    case STATE_DRIVING:
        /* Allow top motor speed calculation. */
        break;

    case STATE_SAFE:
        /* Stop the vehicle. Sent continously to prevent overwriting by other modules. */
        if (nullptr != m_motorSetpointCallback)
        {
            m_motorSetpointCallback(0);
        }

        break;

    default:
        /* Should never happen. */
        m_state = STATE_SAFE;
        break;
    }
}

void LongitudinalController::calculateTopMotorSpeed(const Waypoint& currentVehicleData)
{
    if (STATE_DRIVING == m_state)
    {
        /* TODO: Check follower feedback. Calculate platoon length and react accordingly. */

        /* TODO: Calculate top motor speed. */

        /* Send top motor speed. */
        if (nullptr != m_motorSetpointCallback)
        {
            m_motorSetpointCallback(m_maxMotorSpeed);
        }
    }
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
