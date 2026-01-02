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
 * @file
 * @brief  Startup state
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "StartupState.h"
#include "States/LineSensorsCalibrationState.h"
#include "States/ErrorState.h"
#include <Logging.h>
#include <Board.h>
#include <Util.h>

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

void StartupState::entry()
{
    LOG_INFO("Startup state entered.");

    m_isButtonPressed = false;
}

void StartupState::process(StateMachine& sm)
{
    IButton& button = Board::getInstance().getButton();

    switch (m_subState)
    {
    case SUB_STATE_WAIT_FOR_SYNC:
        if (true == m_serMuxChannelProvider->isInSync())
        {
            LOG_INFO("Synchronization with RU established.");

            SerMuxChannelProvider::MaxMotorSpeedFunc maxMotorSpeedFunc = [this](SMPChannelPayload::RspId status,
                                                                                int16_t maxMotorSpeed) -> void
            {
                if (nullptr == m_motors)
                {
                    m_subState = SUB_STATE_ERROR;
                }
                else if (SMPChannelPayload::RSP_ID_PENDING == status)
                {
                    /* Keep on waiting. */
                    ;
                }
                else if (SMPChannelPayload::RSP_ID_OK == status)
                {
                    m_motors->setMaxSpeed(maxMotorSpeed);

                    LOG_INFO("Press button to start line sensors calibration.");
                    m_subState = SUB_STATE_FINISHED;
                }
                else
                {
                    m_subState = SUB_STATE_ERROR;
                }
            };

            if (false == m_serMuxChannelProvider->requestMaxMotorSpeed(maxMotorSpeedFunc))
            {
                LOG_ERROR("Failed to request max. motor speed from RU.");
                m_subState = SUB_STATE_ERROR;
            }
            else
            {
                m_timer.start(TIMEOUT_MS);
                m_subState = SUB_STATE_WAIT_FOR_MAX_MOTOR_SPEED;
            }
        }
        break;

    case SUB_STATE_WAIT_FOR_MAX_MOTOR_SPEED:
        /* Wait for max. motor speed request response.
         * Do nothing, the callback will change the sub state.
         */
        if (true == m_timer.isTimeout())
        {
            LOG_WARNING("Timeout while waiting for max. motor speed request response.");
            m_subState = SUB_STATE_WAIT_FOR_SYNC;
        }
        break;

    case SUB_STATE_FINISHED:
        if (true == Util::isButtonTriggered(button, m_isButtonPressed))
        {
            sm.setState(LineSensorsCalibrationState::getInstance());
        }
        break;

    case SUB_STATE_ERROR:
        sm.setState(ErrorState::getInstance());
        break;

    default:
        /* Do nothing. */
        break;
    }
}

void StartupState::exit()
{
    m_timer.stop();
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
