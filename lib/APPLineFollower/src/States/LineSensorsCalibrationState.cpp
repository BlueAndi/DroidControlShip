/* MIT License
 *
 * Copyright (c) 2023 - 2025 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Line sensors calibration state
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "LineSensorsCalibrationState.h"
#include <StateMachine.h>
#include <Logging.h>
#include "ReadyState.h"
#include "ErrorState.h"

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

void LineSensorsCalibrationState::entry()
{
    LOG_INFO("Line sensors calibration state entered.");

    if (nullptr == m_serMuxChannelProvider)
    {
        m_isError = true;
    }
    else
    {
        SerMuxChannelProvider::LineSensorCalibFunc lineSensorCalibFunc = [this]() -> void
        {
            LOG_INFO("Line sensor calibration started.");
            this->m_timer.start(TIMEOUT_MS);
        };

        if (false == m_serMuxChannelProvider->requestLineSensorCalibration(lineSensorCalibFunc))
        {
            LOG_ERROR("Failed to request line sensor calibration.");
            m_isError = true;
        }
        else
        {
            m_isError = false;
        }
    }
}

void LineSensorsCalibrationState::process(StateMachine& sm)
{
    if (true == m_isError)
    {
        sm.setState(ErrorState::getInstance());
    }
    else if (true == m_timer.isTimeout())
    {
        sm.setState(ReadyState::getInstance());
    }
    else
    {
        /* Nothing to do, wait for line sensor calibration to complete. */
    }
}

void LineSensorsCalibrationState::exit()
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
