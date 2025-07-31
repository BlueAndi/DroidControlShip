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
 * @brief  Ready state
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "ReadyState.h"
#include <Board.h>
#include <StateMachine.h>
#include "ReleaseTrackState.h"
#include "ErrorState.h"
#include <Logging.h>
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

void ReadyState::entry()
{
    const int32_t SENSOR_VALUE_OUT_PERIOD = 1000; /* ms */

    LOG_INFO("Ready state entered.");
    LOG_INFO("Press button to release track.");
    LOG_INFO("Press several times to configure parameter set.");

    /* The line sensor value shall be output on console cyclic. */
    m_timer.start(SENSOR_VALUE_OUT_PERIOD);
}

void ReadyState::process(StateMachine& sm)
{
    IButton& button = Board::getInstance().getButton();

    if (nullptr == m_lineSensors)
    {
        sm.setState(ErrorState::getInstance());
    }
    /* Shall track be released? */
    else if (true == Util::isButtonTriggered(button, m_isButtonPressed))
    {
        sm.setState(ReleaseTrackState::getInstance());
    }
    /* Shall the line sensor values be printed out on console? */
    else if (true == m_timer.isTimeout())
    {
        uint8_t         index        = 0;
        int16_t         position     = m_lineSensors->readLine();
        const uint16_t* sensorValues = m_lineSensors->getSensorValues();
        char            valueStr[10];

        /* Print line sensor value on console for debug purposes. */
        LOG_DEBUG("%u, %u, %u, %u, %u, pos=%d",
                  sensorValues[0],
                  sensorValues[1],
                  sensorValues[2],
                  sensorValues[3],
                  sensorValues[4],
                  position);

        m_timer.restart();
    }
    else
    {
        /* Nothing to do. */
        ;
    }
}

void ReadyState::exit()
{
    m_timer.stop();
    m_isLapTimeAvailable = false;
}

void ReadyState::setLapTime(uint32_t lapTime)
{
    m_isLapTimeAvailable = true;
    m_lapTime            = lapTime;
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
