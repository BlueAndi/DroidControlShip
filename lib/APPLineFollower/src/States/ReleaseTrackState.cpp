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
 * @brief  Release track state
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "ReleaseTrackState.h"
#include <Board.h>
#include <StateMachine.h>
#include <Util.h>
#include <Logging.h>

#include "DrivingState.h"
#include "ParameterSets.h"

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

void ReleaseTrackState::entry()
{
    LOG_INFO("Release track state entered.");

    /* Start challenge after specific time. */
    m_releaseTimer.start(TRACK_RELEASE_DURATION);

    /* Choose parameter set 0 by default. */
    ParameterSets::getInstance().choose(0);
    showParSet();
}

void ReleaseTrackState::process(StateMachine& sm)
{
    IButton& button = Board::getInstance().getButton();

    /* Change parameter set? */
    if (true == Util::isButtonTriggered(button, m_isButtonPressed))
    {
        /* Choose next parameter set (round-robin). */
        ParameterSets::getInstance().next();
        showParSet();

        m_releaseTimer.restart();
    }

    /* Release track after specific time. */
    if (true == m_releaseTimer.isTimeout())
    {
        sm.setState(DrivingState::getInstance());
    }
}

void ReleaseTrackState::exit()
{
    m_releaseTimer.stop();
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void ReleaseTrackState::showParSet() const
{
    uint8_t     parSetId   = ParameterSets::getInstance().getCurrentSetId();
    const char* parSetName = ParameterSets::getInstance().getParameterSet().name;

    LOG_INFO("Parameter set %d: %s", parSetId, parSetName);
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
