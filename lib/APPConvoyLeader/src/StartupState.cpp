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
 * @brief  Startup State.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "StartupState.h"
#include "DrivingState.h"
#include <SettingsHandler.h>

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
    m_isActive              = true;
    m_pendingCommandCounter = CMD_GET_MAX_SPEED;
}

void StartupState::process(StateMachine& sm)
{
    if (CMD_NONE == m_pendingCommandCounter)
    {
        /* All commands processed. Switch to idle state. */
        sm.setState(DrivingState::getInstance());
    }
}

void StartupState::exit()
{
    m_isActive = false;
}

bool StartupState::getPendingCommand(Command& cmd)
{
    bool             isPending = false;
    SettingsHandler& settings  = SettingsHandler::getInstance();

    if (true == m_isActive)
    {
        switch (m_pendingCommandCounter)
        {
        case CMD_GET_MAX_SPEED:
            cmd.commandId = SMPChannelPayload::CMD_ID_GET_MAX_SPEED;
            isPending     = true;
            break;

        case CMD_SET_INIT_POS:
            cmd.commandId   = SMPChannelPayload::CMD_ID_SET_INIT_POS;
            cmd.xPos        = settings.getInitialXPosition();
            cmd.yPos        = settings.getInitialYPosition();
            cmd.orientation = settings.getInitialHeading();
            isPending       = true;
            break;

        default:
            break;
        }
    }

    return isPending;
}

void StartupState::notifyCommandProcessed()
{
    if (true == m_isActive)
    {
        ++m_pendingCommandCounter;
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
