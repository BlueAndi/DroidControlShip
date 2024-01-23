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
 * @brief  Startup State.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "StartupState.h"
#include "RemoteControl.h"
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
    m_isActive = true;
}

void StartupState::process(StateMachine& sm)
{

    SettingsHandler& settings = SettingsHandler::getInstance();

    switch (m_pendingCommandCounter)
    {
    case CMD_GET_MAX_SPEED:
        if (nullptr == m_pendingCommand)
        {
            /* Create new pending commmand. */
            m_pendingCommand = new Command{RemoteControl::CMD_ID_GET_MAX_SPEED};
        }
        else
        {
            /* Command is still pending. */
        }
        break;

    case CMD_SET_INIT_POS:
        if (nullptr == m_pendingCommand)
        {
            /* Create new pending commmand. */
            m_pendingCommand              = new Command;
            m_pendingCommand->commandId   = RemoteControl::CMD_ID_SET_INIT_POS;
            m_pendingCommand->xPos        = settings.getInitialXPosition();
            m_pendingCommand->yPos        = settings.getInitialYPosition();
            m_pendingCommand->orientation = settings.getInitialHeading();
        }
        else
        {
            /* Command is still pending. */
        }
        break;

    case CMD_NONE:
        /* All commands processed. Switch to idle state. */
        // sm.setState(&IdleState::getInstance());
        break;

    default:
        break;
    }
}

void StartupState::exit()
{
    /* Ensure the pending command is deleted. */
    if (nullptr != m_pendingCommand)
    {
        delete m_pendingCommand;
        m_pendingCommand = nullptr;
    }

    m_isActive = false;
}

Command* StartupState::getPendingCommand()
{
    return (true == m_isActive) ? m_pendingCommand : nullptr;
}

void StartupState::notifyCommandProcessed()
{
    if (nullptr != m_pendingCommand)
    {
        delete m_pendingCommand;
        m_pendingCommand = nullptr;
        m_pendingCommandCounter++;
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
