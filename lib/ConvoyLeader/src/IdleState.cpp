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
 * @brief  Idle State.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "IdleState.h"
#include "DrivingState.h"
#include "RemoteControl.h"
#include <new>

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

void IdleState::entry()
{
    m_isActive            = true;
    m_isReleaseSuccessful = false;

    if (nullptr != m_pendingCommand)
    {
        delete m_pendingCommand;
        m_pendingCommand = nullptr;
    }
}

void IdleState::process(StateMachine& sm)
{
    if (true == m_isReleaseSuccessful)
    {
        /* Release is successful. Switch to driving state. */
        sm.setState(&DrivingState::getInstance());
    }
}

void IdleState::exit()
{
    m_isActive = false;
}

Command* IdleState::getPendingCommand()
{
    return (true == m_isActive) ? m_pendingCommand : nullptr;
}

void IdleState::notifyCommandProcessed()
{
    if (nullptr != m_pendingCommand)
    {
        delete m_pendingCommand;
        m_pendingCommand      = nullptr;
        m_isReleaseSuccessful = true;
    }
}

bool IdleState::requestRelease()
{
    bool isSuccessful = false;

    if (true == m_isActive)
    {
        if (nullptr != m_pendingCommand)
        {
            delete m_pendingCommand;
            m_pendingCommand = nullptr;
        }

        m_pendingCommand = new (std::nothrow) Command{RemoteControl::CMD_ID_START_DRIVING};

        if (nullptr != m_pendingCommand)
        {
            isSuccessful = true;
        }
    }

    return isSuccessful;
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
