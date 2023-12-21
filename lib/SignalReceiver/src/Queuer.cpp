/* MIT License
 *
 * Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Queue implementation
 * @author Paul Gramescu <paul.gramescu@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Queuer.h"

#include <Board.h>

#include <Logging.h>
#include <CoordinateHandler.h>

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

bool Queuer::isFull()
{
    bool isTrue;

    return isTrue;
}

void Queuer::process()
{
    if (true == m_IEQueue.empty())
    {
        InfrastructureElement* selectedIE = m_IEQueue.front();

        CoordinateHandler::getInstance().process(selectedIE->orientation, selectedIE->intervX, selectedIE->intervY,
                                                 selectedIE->entryX, selectedIE->entryY);
    }
}

bool Queuer::enqueueParticipant(InfrastructureElement* enqueuedParticipant)
{
    bool isSuccessful;

    if ((enqueuedParticipant != nullptr) || (enqueuedParticipant->name != ""))
    {
        m_IEQueue.push(enqueuedParticipant);
        LOG_DEBUG("Added %s to the list.", enqueuedParticipant->name);

        isSuccessful = true;
    }
    else
    {
        LOG_ERROR("Invalid name received, participant has not been listed!");

        isSuccessful = false;
    }

    return isSuccessful;
}

void Queuer::dequeueParticipant()
{
    if (m_IEQueue.empty() == false)
    {
        m_IEQueue.pop();
    }
    else
    {
        LOG_DEBUG("Queue is empty.");
    }
}

void Queuer::addParticipant(InfrastructureElement infrastructureElement)
{
    /** If new settings have been received, process them as a new traffic participant. */
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
