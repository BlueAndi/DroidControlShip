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

bool Queuer::process()
{
    bool isSuccessful;

    if (false == m_IEQueue.empty())
    {
        /** Process the first element in queue */
        InfrastructureElement* selectedIE = m_IEQueue.front();
        m_IEQueue.pop();

        /** Set and process the current coordinates with the ones of the selected IE.*/
        Participant::getInstance().setRequiredOrientation(selectedIE->orientation);
        Participant::getInstance().setEntryValues(selectedIE->entryX, selectedIE->entryY);

        if (true == CoordinateHandler::getInstance().isMovingTowards())
        {
            LOG_DEBUG("Robot is moving towards %d.", selectedIE->entryX);

            if (true == CoordinateHandler::getInstance().checkOrientation())
            {
                LOG_DEBUG("Robot pointing towards %d.", selectedIE->entryX);

                if (CoordinateHandler::getInstance().getCurrentDistance() < 250)
                {
                    LOG_DEBUG("Robot is near %d, listening for signals.", selectedIE->entryX);
                    isSuccessful = true;
                }
                else
                {
                    isSuccessful = false;
                    LOG_DEBUG("Robot has some more driving to do.");
                }
            }
            else
            {
                isSuccessful = false;
                LOG_DEBUG("Robot isn't pointing towards %d.", selectedIE->entryX);
            }
        }
        else
        {
            LOG_DEBUG("Robot is moving away from %d.", selectedIE->entryX);
            isSuccessful = false;
        }

        /** Cycle queue. */
        m_IEQueue.push(selectedIE);
    }

    return isSuccessful;
}

bool Queuer::enqueueParticipant(InfrastructureElement* enqueuedParticipant)
{
    bool isSuccessful;

    if (enqueuedParticipant->name != "")
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
