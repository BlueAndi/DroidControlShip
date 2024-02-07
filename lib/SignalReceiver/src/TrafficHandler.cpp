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
 * @brief  Traffic handler implementation
 * @author Paul Gramescu <paul.gramescu@gmail.com>
 *
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "TrafficHandler.h"

#include <Board.h>
#include <MqttClient.h>
#include <DrivingState.h>

#include <Logging.h>
#include <CoordinateHandler.cpp>

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

bool TrafficHandler::process()
{
    bool isSuccessful = false;

    if (true == processList())
    {
        isSuccessful = true;
    }

    return isSuccessful;
}

void TrafficHandler::processColor()
{
    /* Trigger different functions based on what color the robot received. */
    switch (m_colorID)
    {
    case 0:
    {
        /* No new color received. */
        break;
    }
    case 1:
    {
        LOG_DEBUG("COLOR 1");
        DrivingState::getInstance().setMaxMotorSpeed(0);
        break;
    }
    case 2:
    {
        LOG_DEBUG("COLOR 2");
        /* Keep driving. */
        DrivingState::getInstance().setMaxMotorSpeed(1000);
        break;
    }
    case 3:
    {
        LOG_DEBUG("COLOR 3");
        DrivingState::getInstance().setMaxMotorSpeed(500);
        break;
    }
    case 4:
    {
        LOG_DEBUG("COLOR 4");
        DrivingState::getInstance().setMaxMotorSpeed(500);
        break;
    }
    default:
        /* Fatal error */
        break;
    }
}

bool TrafficHandler::processList()
{
    bool isProcessed;

    /** Process the list of IEs. */
    for (int i = 0; i < m_IECounter; i++)
    {
        /** Recalculate distances. */
        listOfElements[i].setPreviousDistance(listOfElements[i].getDistance());
        listOfElements[i].setDistance(CoordinateHandler::getInstance().calculateDistance(
            listOfElements[i].getEntryX(), listOfElements[i].getEntryY()));

        /** Process new coordinates. */
        CoordinateHandler::getInstance().process(
            listOfElements[i].getIEName(), listOfElements[i].getRequiredOrientation(), listOfElements[i].getDistance(),
            listOfElements[i].getPreviousDistance());

        /** Set new robot-IE status after processing. */
        listOfElements[i].setStatus(CoordinateHandler::getInstance().getStatus());

        /** Log status, distances and what IE has been processed. */
        LOG_DEBUG("Status %d | Distance %d & Old Distance %d to %s", listOfElements[i].getStatus(),
                  listOfElements[i].getDistance(), listOfElements[i].getPreviousDistance(),
                  listOfElements[i].getIEName().c_str());

        isProcessed = true;
    }

    return true;
}

bool TrafficHandler::setNewInfrastructureElement(const String& nameAsParameter, int32_t orientationAsParameter,
                                                 int32_t xPosAsParameter, int32_t yPosAsParameter,
                                                 int32_t       defaultDistanceAsParameter,
                                                 int32_t       defaultPreviousDistanceAsParameter,
                                                 const String& topicNameAsParameter)
{
    bool isSuccessful = false;

    /** Setting a new IE if unique and increasing the list for each new IE settings received. */
    for (int CURRENT_ELEMENT = 0; CURRENT_ELEMENT < MAX_ELEMENTS; CURRENT_ELEMENT++)
    {
        /** If IE name isn't empty, add it to the list. */
        if (true == listOfElements[CURRENT_ELEMENT].isEmpty())
        {
            listOfElements[CURRENT_ELEMENT].setIEName(nameAsParameter);
            listOfElements[CURRENT_ELEMENT].setTopicName(topicNameAsParameter);
            listOfElements[CURRENT_ELEMENT].setRequiredOrientation(orientationAsParameter);
            listOfElements[CURRENT_ELEMENT].setEntryValues(xPosAsParameter, yPosAsParameter);

            /** Distances should be 0 when first listed. */
            listOfElements[CURRENT_ELEMENT].setDistance(defaultDistanceAsParameter);
            listOfElements[CURRENT_ELEMENT].setPreviousDistance(defaultPreviousDistanceAsParameter);

            LOG_DEBUG("Listed %s", listOfElements[CURRENT_ELEMENT].getIEName().c_str());

            isSuccessful = true;
            m_IECounter++;
            break;
        }
        else
        {
            if (nameAsParameter == listOfElements[CURRENT_ELEMENT].getIEName())
            {
                isSuccessful = false;
                LOG_DEBUG("Received the same IE settings twice, listing it once.");
                break;
            }
        }
    }

    return isSuccessful;
}

bool TrafficHandler::checkLockIn()
{
    bool isTrue = false;

    for (int i = 0; i < m_IECounter; i++)
    {
        if ((listOfElements[i].getStatus() == 2) || (listOfElements[i].getStatus() == 3))
        {
            if (listOfElements[i].getTopicName() != nullptr)
            {
                lockedOnto = listOfElements[i].getTopicName();

                LOG_DEBUG("Locked onto %s", listOfElements[i].getIEName().c_str());
                isTrue = true;
                break;
            }
            else
            {
                LOG_WARNING("Empty topic, did not lock onto anything!");
            }
        }
        else
        {
            isTrue = false;
        }
    }

    return isTrue;
}

bool TrafficHandler::isNear()
{
    bool isTrue = false;

    for (int i = 0; i < m_IECounter; i++)
    {
        if (listOfElements[i].getStatus() == 3)
        {
            isTrue = true;
            break;
        }
        else
        {
            isTrue = false;
        }
    }

    return isTrue;
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
