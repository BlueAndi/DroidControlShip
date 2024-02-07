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
 * @brief  Coordinate handler implementation
 * @author Paul Gramescu <paul.gramescu@gmail.com>
 *
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "CoordinateHandler.h"
#include <Board.h>
#include <Logging.h>
#include <math.h>

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

bool CoordinateHandler::process(const String& ieName, int32_t ieOrientation, int32_t distanceToIE,
                                int32_t previousDistanceToIE)

{
    bool isSuccessful;

    /* The default status. */
    m_currentStatus = STATUS_IDLE;

    if (true == CoordinateHandler::getInstance().isMovingTowards(distanceToIE, previousDistanceToIE))
    {
        m_currentStatus = STATUS_TOWARDS;

        if (true == CoordinateHandler::getInstance().checkOrientation(ieOrientation))
        {
            /* Robot is moving towards the IE AND has matching orientation, lock in! */
            m_currentStatus = STATUS_LOCKED_IN;

            if (m_distance < 150)
            {
                m_currentStatus = STATUS_NEAR;
                isSuccessful    = true;
            }
            else
            {
                m_currentStatus = STATUS_LOCKED_IN;
                isSuccessful    = false;
            }
        }
        else
        {
            m_currentStatus = STATUS_TOWARDS;
            isSuccessful    = false;
        }
    }
    else
    {
        m_currentStatus = STATUS_PASSED;
        isSuccessful    = false;
    }

    return isSuccessful;
}

int32_t CoordinateHandler::calculateDistance(int32_t xPos, int32_t yPos)
{
    int32_t segmentX = xPos - m_currentX;
    int32_t segmentY = yPos - m_currentY;

    float distanceFloat = sqrtf((segmentX * segmentX) + (segmentY * segmentY)) + 0.5F;

    m_distance = static_cast<int32_t>(distanceFloat);

    return m_distance;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool CoordinateHandler::isMovingTowards(int32_t currentDistance, int32_t previousDistance)
{
    bool isTrue = false;
    if (currentDistance <= 700)
    {

        if (currentDistance <= previousDistance)
        {
            /* Distance is decreasing, robot is moving towards. */
            isTrue = true;
        }
        else
        {
            /* Distance is increasing, robot is moving away. */
            isTrue = false;
        }
    }
    else
    {
        /* Robot is too far away. */
        isTrue = false;
    }

    return isTrue;
}

bool CoordinateHandler::checkOrientation(int32_t orientationIE)
{
    bool isTrue = false;

    if (orientationIE != 0)
    {
        if (((orientationIE - 500) <= getCurrentOrientation()) && (getCurrentOrientation() <= (orientationIE + 500)))
        {
            isTrue = true;
        }
        else
        {
            isTrue = false;
        }
    }
    else
    {
        /* If there is no orientation set, set it to always be true. */
        isTrue = true;
    }

    return isTrue;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
