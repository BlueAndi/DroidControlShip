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
 * @brief  Utilities for calculation of waypoints and related quantities.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "PlatoonUtils.h"
#include <Arduino.h>
#include <math.h>
#include <FPMath.h>

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

int32_t PlatoonUtils::calculateAbsoluteDistance(const Waypoint& waypoint1, const Waypoint& waypoint2)
{
    int32_t absoluteDistance = 0;
    int32_t distanceX        = waypoint1.xPos - waypoint2.xPos;
    int32_t distanceY        = waypoint1.yPos - waypoint2.yPos;

    float distanceFloat = sqrtf((distanceX * distanceX) + (distanceY * distanceY)) + 0.5F;
    absoluteDistance    = static_cast<int32_t>(distanceFloat);

    return absoluteDistance;
}

bool PlatoonUtils::calculateHeading(const Waypoint& targetWaypoint, const Waypoint& referenceWaypoint, int32_t& heading)
{
    bool isSuccessful = false;

    /* Delta position. */
    int32_t deltaX = targetWaypoint.xPos - referenceWaypoint.xPos;
    int32_t deltaY = targetWaypoint.yPos - referenceWaypoint.yPos;

    /* atan2(0,0) has undefined behavior. */
    if ((0 != deltaX) || (0 != deltaY))
    {
        /* Calculate target heading. */
        float angle = atan2(deltaY, deltaX) * 1000.0F;         /* Angle in mrad. */
        angle += (FP_2PI() + 0.5F);                            /* Shift angle by 360 degree. */
        heading      = static_cast<int32_t>(angle) % FP_2PI(); /* Fixed point heading. */
        isSuccessful = true;
    }

    return isSuccessful;
}

bool PlatoonUtils::calculateRelativeHeading(const Waypoint& targetWaypoint, const Waypoint& referenceWaypoint,
                                            int32_t& relativeHeading)
{
    int32_t absoluteHeading   = 0;
    bool    isSuccessful      = calculateHeading(targetWaypoint, referenceWaypoint, absoluteHeading);
    int32_t equivalentHeading = calculateEquivalentHeading(absoluteHeading, referenceWaypoint.orientation);

    relativeHeading = equivalentHeading - referenceWaypoint.orientation;

    return isSuccessful;
}

bool PlatoonUtils::areWaypointsEqual(const Waypoint& waypoint1, const Waypoint& waypoint2, int32_t errorMargin)
{
    int32_t differenceX = abs(waypoint1.xPos - waypoint2.xPos);
    int32_t differenceY = abs(waypoint1.yPos - waypoint2.yPos);

    return ((errorMargin >= differenceX) && (errorMargin >= differenceY));
}

int32_t PlatoonUtils::calculateEquivalentHeading(int32_t targetHeading, int32_t referenceHeading)
{
    int32_t equivalentHeading = targetHeading % FP_2PI();

    if (FP_2PI() == referenceHeading)
    {
        equivalentHeading += FP_2PI();
    }
    else if (-FP_2PI() == referenceHeading)
    {
        equivalentHeading -= FP_2PI();
    }

    /* Calculate delta between the two headings. */
    int32_t absoluteDelta = equivalentHeading - referenceHeading;

    /* Heading are equal. */
    if (0 == absoluteDelta)
    {
        /* Nothing to do. */
    }
    /* Current heading is positive and delta is negative. */
    else if ((absoluteDelta < 0) && (0 <= referenceHeading))
    {
        /* Relative delta when going counter-clockwise. */
        int32_t deltaCounterClockwise = (equivalentHeading + FP_2PI()) - referenceHeading;

        /* Relative delta when going clockwise. */
        int32_t deltaClockwise = referenceHeading - equivalentHeading;

        /* Choose the shortest path. */
        if (deltaCounterClockwise < deltaClockwise)
        {
            equivalentHeading += FP_2PI();
        }
    }
    /* Current heading is positive and delta is positive. */
    else if ((absoluteDelta > 0) && (0 <= referenceHeading))
    {
        /* Relative delta when going counter-clockwise. */
        int32_t deltaCounterClockwise = equivalentHeading - referenceHeading;

        /* Relative delta when going clockwise. */
        int32_t deltaClockwise = referenceHeading + FP_2PI() - equivalentHeading;

        /* Choose the shortest path. */
        if (deltaCounterClockwise > deltaClockwise)
        {
            equivalentHeading -= FP_2PI();
        }
    }
    /* Current heading is negative and delta is negative. */
    else if ((absoluteDelta < 0) && (0 > referenceHeading))
    {
        /* Relative delta when going counter-clockwise. */
        int32_t deltaCounterClockwise = equivalentHeading + FP_2PI() - referenceHeading;

        /* Relative delta when going clockwise. */
        int32_t deltaClockwise = referenceHeading - equivalentHeading;

        /* Choose the shortest path. */
        if (deltaCounterClockwise < deltaClockwise)
        {
            equivalentHeading += FP_2PI();
        }
    }
    /* Current heading is negative and delta is positive. */
    else if ((absoluteDelta > 0) && (0 > referenceHeading))
    {
        /* Relative delta when going counter-clockwise. */
        int32_t deltaCounterClockwise = equivalentHeading - referenceHeading;

        /* Relative delta when going clockwise. */
        int32_t deltaClockwise = referenceHeading + FP_2PI() - equivalentHeading;

        /* Choose the shortest path. */
        if (deltaCounterClockwise > deltaClockwise)
        {
            equivalentHeading -= FP_2PI();
        }
    }

    return equivalentHeading;
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
