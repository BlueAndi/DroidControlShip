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
 *
 * @addtogroup PlatoonService
 *
 * @{
 */
#ifndef PLATOON_UTILS_H
#define PLATOON_UTILS_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <Waypoint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

namespace PlatoonUtils
{
    /**
     * Calculate the absolute distance between two waypoints.
     *
     * @param[in] waypoint1    First waypoint.
     * @param[in] waypoint2    Second waypoint.
     *
     * @return Absolute distance between the two waypoints in mm. It is always positive.
     */
    int32_t calculateAbsoluteDistance(const Waypoint& waypoint1, const Waypoint& waypoint2);

    /**
     * Calculate the heading at which a target waypoint is located relative to other waypoint.
     * The result is the heading in mrad in the range [-PI, PI].
     *
     * @param[in]   targetWaypoint    Target waypoint.
     * @param[in]   referenceWaypoint Reference waypoint.
     * @param[out]  heading           Heading in mrad.
     *
     * @return If heading is valid, it will return true. Otherwise false.
     */
    bool calculateHeading(const Waypoint& targetWaypoint, const Waypoint& referenceWaypoint, int32_t& heading);

    /**
     * Calculate the heading at which a target waypoint is located relative to other waypoint.
     * Takes the orientation of the reference waypoint into account.
     * The result range is [-2PI, 2PI].
     *
     * @param[in] targetWaypoint    Target waypoint.
     * @param[in] referenceWaypoint Reference waypoint.
     * @param[out] relativeHeading  Heading in mrad in range [-2PI, 2PI].
     *
     * @return If heading is valid, it will return true. Otherwise false.
     */
    bool calculateRelativeHeading(const Waypoint& targetWaypoint, const Waypoint& referenceWaypoint,
                                  int32_t& relativeHeading);

    /**
     * Calculate the equivalent of a target heading which results in the shortest delta between two headings.
     *
     * @param[in]   targetHeading       Target heading in range [-2PI, 2PI].
     * @param[in]   referenceHeading    Reference heading in range [-2PI, 2PI].
     *
     * @return Equivalent to target heading in mrad in range [-3PI, 3PI].
     */
    int32_t calculateEquivalentHeading(int32_t targetHeading, int32_t referenceHeading);

    /**
     * Compare two waypoints and check if they are equal.
     * Two waypoints are equal if their x and y position are equal.
     * The orientation is not considered.
     * An error margin may be included in the comparison.
     *
     * @param[in] waypoint1    First waypoint.
     * @param[in] waypoint2    Second waypoint.
     * @param[in] errorMargin  Error margin in mm. Default is 0.
     *
     * @return If the waypoints are equal, it will return true. Otherwise false.
     */
    bool areWaypointsEqual(const Waypoint& waypoint1, const Waypoint& waypoint2, int32_t errorMargin = 0);

} // namespace PlatoonUtils

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* PLATOON_UTILS_H */
/** @} */
