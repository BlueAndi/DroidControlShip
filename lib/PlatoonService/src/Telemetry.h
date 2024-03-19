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
 * @brief  Telemetry struct definition.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup PlatoonService
 *
 * @{
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>
#include "Waypoint.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Vehicle Data struct definition.
 */
struct Telemetry
{
    int32_t xPos;        /**< X position [mm]. */
    int32_t yPos;        /**< Y position [mm]. */
    int32_t orientation; /**< Orientation [mrad]. */
    int16_t left;        /**< Left motor speed [steps/s]. */
    int16_t right;       /**< Right motor speed [steps/s]. */
    int16_t center;      /**< Center speed [steps/s]. */
    uint8_t proximity;   /**< Range at which object is found [range]. */

    /**
     * Default constructor.
     */
    Telemetry() : Telemetry(0, 0, 0, 0, 0, 0, 0)
    {
    }

    /**
     * Constructor
     *
     * @param[in] xPos          X position [mm].
     * @param[in] yPos          Y position [mm].
     * @param[in] orientation   Orientation [mrad].
     * @param[in] left          Left motor speed [steps/s].
     * @param[in] right         Right motor speed [steps/s].
     * @param[in] center        Center speed [steps/s].
     * @param[in] proximity     Range at which object is found [range].
     */
    Telemetry(int32_t xPos, int32_t yPos, int32_t orientation, int16_t left, int16_t right, int16_t center,
              uint8_t proximity) :
        xPos(xPos),
        yPos(yPos),
        orientation(orientation),
        left(left),
        right(right),
        center(center),
        proximity(proximity)
    {
    }

    /**
     * Get vehicle data as Waypoint.
     *
     * @return Vehicle data as Waypoint.
     */
    Waypoint asWaypoint() const
    {
        return Waypoint(xPos, yPos, orientation, left, right, center);
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* TELEMETRY_H */
/** @} */
