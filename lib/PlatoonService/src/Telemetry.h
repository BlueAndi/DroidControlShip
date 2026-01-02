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
 * @file
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
    /**
     * Range in which a detected object may be.
     * Equivalent to the brightness levels.
     * Values estimated from user's guide.
     */
    typedef enum : uint8_t
    {
        RANGE_NO_OBJECT = 0U, /**< No object detected */
        RANGE_25_30,          /**< Object detected in range 25 to 30 cm */
        RANGE_20_25,          /**< Object detected in range 20 to 25 cm */
        RANGE_15_20,          /**< Object detected in range 15 to 20 cm */
        RANGE_10_15,          /**< Object detected in range 10 to 15 cm */
        RANGE_5_10,           /**< Object detected in range 5 to 10 cm */
        RANGE_0_5             /**< Object detected in range 0 to 5 cm */

    } Range; /**< Proximity Sensor Ranges */

    int32_t          xPos;        /**< X position [mm]. */
    int32_t          yPos;        /**< Y position [mm]. */
    int32_t          orientation; /**< Orientation [mrad]. */
    int32_t          left;        /**< Left motor speed [mm/s]. */
    int32_t          right;       /**< Right motor speed [mm/s]. */
    int32_t          center;      /**< Center speed [mm/s]. */
    Telemetry::Range proximity;   /**< Range at which object is found [range]. */

    /**
     * Default constructor.
     */
    Telemetry() : Telemetry(0, 0, 0, 0, 0, 0, Telemetry::Range::RANGE_NO_OBJECT)
    {
    }

    /**
     * Constructor
     *
     * @param[in] xPos          X position [mm].
     * @param[in] yPos          Y position [mm].
     * @param[in] orientation   Orientation [mrad].
     * @param[in] left          Left motor speed [mm/s].
     * @param[in] right         Right motor speed [mm/s].
     * @param[in] center        Center speed [mm/s].
     * @param[in] proximity     Range at which object is found [range].
     */
    Telemetry(int32_t xPos, int32_t yPos, int32_t orientation, int32_t left, int32_t right, int32_t center,
              Telemetry::Range proximity) :
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
     * Copy constructor
     *
     * @param[in] telemetry Telemetry data.
     */
    Telemetry(const Telemetry& telemetry) :
        xPos(telemetry.xPos),
        yPos(telemetry.yPos),
        orientation(telemetry.orientation),
        left(telemetry.left),
        right(telemetry.right),
        center(telemetry.center),
        proximity(telemetry.proximity)
    {
    }

    /**
     * Assignment operator
     *
     * @param[in] telemetry Telemetry data.
     *
     * @returns Reference to telemetry data.
     */
    Telemetry& operator=(const Telemetry& telemetry)
    {
        if (this != &telemetry)
        {
            xPos        = telemetry.xPos;
            yPos        = telemetry.yPos;
            orientation = telemetry.orientation;
            left        = telemetry.left;
            right       = telemetry.right;
            center      = telemetry.center;
            proximity   = telemetry.proximity;
        }

        return *this;
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
