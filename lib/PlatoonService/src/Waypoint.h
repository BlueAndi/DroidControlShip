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
 * @brief  Definition of a Waypoint.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup PlatoonService
 *
 * @{
 */
#ifndef WAYPOINT_H
#define WAYPOINT_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>
#include <WString.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Waypoint structure definition.
 * Defines the position of a waypoint in the map and the speed at which is to be reached.
 */
class Waypoint
{
public:
    int32_t xPos;        /**< X position [mm]. */
    int32_t yPos;        /**< Y position [mm]. */
    int32_t orientation; /**< Orientation [mrad]. */
    int16_t left;        /**< Left motor speed [steps/s]. */
    int16_t right;       /**< Right motor speed [steps/s]. */
    int16_t center;      /**< Center speed [steps/s]. */

    /**
     * Default constructor.
     */
    Waypoint() : Waypoint(0, 0, 0, 0, 0, 0)
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
     */
    Waypoint(int32_t xPos, int32_t yPos, int32_t orientation, int16_t left, int16_t right, int16_t center) :
        xPos(xPos),
        yPos(yPos),
        orientation(orientation),
        left(left),
        right(right),
        center(center)
    {
    }

    /**
     * Deserialize a waypoint.
     * The waypoint is created on the heap and must be deleted by the caller.
     *
     * @param[in]   serializedWaypoint  Serialized waypoint.
     *
     * @return Pointer to a waypoint object. In case of an error, it returns nullptr.
     */
    static Waypoint* deserialize(const String& serializedWaypoint);

    /**
     * Serialize the waypoint.
     *
     * @return Serialized waypoint. Returns an empty string in case of an error.
     */
    String serialize() const;

    /**
     * Print waypoint data to the serial console.
     * Uses the LOG_DEBUG macro, so it can be deactivated.
     */
    void debugPrint() const;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* WAYPOINT_H */
/** @} */
