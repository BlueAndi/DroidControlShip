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
 * @brief  Definition of a Waypoint.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Service
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
typedef struct _Waypoint
{
    int32_t xPos        = 0; /**< X position [mm]. */
    int32_t yPos        = 0; /**< Y position [mm]. */
    int32_t orientation = 0; /**< Orientation [mrad]. */
    int16_t left        = 0; /**< Left motor speed [steps/s]. */
    int16_t right       = 0; /**< Right motor speed [steps/s]. */
    int16_t center      = 0; /**< Center speed [steps/s]. */

    /**
     * Waypoint assignment operator.
     */
    void operator=(const _Waypoint& other)
    {
        xPos        = other.xPos;
        yPos        = other.yPos;
        orientation = other.orientation;
        left        = other.left;
        right       = other.right;
        center      = other.center;
    }

} __attribute__((packed)) Waypoint;

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* WAYPOINT_H */
/** @} */
