/* MIT License
 *
 * Copyright (c) 2023 - 2025 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Global Positioning Sensor (GPS) module.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "Gps.h"
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

/** Index of the x-component in a vector. */
static const size_t X_COMPONENT_INDEX = 0U;

/** Index of the y-component in a vector. */
static const size_t Y_COMPONENT_INDEX = 1U;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

Gps::Gps(webots::GPS* gps, webots::Compass* compass) : IGps(), m_gps(gps), m_compass(compass)
{
}

Gps::~Gps()
{
}

bool Gps::getPosition(int32_t& xPos, int32_t& yPos)
{
    bool isPositionValid = false;

    if (nullptr != m_gps)
    {
        /* Get 3D position vector from Webots in meters. */
        const double* positionVector = m_gps->getValues();

        if (nullptr != positionVector)
        {
            /* Cast to millimeters. */
            xPos            = static_cast<int32_t>((positionVector[X_COMPONENT_INDEX] * 1000.0F) + 1.0F);
            yPos            = static_cast<int32_t>((positionVector[Y_COMPONENT_INDEX] * 1000.0F) + 1.0F);
            isPositionValid = true;
        }
    }

    return isPositionValid;
}

bool Gps::getOrientation(int32_t& orientation)
{
    bool isOrientationValid = false;

    if (nullptr != m_compass)
    {
        /* Get vector that points to the "North" direction. */
        const double* orientationVector = m_compass->getValues();

        if (nullptr != orientationVector)
        {
            double orientationComponentX = orientationVector[X_COMPONENT_INDEX];
            double orientationComponentY = orientationVector[Y_COMPONENT_INDEX];

            /* atan2(0,0) has undefined behavior. */
            if ((0 != orientationComponentX) || (0 != orientationComponentY))
            {
                /* Calculate target heading. */
                float angle = atan2(orientationComponentY, orientationComponentX) * 1000.0F; /* Angle in mrad. */
                orientation = static_cast<int32_t>(angle + 0.5F) % FP_2PI();                 /* Fixed point heading. */
                orientation -= ENU_CORRECTION_ANGLE; /* Correct the angle for ENU convention. */
                orientation *= -1;                   /* Invert the orientation for right-hand rule convention. */
                isOrientationValid = true;
            }
        }
    }

    return isOrientationValid;
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
