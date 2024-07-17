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
 * @brief  Global Positioning Sensor (GPS) module.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "Gps.h"

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

/** Index of the x-coordinate in the position vector. */
static const size_t GPS_POSITION_X_INDEX = 0U;

/** Index of the y-coordinate in the position vector. */
static const size_t GPS_POSITION_Y_INDEX = 1U;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

Gps::Gps(webots::GPS* gps) : IGps(), m_gps(gps)
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
            xPos            = static_cast<int32_t>((positionVector[GPS_POSITION_X_INDEX] * 1000.0F) + 1.0F);
            yPos            = static_cast<int32_t>((positionVector[GPS_POSITION_Y_INDEX] * 1000.0F) + 1.0F);
            isPositionValid = true;
        }
    }

    return isPositionValid;
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
