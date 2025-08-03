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
 * @brief  GPS Interface.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALInterfaces
 *
 * @{
 */
#ifndef IGPS_H
#define IGPS_H

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

/** The abstract GPS interface. */
class IGps
{
public:
    /**
     * Destroys the interface.
     */
    virtual ~IGps()
    {
    }

    /**
     * Get the current position coordinates in millimeters.
     *
     * @param[out] xPos  The X position in mm.
     * @param[out] yPos  The Y position in mm.
     *
     * @return If the position was successfully retrieved, return true. Otherwise, false.
     */
    virtual bool getPosition(int32_t& xPos, int32_t& yPos) = 0;

    /**
     * Get the current orientation in mrad.
     *
     * @param[out] orientation The orientation in mrad.
     *
     * @return If the orientation was successfully retrieved, return true. Otherwise, false.
     */
    virtual bool getOrientation(int32_t& orientation) = 0;

protected:
    /**
     * Construct a GPS interface.
     */
    IGps()
    {
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* IGPS_H */
/** @} */
