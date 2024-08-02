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
 *
 * @addtogroup HALSim
 *
 * @{
 */
#ifndef GPS_H
#define GPS_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <IGps.h>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Global Positioning Sensor (GPS) module.
 */
class Gps : public IGps
{
public:
    /**
     * Construct a GPS module.
     *
     * @param[in] gps Webots GPS device.
     * @param[in] compass Webots compass device.
     */
    Gps(webots::GPS* gps, webots::Compass* compass);

    /**
     * Destruct the GPS module.
     */
    ~Gps();

    /**
     * Get the current position coordinates in millimeters.
     *
     * @param[out] xPos  The X position in mm.
     * @param[out] yPos  The Y position in mm.
     *
     * @return If the position was successfully retrieved.
     */
    bool getPosition(int32_t& xPos, int32_t& yPos) final;

    /**
     * Get the current orientation in mrad.
     *
     * @param[out] orientation The orientation in mrad.
     *
     * @return If the orientation was successfully retrieved, return true. Otherwise, false.
     */
    bool getOrientation(int32_t& orientation) final;

private:
    webots::GPS*     m_gps;     /**< Webots GPS device */
    webots::Compass* m_compass; /**< Webots compass device */

    /**
     * Correction Angle for ENU world coordinate system.
     * The ENU world coordinate system is used in Webots.
     * East is along the x-axis, north is along the y-axis, and up is along the z-axis.
     * Compass points to the north in the ENU world coordinate system,
     * but the robot's system is oriented to the east.
     * The angle is in mrad.
     */
    static const uint32_t ENU_CORRECTION_ANGLE = 1587U;

    /**
     * Default constructor.
     * Not allowed.
     */
    Gps();

    /**
     * Copy construction of an instance.
     * Not allowed.
     *
     * @param[in] src Source instance.
     */
    Gps(const Gps& src);

    /**
     * Assignment operation.
     * Not allowed.
     *
     * @param[in] src Source instance.
     *
     * @returns Reference to GPS instance.
     */
    Gps& operator=(const Gps& src);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* GPS_H */
/** @} */
