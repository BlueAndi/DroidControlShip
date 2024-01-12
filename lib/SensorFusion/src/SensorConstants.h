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
 * @brief  Sensor specific constants
 * @author Juliane Kerpe <juliane.kerpe@web.de>
 *
 * @addtogroup Application
 * 
 * @{
 */

#ifndef SENSORCONSTANTS_H
#define SENSORCONSTANTS_H
/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Abstracts the physical sensor constants.
 */
namespace SensorConstants
{
    /** Sensitivity Factor of the LSM303D and LSM6DS33 accelerometer in mm/s^2/digit at Range of +/- 2 g (g-force).
     * Converts a raw accelerometer value in digits to mm/s^2. The value 0.061 is defined in the Data sheets of the
     * LSM303D and the LSM6DS33 accelerometer. Then the value is converted from mg into mm/s^2. */
    static const float ACCELEROMETER_SENSITIVITY_FACTOR = 0.061F * 9.81F;
    /** Sensitivity Factor of the L3GD20H and LSM6DS33 gyro in mrad/s/digit at Range of +/- 245 dps (degrees per
     * second). Converts a raw gyroscope value in digits to mrad/s. The sensitivity factor 8.75 is defined in the Data
     * sheets of the L3GD20H and the LSM6DS33 gyro. Then the value is converted from mdps/digit into mrad/s/digit. */
    static const float GYRO_SENSITIVITY_FACTOR = 8.75F * 2.0F * M_PI / 360.0F;
}; // namespace SensorConstants

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SENSORCONSTANTS_H */
/** @} */
