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
 * @brief  Sensor specific constants
 * @author Juliane Kerpe <juliane.kerpe@web.de>
 * 
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
#include <stdint.h>

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
    /**
     * The Linear Sensitivity Factors are specified in the data sheets of the sensors LSM303D (accelerometer+gyro), L3GD20H (gyro), LSM6DS33 (gyro + accelerometer), LIS3MDL (magnetometer)
    */
    static const float ACCELEROMETER_SENSITIVITY_FACTOR    = 0.061f * 9.81f;                /**< Sensitivity Factor of the LSM303D and LSM6DS33 accelerometer   in mm/s/s/LSB       at Range of +/- 2 g   */
    static const float GYRO_SENSITIVITY_FACTOR             = 8.75f * 2 * 3.14159 / 360;     /**< Sensitivity Factor of the L3GD20H and LSM6DS33 gyro            in mrad/s/digit     at Range of +/- 245 dps */ 
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SENSORCONSTANTS_H */
/** @} */
