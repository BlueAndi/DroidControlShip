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
 *
 */
/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 *  @brief  Struct for Parameter used for a Kalman Filter
 *  @author Juliane Kerpe <juliane.kerpe@web.de>
 *
 *  @addtogroup Application
 *
 * @{
 */

#ifndef KALMANPARAMETER_H
#define KALMANPARAMETER_H

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

/** Struct of Data used in the Kalman Filter. */
typedef struct _KalmanParameter
{
    float accelerationX;     /**< Acceleration in x direction in mm/s^2 in the Robots Coordinate System. */
    float positionOdometryX; /**< Position calculated by Odometry in x direction in mm. */
    float positionOdometryY; /**< Position calculated by Odometry in y direction in mm. */
    float angleOdometry;     /**< Orientation calculated by Odometry in mrad. */
    float velocityOdometry;  /**< Velocity calculated by Odometry in mm/s. */
    float turnRate;          /**< Turn rate in mrad/s. */
} __attribute__((packed)) KalmanParameter;

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* KALMANPARAMETER_H */
/** @} */
