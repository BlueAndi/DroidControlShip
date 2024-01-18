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
 * @brief  SensorFusion algorithm
 * @author Juliane Kerpe <juliane.kerpe@web.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "SensorFusion.h"
#include "SensorConstants.h"
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

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void SensorFusion::init(void)
{
    m_kalmanFilter.init();
}

void SensorFusion::estimateNewState(const SensorData& newSensorData)
{
    /* Calculate the physical Values via the Sensitivity Factors. */
    int16_t physicalAccelerationX = newSensorData.accelerationX * SensorConstants::ACCELEROMETER_SENSITIVITY_FACTOR;
    int16_t physicalAccelerationY = newSensorData.accelerationY * SensorConstants::ACCELEROMETER_SENSITIVITY_FACTOR;
    int16_t physicalTurnRate      = newSensorData.turnRate * SensorConstants::GYRO_SENSITIVITY_FACTOR;

    /* Transform the acceleration values from the robot coordinate system into the world coordinate system */
    int16_t accelerationInRobotCoordinateSystem[2] = {physicalAccelerationX, physicalAccelerationY};
    int16_t accelerationInGlobalCoordinateSystem[2];
    transformLocalToGlobal(accelerationInGlobalCoordinateSystem, accelerationInRobotCoordinateSystem,
                           m_estimatedPosition.angle);

    /* Perform the Kalman Filter Prediction and Update Steps */
    KalmanParameter kalmanParameter;
    kalmanParameter.accelerationX     = physicalAccelerationX;
    kalmanParameter.accelerationY     = physicalAccelerationY;
    kalmanParameter.positionOdometryX = newSensorData.positionOdometryX;
    kalmanParameter.positionOdometryY = newSensorData.positionOdometryY;
    kalmanParameter.angleOdometry     = newSensorData.orientationOdometry;
    kalmanParameter.turnRate          = physicalTurnRate;

    m_kalmanFilter.predictionStep(newSensorData.timePeriod);
    m_estimatedPosition = m_kalmanFilter.updateStep(kalmanParameter);
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void SensorFusion::transformLocalToGlobal(int16_t* globalResult, const int16_t* localVectorToTransform,
                                          const int16_t& rotationAngle)
{
    /*  Calculate the sin and cos of the rotationAngle; convert the angle from mrad to rad. */
    float cosValue = cosf(static_cast<float>(rotationAngle) / 1000.0F);
    float sinValue = sinf(static_cast<float>(rotationAngle) / 1000.0F);

    globalResult[0] = cosValue * localVectorToTransform[0] - sinValue * localVectorToTransform[1];
    globalResult[1] = sinValue * localVectorToTransform[0] + cosValue * localVectorToTransform[1];
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
