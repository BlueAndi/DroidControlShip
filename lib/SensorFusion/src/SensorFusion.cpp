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

void SensorFusion::estimateNewState(const SensorData& newSensorData)
{
    /* Calculate the physical Values via the Sensitivity Factors. */
    float physicalAccelerationX =
        static_cast<float>(newSensorData.accelerationX) * SensorConstants::ACCELEROMETER_SENSITIVITY_FACTOR;
    float physicalTurnRate = static_cast<float>(newSensorData.turnRate) * SensorConstants::GYRO_SENSITIVITY_FACTOR;

    KalmanParameter kalmanParameter;
    if (true == m_isFirstIteration)
    {
        /* Directely use the Position calculated via Odometry in the first iteration */
        kalmanParameter.positionOdometryX = static_cast<float>(newSensorData.positionOdometryX);
        kalmanParameter.positionOdometryY = static_cast<float>(newSensorData.positionOdometryY);
        kalmanParameter.angleOdometry     = static_cast<float>(newSensorData.orientationOdometry);
        kalmanParameter.turnRate          = physicalTurnRate;
        kalmanParameter.accelerationX     = physicalAccelerationX;
        kalmanParameter.velocityOdometry  = 0.0F;
        m_kalmanFilter.init(kalmanParameter);
        m_estimatedPosition = {kalmanParameter.positionOdometryX, kalmanParameter.positionOdometryY,
                               kalmanParameter.angleOdometry};
        m_isFirstIteration  = false;
    }
    else
    {
        /* Calculate the traveled euclidean Distance based upon the Odometry Values. */
        float deltaXOdometry = static_cast<float>(newSensorData.positionOdometryX) - m_lastOdometryPosition.positionX;
        float deltaYOdometry = static_cast<float>(newSensorData.positionOdometryY) - m_lastOdometryPosition.positionY;
        float euclideanDistance = sqrtf(powf(deltaXOdometry, 2.0F) + powf(deltaYOdometry, 2.0F));

        float deltaAngleOdometry = static_cast<float>(newSensorData.orientationOdometry) - m_lastOdometryPosition.angle;
        float updatedOrientationOdometry = m_estimatedPosition.angle + deltaAngleOdometry;

        float duration = static_cast<float>(newSensorData.timePeriod) / 1000.0F;

        /* Calculate the mean velocity since the last Iteration instead of using the momentary Speed.
        Correct the sign of the distance via the measured velocity if the velocity is negative, hence the robot drives
        backwards.
        For the correction of the sign, the dot product is used. If the robot moves forward, the dot product is
        positive. Otherwise, negative. */
        float dotProduct = deltaXOdometry * cosf(static_cast<float>(newSensorData.orientationOdometry) / 1000.0F) +
                           deltaYOdometry * sinf(static_cast<float>(newSensorData.orientationOdometry) / 1000.0F);
        float directionCorrection = 1.0F;
        if (dotProduct < 0.0F)
        {
            directionCorrection = -1.0F;
        }
        float meanVelocityOdometry = directionCorrection * euclideanDistance / duration;

        /* If the robot moves slowly, the odometry might not be updated in the scope of one iteration.
        The velocity would be falsely assumed to be 0. In this case, the last Velocity shall be used.  */
        if (0.0F == meanVelocityOdometry)
        {
            meanVelocityOdometry = m_lastOdometryVelocity;
        }
        m_lastOdometryVelocity = meanVelocityOdometry;

        /* Update the measured position assuming the robot drives in the estimated Direction.
        Use the Correction Factor in case the robot drives Backwards. */
        kalmanParameter.positionOdometryX =
            m_estimatedPosition.positionX +
            directionCorrection * cosf(m_estimatedPosition.angle / 1000.0F) * euclideanDistance;
        kalmanParameter.positionOdometryY =
            m_estimatedPosition.positionY +
            directionCorrection * sinf(m_estimatedPosition.angle / 1000.0F) * euclideanDistance;
        kalmanParameter.angleOdometry    = updatedOrientationOdometry;
        kalmanParameter.turnRate         = physicalTurnRate;
        kalmanParameter.velocityOdometry = meanVelocityOdometry;
        kalmanParameter.accelerationX    = physicalAccelerationX;

        /* Perform the Prediction Step and the Update Step of the Kalman Filter. */
        m_kalmanFilter.predictionStep(newSensorData.timePeriod);
        m_estimatedPosition = m_kalmanFilter.updateStep(kalmanParameter);
    }

    /* Save the last Odometry values */
    m_lastOdometryPosition.positionX = static_cast<float>(newSensorData.positionOdometryX);
    m_lastOdometryPosition.positionY = static_cast<float>(newSensorData.positionOdometryY);
    m_lastOdometryPosition.angle     = static_cast<float>(newSensorData.orientationOdometry);
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
