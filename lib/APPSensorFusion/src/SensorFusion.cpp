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
    float physicalAccelerationX = static_cast<float>(newSensorData.accelerationX) *
                                  SensorConstants::ACCELEROMETER_SENSITIVITY_FACTOR; /* In mm/s^2 */
    float physicalTurnRate =
        static_cast<float>(newSensorData.turnRate) * SensorConstants::GYRO_SENSITIVITY_FACTOR; /* In mrad/s */

    KalmanParameter kalmanParameter;
    kalmanParameter.turnRate      = physicalTurnRate;      /* In mrad/s */
    kalmanParameter.accelerationX = physicalAccelerationX; /* In mm/s^2 */

    if (true == m_isFirstIteration)
    {
        /* Directely use the Position calculated via Odometry in the first iteration */
        kalmanParameter.positionOdometryX = static_cast<float>(newSensorData.positionOdometryX);   /* In mm */
        kalmanParameter.positionOdometryY = static_cast<float>(newSensorData.positionOdometryY);   /* In mm */
        kalmanParameter.angleOdometry     = static_cast<float>(newSensorData.orientationOdometry); /* In mrad */
        kalmanParameter.velocityOdometry  = 0.0F;                                                  /* In mm/s */
        m_kalmanFilter.init(kalmanParameter);
        m_estimatedPosition = {kalmanParameter.positionOdometryX, kalmanParameter.positionOdometryY,
                               kalmanParameter.angleOdometry};
        m_isFirstIteration  = false;
    }
    else
    {
        /* Calculate the traveled euclidean Distance based upon the Odometry Values. */
        float deltaXOdometry =
            static_cast<float>(newSensorData.positionOdometryX) - m_lastOdometryPosition.positionX; /* In mm */
        float deltaYOdometry =
            static_cast<float>(newSensorData.positionOdometryY) - m_lastOdometryPosition.positionY; /* In mm */
        float euclideanDistance = sqrtf(powf(deltaXOdometry, 2.0F) + powf(deltaYOdometry, 2.0F));   /* In mm */

        /* Compute the change in angle based on odometry data. Determine the current angle by adding this change to the
         * previously estimated angle. */
        float deltaAngleOdometry =
            static_cast<float>(newSensorData.orientationOdometry) - m_lastOdometryPosition.angle; /* In mrad */
        float updatedOrientationOdometry = m_estimatedPosition.angle + deltaAngleOdometry;        /* In mrad */

        float CONVERSION_MS_TO_S = 0.001F;
        float duration           = static_cast<float>(newSensorData.timePeriod) * CONVERSION_MS_TO_S; /* In seconds */
        m_timeSinceLastOdoUpdate += duration;

        /* Calculate the mean velocity since the last Iteration instead of using the momentary Speed.
        Correct the sign of the distance via the measured velocity if the velocity is negative, hence the robot drives
        backwards.
        For the correction of the sign, the dot product is used. If the robot moves forward, the dot product is
        positive. Otherwise, negative. */
        float CONVERSION_MRAD_TO_RAD = 0.001F;
        float dotProduct =
            deltaXOdometry * cosf(static_cast<float>(newSensorData.orientationOdometry) * CONVERSION_MRAD_TO_RAD) +
            deltaYOdometry * sinf(static_cast<float>(newSensorData.orientationOdometry) * CONVERSION_MRAD_TO_RAD);
        float directionCorrection = 1.0F;
        if (dotProduct < 0.0F)
        {
            directionCorrection = -1.0F;
        }
        float meanVelocityOdometry = directionCorrection * euclideanDistance / m_timeSinceLastOdoUpdate; /* In mm/s */

        /* Update the measured position assuming the robot drives in the estimated Direction.
        Use the Correction Factor in case the robot drives Backwards. */
        kalmanParameter.positionOdometryX =
            m_estimatedPosition.positionX + directionCorrection *
                                                cosf(m_estimatedPosition.angle * CONVERSION_MRAD_TO_RAD) *
                                                euclideanDistance; /* In mm */
        kalmanParameter.positionOdometryY =
            m_estimatedPosition.positionY + directionCorrection *
                                                sinf(m_estimatedPosition.angle * CONVERSION_MRAD_TO_RAD) *
                                                euclideanDistance;     /* In mm */
        kalmanParameter.angleOdometry    = updatedOrientationOdometry; /* In mrad */
        kalmanParameter.velocityOdometry = meanVelocityOdometry;       /* In mm/s */

        /* Perform the Prediction Step and the Update Step of the Kalman Filter. */
        m_kalmanFilter.predictionStep(newSensorData.timePeriod, kalmanParameter);

        /* The Odometry is not necessarily updated in every time step. Only Perform the Update step if there are new
         * Odometry values available. */
        bool newOdometryValues = (euclideanDistance > 0.0F);
        if (true == newOdometryValues)
        {
            m_estimatedPosition      = m_kalmanFilter.updateStep();
            m_timeSinceLastOdoUpdate = 0.0F;
        }
    }

    /* Save the last Odometry values */
    m_lastOdometryPosition.positionX = static_cast<float>(newSensorData.positionOdometryX);   /* In mm */
    m_lastOdometryPosition.positionY = static_cast<float>(newSensorData.positionOdometryY);   /* In mm */
    m_lastOdometryPosition.angle     = static_cast<float>(newSensorData.orientationOdometry); /* In mm */
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
