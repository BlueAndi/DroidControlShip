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
    float physicalAccelerationX =
        static_cast<float>(newSensorData.accelerationX) * SensorConstants::ACCELEROMETER_SENSITIVITY_FACTOR;
    float physicalTurnRate      = static_cast<float>(newSensorData.turnRate) * SensorConstants::GYRO_SENSITIVITY_FACTOR;

    /* Perform the Kalman Filter Prediction and Update Steps */
    KalmanParameter kalmanParameter;
    kalmanParameter.accelerationX     = physicalAccelerationX;
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

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
