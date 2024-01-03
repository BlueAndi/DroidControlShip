/* MIT License
 *
 * Copyright (c) 2019 - 2023 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Linear Kalman Filter
 * @author Juliane Kerpe <juliane.kerpe@web.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "LinearKalmanFilter.h"

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
/* TODO: TD124 Determine the right Parameters for LKF and EKF. */

/** The covariance matrix of the process noise Matrix (notation in literature: Q). */
const Eigen::Matrix<float, NUMBER_OF_CONTROL_INPUTS_L, NUMBER_OF_CONTROL_INPUTS_L>
    LinearKalmanFilter::PROCESS_COVARIANCE_MATRIX_Q{{0.0F, 0.0F}, {0.0F, 0.0F}};

/** The observation model Matrix (notation in literature: H). */
const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_STATES_N> LinearKalmanFilter::OBSERVATION_MATRIX_H{
    {1.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 1.0F, 0.0F, 0.0F}};

/** The covariance of the observation noise Matrix (notation in literature: R). */
const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_MEASUREMENTS_M>
    LinearKalmanFilter::OBSERVATION_NOISE_MATRIX_R{{0.0F * 0.0F, 0.0F}, {0.0F, 0.0F * 0.0F}};

/** Initial covariance matrix (notation in literature: P). */
const Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> LinearKalmanFilter::INITIAL_COVARIANCE_MATRIX_P{
    {1.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 1.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 1.0F, 0.0F},
    {0.0F, 0.0F, 0.0F, 1.0F}};

/******************************************************************************
 * Public Methods
 *****************************************************************************/
void LinearKalmanFilter::init()
{
    /* TODO: Implement Kalman Filter in cpp (TD072)  */
}

void LinearKalmanFilter::predictionStep(const uint16_t timeStep)
{
    /* TODO: Implement Kalman Filter in cpp (TD072) */
}

IKalmanFilter::PositionData LinearKalmanFilter::updateStep(KalmanParameter& kalmanParameter)
{
    IKalmanFilter::PositionData currentPosition;
    currentPosition.positionX = kalmanParameter.positionOdometryX;
    currentPosition.positionY = kalmanParameter.positionOdometryY;
    currentPosition.angle     = kalmanParameter.angleOdometry;
    return currentPosition;
    /* TODO: Implement Kalman Filter in cpp (TD072) */
}
/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/
Eigen::VectorXf LinearKalmanFilter::generateMeasurementVector(KalmanParameter& kalmanParameter)
{
    Eigen::Vector<float, NUMBER_OF_MEASUREMENTS_M> measurementVector =
        Eigen::Vector<float, NUMBER_OF_MEASUREMENTS_M>::Zero(NUMBER_OF_MEASUREMENTS_M);
    measurementVector(IDX_ODOMETRY_X_MEASUREMENT_VECTOR) = static_cast<float>(kalmanParameter.positionOdometryX);
    measurementVector(IDX_ODOMETRY_Y_MEASUREMENT_VECTOR) = static_cast<float>(kalmanParameter.positionOdometryY);
    return measurementVector;
}

Eigen::VectorXf LinearKalmanFilter::generateControlInputVector(KalmanParameter& kalmanParameter)
{
    Eigen::Vector<float, NUMBER_OF_CONTROL_INPUTS_L> controlInputVector =
        Eigen::Vector<float, NUMBER_OF_CONTROL_INPUTS_L>::Zero(NUMBER_OF_CONTROL_INPUTS_L);
    controlInputVector(IDX_ACCELERATION_X_CONTROL_INPUT_VECTOR) = static_cast<float>(kalmanParameter.accelerationX);
    controlInputVector(IDX_ACCELERATION_Y_CONTROL_INPUT_VECTOR) = static_cast<float>(kalmanParameter.accelerationY);
    return controlInputVector;
}
/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
