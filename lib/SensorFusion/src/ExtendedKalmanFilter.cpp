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
 * @brief  Implementation of the Extended Kalman Filter
 * @author Juliane Kerpe <juliane.kerpe@web.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "ExtendedKalmanFilter.h"

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
/** Initial covariance matrix (notation in literature: P). */
const Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> ExtendedKalmanFilter::INITIAL_COVARIANCE_MATRIX_P{
    {0.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 0.0F, 0.0F}};

/** The covariance matrix of the process noise Matrix (notation in literature: Q). */
const Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> ExtendedKalmanFilter::PROCESS_COVARIANCE_MATRIX_Q{
    {5.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 5.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 5.0F, 0.0F},
    {0.0F, 0.0F, 0.0F, 5.0F}};

/** The observation model Matrix (notation in literature: H). */
const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_STATES_N> ExtendedKalmanFilter::OBSERVATION_MATRIX_H{
    {1.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 1.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 0.0F, 1.0F}};

/** The covariance of the observation noise Matrix (notation in literature: R). */
const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_MEASUREMENTS_M>
    ExtendedKalmanFilter::OBSERVATION_NOISE_MATRIX_R{{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}};
/******************************************************************************
 * Public Methods
 *****************************************************************************/
void ExtendedKalmanFilter::init()
{
    /* TODO: Implement Kalman Filter in cpp (TD072) */
}

void ExtendedKalmanFilter::predictionStep(const uint16_t timeStep)
{
    /* TODO: Implement Kalman Filter in cpp (TD072) */
}

IKalmanFilter::PositionData ExtendedKalmanFilter::updateStep(KalmanParameter& kalmanParameter)
{
    PositionData currentPosition;
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
Eigen::VectorXf ExtendedKalmanFilter::generateMeasurementVector(KalmanParameter& kalmanParameter)
{
    Eigen::Vector<float, NUMBER_OF_MEASUREMENTS_M> measurementVector =
        Eigen::Vector<float, NUMBER_OF_MEASUREMENTS_M>::Zero(NUMBER_OF_MEASUREMENTS_M);
    measurementVector(IDX_ODOMETRY_X_MEASUREMENT_VECTOR) = static_cast<float>(kalmanParameter.positionOdometryX);
    measurementVector(IDX_ODOMETRY_Y_MEASUREMENT_VECTOR) = static_cast<float>(kalmanParameter.positionOdometryY);
    measurementVector(IDX_ODOMETRY_ORIENTATION_MEASUREMENT_VECTOR) = static_cast<float>(kalmanParameter.angleOdometry);
    return measurementVector;
}

Eigen::VectorXf ExtendedKalmanFilter::generateControlInputVector(KalmanParameter& kalmanParameter)
{
    Eigen::Vector<float, NUMBER_OF_CONTROL_INPUTS_L> controlInputVector =
        Eigen::Vector<float, NUMBER_OF_CONTROL_INPUTS_L>::Zero(NUMBER_OF_CONTROL_INPUTS_L);
    controlInputVector(IDX_ACCELERATION_X_CONTROL_INPUT_VECTOR) = static_cast<float>(kalmanParameter.accelerationX);
    controlInputVector(IDX_ACCELERATION_Y_CONTROL_INPUT_VECTOR) = static_cast<float>(kalmanParameter.accelerationY);
    controlInputVector(IDX_TURNRATE_CONTROL_INPUT_VECTOR)       = static_cast<float>(kalmanParameter.turnRate);
    return controlInputVector;
}
/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
