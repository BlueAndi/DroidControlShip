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

constexpr float ACCELERATION_STD      = 80.0F;              /**< Standard Deviation of the Acceleration. */
constexpr float TURNRATE_STD          = 1.0F;               /**< Standard Deviation of the Turn Rate. */
constexpr float ODOMETRY_POSITION_STD = 4.0F;               /**< Standard Deviation of the Odometry Position. */
constexpr float ODOMETRY_STD_ANGLE    = 100.0F;             /**< Standard Deviation of the Odometry Orientation. */
constexpr float INIT_VELOCITY_STD     = 1.0;                /**< Standard Deviation of the initial Velocity. */
constexpr float INIT_ANGLE_STD        = 1.0 / 180.0 * M_PI; /**< Standard Deviation of the initial Angle. */

/** Initial covariance matrix (notation in literature: P). */
const Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> ExtendedKalmanFilter::INITIAL_COVARIANCE_MATRIX_P{
    {ODOMETRY_POSITION_STD * ODOMETRY_POSITION_STD, 0.0F, 0.0F, 0.0F},
    {0.0F, ODOMETRY_POSITION_STD* ODOMETRY_POSITION_STD, 0.0F, 0.0F},
    {0.0F, 0.0F, INIT_VELOCITY_STD* INIT_VELOCITY_STD, 0.0F},
    {0.0F, 0.0F, 0.0F, INIT_ANGLE_STD* INIT_ANGLE_STD}};

/** The observation model Matrix (notation in literature: H). */
const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_STATES_N> ExtendedKalmanFilter::OBSERVATION_MATRIX_H{
    {1.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 1.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 1.0F, 0.0F},
    {0.0F, 0.0F, 0.0F, 1.0F}};

/** The covariance of the observation noise Matrix (notation in literature: R). */
const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_MEASUREMENTS_M>
    ExtendedKalmanFilter::OBSERVATION_NOISE_MATRIX_R{{ODOMETRY_POSITION_STD * ODOMETRY_POSITION_STD, 0.0F, 0.0F, 0.0F},
                                                     {0.0F, ODOMETRY_POSITION_STD* ODOMETRY_POSITION_STD, 0.0F, 0.0F},
                                                     {0.0F, 0.0F, ODOMETRY_POSITION_STD* ODOMETRY_POSITION_STD, 0.0F},
                                                     {0.0F, 0.0F, 0.0F, ODOMETRY_STD_ANGLE* ODOMETRY_STD_ANGLE}};

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void ExtendedKalmanFilter::init(KalmanParameter& initialParameter)
{
    m_stateVector(IDX_POSITION_X_STATE_VECTOR)  = initialParameter.positionOdometryX;
    m_stateVector(IDX_POSITION_Y_STATE_VECTOR)  = initialParameter.positionOdometryY;
    m_stateVector(IDX_VELOCITY_STATE_VECTOR)    = 0.0F;
    m_stateVector(IDX_ORIENTATION_STATE_VECTOR) = initialParameter.angleOdometry;
    m_covarianceMatrix                          = INITIAL_COVARIANCE_MATRIX_P;
    updateControlInputVector(initialParameter);
    updateMeasurementVector(initialParameter);
}

void ExtendedKalmanFilter::predictionStep(const uint16_t timeStep)
{
    /* TODO: Implement Kalman Filter in cpp (TD072) */
}

IKalmanFilter::PositionData ExtendedKalmanFilter::updateStep(KalmanParameter& kalmanParameter)
{
    PositionData currentPosition;
    currentPosition.positionX = m_stateVector(IDX_POSITION_X_STATE_VECTOR);
    currentPosition.positionY = m_stateVector(IDX_POSITION_Y_STATE_VECTOR);
    currentPosition.angle     = m_stateVector(IDX_ORIENTATION_STATE_VECTOR);
    return currentPosition;
    /* TODO: Implement Kalman Filter in cpp (TD072) */
}
/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/
void ExtendedKalmanFilter::updateMeasurementVector(KalmanParameter& kalmanParameter)
{
    m_measurementVector(IDX_POSITION_X_MEASUREMENT_VECTOR)  = static_cast<float>(kalmanParameter.positionOdometryX);
    m_measurementVector(IDX_POSITION_Y_MEASUREMENT_VECTOR)  = static_cast<float>(kalmanParameter.positionOdometryY);
    m_measurementVector(IDX_ORIENTATION_MEASUREMENT_VECTOR) = static_cast<float>(kalmanParameter.angleOdometry);
}

void ExtendedKalmanFilter::updateControlInputVector(KalmanParameter& kalmanParameter)
{
    m_controlInputVector[IDX_TURNRATE_CONTROL_INPUT_VECTOR]       = static_cast<float>(kalmanParameter.turnRate);
    m_controlInputVector[IDX_ACCELERATION_X_CONTROL_INPUT_VECTOR] = static_cast<float>(kalmanParameter.accelerationX);
}

float ExtendedKalmanFilter::wrapAngle(float inputAngle)
{
    if (inputAngle < -1000.0F * M_PI)
    {
        inputAngle += 2000.0F * M_PI;
    }
    float outputAngle = fmodf(inputAngle + 1000.0F * M_PI, 2000.0F * M_PI) - 1000.0F * M_PI;
    return outputAngle;
}
/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
