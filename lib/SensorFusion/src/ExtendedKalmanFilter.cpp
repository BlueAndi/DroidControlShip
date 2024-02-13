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
/** The expected Duration of one Cycle.
 *  It does not neccessary be precise since it is only used for a rough estimation of the Variance. */
constexpr float EXPECTED_DURATION_IN_S = 0.025F;

/** The Variance of the Acceleration.
 *  The standard deviation value 20.0F has been determined by standstill measurements.
 *  The variance then needs to be multiplied with the expected duration of one cycle. */
constexpr float ACCELERATION_VARIANCE = 20.0F * 20.0F * EXPECTED_DURATION_IN_S * EXPECTED_DURATION_IN_S;

/** Variance of the Turn Rate.
 *  The standard deviation value 4.0F has been determined by standstill measurements.
 *  The variance then needs to be multiplied with the expected duration of one cycle. */
constexpr float TURNRATE_VARIANCE = 4.0F * 4.0F * EXPECTED_DURATION_IN_S * EXPECTED_DURATION_IN_S;

/** Variance of the Odometry Position. Determined empirically. */
constexpr float ODOMETRY_POSITION_VARIANCE = 2.0F / 1000.0F;

/** Variance of the Odometry Velocity. Determined empirically. */
constexpr float ODOMETRY_VELOCITY_VARIANCE = 3.0F / 1000.0F;

/** Variance of the Odometry Angle. Determined empirically. */
constexpr float ODOMETRY_ANGLE_VARIANCE = 1.0F;

/** The observation model Matrix (notation in literature: H). */
const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_STATES_N> ExtendedKalmanFilter::OBSERVATION_MATRIX_H{
    {1.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 1.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 1.0F, 0.0F},
    {0.0F, 0.0F, 0.0F, 1.0F}};

/** The covariance of the observation noise Matrix (notation in literature: R). */
const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_MEASUREMENTS_M>
    ExtendedKalmanFilter::OBSERVATION_NOISE_MATRIX_R{{ODOMETRY_POSITION_VARIANCE, 0.0F, 0.0F, 0.0F},
                                                     {0.0F, ODOMETRY_POSITION_VARIANCE, 0.0F, 0.0F},
                                                     {0.0F, 0.0F, ODOMETRY_VELOCITY_VARIANCE, 0.0F},
                                                     {0.0F, 0.0F, 0.0F, ODOMETRY_ANGLE_VARIANCE}};

/** The covariance matrix of the process noise Matrix (notation in literature: Q). */
const Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> ExtendedKalmanFilter::PROCESS_COVARIANCE_MATRIX_Q{
    {0.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, ACCELERATION_VARIANCE, 0.0F},
    {0.0F, 0.0F, 0.0F, TURNRATE_VARIANCE}};

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void ExtendedKalmanFilter::init(KalmanParameter& initialParameter)
{
    m_stateVector(IDX_POSITION_X_STATE_VECTOR)  = initialParameter.positionOdometryX;
    m_stateVector(IDX_POSITION_Y_STATE_VECTOR)  = initialParameter.positionOdometryY;
    m_stateVector(IDX_VELOCITY_STATE_VECTOR)    = 0.0F;
    m_stateVector(IDX_ORIENTATION_STATE_VECTOR) = initialParameter.angleOdometry;
    updateControlInputVector(initialParameter);
    updateMeasurementVector(initialParameter);
}

void ExtendedKalmanFilter::predictionStep(const uint16_t timeStep)
{
    /* Extract individual values into variables in favor of readability. */
    float positionX   = m_stateVector(IDX_POSITION_X_STATE_VECTOR);  /* In mm */
    float positionY   = m_stateVector(IDX_POSITION_Y_STATE_VECTOR);  /* In mm */
    float velocity    = m_stateVector(IDX_VELOCITY_STATE_VECTOR);    /* In mm/s */
    float orientation = m_stateVector(IDX_ORIENTATION_STATE_VECTOR); /* In mrad */

    float CONVERSION_MRAD_TO_RAD = 0.001F;
    float cosOrienation          = cosf(orientation * CONVERSION_MRAD_TO_RAD);
    float sinOrienation          = sinf(orientation * CONVERSION_MRAD_TO_RAD);

    float CONVERSION_MS_TO_S = 0.001F;
    float duration           = static_cast<float>(timeStep) * CONVERSION_MS_TO_S; /* In seconds */

    /* Perform the prediction step of the state vector according to the State Model */
    m_stateVector[IDX_POSITION_X_STATE_VECTOR] = positionX + duration * velocity * cosOrienation;
    m_stateVector[IDX_POSITION_Y_STATE_VECTOR] = positionY + duration * velocity * sinOrienation;
    m_stateVector[IDX_VELOCITY_STATE_VECTOR] =
        velocity + m_controlInputVector[IDX_ACCELERATION_X_CONTROL_INPUT_VECTOR] * duration;
    m_stateVector[IDX_ORIENTATION_STATE_VECTOR] =
        orientation + duration * m_controlInputVector[IDX_TURNRATE_CONTROL_INPUT_VECTOR];

    /* Perform the prediction step of the covariance matrix with the help of the Jacobian Matrix */
    Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> jacobianMatrix;
    jacobianMatrix << 1, 0, duration * cosOrienation, -duration * velocity * sinOrienation, 0, 1,
        duration * sinOrienation, duration * velocity * cosOrienation, 0, 0, 1, 0, 0, 0, 0, 1;

    m_covarianceMatrix = jacobianMatrix * m_covarianceMatrix * jacobianMatrix.transpose() + PROCESS_COVARIANCE_MATRIX_Q;
}

IKalmanFilter::PositionData ExtendedKalmanFilter::updateStep(KalmanParameter& kalmanParameter)
{
    updateControlInputVector(kalmanParameter);
    updateMeasurementVector(kalmanParameter);

    /* Calculate the Kalman Gain Matrix according to Kalman Formula. */
    Eigen::VectorXf z_hat             = OBSERVATION_MATRIX_H * m_stateVector;
    Eigen::VectorXf innovationVectorY = m_measurementVector - z_hat;
    innovationVectorY(IDX_ORIENTATION_MEASUREMENT_VECTOR) =
        wrapAngle(innovationVectorY(IDX_ORIENTATION_MEASUREMENT_VECTOR));
    Eigen::MatrixXf kalmanGainMatrix =
        m_covarianceMatrix * OBSERVATION_MATRIX_H.transpose() *
        (OBSERVATION_MATRIX_H * m_covarianceMatrix * OBSERVATION_MATRIX_H.transpose() + OBSERVATION_NOISE_MATRIX_R)
            .inverse();

    /* Perform the update step of the state vector  */
    m_stateVector                               = m_stateVector + kalmanGainMatrix * innovationVectorY;
    m_stateVector(IDX_ORIENTATION_STATE_VECTOR) = wrapAngle(m_stateVector(IDX_ORIENTATION_STATE_VECTOR));

    /* Perform the update step of the covariance matrix  */
    Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> identityMatrix =
        Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N>::Identity();
    m_covarianceMatrix = (identityMatrix - kalmanGainMatrix * OBSERVATION_MATRIX_H) * m_covarianceMatrix;

    /* Fill the PositionData struct  */
    PositionData currentPosition;
    currentPosition.positionX = m_stateVector(IDX_POSITION_X_STATE_VECTOR);  /* In mm */
    currentPosition.positionY = m_stateVector(IDX_POSITION_Y_STATE_VECTOR);  /* In mm */
    currentPosition.angle     = m_stateVector(IDX_ORIENTATION_STATE_VECTOR); /* In mrad */
    return currentPosition;
}
/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/
void ExtendedKalmanFilter::updateMeasurementVector(KalmanParameter& kalmanParameter)
{
    m_measurementVector[IDX_POSITION_X_MEASUREMENT_VECTOR]  = kalmanParameter.positionOdometryX;        /* In mm */
    m_measurementVector[IDX_POSITION_Y_MEASUREMENT_VECTOR]  = kalmanParameter.positionOdometryY;        /* In mm */
    m_measurementVector[IDX_VELOCITY_MEASUREMENT_VECTOR]    = kalmanParameter.velocityOdometry;         /* In mm/s */
    m_measurementVector[IDX_ORIENTATION_MEASUREMENT_VECTOR] = wrapAngle(kalmanParameter.angleOdometry); /* In mrad */
}

void ExtendedKalmanFilter::updateControlInputVector(KalmanParameter& kalmanParameter)
{
    m_controlInputVector[IDX_ACCELERATION_X_CONTROL_INPUT_VECTOR] = kalmanParameter.accelerationX; /* In mm/s^2 */
    m_controlInputVector[IDX_TURNRATE_CONTROL_INPUT_VECTOR]       = kalmanParameter.turnRate;      /* In mrad/s */
}

float ExtendedKalmanFilter::wrapAngle(float inputAngle)
{
    const float pi          = 1000.0F * M_PI; /* 1 Pi in mrad converted from rad to mrad. */
    const float twoPi       = 2.0F * pi;      /* Two Pi in mrad */
    float       outputAngle = 0.0F;

    if (inputAngle < -pi)
    {
        inputAngle += twoPi;
    }

    outputAngle = fmodf(inputAngle + pi, twoPi) - pi;
    return outputAngle;
}
/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
