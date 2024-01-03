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
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <ArduinoEigen.h>
#include <stdint.h>
#include "IKalmanFilter.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Number of states used in the state vector x (notation in literature: N). */
static const uint8_t NUMBER_OF_STATES_N = 4U;

/** Number of measurements in the measurement vector z (notation in literature: M). */
static const uint8_t NUMBER_OF_MEASUREMENTS_M = 3U;

/** Number of control inputs in the control input vector u (notation in literature: L). */
static const uint8_t NUMBER_OF_CONTROL_INPUTS_L = 3U;

/** This class provides an Extended Kalman filter. */
class ExtendedKalmanFilter : public IKalmanFilter
{
public:
    /**
     * Constructs the Extended Kalman Filter
     */
    ExtendedKalmanFilter() :
        m_stateVector(Eigen::Vector<float, NUMBER_OF_STATES_N>::Zero(NUMBER_OF_STATES_N)),
        m_covarianceMatrix(
            Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N>::Zero(NUMBER_OF_STATES_N, NUMBER_OF_STATES_N)),
        m_controlInputVector(Eigen::Vector<float, NUMBER_OF_CONTROL_INPUTS_L>::Zero(NUMBER_OF_CONTROL_INPUTS_L))
    {
    }

    /**
     * Destroys the Extended Kalman Filter
     */
    ~ExtendedKalmanFilter()
    {
    }

    /**
     * Initializes the variables of the Extended Kalman Filter.
     */
    void init() final;

    /**
     * Prediction of the covariance and the state of the Extended Kalman Filter.
     * @param[in] timeStep Measured Time Step in ms.
     */
    void predictionStep(const uint16_t timeStep) final;

    /**
     * Update of the covariance and the state of the Kalman Filter.
     * @param[in] kalmanParameter   Input Parameters for the Kalman Filter as a KalmanParameter struct.
     * @return Estimated Position as a PositionData struct.
     */
    PositionData updateStep(KalmanParameter& kalmanParameter) final;

private:
    /** Estimated state vector [positionX, positionY, velocity, orientation]^T */
    Eigen::Vector<float, NUMBER_OF_STATES_N> m_stateVector;

    /** Covariance Matrix of the state */
    Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> m_covarianceMatrix;

    /** Control Input Vector u=[accelerationX, accelerationY, turnRate]^T */
    Eigen::Vector<float, NUMBER_OF_CONTROL_INPUTS_L> m_controlInputVector;

    /** Index of Position in x-direction in the state vector x. */
    static const uint8_t IDX_POSITION_X_STATE_VECTOR = 0U;

    /** Index of Position in y-direction in the state vector x. */
    static const uint8_t IDX_POSITION_Y_STATE_VECTOR = 1U;

    /** Index of the velocity in the state vector x. */
    static const uint8_t IDX_VELOCITY_STATE_VECTOR = 2U;

    /** Index of the orientation in the state vector x. */
    static const uint8_t IDX_ORIENTATION_STATE_VECTOR = 3U;

    /** Index of Position in x-direction of the odometry in the measurement vector z. */
    static const uint8_t IDX_ODOMETRY_X_MEASUREMENT_VECTOR = 0U;

    /** Index of Position in y-direction of the odometry in the measurement vector z. */
    static const uint8_t IDX_ODOMETRY_Y_MEASUREMENT_VECTOR = 1U;

    /** Index of Orientation of the odometry in the measurement vector z. */
    static const uint8_t IDX_ODOMETRY_ORIENTATION_MEASUREMENT_VECTOR = 2U;

    /** Index of Acceleration in x-direction in the control input vector u. */
    static const uint8_t IDX_ACCELERATION_X_CONTROL_INPUT_VECTOR = 0U;

    /** Index of Acceleration in y-direction in the control input vector u. */
    static const uint8_t IDX_ACCELERATION_Y_CONTROL_INPUT_VECTOR = 1U;

    /** Index of Turn Rate in the control input vector u. */
    static const uint8_t IDX_TURNRATE_CONTROL_INPUT_VECTOR = 2U;

    /** Initial covariance matrix (notation in literature: P). */
    static const Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> INITIAL_COVARIANCE_MATRIX_P;

    /** The covariance matrix of the process noise Matrix (notation in literature: Q). */
    static const Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> PROCESS_COVARIANCE_MATRIX_Q;

    /** The observation model Matrix (notation in literature: H). */
    static const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_STATES_N> OBSERVATION_MATRIX_H;

    /** The covariance of the observation noise Matrix (notation in literature: R). */
    static const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_MEASUREMENTS_M> OBSERVATION_NOISE_MATRIX_R;

    /**
     * Writes data from a KalmanParameter Struct into a measurement vector
     * @param[in] kalmanParameter   Input Parameters for the Kalman Filter as a KalmanParameter struct.
     * @return Measurement Vector z_k with structure: [positionOdometryX, positionOdometryY, orientationOdometry]^T
     */
    Eigen::VectorXf generateMeasurementVector(KalmanParameter& kalmanParameter);

    /**
     * Writes data from a KalmanParameter Struct into a control Input Vector
     * @param[in] kalmanParameter   Input Parameters for the Kalman Filter as a KalmanParameter struct.
     * @return Control Input Vector u_k with structure: [accelerationX, accelerationY, turnRate]^T
     */
    Eigen::VectorXf generateControlInputVector(KalmanParameter& kalmanParameter);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* EXTENDEDKALMANFILTER_H */
/** @} */
