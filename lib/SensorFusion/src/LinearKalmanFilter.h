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
 * @brief  Implementation of the Linear Kalman Filter
 * @author Juliane Kerpe <juliane.kerpe@web.de>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef LINEARKALMANFILTER_H
#define LINEARKALMANFILTER_H

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
static const uint8_t NUMBER_OF_MEASUREMENTS_M = 2U;

/** Number of control inputs in the control input vector u (notation in literature: L). */
static const uint8_t NUMBER_OF_CONTROL_INPUTS_L = 2U;

/** This class provides a Linear Kalman filter. */
class LinearKalmanFilter : public IKalmanFilter
{
public:
    /**
     * Constructs the Linear Kalman Filter
     */
    LinearKalmanFilter() :
        m_stateVector(Eigen::Vector<float, NUMBER_OF_STATES_N>::Zero(NUMBER_OF_STATES_N)),
        m_covarianceMatrix(
            Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N>::Zero(NUMBER_OF_STATES_N, NUMBER_OF_STATES_N)),
        m_controlInputVector(Eigen::Vector<float, NUMBER_OF_CONTROL_INPUTS_L>::Zero(NUMBER_OF_CONTROL_INPUTS_L)),
        m_measurementVector(Eigen::Vector<float, NUMBER_OF_MEASUREMENTS_M>::Zero(NUMBER_OF_MEASUREMENTS_M))
    {
    }

    /**
     * Destroys the Linear Kalman Filter
     */
    ~LinearKalmanFilter()
    {
    }

    /**
     * Initializes the variables of the Linear Kalman Filter.
     */
    void init() final;

    /**
     * Prediction of the covariance and the state of the Linear Kalman Filter.
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
    /** Estimated state vector x=[p_x, p_y, v_x, v_y]^T */
    Eigen::Vector<float, NUMBER_OF_STATES_N> m_stateVector;

    /** Covariance Matrix of the state */
    Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> m_covarianceMatrix;

    /** Control Input Vector u=[a_x, a_y]^T */
    Eigen::Vector<float, NUMBER_OF_CONTROL_INPUTS_L> m_controlInputVector;

    /** Measurement Vector z=[positionOdometryX, positionOdometryY]^T */
    Eigen::Vector<float, NUMBER_OF_MEASUREMENTS_M> m_measurementVector;

    /** Index of Position in x-direction in the state vector x. */
    static const uint8_t IDX_POSITION_X_STATE_VECTOR = 0U;

    /** Index of Position in y-direction in the state vector x. */
    static const uint8_t IDX_POSITION_Y_STATE_VECTOR = 1U;

    /** Index of Velocity in x-direction in the state vector x. */
    static const uint8_t IDX_VELOCITY_X_STATE_VECTOR = 2U;

    /** Index of Velocity in y-direction in the state vector x. */
    static const uint8_t IDX_VELOCITY_Y_STATE_VECTOR = 3U;

    /** Index of Position in x-direction of the odometry in the measurement vector z. */
    static const uint8_t IDX_ODOMETRY_X_MEASUREMENT_VECTOR = 0U;

    /** Index of Position in y-direction of the odometry in the measurement vector z. */
    static const uint8_t IDX_ODOMETRY_Y_MEASUREMENT_VECTOR = 1U;

    /** Index of Acceleration in x-direction in the control input vector u. */
    static const uint8_t IDX_ACCELERATION_X_CONTROL_INPUT_VECTOR = 0U;

    /** Index of Acceleration in y-direction in the control input vector u. */
    static const uint8_t IDX_ACCELERATION_Y_CONTROL_INPUT_VECTOR = 1U;

    /** The covariance matrix of the process noise Matrix (notation in literature: Q). */
    static const Eigen::Matrix<float, NUMBER_OF_CONTROL_INPUTS_L, NUMBER_OF_CONTROL_INPUTS_L>
        PROCESS_COVARIANCE_MATRIX_Q;

    /** The observation model Matrix (notation in literature: H). */
    static const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_STATES_N> OBSERVATION_MATRIX_H;

    /** The covariance of the observation noise Matrix (notation in literature: R). */
    static const Eigen::Matrix<float, NUMBER_OF_MEASUREMENTS_M, NUMBER_OF_MEASUREMENTS_M> OBSERVATION_NOISE_MATRIX_R;

    /** Initial covariance matrix (notation in literature: P). */
    static const Eigen::Matrix<float, NUMBER_OF_STATES_N, NUMBER_OF_STATES_N> INITIAL_COVARIANCE_MATRIX_P;

    /**
     * Writes data from a KalmanParameter Struct into the measurement Vector as a member variable m_measurementVector
     * with the Structure: [positionOdometryX, positionOdometryY]
     *
     * @param[in] kalmanParameter   Input Parameters for the Kalman Filter as a KalmanParameter struct.
     */
    void updateMeasurementVector(KalmanParameter& kalmanParameter);

    /**
     * Writes data from a KalmanParameter Struct into the Control Input Vector as a member variable m_controlInputVector
     * [accelerationX, accelerationY]^T
     *
     * @param[in] kalmanParameter   Input Parameters for the Kalman Filter as a KalmanParameter struct.
     */
    void updateControlInputVector(KalmanParameter& kalmanParameter);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* LINEARKALMANFILTER_H */
/** @} */
