/* MIT License
 *
 * Copyright (c) 2023 - 2025 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Extended Kalman Filter with 5D state
 * @author Tobias Haeckel
 *
 * State vector:
 *   x = [p_x, p_y, theta, v, omega]^T
 *   p_x, p_y   : position in mm
 *   theta      : heading in mrad
 *   v          : linear velocity in mm/s
 *   omega      : yaw rate in mrad/s
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef EKF_H
#define EKF_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <ArduinoEigen.h>
#include <stdint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * @brief Extended Kalman Filter implementation for a 5D state.
 */
class ExtendedKalmanFilter5D
{
public:
    /** State dimension. */
    static constexpr uint8_t STATE_DIM    = 5U;
    /** Camera measurement dimension. */
    static constexpr uint8_t CAM_MEAS_DIM = 5U;
    /** Odometry measurement dimension. */
    static constexpr uint8_t ODO_MEAS_DIM = 4U;
    /** IMU measurement dimension. */
    static constexpr uint8_t IMU_MEAS_DIM = 1U;

    /** State vector type: [p_x, p_y, theta, v, omega]^T. */
    using StateVector = Eigen::Matrix<float, STATE_DIM, 1>;
    /** State covariance matrix type. */
    using StateMatrix = Eigen::Matrix<float, STATE_DIM, STATE_DIM>;

    /** Camera measurement vector type. */
    using CamMeasVector = Eigen::Matrix<float, CAM_MEAS_DIM, 1>;
    /** Camera measurement covariance matrix type. */
    using CamMeasMatrix = Eigen::Matrix<float, CAM_MEAS_DIM, CAM_MEAS_DIM>;

    /** Odometry measurement vector type. */
    using OdoMeasVector = Eigen::Matrix<float, ODO_MEAS_DIM, 1>;
    /** Odometry measurement covariance matrix type. */
    using OdoMeasMatrix = Eigen::Matrix<float, ODO_MEAS_DIM, ODO_MEAS_DIM>;

    /** IMU measurement vector type. */
    using ImuMeasVector = Eigen::Matrix<float, IMU_MEAS_DIM, 1>;
    /** IMU measurement covariance matrix type. */
    using ImuMeasMatrix = Eigen::Matrix<float, IMU_MEAS_DIM, IMU_MEAS_DIM>;

    /**
     * @brief Construct the EKF with default noise matrices.
     */
    ExtendedKalmanFilter5D();

    /**
     * @brief Initialize filter with a given state and covariance.
     *
     * @param[in] x0 Initial state vector.
     * @param[in] P0 Initial covariance matrix.
     *
     * @return true on success.
     */
    bool init(const StateVector& x0, const StateMatrix& P0);

    /**
     * @brief Initialize filter with default state and identity covariance.
     *
     * Default state is configured in EKF.cpp.
     *
     * @return true on success.
     */
    bool init();

    /**
     * @brief Prediction step of the EKF.
     *
     * State model:
     *   p_x(k+1) = p_x + v cos(theta) dt
     *   p_y(k+1) = p_y + v sin(theta) dt
     *   theta(k+1) = theta + omega dt
     *   v(k+1) = v + a_x dt
     *   omega(k+1) = omega
     *
     * @param[in] a_x Longitudinal acceleration in mm/s^2.
     * @param[in] dt  Time step in seconds.
     */
    void predict(float a_x, float dt);

    /**
     * @brief Camera / Space Ship Radar measurement update.
     *
     * Measurement model:
     *   z_cam(0) = p_x
     *   z_cam(1) = p_y
     *   z_cam(2) = theta [mrad]
     *   z_cam(3) = v_x
     *   z_cam(4) = v_y
     *
     * @param[in] z_cam Camera measurement vector.
     */
    void updateCamera(const CamMeasVector& z_cam);

    /**
     * @brief Odometry measurement update.
     *
     * Measurement model:
     *   z_odo(0) = p_x
     *   z_odo(1) = p_y
     *   z_odo(2) = v
     *   z_odo(3) = theta
     *
     * @param[in] z_odo Odometry measurement vector.
     */
    void updateOdometry(const OdoMeasVector& z_odo);

    /**
     * @brief IMU yaw-rate measurement update.
     *
     * Measurement model:
     *   z_imu(0) = omega
     *
     * @param[in] z_imu IMU measurement vector.
     */
    void updateImu(const ImuMeasVector& z_imu);

    /**
     * @brief Get current state estimate.
     *
     * @return const reference to internal state vector.
     */
    const StateVector& getState() const
    {
        return m_state;
    }

private:
    /** State vector. */
    StateVector m_state;

    /** State covariance matrix. */
    StateMatrix m_covariance;

    /** Process noise covariance matrix. */
    StateMatrix m_Q;

    /** Camera measurement noise covariance matrix. */
    CamMeasMatrix m_R_cam;

    /** Odometry measurement noise covariance matrix. */
    OdoMeasMatrix m_R_odo;

    /** IMU measurement noise covariance matrix. */
    ImuMeasMatrix m_R_imu;

    /**
     * @brief Nonlinear process model.
     *
     * @param[in] x   Current state.
     * @param[in] a_x Longitudinal acceleration in mm/s^2.
     * @param[in] dt  Time step in seconds.
     *
     * @return Predicted next state.
     */
    StateVector processModel(const StateVector& x, float a_x, float dt) const;

    /**
     * @brief Jacobian of the process model w.r.t. the state.
     *
     * @param[in] x   Current state.
     * @param[in] a_x Longitudinal acceleration in mm/s^2 (currently unused in Jacobian).
     * @param[in] dt  Time step in seconds.
     *
     * @return Process Jacobian matrix F.
     */
    StateMatrix processJacobianF(const StateVector& x, float a_x, float dt) const;

    /**
     * @brief Camera measurement model.
     *
     * @param[in] x Current state.
     *
     * @return Predicted camera measurement.
     */
    CamMeasVector cameraModel(const StateVector& x) const;

    /**
     * @brief Jacobian of the camera measurement model.
     *
     * @param[in] x Current state.
     *
     * @return Camera Jacobian matrix H_cam.
     */
    Eigen::Matrix<float, CAM_MEAS_DIM, STATE_DIM> cameraJacobianH(const StateVector& x) const;

    /**
     * @brief Odometry measurement model.
     *
     * @param[in] x Current state.
     *
     * @return Predicted odometry measurement.
     */
    OdoMeasVector odometryModel(const StateVector& x) const;

    /**
     * @brief Jacobian of the odometry measurement model.
     *
     * @param[in] x Current state (currently unused).
     *
     * @return Odometry Jacobian matrix H_odo.
     */
    Eigen::Matrix<float, ODO_MEAS_DIM, STATE_DIM> odometryJacobianH(const StateVector& x) const;

    /**
     * @brief IMU measurement model.
     *
     * @param[in] x Current state.
     *
     * @return Predicted IMU measurement.
     */
    ImuMeasVector imuModel(const StateVector& x) const;

    /**
     * @brief Jacobian of the IMU measurement model.
     *
     * @param[in] x Current state (currently unused).
     *
     * @return IMU Jacobian matrix H_imu.
     */
    Eigen::Matrix<float, IMU_MEAS_DIM, STATE_DIM> imuJacobianH(const StateVector& x) const;

    /**
     * @brief Wrap an angle in mrad to [-pi, pi).
     *
     * @param[in] angleMrad Angle in mrad.
     *
     * @return Wrapped angle in mrad.
     */
    static float wrapAngleMrad(float angleMrad);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* EKF_H */
/** @} */
