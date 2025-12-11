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
 *   x = [ p_x, p_y, theta, v, omega ]^T
 *
 * Units:
 *   p_x, p_y : [mm]
 *   theta    : [mrad]
 *   v        : [mm/s]
 *   omega    : [mrad/s]
 *
 * Measurement models:
 *   - Camera (SSR): absolute pose and Cartesian velocity in world frame
 *   - Odometry: longitudinal velocity and heading
 *   - IMU: yaw rate (simplified model, can be extended later)
 */

#ifndef EKF_H
#define EKF_H

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <ArduinoEigen.h>
#include <stdint.h>

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
    static constexpr std::uint8_t STATE_DIM    = 5U;
    /** Camera measurement dimension: [x, y, theta, v_x, v_y]. */
    static constexpr std::uint8_t CAM_MEAS_DIM = 5U;
    /** Odometry measurement dimension: [v, theta]. */
    static constexpr std::uint8_t ODO_MEAS_DIM = 2U;
    /** IMU measurement dimension: [omega]. */
    static constexpr std::uint8_t IMU_MEAS_DIM = 1U;

    /** @brief State vector type x = [p_x, p_y, theta, v, omega]^T. */
    using StateVector   = Eigen::Matrix<float, STATE_DIM, 1>;

    /** @brief State covariance matrix type P (STATE_DIM x STATE_DIM). */
    using StateMatrix   = Eigen::Matrix<float, STATE_DIM, STATE_DIM>;

    /** @brief Camera measurement vector z_cam = [p_x, p_y, theta, v_x, v_y]^T. */
    using CamMeasurementVector = Eigen::Matrix<float, CAM_MEAS_DIM, 1>;
    /** @brief Camera measurement covariance matrix R_cam. */
    using CamMeasMatrix = Eigen::Matrix<float, CAM_MEAS_DIM, CAM_MEAS_DIM>;

    /** @brief Odometry measurement vector z_odo = [v, theta]^T. */
    using OdoMeasurementVector = Eigen::Matrix<float, ODO_MEAS_DIM, 1>;
    /** @brief Odometry measurement covariance matrix R_odo. */
    using OdoMeasMatrix = Eigen::Matrix<float, ODO_MEAS_DIM, ODO_MEAS_DIM>;

    /** @brief IMU measurement vector z_imu = [omega]^T. */
    using ImuMeasurementVector = Eigen::Matrix<float, IMU_MEAS_DIM, 1>;
    /** @brief IMU measurement covariance matrix R_imu. */
    using ImuMeasMatrix = Eigen::Matrix<float, IMU_MEAS_DIM, IMU_MEAS_DIM>;


public:
    /**
     * Constructs the EKF with default noise parameters and zero-initialized state.
     */
    ExtendedKalmanFilter5D();

    /**
     * Initializes the EKF with a given state and covariance.
     *
     * @param[in] x0 Initial state vector (physical units).
     * @param[in] P0 Initial covariance matrix.
     *
     */
    void init(const StateVector& x0, const StateMatrix& P0);

    /**
     * Initializes the EKF with a fixed default start pose and identity covariance.
     *
     * State is initialized to:
     *   p_x   = EKF_START_X_MM
     *   p_y   = EKF_START_Y_MM
     *   theta = EKF_START_THETA_MRAD
     *   v     = 0
     *   omega = 0
     */
    void init();

    /**
     * EKF prediction step.
     *
     * Nonlinear state propagation using the motion model:
     *   p_x   += v cos(theta) dt
     *   p_y   += v sin(theta) dt
     *   theta += omega dt
     *   v     += a_x dt
     *   omega  = omega          (no angular acceleration modeled)
     *
     * @param[in] accX_raw Raw longitudinal acceleration from IMU [digits].
     * @param[in] dt       Time step [s].
     *
     * The raw acceleration is internally converted into [mm/s^2].
     */
    void predict(float accX_raw, float dt);

    /**
     * EKF update step for camera (SSR) measurements.
     *
     * Camera measurement:
     *   z_cam = [p_x, p_y, theta, v_x, v_y]^T + v_cam
     *
     * All quantities are expected in the same unit system as the state.
     *
     * @param[in] z_cam Camera measurement vector.
     */
    void updateCamera(const CamMeasurementVector& z_cam);

    /**
     * EKF update step for odometry measurements.
     *
     * Odometry measurement (drift-reduced):
     *   z_odo = [v_odo, theta_odo]^T
     *
     * Measurement model:
     *   h_odo(x) = [ v, theta ]^T
     *
     * @param[in] z_odo Odometry measurement vector.
     */
    void updateOdometry(const OdoMeasurementVector& z_odo);

    /**
     * EKF update step for IMU measurements (physical units).
     *
     * IMU measurement (simplified):
     *   z_imu = [omega]^T, omega in [mrad/s]
     *
     * Measurement model:
     *   h_imu(x) = [ omega ]^T
     *
     * @param[in] z_imu IMU measurement vector (omega in mrad/s).
     */
    void updateImu(const ImuMeasurementVector& z_imu);

    /**
     * EKF update step for IMU yaw rate using raw gyro digits.
     *
     * The raw gyro Z-axis value is internally converted to [mrad/s]
     * using SensorConstants::GYRO_SENSITIVITY_FACTOR.
     *
     * @param[in] rawGyroZ Raw gyroscope value around Z [digits].
     */
    void updateImuFromDigits(int16_t rawGyroZ);

    /**
     * Get current state estimate.
     *
     * @return Const reference to state vector.
     */
    const StateVector& getState() const
    {
        return m_state;
    }

    /**
     * Get current covariance estimate.
     *
     * @return Const reference to covariance matrix.
     */
    const StateMatrix& getCovariance() const
    {
        return m_covariance;
    }

private:
    /** Current state estimate. */
    StateVector m_state;
    /** Current covariance estimate. */
    StateMatrix m_covariance;

    /** Process noise covariance Q. */
    StateMatrix   m_Q;
    /** Camera measurement noise covariance R_cam. */
    CamMeasMatrix m_R_cam;
    /** Odometry measurement noise covariance R_odo. */
    OdoMeasMatrix m_R_odo;
    /** IMU measurement noise covariance R_imu. */
    ImuMeasMatrix m_R_imu;

private:
    /**
     * Nonlinear process model f(x, a_x, dt).
     *
     * @param[in] x        Current state (physical units).
     * @param[in] a_x_mms  Longitudinal acceleration [mm/s^2].
     * @param[in] dt       Time step [s].
     *
     * @return Predicted next state.
     */
    StateVector processModel(const StateVector& x, float a_x_mms, float dt) const;

    /**
     * Jacobian of the process model F = df/dx.
     *
     * @param[in] x  Current state vector.
     * @param[in] dt Time step [s].
     *
     * @return Process Jacobian matrix.
     */
    StateMatrix processJacobianF(const StateVector& x, float dt) const;

    /**
     * Camera measurement model h_cam(x).
     *
     * @param[in] x Current state.
     *
     * @return Camera measurement prediction.
     */
    CamMeasurementVector cameraModel(const StateVector& x) const;

    /**
     * Camera measurement Jacobian H_cam = dh_cam/dx.
     *
     * @param[in] x Current state.
     *
     * @return Camera measurement Jacobian.
     */
    Eigen::Matrix<float, CAM_MEAS_DIM, STATE_DIM> cameraJacobianH(const StateVector& x) const;

    /**
     * Odometry measurement model h_odo(x) = [v, theta]^T.
     *
     * @param[in] x Current state.
     *
     * @return Odometry measurement prediction.
     */
    OdoMeasurementVector odometryModel(const StateVector& x) const;

    /**
     * Odometry measurement Jacobian H_odo = dh_odo/dx.
     *
     * @return Odometry measurement Jacobian.
     * 
     * @param[in] x Current state vector.
     */
    Eigen::Matrix<float, ODO_MEAS_DIM, STATE_DIM> odometryJacobianH(const StateVector& x) const;

    /**
     * IMU measurement model h_imu(x) = [omega]^T.
     *
     * @param[in] x Current state vector.
     *
     * @return IMU measurement prediction.
     */
    ImuMeasurementVector imuModel(const StateVector& x) const;

    /**
     * IMU measurement Jacobian H_imu = dh_imu/dx.
     *
     * @param[in] x Current state vector.
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

#endif /* EKF_H */
