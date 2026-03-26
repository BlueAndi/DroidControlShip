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
 * @brief  Extended Kalman Filter with fixed 4D state
 * @author Tobias Haeckel
 *
 * State vector:
 *   x = [ p_x, p_y, theta, v ]^T
 *
 * Units:
 *   p_x, p_y : [mm]
 *   theta    : [mrad]
 *   v        : [mm/s]
 *   omega    : [mrad/s] as control input
 *
 * Measurement models:
 *   - Camera (SSR): absolute position and heading
 *   - Odometry: longitudinal velocity
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
 * @brief Extended Kalman Filter implementation for a fixed 4D state.
 */
class ExtendedKalmanFilter4D
{
public:
    /** State dimension. */
    static constexpr std::uint8_t STATE_DIM = 4U;
    /** Camera measurement dimension: [x, y, theta]. */
    static constexpr std::uint8_t CAM_MEAS_DIM = 3U;
    /** Odometry measurement dimension: [v]. */
    static constexpr std::uint8_t ODO_MEAS_DIM = 1U;

    /** @brief State vector type x = [p_x, p_y, theta, v]^T. */
    using StateVector = Eigen::Matrix<float, STATE_DIM, 1>;

    /** @brief State covariance matrix type P (STATE_DIM x STATE_DIM). */
    using StateMatrix = Eigen::Matrix<float, STATE_DIM, STATE_DIM>;

    /** @brief Camera measurement vector z_cam = [p_x, p_y, theta]^T. */
    using CamMeasurementVector = Eigen::Matrix<float, CAM_MEAS_DIM, 1>;
    /** @brief Camera measurement covariance matrix R_cam. */
    using CamMeasMatrix = Eigen::Matrix<float, CAM_MEAS_DIM, CAM_MEAS_DIM>;

    /** @brief Odometry measurement vector z_odo = [v]^T. */
    using OdoMeasurementVector = Eigen::Matrix<float, ODO_MEAS_DIM, 1>;
    /** @brief Odometry measurement covariance matrix R_odo. */
    using OdoMeasMatrix = Eigen::Matrix<float, ODO_MEAS_DIM, ODO_MEAS_DIM>;

    /**
     * @brief Last valid NIS information for one sensor.
     */
    struct NisData
    {
        float    value;       /**< Last computed NIS value. */
        uint32_t timestampMs; /**< Timestamp of NIS computation in milliseconds. */
        bool     isValid;     /**< True if the NIS value is valid and up to date. */
    };

public:
    /**
     * Constructs the EKF with default noise parameters and zero-initialized state.
     */
    ExtendedKalmanFilter4D();

    /**
     * Initializes the EKF with a given state and covariance.
     *
     * @param[in] x0 Initial state vector (physical units).
     * @param[in] P0 Initial covariance matrix.
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
     */
    void init();

    /**
     * EKF prediction step.
     *
     * Nonlinear state propagation using the motion model:
     *   p_x   += v cos(theta) dt
     *   p_y   += v sin(theta) dt
     *   theta += omega dt
     *   v      = v
     *
     * @param[in] omegaMradPerSec Angular rate input [mrad/s].
     * @param[in] dt              Time step [s].
     */
    void predict(float omegaMradPerSec, float dt);

    /**
     * EKF update step for camera (SSR) measurements.
     *
     * Camera measurement:
     *   z_cam = [p_x, p_y, theta]^T + v_cam
     *
     * All quantities are expected in the same unit system as the state.
     *
     * @param[in] z_cam        Camera measurement vector.
     * @param[in] timestampMs  Measurement timestamp in local ms.
     */
    void updateCamera(const CamMeasurementVector& z_cam, uint32_t timestampMs);

    /**
     * EKF update step for odometry measurements.
     *
     * Odometry measurement:
     *   z_odo = [v_odo]^T
     *
     * Measurement model:
     *   h_odo(x) = [v]^T
     *
     * @param[in] z_odo       Odometry measurement vector.
     * @param[in] timestampMs Measurement timestamp in local ms.
     */
    void updateOdometry(const OdoMeasurementVector& z_odo, uint32_t timestampMs);

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

    /**
     * Get last valid odometry NIS data.
     *
     * @return Const reference to NIS metadata.
     */
    const NisData& getLastOdometryNis() const
    {
        return m_lastNisOdometry;
    }

    /**
     * Get last valid camera NIS data.
     *
     * @return Const reference to NIS metadata.
     */
    const NisData& getLastCameraNis() const
    {
        return m_lastNisCamera;
    }

    /**
     * Convert raw gyro digits to angular rate in mrad/s.
     *
     * @param[in] rawGyroZ Raw gyroscope value around Z [digits].
     *
     * @return Angular rate in mrad/s.
     */
    static float gyroDigitsToMradPerSec(int16_t rawGyroZ);

private:
    /** Current state estimate. */
    StateVector m_state;
    /** Current covariance estimate. */
    StateMatrix m_covariance;

    /** Process noise covariance Q. */
    StateMatrix m_Q;
    /** Camera measurement noise covariance R_cam. */
    CamMeasMatrix m_R_cam;
    /** Odometry measurement noise covariance R_odo. */
    OdoMeasMatrix m_R_odo;
    /** Last valid odometry NIS. */
    NisData m_lastNisOdometry;
    /** Last valid camera NIS. */
    NisData m_lastNisCamera;

private:
    /**
     * Nonlinear process model f(x, omega, dt).
     *
     * @param[in] x                Current state (physical units).
     * @param[in] omegaMradPerSec  Angular rate input [mrad/s].
     * @param[in] dt               Time step [s].
     *
     * @return Predicted next state.
     */
    StateVector processModel(const StateVector& x, float omegaMradPerSec, float dt) const;

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
     * Odometry measurement model h_odo(x) = [v]^T.
     *
     * @param[in] x Current state.
     *
     * @return Odometry measurement prediction.
     */
    OdoMeasurementVector odometryModel(const StateVector& x) const;

    /**
     * Odometry measurement Jacobian H_odo = dh_odo/dx.
     *
     * @param[in] x Current state vector.
     *
     * @return Odometry measurement Jacobian.
     */
    Eigen::Matrix<float, ODO_MEAS_DIM, STATE_DIM> odometryJacobianH(const StateVector& x) const;

    /**
     * Reset NIS metadata after construction or reinitialization.
     */
    void resetNisData();

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
