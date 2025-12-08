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
 * @brief  Implementation of the Extended Kalman Filter
 * @author Tobias Haeckel
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "EKF.h"
#include <cmath>
#include "SensorConstants.h"

namespace
{
    constexpr float PI_MRAD     = 1000.0F * static_cast<float>(M_PI);
    constexpr float TWO_PI_MRAD = 2.0F * PI_MRAD;

    // Default process noise standard deviations.
    constexpr float SIGMA_PX      = 1.0F;  // [mm]
    constexpr float SIGMA_PY      = 1.0F;  // [mm]
    constexpr float SIGMA_THETA_P = 5.0F;  // [mrad]
    constexpr float SIGMA_V_P     = 5.0F;  // [mm/s]
    constexpr float SIGMA_OMEGA_P = 2.0F;  // [mrad/s]

    // Camera noise.
    constexpr float SIGMA_CAM_POS   = 2.0F; // [mm]
    constexpr float SIGMA_CAM_THETA = 5.0F;  // [mrad]
    constexpr float SIGMA_CAM_V     = 20.0F; // [mm/s]

    // Odometry noise (only v and theta).
    constexpr float SIGMA_ODO_V     = 10.0F; // [mm/s]
    constexpr float SIGMA_ODO_THETA = 5.0F;  // [mrad]

    // IMU yaw noise.
    constexpr float SIGMA_IMU_OMEGA = 1.0F;  // [mrad/s]

    // Default initial state.
    constexpr float EKF_START_X_MM         = 705.0F; // [mm]
    constexpr float EKF_START_Y_MM         = 939.0F; // [mm]
    constexpr float EKF_START_THETA_MRAD   = 0.0F;   // [mrad]
    constexpr float EKF_START_V_MMS        = 0.0F;   // [mm/s]
    constexpr float EKF_START_OMEGA_MRADPS = 0.0F;   // [mrad/s]
}

/******************************************************************************
 * Public Methods
 *****************************************************************************/

ExtendedKalmanFilter5D::ExtendedKalmanFilter5D()
{
    /* Initialize state and covariance. */
    m_state.setZero();
    m_covariance.setIdentity();

    /* Process noise Q (diagonal). */
    m_Q.setZero();
    m_Q(0, 0) = SIGMA_PX      * SIGMA_PX;
    m_Q(1, 1) = SIGMA_PY      * SIGMA_PY;
    m_Q(2, 2) = SIGMA_THETA_P * SIGMA_THETA_P;
    m_Q(3, 3) = SIGMA_V_P     * SIGMA_V_P;
    m_Q(4, 4) = SIGMA_OMEGA_P * SIGMA_OMEGA_P;

    /* Camera measurement noise R_cam. */
    m_R_cam.setZero();
    m_R_cam(0, 0) = SIGMA_CAM_POS   * SIGMA_CAM_POS;
    m_R_cam(1, 1) = SIGMA_CAM_POS   * SIGMA_CAM_POS;
    m_R_cam(2, 2) = SIGMA_CAM_THETA * SIGMA_CAM_THETA;
    m_R_cam(3, 3) = SIGMA_CAM_V     * SIGMA_CAM_V;
    m_R_cam(4, 4) = SIGMA_CAM_V     * SIGMA_CAM_V;

    /* Odometry measurement noise R_odo. */
    m_R_odo.setZero();
    m_R_odo(0, 0) = SIGMA_ODO_V     * SIGMA_ODO_V;
    m_R_odo(1, 1) = SIGMA_ODO_THETA * SIGMA_ODO_THETA;

    /* IMU yaw noise R_imu. */
    m_R_imu.setZero();
    m_R_imu(0, 0) = SIGMA_IMU_OMEGA * SIGMA_IMU_OMEGA;
}

bool ExtendedKalmanFilter5D::init(const StateVector& x0, const StateMatrix& P0)
{
    m_state      = x0;
    m_covariance = P0;
    return true;
}

bool ExtendedKalmanFilter5D::init()
{
    m_state.setZero();
    m_state(0) = EKF_START_X_MM;
    m_state(1) = EKF_START_Y_MM;
    m_state(2) = EKF_START_THETA_MRAD;
    m_state(3) = EKF_START_V_MMS;
    m_state(4) = EKF_START_OMEGA_MRADPS;

    m_covariance.setIdentity();
    return true;
}

void ExtendedKalmanFilter5D::predict(float accX_raw, float dt)
{
    /* Convert raw accelerometer digits to physical acceleration [mm/s^2]. */
    const float a_x_mms =
        accX_raw * SensorConstants::ACCELEROMETER_SENSITIVITY_FACTOR;

    /* Nonlinear prediction. */
    StateVector x_pred = processModel(m_state, a_x_mms, dt);

    /* Wrap heading angle (index 2). */
    x_pred(2) = wrapAngleMrad(x_pred(2));

    /* Linearization. */
    StateMatrix jacobianF = processJacobianF(m_state, dt);

    /* Covariance prediction. */
    StateMatrix P_pred = jacobianF * m_covariance * jacobianF.transpose() + m_Q;

    /* Commit. */
    m_state      = x_pred;
    m_covariance = P_pred;
}

void ExtendedKalmanFilter5D::updateCamera(const CamMeasVector& z_cam)
{
    /* Predicted measurement. */
    CamMeasVector z_pred = cameraModel(m_state);
    auto          H      = cameraJacobianH(m_state);

    /* Innovation. */
    CamMeasVector y = z_cam - z_pred;

    /* Wrap angle innovation (index 2 is theta). */
    y(2) = wrapAngleMrad(y(2));

    /* EKF update. */
    const CamMeasMatrix S = H * m_covariance * H.transpose() + m_R_cam;
    const Eigen::Matrix<float, STATE_DIM, CAM_MEAS_DIM> K =
        m_covariance * H.transpose() * S.inverse();

    m_state      = m_state + K * y;
    m_state(2)   = wrapAngleMrad(m_state(2));
    m_covariance = (StateMatrix::Identity() - K * H) * m_covariance;
}

void ExtendedKalmanFilter5D::updateOdometry(const OdoMeasVector& z_odo)
{
    /* Predicted measurement. */
    OdoMeasVector z_pred = odometryModel(m_state);
    auto          H      = odometryJacobianH(m_state);

    /* Innovation. */
    OdoMeasVector y = z_odo - z_pred;

    /* Wrap angle innovation (index 1 is theta). */
    y(1) = wrapAngleMrad(y(1));

    /* EKF update. */
    const OdoMeasMatrix S = H * m_covariance * H.transpose() + m_R_odo;
    const Eigen::Matrix<float, STATE_DIM, ODO_MEAS_DIM> K =
        m_covariance * H.transpose() * S.inverse();

    m_state      = m_state + K * y;
    m_state(2)   = wrapAngleMrad(m_state(2));
    m_covariance = (StateMatrix::Identity() - K * H) * m_covariance;
}

void ExtendedKalmanFilter5D::updateImu(const ImuMeasVector& z_imu)
{
    /* Predicted measurement. */
    ImuMeasVector z_pred = imuModel(m_state);
    auto          H      = imuJacobianH(m_state);

    /* Innovation (omega, no wrapping). */
    ImuMeasVector y = z_imu - z_pred;

    /* EKF update. */
    const ImuMeasMatrix S = H * m_covariance * H.transpose() + m_R_imu;
    const Eigen::Matrix<float, STATE_DIM, IMU_MEAS_DIM> K =
        m_covariance * H.transpose() * S.inverse();

    m_state      = m_state + K * y;
    m_state(2)   = wrapAngleMrad(m_state(2));
    m_covariance = (StateMatrix::Identity() - K * H) * m_covariance;
}

void ExtendedKalmanFilter5D::updateImuFromDigits(int16_t rawGyroZ)
{
    ImuMeasVector z_imu;
    /* Convert raw gyro digits to physical yaw rate [mrad/s]. */
    z_imu(0) = static_cast<float>(rawGyroZ) *
               SensorConstants::GYRO_SENSITIVITY_FACTOR;

    updateImu(z_imu);
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

ExtendedKalmanFilter5D::StateVector
ExtendedKalmanFilter5D::processModel(const StateVector& x,
                                     float              a_x_mms,
                                     float              dt) const
{
    const float px        = x(0);
    const float py        = x(1);
    const float thetaMrad = x(2);
    const float v         = x(3);
    const float omegaMrad = x(4);          /* already in mrad/s */

    const float thetaRad  = thetaMrad / 1000.0F;

    StateVector x_next;
    x_next(0) = px + v * std::cos(thetaRad) * dt; /* p_x */
    x_next(1) = py + v * std::sin(thetaRad) * dt; /* p_y */
    x_next(2) = thetaMrad + omegaMrad * dt;       /* theta [mrad] */
    x_next(3) = v + a_x_mms * dt;                 /* v */
    x_next(4) = omegaMrad;                        /* omega */

    return x_next;
}

ExtendedKalmanFilter5D::StateMatrix
ExtendedKalmanFilter5D::processJacobianF(const StateVector& x, float dt) const
{
    const float thetaMrad = x(2);
    const float v         = x(3);
    const float thetaRad  = thetaMrad / 1000.0F;

    StateMatrix F_jacobian = StateMatrix::Identity();

    /* d p_x / d theta. */
    F_jacobian(0, 2) = -v * std::sin(thetaRad) * dt / 1000.0F;
    /* d p_x / d v. */
    F_jacobian(0, 3) =  std::cos(thetaRad) * dt;

    /* d p_y / d theta. */
    F_jacobian(1, 2) =  v * std::cos(thetaRad) * dt / 1000.0F;
    /* d p_y / d v. */
    F_jacobian(1, 3) =  std::sin(thetaRad) * dt;

    /* d theta / d omega. */
    F_jacobian(2, 4) = dt;

    return F_jacobian;
}

/* ---------------- Camera model ---------------- */

ExtendedKalmanFilter5D::CamMeasVector
ExtendedKalmanFilter5D::cameraModel(const StateVector& x) const
{
    const float px        = x(0);
    const float py        = x(1);
    const float thetaMrad = x(2);
    const float v         = x(3);

    const float thetaRad  = thetaMrad / 1000.0F;

    CamMeasVector z;
    z(0) = px;                     /* p_x */
    z(1) = py;                     /* p_y */
    z(2) = thetaMrad;              /* theta [mrad] */
    z(3) = v * std::cos(thetaRad); /* v_x */
    z(4) = v * std::sin(thetaRad); /* v_y */
    return z;
}

Eigen::Matrix<float, ExtendedKalmanFilter5D::CAM_MEAS_DIM, ExtendedKalmanFilter5D::STATE_DIM>
ExtendedKalmanFilter5D::cameraJacobianH(const StateVector& x) const
{
    const float thetaMrad = x(2);
    const float v         = x(3);
    const float thetaRad  = thetaMrad / 1000.0F;

    Eigen::Matrix<float, CAM_MEAS_DIM, STATE_DIM> H;
    H.setZero();

    /* p_x */
    H(0, 0) = 1.0F;
    /* p_y */
    H(1, 1) = 1.0F;
    /* theta */
    H(2, 2) = 1.0F;

    /* v_x = v cos(theta) */
    H(3, 2) = -v * std::sin(thetaRad) * (1.0F / 1000.0F); /* d v_x / d theta */
    H(3, 3) =  std::cos(thetaRad);                        /* d v_x / d v */

    /* v_y = v sin(theta) */
    H(4, 2) =  v * std::cos(thetaRad) * (1.0F / 1000.0F); /* d v_y / d theta */
    H(4, 3) =  std::sin(thetaRad);                        /* d v_y / d v */

    return H;
}

/* ---------------- Odometry model ---------------- */

ExtendedKalmanFilter5D::OdoMeasVector
ExtendedKalmanFilter5D::odometryModel(const StateVector& x) const
{
    OdoMeasVector z;
    z(0) = x(3); /* v */
    z(1) = x(2); /* theta */
    return z;
}

Eigen::Matrix<float, ExtendedKalmanFilter5D::ODO_MEAS_DIM, ExtendedKalmanFilter5D::STATE_DIM>
ExtendedKalmanFilter5D::odometryJacobianH(const StateVector& /*x*/) const
{
    Eigen::Matrix<float, ODO_MEAS_DIM, STATE_DIM> H;
    H.setZero();

    /* v */
    H(0, 3) = 1.0F;
    /* theta */
    H(1, 2) = 1.0F;

    return H;
}

/* ---------------- IMU yaw model ---------------- */

ExtendedKalmanFilter5D::ImuMeasVector
ExtendedKalmanFilter5D::imuModel(const StateVector& x) const
{
    ImuMeasVector z;
    z(0) = x(4); /* omega [mrad/s] */
    return z;
}

Eigen::Matrix<float, ExtendedKalmanFilter5D::IMU_MEAS_DIM, ExtendedKalmanFilter5D::STATE_DIM>
ExtendedKalmanFilter5D::imuJacobianH(const StateVector& /*x*/) const
{
    Eigen::Matrix<float, IMU_MEAS_DIM, STATE_DIM> H;
    H.setZero();
    H(0, 4) = 1.0F; /* d omega / d state */
    return H;
}

/* ---------------- Angle wrapping ---------------- */

float ExtendedKalmanFilter5D::wrapAngleMrad(float angleMrad)
{
    /* Wrap to [-pi, pi). */
    float x = std::fmod(angleMrad + PI_MRAD, TWO_PI_MRAD);
    if (x < 0.0F)
    {
        x += TWO_PI_MRAD;
    }

    return x - PI_MRAD;
}
