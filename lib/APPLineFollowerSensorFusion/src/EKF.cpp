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

    /** Default process noise standard deviations. */
    constexpr float SIGMA_PX      = 1.0F;  /* [mm] */
    constexpr float SIGMA_PY      = 1.0F;  /* [mm] */
    constexpr float SIGMA_THETA_P = 5.0F;  /* [mrad] */
    constexpr float SIGMA_V_P     = 10.0F; /* [mm/s] */

    /** Camera noise. */
    constexpr float SIGMA_CAM_POS   = 2.0F; /* [mm] */
    constexpr float SIGMA_CAM_THETA = 5.0F; /* [mrad] */

    /** Odometry noise. */
    constexpr float SIGMA_ODO_V = 10.0F; /* [mm/s] */

    /** Default initial state. */
    constexpr float EKF_START_X_MM       = 705.0F; /* [mm] */
    constexpr float EKF_START_Y_MM       = 939.0F; /* [mm] */
    constexpr float EKF_START_THETA_MRAD = 0.0F;   /* [mrad] */
    constexpr float EKF_START_V_MMS      = 0.0F;   /* [mm/s] */
} // namespace

/******************************************************************************
 * Public Methods
 *****************************************************************************/

ExtendedKalmanFilter4D::ExtendedKalmanFilter4D()
{
    /* Initialize state and covariance. */
    m_state.setZero();
    m_covariance.setIdentity();

    /* Process noise Q (diagonal). */
    m_Q.setZero();
    m_Q(0, 0) = SIGMA_PX * SIGMA_PX;
    m_Q(1, 1) = SIGMA_PY * SIGMA_PY;
    m_Q(2, 2) = SIGMA_THETA_P * SIGMA_THETA_P;
    m_Q(3, 3) = SIGMA_V_P * SIGMA_V_P;

    /* Camera measurement noise R_cam. */
    m_R_cam.setZero();
    m_R_cam(0, 0) = SIGMA_CAM_POS * SIGMA_CAM_POS;
    m_R_cam(1, 1) = SIGMA_CAM_POS * SIGMA_CAM_POS;
    m_R_cam(2, 2) = SIGMA_CAM_THETA * SIGMA_CAM_THETA;

    /* Odometry measurement noise R_odo. */
    m_R_odo.setZero();
    m_R_odo(0, 0) = SIGMA_ODO_V * SIGMA_ODO_V;

    resetNisData();
}

void ExtendedKalmanFilter4D::init(const StateVector& x0, const StateMatrix& P0)
{
    m_state      = x0;
    m_covariance = P0;
    m_state(2)   = wrapAngleMrad(m_state(2));
    resetNisData();
}

void ExtendedKalmanFilter4D::init()
{
    m_state.setZero();
    m_state(0) = EKF_START_X_MM;
    m_state(1) = EKF_START_Y_MM;
    m_state(2) = EKF_START_THETA_MRAD;
    m_state(3) = EKF_START_V_MMS;

    m_covariance.setIdentity();
    resetNisData();
}

void ExtendedKalmanFilter4D::predict(float omegaMradPerSec, float dt)
{
    /* Nonlinear prediction. */
    StateVector xPred = processModel(m_state, omegaMradPerSec, dt);

    /* Wrap heading angle (index 2). */
    xPred(2) = wrapAngleMrad(xPred(2));

    /* Linearization. */
    StateMatrix jacobianF = processJacobianF(m_state, dt);

    /* Covariance prediction. */
    StateMatrix pPred = jacobianF * m_covariance * jacobianF.transpose() + m_Q;

    /* Commit. */
    m_state      = xPred;
    m_covariance = pPred;
}

void ExtendedKalmanFilter4D::updateCamera(const CamMeasurementVector& z_cam, uint32_t timestampMs)
{
    using CamJacobian = Eigen::Matrix<float, CAM_MEAS_DIM, STATE_DIM>;
    using CamGain     = Eigen::Matrix<float, STATE_DIM, CAM_MEAS_DIM>;

    /* Predicted measurement. */
    const CamMeasurementVector zPred = cameraModel(m_state);
    const CamJacobian          H     = cameraJacobianH(m_state);

    /* Innovation. */
    CamMeasurementVector y = z_cam - zPred;
    y(2)                   = wrapAngleMrad(y(2));

    /* EKF update. */
    const CamMeasMatrix S    = H * m_covariance * H.transpose() + m_R_cam;
    const CamMeasMatrix SInv = S.inverse();
    const CamGain       K    = m_covariance * H.transpose() * SInv;
    const StateMatrix   I    = StateMatrix::Identity();
    const StateMatrix   IKH  = I - K * H;

    m_state      = m_state + K * y;
    m_state(2)   = wrapAngleMrad(m_state(2));
    m_covariance = IKH * m_covariance * IKH.transpose() + K * m_R_cam * K.transpose();

    m_lastNisCamera.value       = y.dot(SInv * y);
    m_lastNisCamera.timestampMs = timestampMs;
    m_lastNisCamera.isValid     = true;
}

void ExtendedKalmanFilter4D::updateOdometry(const OdoMeasurementVector& z_odo, uint32_t timestampMs)
{
    using OdoJacobian = Eigen::Matrix<float, ODO_MEAS_DIM, STATE_DIM>;
    using OdoGain     = Eigen::Matrix<float, STATE_DIM, ODO_MEAS_DIM>;

    /* Predicted measurement. */
    const OdoMeasurementVector zPred = odometryModel(m_state);
    const OdoJacobian          H     = odometryJacobianH(m_state);

    /* Innovation. */
    const OdoMeasurementVector y = z_odo - zPred;

    /* EKF update. */
    const OdoMeasMatrix S    = H * m_covariance * H.transpose() + m_R_odo;
    const OdoMeasMatrix SInv = S.inverse();
    const OdoGain       K    = m_covariance * H.transpose() * SInv;
    const StateMatrix   I    = StateMatrix::Identity();
    const StateMatrix   IKH  = I - K * H;

    m_state      = m_state + K * y;
    m_state(2)   = wrapAngleMrad(m_state(2));
    m_covariance = IKH * m_covariance * IKH.transpose() + K * m_R_odo * K.transpose();

    m_lastNisOdometry.value       = y.dot(SInv * y);
    m_lastNisOdometry.timestampMs = timestampMs;
    m_lastNisOdometry.isValid     = true;
}

float ExtendedKalmanFilter4D::gyroDigitsToMradPerSec(int16_t rawGyroZ)
{
    return static_cast<float>(rawGyroZ) * SensorConstants::GYRO_SENSITIVITY_FACTOR;
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

ExtendedKalmanFilter4D::StateVector ExtendedKalmanFilter4D::processModel(const StateVector& x, float omegaMradPerSec,
                                                                         float dt) const
{
    const float px        = x(0);
    const float py        = x(1);
    const float thetaMrad = x(2);
    const float v         = x(3);
    const float thetaRad  = thetaMrad / 1000.0F;

    StateVector xNext;
    xNext(0) = px + v * std::cos(thetaRad) * dt;
    xNext(1) = py + v * std::sin(thetaRad) * dt;
    xNext(2) = thetaMrad + omegaMradPerSec * dt;
    xNext(3) = v;

    return xNext;
}

ExtendedKalmanFilter4D::StateMatrix ExtendedKalmanFilter4D::processJacobianF(const StateVector& x, float dt) const
{
    const float thetaMrad = x(2);
    const float v         = x(3);
    const float thetaRad  = thetaMrad / 1000.0F;

    StateMatrix F_jacobian = StateMatrix::Identity();

    /* d p_x / d theta */
    F_jacobian(0, 2) = -v * std::sin(thetaRad) * dt / 1000.0F;
    /* d p_x / d v */
    F_jacobian(0, 3) = std::cos(thetaRad) * dt;

    /* d p_y / d theta */
    F_jacobian(1, 2) = v * std::cos(thetaRad) * dt / 1000.0F;
    /* d p_y / d v */
    F_jacobian(1, 3) = std::sin(thetaRad) * dt;

    return F_jacobian;
}

ExtendedKalmanFilter4D::CamMeasurementVector ExtendedKalmanFilter4D::cameraModel(const StateVector& x) const
{
    CamMeasurementVector z;
    z(0) = x(0);
    z(1) = x(1);
    z(2) = x(2);
    return z;
}

Eigen::Matrix<float, ExtendedKalmanFilter4D::CAM_MEAS_DIM, ExtendedKalmanFilter4D::STATE_DIM> ExtendedKalmanFilter4D::
    cameraJacobianH(const StateVector& /*x*/) const
{
    Eigen::Matrix<float, CAM_MEAS_DIM, STATE_DIM> H;
    H.setZero();

    H(0, 0) = 1.0F;
    H(1, 1) = 1.0F;
    H(2, 2) = 1.0F;

    return H;
}

ExtendedKalmanFilter4D::OdoMeasurementVector ExtendedKalmanFilter4D::odometryModel(const StateVector& x) const
{
    OdoMeasurementVector z;
    z(0) = x(3);
    return z;
}

Eigen::Matrix<float, ExtendedKalmanFilter4D::ODO_MEAS_DIM, ExtendedKalmanFilter4D::STATE_DIM> ExtendedKalmanFilter4D::
    odometryJacobianH(const StateVector& /*x*/) const
{
    Eigen::Matrix<float, ODO_MEAS_DIM, STATE_DIM> H;
    H.setZero();
    H(0, 3) = 1.0F;
    return H;
}

void ExtendedKalmanFilter4D::resetNisData()
{
    m_lastNisOdometry.value       = 0.0F;
    m_lastNisOdometry.timestampMs = 0U;
    m_lastNisOdometry.isValid     = false;

    m_lastNisCamera.value       = 0.0F;
    m_lastNisCamera.timestampMs = 0U;
    m_lastNisCamera.isValid     = false;
}

float ExtendedKalmanFilter4D::wrapAngleMrad(float angleMrad)
{
    /* Wrap to [-pi, pi). */
    float x = std::fmod(angleMrad + PI_MRAD, TWO_PI_MRAD);
    if (x < 0.0F)
    {
        x += TWO_PI_MRAD;
    }

    return x - PI_MRAD;
}
