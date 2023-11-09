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

/** This class provides a Linear Kalman filter. */
class LinearKalmanFilter : public IKalmanFilter
{
public:
    /**
     * Constructs the Linear Kalman Filter
     */
    LinearKalmanFilter()
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
    void init()
    {
        // TODO: Implement Kalman Filter in cpp (TD072)
    }

    /**
     * Prediction of the covariance and the state of the Linear Kalman Filter.
     */
    void predictionStep()
    {
        // TODO: Implement Kalman Filter in cpp (TD072)
    }

    /**
     * Update of the covariance and the state of the Kalman Filter.
     * @param[in] kalmanParameter   Input Parameters for the Kalman Filter as a KalmanParameter struct.
     * @return Estimated Position as a PositionData struct.
     */
    PositionData updateStep(KalmanParameter& kalmanParameter)
    {
        PositionData currentPosition;
        currentPosition.currentXPos    = 0;
        currentPosition.currentYPos    = 0;
        currentPosition.currentHeading = 0;
        return currentPosition;
        // TODO: Implement Kalman Filter in cpp (TD072)
    }

private:
    Eigen::VectorXf m_state;      /**< Estimated state vector [p_x, p_y, v_x, v_y, a_x, a_y]*/
    Eigen::MatrixXf m_covariance; /**< Covariance Matrix of the state */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* LINEARKALMANFILTER_H */
/** @} */
