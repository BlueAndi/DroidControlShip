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
 *
 */
/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 *  @brief  Interface Class of a Kalman Filter Implementation
 *  @author Juliane Kerpe <juliane.kerpe@web.de>
 *
 *  @addtogroup Application
 *
 * @{
 */

#ifndef IKALMANFILTER_H
#define IKALMANFILTER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "KalmanParameter.h"
/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/
/** The Kalman Filter Interface. */
class IKalmanFilter
{

public:
    /** Results of the Sensor Fusion Algorithm. */
    typedef struct _PositionData
    {
        float positionX; /**< Current Position in x direction in mm. */
        float positionY; /**< Current Position in y direction in mm. */
        float angle;     /**< Current Orientation in in mrad. */
    } PositionData;

    /**
     * Initializes the variables of the Kalman Filter.
     *
     * @param[in] initialParameter Parameters which should be used to initialize the filter.
     */
    virtual void init(KalmanParameter& initialParameter) = 0;

    /**
     * Prediction of the covariance and the state of the Kalman Filter.
     * @param[in] timeStep Measured Time Step in ms.
     */
    virtual void predictionStep(const uint16_t timeStep) = 0;

    /**
     * Update of the covariance and the state of the Kalman Filter.
     * @param[in] kalmanParameter   Input Parameters for the Kalman Filter
     * @return Estimated Position as a PositionData struct.
     *
     */
    virtual PositionData updateStep(KalmanParameter& kalmanParameter) = 0;

private:
    /**
     * Writes data from a KalmanParameter Struct into the Measurement Vector as a member variable
     *
     * @param[in] kalmanParameter   Input Parameters for the Kalman Filter as a KalmanParameter struct.
     */
    virtual void updateMeasurementVector(KalmanParameter& kalmanParameter) = 0;

    /**
     * Writes data from a KalmanParameter Struct into the Control Input Vector as a member variable
     *
     * @param[in] kalmanParameter   Input Parameters for the Kalman Filter as a KalmanParameter struct.
     */
    virtual void updateControlInputVector(KalmanParameter& kalmanParameter) = 0;
};

#endif /* IKALMANFILTER_H */
/** @} */
