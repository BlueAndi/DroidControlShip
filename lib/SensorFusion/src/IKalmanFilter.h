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
 *
 */
/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 *  @brief  Interface Class of a Kalman Filter Implementation
 *  @author Juliane Kerpe <juliane.kerpe@web.de>
 */
#ifndef IKALMANFILTER_H
#define IKALMANFILTER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "IKalmanParameter.h"
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
    /**
     * Initializes the variables of the Kalman Filter.
     */
    virtual void init() = 0;

    /**
     * Prediction of the covariance and the state of the Kalman Filter.
     */
    virtual void predictionStep() = 0;

    /**
     * Update of the covariance and the state of the Kalman Filter.
     * @param[in] kalmanParameters   Input Parameters for the Kalman Filter
     *
     */
    virtual void updateStep(IKalmanParameter& kalmanParameters) = 0;
};

#endif /* IKALMANFILTER_H */
