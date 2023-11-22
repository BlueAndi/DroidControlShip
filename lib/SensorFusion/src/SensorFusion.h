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
 * @brief  Implementation of the Sensor Fusion
 * @author Juliane Kerpe <juliane.kerpe@web.de>
 *
 *
 * @{
 */

#ifndef SENSORFUSION_H
#define SENSORFUSION_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "SerialMuxChannels.h"
#include "LinearKalmanFilter.h"
#include <stdint.h>
#include <SensorConstants.h>
/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class provides a SensorFusion Algorithm. */
class SensorFusion
{
public:
    /**
     * Constructs the SensorFusion Algorithm.
     */
    SensorFusion() : m_linearKalmanFilter(), m_currentPosition{0, 0, 0}
    {
    }

    /**
     * Destroys the SensorFusion
     */
    ~SensorFusion()
    {
    }

    /**
     * Initialize the Sensor Fusion.
     */
    void init();

    /**
     * Perform an update of the Estimated State.
     *
     * @param[in] newSensorData New Sensor Data.
     */
    void estimateNewState(SensorData newSensorData);

    /**
     * Get the Latest calculated State
     *
     * @return Latest Position as a PositionData Struct.
     */
    IKalmanFilter::PositionData getLatestPosition()
    {
        return m_currentPosition;
    }

private:
    LinearKalmanFilter m_linearKalmanFilter; /**< An Instance of the Kalman Filter algorithm class. */

    IKalmanFilter::PositionData m_currentPosition; /**< Variable where the current estimated Position is saved in. */

    /**
     * Transform the local acceleration vector [acc_x, acc_y] into a global vector using the provided angle.
     *
     * @param[in] globalResult                  The array to store the transformed vector [result_x, result_y].
     * @param[in] localVectorToTransform        The local acceleration vector [acc_x, acc_y] to be transformed.
     * @param[in] rotationAngle                 The angle used for the transformation, given in mrad.
     */
    void transformLocalToGlobal(int16_t* globalResult, const int16_t* localVectorToTransform,
                                const int16_t& rotationAngle);

    /**
     * This function combines encoder and magnetometer data to estimate the current angle.
     *
     * @param[in] estimatedAngle   Reference to the variable storing the estimated angle.
     * @param[in] encoderAngle     The angle value obtained from an encoder.
     * @param[in] magnetometerValueX  The X-axis value from a magnetometer.
     * @param[in] magnetometerValueY  The Y-axis value from a magnetometer.
     *
     * @note The function updates the estimatedAngle parameter with the combined angle estimation.
     */
    void estimateAngle(int16_t& estimatedAngle, const int32_t& encoderAngle, const int16_t& magnetometerValueX,
                       const int16_t& magnetometerValueY);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SENSORFUSION_H */
/** @} */
