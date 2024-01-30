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
 */

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  Implementation of the Sensor Fusion
 * @author Juliane Kerpe <juliane.kerpe@web.de>
 *
 * @addtogroup Application
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
#include "ExtendedKalmanFilter.h"
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
    SensorFusion() :
        m_kalmanFilter(),
        m_estimatedPosition{0.0F, 0.0F, 0.0F},
        m_lastOdometryPosition{0.0F, 0.0F, 0.0F},
        m_isFirstIteration(true),
        m_lastOdometryVelocity(0.0F)
    {
    }

    /**
     * Destroys the SensorFusion
     */
    ~SensorFusion()
    {
    }

    /**
     * Perform an update of the Estimated State.
     *
     * @param[in] newSensorData New Sensor Data.
     */
    void estimateNewState(const SensorData& newSensorData);

    /**
     * Get the Latest calculated State
     *
     * @return Latest Position as a PositionData Struct.
     */
    IKalmanFilter::PositionData getLatestPosition()
    {
        return m_estimatedPosition;
    }

private:
    ExtendedKalmanFilter m_kalmanFilter; /**< An Instance of the Kalman Filter algorithm class. */

    IKalmanFilter::PositionData m_estimatedPosition; /**< Variable where the current estimated Position is saved in. */

    IKalmanFilter::PositionData
        m_lastOdometryPosition; /**< Variable where the previous odometry Position is saved in. */

    bool m_isFirstIteration; /**< Flag if the current Iteration is the first one */

    float m_lastOdometryVelocity = 0.0F; /**< The velocity calculated via Odometry of the last iteration. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SENSORFUSION_H */
/** @} */
