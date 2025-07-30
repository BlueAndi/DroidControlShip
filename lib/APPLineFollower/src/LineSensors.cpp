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
 * @brief  Line sensors
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "LineSensors.h"
#include <Logging.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

LineSensors::LineSensors(SerMuxChannelProvider& serMuxChannelProvider) :
    m_serMuxChannelProvider(serMuxChannelProvider),
    m_lineSensorData{0, 0, 0, 0, 0},
    m_lastPosValue(0)
{
    /* Register the line sensor channel callback. */
    m_serMuxChannelProvider.registerLineSensorCallback([this](const LineSensorData& data) -> void
                                                       { this->serMuxLineSensorChannelCallback(data); });
}

int16_t LineSensors::readLine()
{
    uint32_t       estimatedPos = 0U;
    uint32_t       numerator    = 0U;
    uint32_t       denominator  = 0U;
    const uint32_t WEIGHT       = 1000U;
    bool           isOnLine     = false;

    for (uint32_t idx = 0U; idx < MAX_SENSORS; ++idx)
    {
        uint32_t sensorValue = m_lineSensorData[idx];

        numerator += idx * WEIGHT * sensorValue;
        denominator += sensorValue;

        /* Keep track of whether we see the line at all. */
        if (SENSOR_OFF_LINE_THRESHOLD < sensorValue)
        {
            isOnLine = true;
        }
    }

    if (false == isOnLine)
    {
        /* If it last read to the left of center, return 0. */
        if (m_lastPosValue < (((MAX_SENSORS - 1U) * SENSOR_MAX_VALUE) / 2U))
        {
            estimatedPos = 0U;
        }
        /* If it last read to the right of center, return the max. value. */
        else
        {
            estimatedPos = (MAX_SENSORS - 1U) * SENSOR_MAX_VALUE;
        }
    }
    else
    {
        /* Check to avoid division by zero. */
        if (0 != denominator)
        {
            estimatedPos = numerator / denominator;
        }

        m_lastPosValue = estimatedPos;
    }

    return static_cast<int16_t>(estimatedPos);
}

const uint16_t* LineSensors::getSensorValues() const
{
    return m_lineSensorData;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void LineSensors::serMuxLineSensorChannelCallback(const LineSensorData& data)
{
    size_t idx;

    /* Copy the line sensor data. */
    for (idx = 0U; idx < MAX_SENSORS; ++idx)
    {
        m_lineSensorData[idx] = data.lineSensorData[idx];
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
