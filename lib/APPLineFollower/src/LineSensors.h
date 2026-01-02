/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
 * @file
 * @brief  Line sensors
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include "SerMuxChannelProvider.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The line sensors class provides the interface to the line sensors.
 * It is used to read the line sensor values and to determine the position of
 * the robot with respect to the line.
 */
class LineSensors
{
public:
    /**
     * Construct line sensors.
     *
     * @param[in] serMuxChannelProvider Serial multiplexer channel provider.
     */
    LineSensors(SerMuxChannelProvider& serMuxChannelProvider);

    /**
     * Destroy line sensors.
     */
    ~LineSensors()
    {
    }

    /**
     * Determines the deviation and returns an estimated position of the robot
     * with respect to a line. The estimate is made using a weighted average of
     * the sensor indices multiplied by 1000, so that a return value of 0
     * indicates that the line is directly below sensor 0, a return value of
     * 1000 indicates that the line is directly below sensor 1, 2000
     * indicates that it's below sensor 2000, etc.  Intermediate values
     * indicate that the line is between two sensors. The formula is:
     *
     *   0*value0 + 1000*value1 + 2000*value2 + ...
     *  --------------------------------------------
     *      value0  +  value1  +  value2 + ...
     *
     * This function assumes a dark line (high values) surrounded by white
     * (low values).
     *
     * @return Estimated position with respect to track.
     */
    int16_t readLine();

    /**
     * Get last line sensor values.
     *
     * @return Line sensor values.
     */
    const uint16_t* getSensorValues() const;

    /**
     * Get number of used line sensors.
     *
     * @return Number of used line sensors
     */
    static uint8_t getNumLineSensors()
    {
        return MAX_SENSORS;
    }

    /**
     * Get max. value of a single line sensor in digits.
     * The sensor value is indirect proportional to the reflectance.
     *
     * @return Max. line sensor value
     */
    static int16_t getSensorValueMax()
    {
        return SENSOR_MAX_VALUE;
    }

private:
    /**
     * Maximum number of line sensors.
     */
    static const size_t MAX_SENSORS =
        sizeof(LineSensorData::lineSensorData) / sizeof(LineSensorData::lineSensorData[0]);

    /**
     * Max. value of a single line sensor in digits.
     * It depends on the Zumo32U4LineSensors implementation.
     */
    static const int16_t SENSOR_MAX_VALUE = 1000;

    /**
     * If above the threshold, it will be assumed that the sensor see's the line.
     */
    static const uint16_t SENSOR_OFF_LINE_THRESHOLD = 200U;

    /**
     * Serial multiplexer protocol channel provider.
     */
    SerMuxChannelProvider& m_serMuxChannelProvider;

    /**
     * Line sensor data [digits] normalized to max. 1000 digits.
     */
    uint16_t m_lineSensorData[MAX_SENSORS];

    /**
     * Last valid line sensor position value.
     */
    int16_t m_lastPosValue;

    /**
     * No default constructor.
     */
    LineSensors();

    /**
     * No copy constructor.
     *
     * @param[in] other Instance to copy from.
     */
    LineSensors(const LineSensors& other);

    /**
     * No assignment operator.
     *
     * @param[in] other Instance to copy from.
     *
     * @return Reference to this instance.
     */
    LineSensors& operator=(const LineSensors& other);

    /**
     * Callback for line sensor channel.
     *
     * @param[in] data Line sensor data.
     */
    void serMuxLineSensorChannelCallback(const LineSensorData& data);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* PARAMETER_SET_H */
/** @} */
