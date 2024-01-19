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
 *  @brief  Channel structure definition for the SerialMuxProt.
 *  @author Juliane Kerpe <juliane.kerpe@web.de>
 *
 *  @addtogroup Application
 *
 * @{
 */

#ifndef SERIAL_MUX_CHANNELS_H_
#define SERIAL_MUX_CHANNELS_H_

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/** Maximum number of SerialMuxProt Channels. */
#define MAX_CHANNELS (10U)

/** Name of Channel to send Sensor Data to. */
#define SENSORDATA_CHANNEL_NAME "SENSOR_DATA"

/** DLC of Sensordata Channel */
#define SENSORDATA_CHANNEL_DLC (sizeof(SensorData))

/** Name of Channel to End Line Detected Signal. */
#define ENDLINE_CHANNEL_NAME "END_LINE"

/** DLC of End Line Channel */
#define ENDLINE_CHANNEL_DLC (sizeof(EndLineFlag))

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Struct of the Sensor Data channel payload. */
typedef struct _SensorData
{
    /** Position in x direction in mm calculated by odometry. */
    int32_t positionOdometryX;

    /** Position in y direction in mm calculated by odometry. */
    int32_t positionOdometryY;

    /** Orientation in mrad calculated by odometry. */
    int32_t orientationOdometry;

    /** Velocity in mm/s calculated by odometry. */
    int16_t velocityOdometry;

    /** Acceleration in x direction as a raw sensor value in digits.
     * It can be converted into a physical acceleration value in mm/s^2 via the
     * multiplication with a sensitivity factor in mm/s^2/digit.
     */
    int16_t accelerationX;

    /** Gyro value around z axis as a raw sensor value in digits.
     * It can be converted into a physical turn rate in mrad/s via the multiplication
     * with a sensitivity factor in mrad/s/digit.
     */
    int16_t turnRate;

    /** Time passed since the last sensor value in milliseconds. */
    uint16_t timePeriod;
} __attribute__((packed)) SensorData;

/** Struct of the End Line Detection payload. */
typedef struct _EndLineFlag
{
    /** Indicates if the End Line has been detected. */
    bool isEndLineDetected;
} __attribute__((packed)) EndLineFlag;

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SERIAL_MUX_CHANNELS_H_ */