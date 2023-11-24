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
 *  @brief  Channel structure definition for the SerialMuxProt.
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

#ifndef SERIAL_MUX_CHANNELS_H_
#define SERIAL_MUX_CHANNELS_H_

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <Arduino.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/** Maximum number of SerialMuxProt Channels. */
#define MAX_CHANNELS (10U)

/** Name of Channel to send Odometry Data to. */
#define ODOMETRY_CHANNEL_NAME "ODOMETRY"

/** DLC of Odometry Channel */
#define ODOMETRY_CHANNEL_DLC (sizeof(OdometryData))

/** DLC of Line Sensor Channel */
#define LINE_SENSOR_CHANNEL_DLC (sizeof(LineSensorData))

/** Name of Channel to send Traffic Light Color IDs to. */
#define TRAFFIC_LIGHT_COLORS_CHANNEL_NAME "TL_COLORS"

/** DLC of Traffic Light Channel */
#define TRAFFIC_LIGHT_COLORS_CHANNEL_DLC (sizeof(Color))

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Struct of the "Odometry" channel payload. */
typedef struct _OdometryData
{
    int32_t xPos;        /**< X position [mm]. */
    int32_t yPos;        /**< Y position [mm]. */
    int32_t orientation; /**< Orientation [mrad]. */
} __attribute__((packed)) OdometryData;

/** Struct of the "Color" channel payload. */
typedef struct _Color
{
    uint8_t colorId; /**< Command ID */
} __attribute__((packed)) Color;

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SERIAL_MUX_CHANNELS_H_ */