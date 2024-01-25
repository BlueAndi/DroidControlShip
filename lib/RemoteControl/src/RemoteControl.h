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
 * @brief  RemoteControl common constants.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup App
 *
 * @{
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** RemoteControl application constants */
namespace RemoteControl
{
    /** Remote control commands. */
    typedef enum : uint8_t
    {
        CMD_ID_IDLE = 0,                /**< Nothing to do. */
        CMD_ID_START_LINE_SENSOR_CALIB, /**< Start line sensor calibration. */
        CMD_ID_START_MOTOR_SPEED_CALIB, /**< Start motor speed calibration. */
        CMD_ID_REINIT_BOARD,            /**< Re-initialize the board. Required for webots simulation. */
        CMD_ID_GET_MAX_SPEED,           /**< Get maximum speed. */

    } CmdId;

    /** Remote control command responses. */
    typedef enum : uint8_t
    {
        RSP_ID_OK = 0,  /**< Command successful executed. */
        RSP_ID_PENDING, /**< Command is pending. */
        RSP_ID_ERROR    /**< Command failed. */

    } RspId;
} /* namespace RemoteControl */

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* REMOTE_CONTROL_H */
/** @} */
