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
 * @brief  Platoon controller class for calculating each step inside a platoon context.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Service
 *
 * @{
 */
#ifndef PLATOON_CONTROLLER_H
#define PLATOON_CONTROLLER_H

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

/**
 * Waypoint structure definition.
 * Defines the position of a waypoint in the map and the speed at which is to be reached.
 */
typedef struct _Waypoint
{
    int32_t xPos;        /**< X position [mm]. */
    int32_t yPos;        /**< Y position [mm]. */
    int32_t orientation; /**< Orientation [mrad]. */
    int16_t left;        /**< Left motor speed [steps/s]. */
    int16_t right;       /**< Right motor speed [steps/s]. */
    int16_t center;      /**< Center speed [steps/s]. */
} __attribute__((packed)) Waypoint;

/**
 * Input waypoint callback.
 * Called in order to get the next waypoint into the platoon controller.
 *
 * @param[out] waypoint   Next waypoint.
 */
typedef void (*InputWaypointCallback)(Waypoint& waypoint);

/**
 * Output waypoint callback.
 * Called in order to send the last waypoint to the next platoon participant.
 *
 * @param[in] waypoint    Last waypoint.
 */
typedef void (*OutputWaypointCallback)(const Waypoint& waypoint);

/**
 * Motor setpoint callback.
 * Called in order to set the motor speeds.
 *
 * @param[in] left      Left motor speed [steps/s].
 * @param[in] right     Right motor speed [steps/s].
 * @param[in] center    Center speed [steps/s].
 */
typedef void (*MotorSetpointCallback)(const int16_t left, const int16_t right, const int16_t center);

/**
 * Platoon controller class for calculating each step inside a platoon context.
 * Follows a waypoint by calculating the speed setpoints for each motor.
 */
class PlatoonController
{
public:
    /**
     * PlatoonController constructor.
     *
     * @param[in] inputWaypointCallback   Input waypoint callback.
     * @param[in] outputWaypointCallback  Output waypoint callback.
     * @param[in] motorSetpointCallback   Motor setpoint callback.
     */
    PlatoonController(InputWaypointCallback inputWaypointCallback, OutputWaypointCallback outputWaypointCallback,
                      MotorSetpointCallback motorSetpointCallback);

    /**
     * PlatoonController default destructor.
     */
    ~PlatoonController();

private:
    /**
     * Input waypoint callback.
     */
    InputWaypointCallback m_inputWaypointCallback;

    /**
     * Output waypoint callback.
     */
    OutputWaypointCallback m_outputWaypointCallback;

    /**
     * Motor setpoint callback.
     */
    MotorSetpointCallback m_motorSetpointCallback;

private:
    /**
     * PlatoonController default constructor.
     */
    PlatoonController();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* PLATOON_CONTROLLER_H */
/** @} */
