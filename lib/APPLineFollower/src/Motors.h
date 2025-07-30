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
 * @brief  Motors
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef MOTORS_H
#define MOTORS_H

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
 * The motors class provides the interface to the motors.
 * It is used to set the speed of the motors.
 */
class Motors
{
public:
    /**
     * Construct motors.
     *
     * @param[in] serMuxChannelProvider Serial multiplexer channel provider.
     */
    Motors(SerMuxChannelProvider& serMuxChannelProvider);

    /**
     * Destroy motors.
     */
    ~Motors()
    {
    }

    /**
     * Sets the speeds for both motors.
     *
     * @param[in] leftSpeed A number from -400 to 400 representing the speed and
     * direction of the right motor. Values of -400 or less result in full speed
     * reverse, and values of 400 or more result in full speed forward.
     * @param[in] rightSpeed A number from -400 to 400 representing the speed and
     * direction of the right motor. Values of -400 or less result in full speed
     * reverse, and values of 400 or more result in full speed forward.
     */
    void setSpeeds(int16_t leftSpeed, int16_t rightSpeed);

    /**
     * Get maximum speed of the motors in digits.
     *
     * @return Max. speed in digits
     */
    int16_t getMaxSpeed() const
    {
        return m_maxMotorSpeed;
    }

    /**
     * Set maximum speed of the motors in digits.
     *
     * @param[in] maxSpeed Max. speed in digits
     */
    void setMaxSpeed(int16_t maxSpeed)
    {
        m_maxMotorSpeed = maxSpeed;
    }

private:

    /**
     * Serial multiplexer protocol channel provider.
     */
    SerMuxChannelProvider& m_serMuxChannelProvider;

    /**
     * Max. motor speed in digits.
     */
    int16_t m_maxMotorSpeed;

    /**
     * No default constructor.
     */
    Motors() = delete;

    /**
     * No copy constructor.
     *
     * @param[in] other Instance to copy from.
     */
    Motors(const Motors&) = delete;

    /**
     * No assignment operator.
     *
     * @param[in] other Instance to copy from.
     *
     * @return Reference to this instance.
     */
    Motors& operator=(const Motors&) = delete;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* MOTORS_H */
/** @} */
