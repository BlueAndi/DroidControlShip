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
 * @brief  Interface for a Lateral Safety Policy.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Service
 *
 * @{
 */
#ifndef I_LATERAL_SAFETY_POLICY_H
#define I_LATERAL_SAFETY_POLICY_H

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

/** Abstract lateral safety policy interface. */
class ILateralSafetyPolicy
{
public:
    /**
     * ILateralSafetyPolicy creation function, used by the ProcessingChainFactory to create a
     * ILateralSafetyPolicy instance.
     */
    typedef ILateralSafetyPolicy* (*CreateFunc)(void);

    /**
     * Destroys the interface.
     */
    virtual ~ILateralSafetyPolicy()
    {
    }

    /**
     * Checks if the lateral safety policy is satisfied by the calculated motor speeds.
     * If not, the motor speeds are adjusted accordingly.
     *
     * @param[in,out]   leftMotorSpeedSetpoint  Left motor speed [steps/s].
     * @param[in,out]   rightMotorSpeedSetpoint Right motor speed [steps/s].
     *
     * @return True is satisfied, otherwise false.
     */
    virtual bool check(int16_t& leftMotorSpeedSetpoint, int16_t& rightMotorSpeedSetpoint) = 0;

private:
    /**
     * Constructs the interface.
     */
    ILateralSafetyPolicy()
    {
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* I_LATERAL_SAFETY_POLICY_H */
/** @} */
