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
 * @brief  Concrete Longitudinal Safety Policy.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef LONGITUDINAL_SAFETY_POLICY_H
#define LONGITUDINAL_SAFETY_POLICY_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <ILongitudinalSafetyPolicy.h>
#include <new>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Concrete Longitudinal Safety Policy. */
class LongitudinalSafetyPolicy : public ILongitudinalSafetyPolicy
{
public:
    /**
     * Longitudinal Safety Policy constructor.
     */
    LongitudinalSafetyPolicy();

    /**
     * Longitudinal Safety Policy destructor.
     */
    virtual ~LongitudinalSafetyPolicy();

    /**
     * Creates a LongitudinalSafetyPolicy instance, for registering in the ProcessingChainFactory.
     *
     * @return If successful, returns a pointer to the LongitudinalSafetyPolicy instance. Otherwise nullptr.
     */
    static ILongitudinalSafetyPolicy* create()
    {
        return new (std::nothrow) LongitudinalSafetyPolicy();
    }

    /**
     * Checks if the longitudinal safety policy is satisfied by the calculated center speed.
     * If not, the center speed is adjusted accordingly.
     *
     * @param[in,out]   centerSpeedSetpoint  Center speed [steps/s].
     *
     * @return True is satisfied, otherwise false.
     */
    bool check(int16_t& centerSpeedSetpoint) final;

private:
    /** Maximum motor speed in encoder steps/s */
    static const int16_t MAX_MOTOR_SPEED = 2400;

    /** Minimum motor speed in encoder steps/s */
    static const int16_t MIN_MOTOR_SPEED = 0;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* LONGITUDINAL_SAFETY_POLICY_H */
/** @} */
