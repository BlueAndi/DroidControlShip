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
 * @brief  Battery realization
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALTargetCommon
 *
 * @{
 */

#ifndef BATTERY_H
#define BATTERY_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "IBattery.h"
#include <MovAvg.hpp>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class provides access to the robot's battery. */
class Battery : public IBattery
{
public:
    /**
     * Constructs the battery adapter.
     */
    Battery() : IBattery(), m_voltMovAvg()
    {
        m_voltMovAvg.clear();
    }

    /**
     * Destroys the battery adapter.
     */
    virtual ~Battery()
    {
    }

    /**
     * Get battery voltage read.
     *
     * @return Battery voltage in millivolts.
     */
    uint32_t getVoltage() final;

    /**
     * Get battery charge level.
     *
     * @return Charge level in percentage.
     */
    uint8_t getChargeLevel() final;

private:
    static const uint32_t VOLTAGE_MIN       = 6000U;  /**< Minimum voltage in millivolts. */
    static const uint32_t VOLTAGE_MAX       = 7000U;  /**< Maximum voltage in millivolts. */
    static const uint32_t REFERENCE_VOLTAGE = 3300U;  /**< Reference voltage of the ADCs in millivolts*/
    static const uint32_t CONVERSION_FACTOR = 10000U; /**< Conversion factor from measured to real battery voltage. */

    MovAvg<uint32_t, uint32_t, 2U>
        m_voltMovAvg; /**< The moving average of the measured voltage over 2 calling cycles. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* BATTERY_H */
/** @} */
