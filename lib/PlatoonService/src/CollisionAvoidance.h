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
 * @brief  Collision Avoidance Module.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup PlatoonService
 *
 * @{
 */
#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

#include "Telemetry.h"
#include <Logging.h>

/******************************************************************************
 * Includes
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Collision avoidance class.
 */
class CollisionAvoidance
{
public:
    /**
     * Collision avoidance constructor
     *
     * @param[in] closestProximityRangeValue Value equivalent to the closest proximity range that can be measured.
     * @param[in] rangeThreshold Range at which the vehicle should stop to avoid collision.
     */
    CollisionAvoidance(Telemetry::Range closestProximityRangeValue, Telemetry::Range rangeThreshold) :
        m_closestProximityRangeValue(closestProximityRangeValue),
        m_rangeThreshold(rangeThreshold) {};

    /**
     * Default Destructor.
     */
    ~CollisionAvoidance()
    {
    }

    /**
     * Limit the speed of the vehicle to avoid collision.
     *
     * @param[out] speedSetpoint The speed setpoint to be limited in mm/s.
     * @param[in] vehicleData The vehicle data structure.
     */
    void limitSpeedToAvoidCollision(int32_t& speedSetpoint, const Telemetry& vehicleData) const
    {
        /* Is vehicle is closer than the threshold? */
        if (vehicleData.proximity >= m_rangeThreshold)
        {
            speedSetpoint = 0;
            LOG_WARNING("Collision Avoidance: Setpoint limited!");
        }
    }

    /**
     * Limit the speed of the vehicle to avoid collision.
     *
     * @param[out] leftSpeedSetpoint The left motor speed setpoint to be limited in mm/s.
     * @param[out] rightSpeedSetpoint The right motor speed setpoint to be limited in mm/s.
     * @param[in] vehicleData The vehicle data structure.
     */
    void limitSpeedToAvoidCollision(int32_t& leftSpeedSetpoint, int32_t& rightSpeedSetpoint,
                                    const Telemetry& vehicleData) const
    {
        /* Is vehicle is closer than the threshold? */
        if (vehicleData.proximity >= m_rangeThreshold)
        {
            leftSpeedSetpoint  = 0;
            rightSpeedSetpoint = 0;
            LOG_WARNING("Collision Avoidance: Setpoint limited!");
        }
    }

private:
    /** Closest proximity range value [brightness levels]. */
    Telemetry::Range m_closestProximityRangeValue;

    /** Range threshold [brightness levels]. */
    Telemetry::Range m_rangeThreshold;

    /**
     * Default constructor.
     */
    CollisionAvoidance();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* COLLISION_AVOIDANCE_H */
/** @} */
