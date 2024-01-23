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
 * @brief  Leader Longitudinal Controller.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef LONGITUDINAL_CONTROLLER_H
#define LONGITUDINAL_CONTROLLER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>
#include <Waypoint.h>
#include <functional>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Leader Longitudinal Controller. */
class LongitudinalController
{
public:
    /**
     * Motor setpoint callback.
     * Called in order to set the motor speeds.
     */
    typedef std::function<bool(const int16_t topCenterSpeed)> MotorSetpointCallback;

    /** Controller states. */
    enum STATE : uint8_t
    {
        STATE_STARTUP = 0, /**< Startup state. */
        STATE_READY,       /**< Ready state. */
        STATE_DRIVING,     /**< Driving state. */
        STATE_SAFE,        /**< Safe state. */
    };

    /**
     * LongitudinalController constructor.
     */
    LongitudinalController();

    /**
     * Default destructor.
     */
    ~LongitudinalController();

    /**
     * Process the longitudinal controller.
     */
    void process();

    /**
     * Set the maximum motor speed of the robot.
     *
     * @param[in] maxMotorSpeed Maximum motor speed.
     */
    void setMaxMotorSpeed(int16_t maxMotorSpeed)
    {
        m_maxMotorSpeed = maxMotorSpeed;
    }

    /**
     * Release robot for driving state, only if in ready state.
     *
     * @return If the robot was released, returns true. Otherwise false.
     */
    bool release()
    {
        bool isReleased = false;

        if (STATE_READY == m_state)
        {
            m_state    = STATE_DRIVING;
            isReleased = true;
        }

        return isReleased;
    }

    /**
     * Set incoming feedback from last follower.
     *
     * @param[in] feedback Feedback from last follower.
     */
    void setLastFollowerFeedback(const Waypoint& feedback)
    {
        m_lastFollowerFeedback = feedback;
    }

    /**
     * Get current state.
     *
     * @return Current state.
     */
    STATE getState() const
    {
        return m_state;
    }

    /**
     * Set safe state. This will stop the robot.
     */
    void setSafeState()
    {
        m_state = STATE_SAFE;
    }

    /**
     * Set motor setpoint callback.
     *
     * @param[in] motorSetpointCallback Motor setpoint callback.
     */
    void setMotorSetpointCallback(const MotorSetpointCallback& motorSetpointCallback)
    {
        m_motorSetpointCallback = motorSetpointCallback;
    }

    /**
     * Calculate the top motor speed and send it to the robot.
     *
     * @param[in] currentVehicleData Current vehicle data.
     */
    void calculateTopMotorSpeed(const Waypoint& currentVehicleData);

private:
    /** Maximum motor speed. */
    int16_t m_maxMotorSpeed;

    /** Current state. */
    STATE m_state;

    /** Feedback from last follower. */
    Waypoint m_lastFollowerFeedback;

    /**
     * Motor setpoint callback.
     */
    MotorSetpointCallback m_motorSetpointCallback;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* LONGITUDINAL_CONTROLLER_H */
/** @} */
