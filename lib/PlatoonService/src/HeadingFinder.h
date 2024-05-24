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
 * @brief  Heading Finder Module.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Service
 *
 * @{
 */
#ifndef HEADING_FINDER_H
#define HEADING_FINDER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <Arduino.h>
#include <PIDController.h>
#include <SimpleTimer.hpp>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Heading Finder Class.
 * This class is used to calculate the heading of the robot using the odometry data.
 * It uses a PID controller to calculate the target speed of the motors.
 * The target heading is calculated using the current position and the target position.
 */
class HeadingFinder
{
public:
    /** Heading finder data structure. Contains all the data necessary for the calculation */
    typedef struct _HeadingFinderData
    {
        int32_t currentXPos;       /**< Current X position [mm]. */
        int32_t currentYPos;       /**< Current Y position[mm]. */
        int32_t currentHeading;    /**< Current heading [mrad]. */
        int16_t currentSpeedLeft;  /**< Current speed of the left motor [steps/s]. */
        int16_t currentSpeedRight; /**< Current speed of the right motor [steps/s]. */
        int32_t targetXPos;        /**< Target X position[mm]. */
        int32_t targetYPos;        /**< Target Y position[mm]. */
        int32_t targetHeading;     /**< Target heading [mrad]. */
    } HeadingFinderData;

    /** HeadingFinder constructor.*/
    HeadingFinder();

    /** HeadingFinder destructor. */
    ~HeadingFinder();

    /**
     * Set the factors for the PID controller.
     *
     * @param[in] pNumerator Numerator of the proportional factor.
     * @param[in] pDenominator Denominator of the proportional factor.
     * @param[in] iNumerator Numerator of the integral factor.
     * @param[in] iDenominator Denominator of the integral factor.
     * @param[in] dNumerator Numerator of the derivative factor.
     * @param[in] dDenominator Denominator of the derivative factor.
     */
    void setPIDFactors(int32_t pNumerator, int32_t pDenominator, int32_t iNumerator, int32_t iDenominator,
                       int32_t dNumerator, int32_t dDenominator);

    /**
     * Process the Heading finder and calculate the target speed for the motors.
     * @param[out] targetSpeedLeft Target speed of the left motor [steps/s].
     * @param[out] targetSpeedRight Target speed of the right motor [steps/s].
     *
     * @return the delta speed [steps/s] calculated by the PID controller, if available.
     *        Otherwise 0.
     */
    int16_t process(int16_t& targetSpeedLeft, int16_t& targetSpeedRight);

    /**
     * Set lastest odometry data.
     * @param[in] xPos X position [mm].
     * @param[in] yPos Y position [mm].
     * @param[in] heading Heading [mrad].
     */
    void setOdometryData(int32_t xPos, int32_t yPos, int32_t heading);

    /**
     * Set latest motor speed data.
     * @param[in] speedLeft Speed of the left motor [steps/s].
     * @param[in] speedRight Speed of the right motor [steps/s].
     */
    void setMotorSpeedData(int16_t speedLeft, int16_t speedRight);

    /**
     * Set the target heading the robot shall face to using its x and y coordinates.
     * @param[in] xPos X position [mm].
     * @param[in] yPos Y position [mm].
     */
    void setTargetHeading(int32_t xPos, int32_t yPos);

    /**
     * Get the latest calculation data. May include new odometry and speed data, but old calculated heading data.
     *
     * @return Latest calculation data.
     */
    HeadingFinderData getLatestData() const;

private:
    /** Period in ms for PID processing. */
    static const uint32_t PID_PROCESS_PERIOD = 40U;

    /** Maximum motor speed in encoder steps/s */
    static const int16_t MAX_MOTOR_SPEED = 3000;

    /** The PID proportional factor numerator for the heading controller. */
    static const int32_t PID_P_NUMERATOR = 4;

    /** The PID proportional factor denominator for the heading controller.*/
    static const int32_t PID_P_DENOMINATOR = 4;

    /** The PID integral factor numerator for the heading controller. */
    static const int32_t PID_I_NUMERATOR = 0;

    /** The PID integral factor denominator for the heading controller. */
    static const int32_t PID_I_DENOMINATOR = 25;

    /** The PID derivative factor numerator for the heading controller. */
    static const int32_t PID_D_NUMERATOR = 0;

    /** The PID derivative factor denominator for the heading controller. */
    static const int32_t PID_D_DENOMINATOR = 1;

    /** Calculation Data. */
    HeadingFinderData m_data;

    /** New odometry data flag. */
    bool m_newOdometryData;

    /** New motor speed data flag. */
    bool m_newMotorSpeedData;

    /** PID controller, used for heading finding. */
    PIDController<int32_t> m_pidCtrl;

    /** Timer used for periodically PID processing. */
    SimpleTimer m_pidProcessTime;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* HEADING_FINDER_H */
/** @} */
