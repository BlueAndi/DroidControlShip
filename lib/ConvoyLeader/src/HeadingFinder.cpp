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
 * @brief  Heading Finder Module
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "HeadingFinder.h"
#include <math.h>
#include <FPMath.h>
#include <Logging.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

HeadingFinder::HeadingFinder() :
    m_data(),
    m_newOdometryData(false),
    m_newMotorSpeedData(false),
    m_pidCtrl(),
    m_pidProcessTime()
{
}

HeadingFinder::~HeadingFinder()
{
}

void HeadingFinder::init()
{
    /* Configure PID. */
    m_pidCtrl.clear();
    setPIDFactors(PID_P_NUMERATOR, PID_P_DENOMINATOR, PID_I_NUMERATOR, PID_I_DENOMINATOR, PID_D_NUMERATOR,
                  PID_D_DENOMINATOR);
    m_pidCtrl.setSampleTime(PID_PROCESS_PERIOD);
    m_pidCtrl.setLimits(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    m_pidCtrl.setDerivativeOnMeasurement(true);

    m_pidProcessTime.start(0); /* Immediate */
}

void HeadingFinder::setPIDFactors(int32_t pNum, int32_t pDen, int32_t iNum, int32_t iDen, int32_t dNum, int32_t dDen)
{
    m_pidCtrl.setPFactor(pNum, pDen);
    m_pidCtrl.setIFactor(iNum, iDen);
    m_pidCtrl.setDFactor(dNum, dDen);
}

int16_t HeadingFinder::process(int16_t& targetSpeedLeft, int16_t& targetSpeedRight)
{
    int16_t pidDelta = 0;

    /* Process PID controller when timer is done and new data is found. */
    if ((true == m_pidProcessTime.isTimeout()) && (true == m_newOdometryData) && (true == m_newMotorSpeedData))
    {
        /* Delta position. */
        int32_t deltaX = m_data.targetXPos - m_data.currentXPos;
        int32_t deltaY = m_data.targetYPos - m_data.currentYPos;

        /* Calculate target heading. */
        float angle          = atan2(deltaY, deltaX) * 1000.0F;        /* Angle in mrad. */
        m_data.targetHeading = static_cast<int32_t>(angle) % FP_2PI(); /* Fixed point heading. */

        /* Calculate PID delta. */
        pidDelta = m_pidCtrl.calculate(m_data.targetHeading, m_data.currentHeading);
        LOG_DEBUG("Heading: %d, Target: %d, PID Delta: %d", m_data.currentHeading, m_data.targetHeading, pidDelta);

        /* Calculate target speed. */
        targetSpeedLeft  = m_data.currentSpeedLeft - pidDelta;
        targetSpeedRight = -targetSpeedLeft;

        /* Reset new data flags. */
        m_newOdometryData   = false;
        m_newMotorSpeedData = false;

        /* Restart timer. */
        m_pidProcessTime.start(PID_PROCESS_PERIOD);
    }

    return pidDelta;
}

void HeadingFinder::setOdometryData(int32_t xPos, int32_t yPos, int32_t heading)
{
    /* Set odometry data. */
    m_data.currentXPos    = xPos;
    m_data.currentYPos    = yPos;
    m_data.currentHeading = heading;

    /* Set new odometry data flag. */
    m_newOdometryData = true;
}

void HeadingFinder::setMotorSpeedData(int16_t speedLeft, int16_t speedRight)
{
    /* Set motor speeds. */
    m_data.currentSpeedLeft  = speedLeft;
    m_data.currentSpeedRight = speedRight;

    /* Set new motor speed data flag. */
    m_newMotorSpeedData = true;
}

void HeadingFinder::setTargetHeading(int32_t xPos, int32_t yPos)
{
    /* Set internal values.*/
    m_data.targetXPos = xPos;
    m_data.targetYPos = yPos;
}

HeadingFinder::HeadingFinderData HeadingFinder::getLatestData() const
{
    return m_data;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
