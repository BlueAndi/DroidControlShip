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
 * @brief The green light phase realization.
 * @author Paul Gramescu <paul.gramescu@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef COORDINATE_HANDLER_H
#define COORDINATE_HANDLER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>

#include <Logging.h>
#include <Board.h>

#include <Participants.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The green light phase.
 */
class CoordinateHandler
{
public:
    /** The CoordinateHandler instance. */
    static CoordinateHandler& getInstance()
    {
        static CoordinateHandler instance;

        return instance;
    }

    /**
     * Set current coordinates received from robots
     * into values to be processed in the program.
     */
    void setCurrentCoordinates(int32_t xPos, int32_t yPos)
    {
        m_currentX = xPos;
        m_currentY = yPos;
    }

    /**
     * Get current x position.
     *
     * @return Current x position
     */
    int32_t getCurrentXCoord() const
    {
        return m_currentX;
    }

    /**
     * Get current y position.
     *
     * @return Current y position
     */
    int32_t getCurrentYCoord() const
    {
        return m_currentY;
    }

    /**
     * Set orientation of infrastructure element.
     */
    void setCurrentOrientation(int32_t currentOrientation)
    {
        m_currentOrientation = currentOrientation;
    }

    /**
     * Get orientation.
     *
     * @returns the orientation value
     */
    int32_t getCurrentOrientation()
    {
        return m_currentOrientation;
    }

    // bool process(int32_t orientationValue, int32_t intervalValueX, int32_t intervalValueY, int32_t entryValueX,
    //              int32_t entryValueY)
    // {
    //     bool isTrue;

    //     return isTrue;
    // }

    /**
     * Check if robot has entered the trigger area.
     * Using intervals because of slight odometry imprecisions.
     *
     * A bigger interval is needed the faster the PID algorithm is.
     *
     * Current interval and coordinate map-outs have been done
     * using the first parameter set of the PID controller on the robot.
     *
     * @return true if entered trigger area
     */
    bool CheckEntryCoordinates()
    {
        bool isTrue = false;

        // LOG_DEBUG("Comparing x %d < %d < %d", getEntryX() - m_intervalValueX, getCurrentXCoord(), getEntryX() +
        // m_intervalValueX); LOG_DEBUG("Comparing y %d < %d < %d", getEntryY() - m_intervalValueY, getCurrentYCoord(),
        // getEntryY() + m_intervalValueY);

        if (((Participant::getInstance().getEntryX() - Participant::getInstance().getIntervalValueX() <=
              getCurrentXCoord()) &&
             (getCurrentXCoord() <=
              Participant::getInstance().getEntryX() + Participant::getInstance().getIntervalValueX())) &&
            ((Participant::getInstance().getEntryY() - Participant::getInstance().getIntervalValueY() <=
              getCurrentYCoord()) &&
             (getCurrentYCoord() <=
              Participant::getInstance().getEntryY() + Participant::getInstance().getIntervalValueY())))
        {
            /* Robot is now in the entry area, toggle flag. */
            isTrue = true;
        }
        else
        {
            isTrue = false;
        }

        return isTrue;
    }

    /**
     * Check if robot is near exit.
     *
     * @return true if near exit
     */
    bool isNearExit()
    {
        bool isTrue = false;

        if (true == (Participant::getInstance().getEntryY() < 0))
        {
            if ((((Participant::getInstance().getEntryY() + Participant::getInstance().getIntervalValueY()) - 200) <=
                 getCurrentYCoord()) &&
                (getCurrentYCoord() <=
                 (Participant::getInstance().getEntryY() + Participant::getInstance().getIntervalValueY())))
            {
                /** Robot is near exit, toggle flag. */
                isTrue = true;

                LOG_DEBUG("Robot is near exit");
            }
            else
            {
                isTrue = false;

                LOG_DEBUG("Robot has more driving to do.");
            }
        }
        else if (true == (Participant::getInstance().getEntryY() >= 0))
        {
            if ((((Participant::getInstance().getEntryY() - Participant::getInstance().getIntervalValueX()) + 40) <=
                 getCurrentYCoord()) &&
                (getCurrentYCoord() <=
                 (Participant::getInstance().getEntryY() - Participant::getInstance().getIntervalValueY())))
            {
                /** Robot is near exit, toggle flag. */
                isTrue = true;

                LOG_DEBUG("Robot is near exit");
            }
            else
            {
                isTrue = false;

                LOG_DEBUG("Robot has more driving to do.");
            }
        }
        else
        {
            /** Do nothing */
        }

        return isTrue;
    }

    /**
     * Check if orientation matches the one given by the infrastructure element.
     *
     * @returns true if orientations match.
     */
    bool checkOrientation()
    {
        bool isTrue = false;

        if (Participant::getInstance().getRequiredOrientation() != 0)
        {
            // LOG_DEBUG("Comparing %d < %d < %d", Participant::getInstance().getRequiredOrientation() - 500,
            //           getCurrentOrientation(), Participant::getInstance().getRequiredOrientation() + 500);

            if (((Participant::getInstance().getRequiredOrientation() - 500) <= getCurrentOrientation()) &&
                (getCurrentOrientation() <= (Participant::getInstance().getRequiredOrientation() + 500)))
            {
                isTrue = true;
            }
            else
            {
                isTrue = false;
            }
        }
        else
        {
            /** If no there is no orientation set, set it to always be true. */
            isTrue = true;
        }

        return isTrue;
    }

private:
    /** Current coordinates */
    int32_t m_currentX;
    int32_t m_currentY;

    /** Current orientation */
    int32_t m_currentOrientation;

    /** Green state constructor. */
    CoordinateHandler() : m_currentOrientation(0U), m_currentX(0U), m_currentY(0U)
    {
    }

    /** Green state deconstructor. */
    ~CoordinateHandler()
    {
    }

    CoordinateHandler(const CoordinateHandler& state);            /**< Copy construction of an instance. */
    CoordinateHandler& operator=(const CoordinateHandler& state); /**< Assignment of an instance. */
};

#endif /*COORDINATE_HANDLER_H*/
       /** @} */