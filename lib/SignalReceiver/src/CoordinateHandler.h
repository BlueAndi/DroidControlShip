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
    static CoordinateHandler& getInstance()
    {
        static CoordinateHandler instance;

        return instance;
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
     * Set current coordinates received from robots
     * into values to be processed in the program.
     */
    void setCurrentCoordinates(int32_t xPos, int32_t yPos)
    {
        m_currentX = xPos;
        m_currentY = yPos;
    }

    /**
     * Set interval values for the trigger area.
     * This in combination with setting the required entry
     * coordinate setup an area.
     *
     * The TL shall send relevant signals only once the robot
     * is in this area.
     *
     * @param[in] intervValueX value of x that shall be used to form a line on the x-Axis.
     * @param[in] intervValueY value of y that shall be used to form a line on the y-Axis.
     */
    void setIntervalValues(int32_t intervValueX, int32_t intervValueY)
    {
        m_intervalValueX = intervValueX;
        m_intervalValueY = intervValueY;
    }

    /**
     * Get interval x value.
     *
     * @return interval x value
     */
    int32_t getIntervalValueX()
    {
        return m_intervalValueX;
    }

    /**
     * Get interval y value.
     *
     * @return interval y value
     */
    int32_t getIntervalValueY()
    {
        return m_intervalValueY;
    }

    /**
     * Set entry requirement for the trigger area.
     *
     * Coordinate settings can be found in documentation.
     */
    void setEntryValues(int32_t entryX, int32_t entryY)
    {
        m_entryX = entryX;
        m_entryY = entryY;
    }

    /**
     * Get entry x value.
     *
     * @return entry x value
     */
    int32_t getEntryX() const
    {
        return m_entryX;
    }

    /**
     * Get entry y value.
     *
     * @return entry y value
     */
    int32_t getEntryY() const
    {
        return m_entryY;
    }

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

        if (((getEntryX() - m_intervalValueX <= getCurrentXCoord()) &&
             (getCurrentXCoord() <= getEntryX() + m_intervalValueX)) &&
            ((getEntryY() - m_intervalValueY <= getCurrentYCoord()) &&
             (getCurrentYCoord() <= getEntryY() + m_intervalValueY)))
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

        if (true == (getEntryY() < 0))
        {
            if ((((getEntryY() + m_intervalValueY) - 200) <= getCurrentYCoord()) &&
                (getCurrentYCoord() <= (getEntryY() + m_intervalValueY)))
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
        else if (true == (getEntryY() >= 0))
        {
            if ((((getEntryY() - m_intervalValueY) + 40) <= getCurrentYCoord()) &&
                (getCurrentYCoord() <= (getEntryY() - m_intervalValueY)))
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

private:
    /** Current coordinates */
    int32_t m_currentX;
    int32_t m_currentY;

    /** Interval values to be used for the trigger area. */
    int32_t m_intervalValueX;
    int32_t m_intervalValueY;

    /** Required entry values used for the trigger area. */
    int32_t m_entryX;
    int32_t m_entryY;

    /** Green state constructor. */
    CoordinateHandler()
    {
    }

    /** Green state deconstructor. */
    ~CoordinateHandler()
    {
    }

    CoordinateHandler(const CoordinateHandler& state);
    CoordinateHandler& operator=(const CoordinateHandler& state);
};

#endif /*COORDINATE_HANDLER_H*/
       /** @} */