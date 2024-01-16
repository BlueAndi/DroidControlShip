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
 * @brief The coordinate handler realization.
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
    /** The CoordinateHandler instance. */
    static CoordinateHandler& getInstance()
    {
        static CoordinateHandler instance;

        return instance;
    }

    /**
     * Set current coordinates received from robot
     * into values to be processed in the program.
     *
     * @param[in] xPos is current x position of the robot
     * @param[in] yPos is current y position of the robot
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
     * Set current orientation of the robot
     *
     * @param[in] currentOrientation is the orientation value
     */
    void setCurrentOrientation(int32_t currentOrientation)
    {
        m_currentOrientation = currentOrientation;
    }

    /**
     * Get current orientation.
     *
     * @returns the current orientation of the robot
     */
    int32_t getCurrentOrientation()
    {
        return m_currentOrientation;
    }

    /**
     * Get current distance between robot and IE.
     *
     * @returns the current distance.
     */
    int32_t getCurrentDistance()
    {
        return m_distance;
    }

    /**
     * Get old distance between robot and IE.
     *
     * @returns the old distance.
     */
    int32_t getOldDistance()
    {
        return m_oldDistance;
    }

    /**
     * Process current coordinates with the ones of the IE.
     *
     * @param[in] ieName is name of IE to be processed.
     * @param[in] ieOrientation is orientation of IE to be processed.
     * @param[in] distanceToIE is current robot-IE distance to be processed.
     * @param[in] previousDistanceToIE is older robot-IE distance to be processed.
     * 
     * @returns true if robot is successfully driving towards IE
     */
    bool process(const String& ieName, int32_t ieOrientation, int32_t distanceToIE, int32_t previousDistanceToIE);

    /**
     * Checks distance of robot to selected IE using vector absolute value logic:
     * dist = sqrt ((x1 - x2)^2 + (y1 - y2)^2)
     *
     * @param[in] xPos is position of IE on x axis.
     * @param[in] yPos is position of IE on y axis.
     * 
     * @returns the calculated distance.
     */
    int32_t checkDistance(int32_t xPos, int32_t yPos);

    /**
     * Checks current status between robot and IE.
     *
     * @returns the current status.
     */
    int8_t getStatus()
    {
        return m_currentStatus;
    }

private:
    /**
     * Status between robot and current IE.
     */
    enum CurrentStatus
    {
        STATUS_IDLE = 0,  /**< robot isn't moving towards and not pointing towards IE.  */
        STATUS_TOWARDS,   /**< robot is moing towards, but not pointing to IE. */
        STATUS_LOCKED_IN, /**< robot is moving towards, and pointing to IE. */
        STATUS_NEAR,      /**< robot is near IE, listen to signals.  */
        STATUS_PASSED     /**< robot passed this IE. */
    };

    /** Current distance between robot and infrastructure element. */
    int32_t m_distance;

    /** Previous value of distance. */
    int32_t m_oldDistance;

    /** Current coordinate of robot on the x Axis*/
    int32_t m_currentX;

    /** Current coordinate of robot on the x Axis*/
    int32_t m_currentY;

    /** Current orientation */
    int32_t m_currentOrientation;

    /** Status of robot in relation to IE. */
    CurrentStatus m_currentStatus;

    /**
     * Check if robot is moving towards an IE. If distance gets smaller then
     * robot is moving towards it.
     *
     * @param[in] currentDistance current distance between robot and IE.
     * @param[in] previousDistance older distance between robot and IE.
     * 
     * @returns true if driving towards
     */
    bool isMovingTowards(int32_t currentDistance, int32_t previousDistance);

    /**
     * Check if orientation matches the one given by the infrastructure element.
     *
     * @param[in] orientationIE orientation of infrastructure element.
     * @returns true if orientations match.
     */
    bool checkOrientation(int32_t orientationIE);

    /** Coordinate handler constructor. */
    CoordinateHandler() :
        m_distance(0U),
        m_oldDistance(0U),
        m_currentOrientation(0U),
        m_currentX(0U),
        m_currentY(0U),
        m_currentStatus(STATUS_IDLE)
    {
    }

    /** Coordinate handler deconstructor. */
    ~CoordinateHandler()
    {
    }

    CoordinateHandler(const CoordinateHandler& state);            /**< Copy construction of an instance. */
    CoordinateHandler& operator=(const CoordinateHandler& state); /**< Assignment of an instance. */
};

#endif /*COORDINATE_HANDLER_H*/
       /** @} */