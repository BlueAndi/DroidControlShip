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

#ifndef PARTICIPANTS_H
#define PARTICIPANTS_H

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

/** the InfrastructureElement struct */
struct InfrastructureElement
{
    String  name;        /** Name of Infrastructure Element */
    int32_t orientation; /** Orientation of the infrastructure element. */
    int32_t intervX;     /** Interval values on the x axis. */
    int32_t intervY;     /** Interval values on the y axis. */
    int32_t entryX;      /** Entry value of x. */
    int32_t entryY;      /** Entry value of y. */
};

/**
 * The green light phase.
 */
class Participant
{
public:
    static Participant& getInstance()
    {
        static Participant instance;

        return instance;
    }

    /**
     * Set orientation of infrastructure element.
     *
     * @param[in] requiredOrientation orientation of infrastructure element
     */
    void setRequiredOrientation(int32_t requiredOrientation)
    {
        m_orientation = requiredOrientation;
    }

    /**
     * Get orientation.
     *
     * @returns the orientation value
     */
    int32_t getRequiredOrientation()
    {
        return m_orientation;
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
        m_intervX = intervValueX;
        m_intervY = intervValueY;
    }

    /**
     * Get interval x value.
     *
     * @return interval x value
     */
    int32_t getIntervalValueX()
    {
        return m_intervX;
    }

    /**
     * Get interval y value.
     *
     * @return interval y value
     */
    int32_t getIntervalValueY()
    {
        return m_intervY;
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

private:
    /** Infrastructure element's name */
    String m_participantName;

    /** Orientation of the infrastructure element. */
    int32_t m_orientation;

    /** Interval values on the x axis. */
    int32_t m_intervX;

    /** Interval value on the y axis. */
    int32_t m_intervY;

    /** Entry value of x. */
    int32_t m_entryX;

    /** Entry value of y. */
    int32_t m_entryY;

    /** Participant constructor. */
    Participant() : m_participantName(""), m_orientation(0U), m_intervX(0U), m_intervY(0U), m_entryX(0U), m_entryY(0U)
    {
    }

    /** Participant deconstructor. */
    ~Participant()
    {
    }

    Participant(const Participant& state);            /**< Copy construction of an instance. */
    Participant& operator=(const Participant& state); /**< Assignment of an instance. */
};

#endif /*PARTICIPANTS_H*/
       /** @} */