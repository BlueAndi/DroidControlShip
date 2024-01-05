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
    String  name;        /**< Name of Infrastructure Element */
    int32_t orientation; /**< Orientation of the infrastructure element. */
    int32_t entryX;      /**< Entry value of x. */
    int32_t entryY;      /**< Entry value of y. */
};

/**
 * The green light phase.
 */
class Participant
{
public:
    /** The Participant instance. */
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
    /** Name of Infrastructure Element. */
    String m_name;

    /** Orientation of the infrastructure element. */
    int32_t m_orientation;

    /** Entry value of x. */
    int32_t m_entryX;

    /** Entry value of y. */
    int32_t m_entryY;

    /** Participant constructor. */
    Participant() : m_name(""), m_orientation(0U), m_entryX(0U), m_entryY(0U)
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