/* MIT License
 *
 * Copyright (c) 2023 - 2025 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Keyboard realization
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef KEYBOARD_H
#define KEYBOARD_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include <webots/Keyboard.hpp>
#include <webots/Robot.hpp>

#include "SimTime.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class implements a custom keyboard for better functionality. */
class Keyboard
{
public:
    /**
     * Constructs the encoders adapter and initialize it.
     */
    Keyboard(SimTime& simTime, webots::Keyboard* keyboard) :
        m_keys(),
        m_simTime(simTime),
        m_keyboard(keyboard)
    {
    }

    /**
     * Destroys the encoders adapter.
     */
    ~Keyboard()
    {
    }

    /**
     * Gets the current buttons pressed on the keyboard. Needs to be called
     * cyclically to work correct.
     */
    void getPressedButtons()
    {
        /* Getting the new key values. Currently the limit of the simulation is seven
         * keypresses that can be detected simultaniously.
         */
        for (uint8_t keyIndex = 0; keyIndex < (sizeof(m_keys) / sizeof(*m_keys)); ++keyIndex)
        {
            m_keys[keyIndex] = m_keyboard->getKey();
        }
    }

    /**
     * Checks weather the button S was pressed.
     *
     * @return Return true if button S was pressed.
     */
    bool buttonSPressed()
    {
        return isButtonPressed(KEY_CODE_S_LOWER_CASE, KEY_CODE_S_UPPER_CASE);
    }

    /**
     * Checks weather the button S is released.
     *
     * @return Return true if button S is released.
     */
    bool buttonSReleased()
    {
        return isButtonReleased(KEY_CODE_S_LOWER_CASE, KEY_CODE_S_UPPER_CASE);
    }

    /**
     * Waits until Button S gets released.
     * Needs to call the robots step() method and getPressedButtons()
     * to update the keypresses correctly and avoid getting stuck
     * in the while loop.
     */
    void waitForReleaseS()
    {
        while (!buttonSReleased())
        {
            if (false == m_simTime.step())
            {
                break;
            }
            getPressedButtons();
        }
    }

private:
    /** The key code of the lower case s character, which simulates the button. */
    static const char KEY_CODE_S_LOWER_CASE = 's';

    /** The key code of the upper case S character, which simulates the button. */
    static const char KEY_CODE_S_UPPER_CASE = 'S';

    /** The maximum number of keys pressed simultaniously, that the simulation can process. */
    static const uint8_t MAX_KEY_NUMBER = 7;

    /** The keys pressed during this update. */
    uint16_t m_keys[MAX_KEY_NUMBER];

    SimTime &m_simTime; /**< Simulation time */

    webots::Keyboard *m_keyboard; /**< Robot keyboard */

    /**
     * Is the button pressed?
     *
     * @param[in] lowerCaseChar Lower case character ASCII value
     * @param[in] upperCaseChar Upper case character ASCII value
     *
     * @return If pressed, it will return true otherwise false.
     */
    bool isButtonPressed(char lowerCaseChar, char upperCaseChar) const;

    /**
     * Is the button released?
     *
     * @param[in] lowerCaseChar Lower case character ASCII value
     * @param[in] upperCaseChar Upper case character ASCII value
     *
     * @return If released, it will return true otherwise false.
     */
    bool isButtonReleased(char lowerCaseChar, char upperCaseChar) const;

    /**
     * Checks whether the given array contains a element.
     *
     * @param[in]   array           The array where to search for the element.
     * @param[in]   arraySize       Number of array elements.
     * @param[in]   elemLowerCase   Lower case character
     * @param[in]   elemUppercase   Upper case character
     *
     * @return If found, it will return true otherwise false.
     */
    bool arrayContains(const uint16_t array[], uint16_t arraySize, char elemLowerCase, char elemUppercase) const;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* KEYBOARD_H */
/** @} */
