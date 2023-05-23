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
 * @brief  The simulation board realization.
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALSim
 *
 * @{
 */
#ifndef BOARD_H
#define BOARD_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <IBoard.h>
#include "Battery.h"
#include "Button.h"
#include "Device.h"
#include "LedBlue.h"
#include "LedGreen.h"
#include "LedRed.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The concrete simulation robot board.
 */
class Board : public IBoard
{
public:
    /**
     * Get board instance.
     *
     * @return Board instance
     */
    static Board& getInstance()
    {
        static Board instance; /* idiom */

        return instance;
    }

    /**
     * Initialize the hardware.
     *
     * @returns If all components are correctly initialized, returns true. Otherwise, false.
     */
    bool init() final
    {
        bool isReady = false;

        if (false == m_device.init())
        {
            /* Handle Device error */
        }
        else
        {
            /* Ready */
            isReady = true;
        }

        return isReady;
    }

    /**
     * Process board components.
     *
     * @returns If all components are correct, returns true. Otherwise, false;
     */
    bool process() final
    {
        bool isSuccess = false;

        if (false == m_device.process())
        {
            /* Handle Device error */
        }
        else
        {
            /* No Errors */
            isSuccess = true;
        }

        return isSuccess;
    }

    /**
     * Get battery driver.
     *
     * @return Battery driver.
     */
    IBattery& getBattery() final
    {
        return m_battery;
    }

    /**
     * Get button driver.
     *
     * @return Button driver.
     */
    IButton& getButton() final
    {
        return m_button;
    }

    /**
     * Get Device driver.
     *
     * @return Device driver.
     */
    IDevice& getDevice() final
    {
        return m_device;
    }

    /**
     * Get yellow LED driver.
     *
     * @return Yellow LED driver.
     */
    ILed& getBlueLed() final
    {
        return m_ledBlue;
    }

    /**
     * Get green LED driver.
     *
     * @return Green LED driver.
     */
    ILed& getGreenLed() final
    {
        return m_ledGreen;
    }

    /**
     * Get red LED driver.
     *
     * @return Red LED driver.
     */
    ILed& getRedLed() final
    {
        return m_ledRed;
    }

    /**
     * Get Network driver.
     *
     * @return Network driver.
     */
    INetwork& getNetwork() final
    {
        /* Not implemented. */
    }

protected:
private:
    /** Battery driver */
    Battery m_battery;

    /** Button driver */
    Button m_button;

    /** Device driver */
    Device m_device;

    /** Red LED driver */
    LedBlue m_ledBlue;

    /** Red LED driver */
    LedGreen m_ledGreen;

    /** Red LED driver */
    LedRed m_ledRed;

    /**
     * Constructs the concrete board.
     */
    Board() : IBoard(), m_battery(), m_button(), m_device(), m_ledBlue(), m_ledGreen(), m_ledRed()
    {
    }

    /**
     * Destroys the concrete board.
     */
    ~Board()
    {
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* BOARD_H */
/** @} */
