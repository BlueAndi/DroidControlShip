/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
 * @file
 * @brief  Board interface, which abstracts the physical board
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALInterfaces
 *
 * @{
 */

#ifndef IBOARD_H
#define IBOARD_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include "IBattery.h"
#include "IButton.h"
#include "ILed.h"
#include "INetwork.h"
#include "IRobot.h"
#include "IGps.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Abstracts the physical board interface.
 */
class IBoard
{
public:
    /**
     * Destroys the board interface.
     */
    virtual ~IBoard()
    {
    }

    /**
     * Initialize the hardware.
     *
     * @returns If all components are correctly initialized, returns true. Otherwise, false.
     */
    virtual bool init() = 0;

    /**
     * Process board components.
     */
    virtual void process() = 0;

    /**
     * Get battery driver.
     *
     * @return Battery driver.
     */
    virtual IBattery& getBattery() = 0;

    /**
     * Get button "Reset" driver.
     *
     * @return Button driver.
     */
    virtual IButton& getButton() = 0;

    /**
     * Get button "A" driver.
     *
     * @return Button "A" driver.
     */
    virtual IButton& getButtonA() = 0;

    /**
     * Get button "B" driver.
     *
     * @return Button "B" driver.
     */
    virtual IButton& getButtonB() = 0;

    /**
     * Get button "C" driver.
     *
     * @return Button "C" driver.
     */
    virtual IButton& getButtonC() = 0;

    /**
     * Get red LED driver.
     *
     * @return Red LED driver.
     */
    virtual ILed& getRedLed() = 0;

    /**
     * Get green LED driver.
     *
     * @return Green LED driver.
     */
    virtual ILed& getGreenLed() = 0;

    /**
     * Get yellow LED driver.
     *
     * @return Yellow LED driver.
     */
    virtual ILed& getBlueLed() = 0;

    /**
     * Get LED "A" driver.
     *
     * @return LED "A" driver.
     */
    virtual ILed& getLedA() = 0;

    /**
     * Get LED "B" driver.
     *
     * @return LED "B" driver.
     */
    virtual ILed& getLedB() = 0;

    /**
     * Get LED "C" driver.
     *
     * @return LED "C" driver.
     */
    virtual ILed& getLedC() = 0;

    /**
     * Get Network driver.
     *
     * @return Network driver.
     */
    virtual INetwork& getNetwork() = 0;

    /**
     * Get robot driver.
     *
     * @return Robot driver.
     */
    virtual IRobot& getRobot() = 0;

    /**
     * Get the file path of the configuration (settings).
     *
     * @return Configuration file path
     */
    virtual const String& getConfigFilePath() const = 0;

    /**
     * Get GPS driver.
     *
     * @return If GPS is available, it will return a pointer to it, otherwise nullptr.
     */
    virtual IGps* getGps() = 0;

protected:
    /**
     * Constructs the board interface.
     */
    IBoard()
    {
    }

private:
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* IBOARD_H */
/** @} */
