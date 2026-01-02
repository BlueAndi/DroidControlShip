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
 * @brief  The target board realization.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALTargetCommon
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
#include "ButtonReset.h"
#include "ButtonA.h"
#include "ButtonB.h"
#include "ButtonC.h"
#include "LedBlue.h"
#include "LedGreen.h"
#include "LedRed.h"
#include "LedA.h"
#include "LedB.h"
#include "LedC.h"
#include "Network.h"
#include "ButtonDrv.h"
#include "Robot.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The concrete target board.
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
    bool init() final;

    /**
     * Process board components.
     */
    void process() final;

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
     * Get button "Reset" driver.
     *
     * @return Button driver.
     */
    IButton& getButtonReset() final
    {
        return m_buttonReset;
    }

    /**
     * Get button "A" driver.
     *
     * @return Button "A" driver.
     */
    IButton& getButtonA() final
    {
        return m_buttonA;
    }

    /**
     * Get button "B" driver.
     *
     * @return Button "B" driver.
     */
    IButton& getButtonB() final
    {
        return m_buttonB;
    }

    /**
     * Get button "C" driver.
     *
     * @return Button "C" driver.
     */
    IButton& getButtonC() final
    {
        return m_buttonC;
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
     * Get LED "A" driver.
     *
     * @return LED "A" driver.
     */
    ILed& getLedA() final
    {
        return m_ledA;
    }

    /**
     * Get LED "B" driver.
     *
     * @return LED "B" driver.
     */
    ILed& getLedB() final
    {
        return m_ledB;
    }

    /**
     * Get LED "C" driver.
     *
     * @return LED "C" driver.
     */
    ILed& getLedC() final
    {
        return m_ledC;
    }

    /**
     * Get Network driver.
     *
     * @return Network driver.
     */
    INetwork& getNetwork() final
    {
        return m_network;
    }

    /**
     * Get robot driver.
     *
     * @return Robot driver.
     */
    IRobot& getRobot() final
    {
        return m_hostRobot;
    }

    /**
     * Get the file path of the configuration (settings).
     *
     * @return Configuration file path
     */
    const String& getConfigFilePath() const
    {
        return m_configFilePath;
    }

    /**
     * Get GPS driver.
     *
     * @return If GPS is available, it will return a pointer to it, otherwise nullptr.
     */
    IGps* getGps() final
    {
        return nullptr;
    }

protected:
private:
    /** Battery driver */
    Battery m_battery;

    /** Button "Reset" driver */
    ButtonReset m_buttonReset;

    /** Button "A" driver */
    ButtonA m_buttonA;

    /** Button "B" driver */
    ButtonB m_buttonB;

    /** Button "C" driver */
    ButtonC m_buttonC;

    /** Blue LED driver */
    LedBlue m_ledBlue;

    /** Green LED driver */
    LedGreen m_ledGreen;

    /** Red LED driver */
    LedRed m_ledRed;

    /** LED "A" driver */
    LedA m_ledA;

    /** LED "B" driver */
    LedB m_ledB;

    /** LED "C" driver */
    LedC m_ledC;

    /** Network driver */
    Network m_network;

    /** Robot driver to communicate with the host robot, DCS is connected to. */
    Robot m_hostRobot;

    /** Configuration file path */
    const String m_configFilePath;

    /**
     * Constructs the concrete board.
     */
    Board();

    /**
     * Destroys the concrete board.
     */
    virtual ~Board()
    {
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* BOARD_H */
/** @} */
