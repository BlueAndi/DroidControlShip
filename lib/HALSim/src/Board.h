/* MIT License
 *
 * Copyright (c) 2023 - 2024 Andreas Merkle <web@blue-andi.de>
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
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
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
#include <WString.h>
#include "Battery.h"
#include "Button.h"
#include "LedBlue.h"
#include "LedGreen.h"
#include "LedRed.h"
#include "Network.h"
#include "Robot.h"
#include "WebotsSerialDrv.h"

#include <webots/Robot.hpp>
#include <SimTime.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The concrete simulation board.
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
     *
     * @returns If all components are processed correctly, returns true. Otherwise, false
     */
    bool process() final;

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

private:
    /** Name of the serial emitter in the DCS simulation. */
    static const char* EMITTER_NAME_SERIAL;

    /** Name of the serial receiver in the DCS simulation. */
    static const char* RECEIVER_NAME_SERIAL;

    /** Simulated DCS robot instance. */
    webots::Robot m_robot;

    /** Simulation time handler */
    SimTime m_simTime;

    /** Serial driver to communicate with the host robot, DCS is connected to. */
    WebotsSerialDrv m_serialDrv;

    /** Battery driver */
    Battery m_battery;

    /** Button driver */
    Button m_button;

    /** Blue LED driver */
    LedBlue m_ledBlue;

    /** Green LED driver */
    LedGreen m_ledGreen;

    /** Red LED driver */
    LedRed m_ledRed;

    /** Network driver */
    Network m_network;

    /** Robot driver to communicate with the host robot, DCS is connected to. */
    Robot m_hostRobot;

    /** Configuration file path */
    String m_configFilePath;

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

    /**
     * Set the file path of the configuration (settings).
     *
     * @param[in] configFilePath Configuration file path
     */
    void setConfigFilePath(const char* configFilePath)
    {
        m_configFilePath = configFilePath;
    }

    /**
     * Get the simulation time handler.
     *
     * @return Simulation time handler
     */
    SimTime& getSimTime()
    {
        return m_simTime;
    }

    /**
     * Get the simulation serial driver, which is connected within Webots.
     *
     * @return If serial driver is available, it will return a pointer to it, otherwise nullptr.
     */
    WebotsSerialDrv* getSimSerial()
    {
        return &m_serialDrv;
    }

    /**
     * Enable all simulation devices.
     * It is called by the main entry only.
     * Devices must be enabled before they can be used, and a simulation step must be performed before the application
     * initialization.
     */
    void enableSimulationDevices();

    /**
     * The main entry needs access to be able to set the configuration file path.
     * But all other application parts shall have no access, which is
     * solved by this friend.
     *
     * @param[in] argc  Number of arguments
     * @param[in] argv  Arguments
     *
     * @return Exit code
     */
    friend int main(int argc, char** argv);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* BOARD_H */
/** @} */
