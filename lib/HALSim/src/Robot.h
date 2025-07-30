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
 * @brief  Robot realization
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef ROBOT_H
#define ROBOT_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "IRobot.h"
#include "IRobotNative.h"
#include "WebotsSerialDrv.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class provides access to the simulation robot. */
class Robot : public IRobot, public IRobotNative
{
public:
    /**
     * Constructs the robot adapter.
     *
     * @param[in] serialDrv Webots serial driver.
     */
    Robot(WebotsSerialDrv& serialDrv) : IRobot(), IRobotNative(), m_serialDrv(serialDrv)
    {
    }

    /**
     * Destroys the robot adapter.
     */
    ~Robot() final
    {
    }

    /**
     * Initialize robot driver.
     *
     * @return If successfully initialized, returns true. Otherwise, false.
     */
    bool init() final;

    /**
     * Process communication with the robot.
     */
    void process() final;

    /**
     * Get comunication stream.
     *
     * @return Robot data stream.
     */
    Stream& getStream() final;

    /**
     * Reset the robot.
     */
    void reset() final;

    /**
     * Enter bootloader mode.
     */
    void enterBootloader() final;

    /**
     * Is the robot in bootloader mode?
     *
     * @return If robot is in bootloader mode, it will return true. Otherwise false.
     */
    bool isInBootloaderMode() const final;

    /**
     * Set the serial receive channel id.
     *
     * @param[in] channelId Channel ID, shall be positive for inter-robot communication.
     */
    void setRxChannel(int32_t channelId) final;

    /**
     * Set the serial sender channel id.
     *
     * @param[in] channelId Channel ID, shall be positive for inter-robot communication.
     */
    void setTxChannel(int32_t channelId) final;

private:
    WebotsSerialDrv& m_serialDrv; /**< Webots serial driver. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* ROBOT_H */
/** @} */
