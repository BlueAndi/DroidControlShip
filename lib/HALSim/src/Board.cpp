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
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Board.h"
#include <Logging.h>
#include "RobotDeviceNames.h"

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

bool Board::init()
{
    bool isReady = true;

    /* Nothing to do. */

    return isReady;
}

bool Board::process()
{
    bool isSuccess = false;

    if (false == m_network.process())
    {
        /* Log Network error */
        LOG_ERROR("Network process failed.");
    }
    else
    {
        /* No Errors */
        isSuccess = true;
    }

    return isSuccess;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

Board::Board() :
    IBoard(),
    m_robot(),
    m_simTime(m_robot),
    m_serialDrv(m_robot.getEmitter(RobotDeviceNames::EMITTER_NAME_SERIAL),
                m_robot.getReceiver(RobotDeviceNames::RECEIVER_NAME_SERIAL)),
    m_battery(),
    m_button(),
    m_ledBlue(),
    m_ledGreen(),
    m_ledRed(),
    m_network(),
    m_hostRobot(m_serialDrv),
    m_configFilePath()
{
}

void Board::enableSimulationDevices()
{
    const int timeStep = m_simTime.getTimeStep();

    m_robot.getReceiver(RobotDeviceNames::RECEIVER_NAME_SERIAL)->enable(timeStep);
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
