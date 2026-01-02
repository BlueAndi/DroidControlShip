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
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Board.h"
#include "GPIO.h"
#include <Logging.h>

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
    bool isReady = false;

    GpioPins::init();

    /* Turn LEDs off. */
    m_ledRed.enable(false);
    m_ledGreen.enable(false);
    m_ledBlue.enable(false);

    if (false == ButtonDrv::getInstance().init())
    {
        /* Log Button Driver error */
        LOG_ERROR("Button driver initialization failed.");
    }
    else if (false == m_hostRobot.init())
    {
        /* Log Robot error */
        LOG_ERROR("Robot initialization failed.");
    }
    else if (false == m_network.init())
    {
        /* Log Network error */
        LOG_ERROR("Network initialization failed.");
    }
    else
    {
        /* Ready */
        isReady = true;
    }

    return isReady;
}

void Board::process()
{
    m_hostRobot.process();
    m_network.process();
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

Board::Board() :
    IBoard(),
    m_battery(),
    m_button(),
    m_ledBlue(),
    m_ledGreen(),
    m_ledRed(),
    m_network(),
    m_hostRobot(),
    m_configFilePath(CONFIG_FILE_PATH)
{
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
