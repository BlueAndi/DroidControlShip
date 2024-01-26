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
 * @brief  Device realization
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Device.h"
#include "GPIO.h"

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
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

bool Device::init()
{
    reset();
    return m_usbHost.init();
}

bool Device::process()
{
    if (true == m_resetTimer.isTimeout())
    {
        GpioPins::resetDevicePin.write(LOW);
        m_resetTimer.stop();

        if (true == m_bootloaderModeRequest)
        {
            m_bootloaderModeRequest = false;
            m_waitTimer.start(WAIT_TIME_BOOTLOADER_MODE_MS);
        }
    }

    if (true == m_waitTimer.isTimeout())
    {
        reset();
        m_waitTimer.stop();
    }

    return m_usbHost.process();
}

Stream& Device::getStream()
{
    return m_usbHost;
}

void Device::reset()
{
    if (false == m_resetTimer.isTimerRunning())
    {
        GpioPins::resetDevicePin.write(HIGH);
        m_resetTimer.start(RESET_TIME_MS);
    }
}

void Device::enterBootloader()
{
    m_bootloaderModeRequest = true;
    m_usbHost.requestBootloaderMode();
    reset();
}

bool Device::isInBootloaderMode() const
{
    return m_usbHost.isBootloaderModeActive();
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
