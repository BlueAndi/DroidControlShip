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
 *
 * @addtogroup HALTarget
 *
 * @{
 */

#ifndef DEVICE_H
#define DEVICE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "IDevice.h"
#include "USBHost.h"
#include <SimpleTimer.hpp>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class provides access to the robot's device. */
class Device : public IDevice
{
public:
    /**
     * Constructs the device adapter.
     */
    Device() : IDevice(), m_usbHost(), m_resetTimer(), m_waitTimer(), m_bootloaderModeRequest(false)
    {
    }

    /**
     * Destroys the device adapter.
     */
    virtual ~Device()
    {
    }

    /**
     * Initialize device driver.
     *
     * @return If successfully initialized, returns true. Otherwise, false.
     */
    bool init() final;

    /**
     * Process communication with the device.
     *
     * @return If communication is successful, returns true. Otherwise, false.
     */
    bool process() final;

    /**
     * Get comunication Stream.
     *
     * @return Device data Stream.
     */
    Stream& getStream() final;

    /**
     * Reset the device.
     */
    void reset() final;

    /**
     * Enter Bootloader mode.
     */
    void enterBootloader() final;

    /**
     * Is the device in bootloader mode?
     *
     * @return If device is in bootloader mode, it will return true. Otherwise false.
     */
    bool isInBootloaderMode() const final;

private:
    /** Time to hold the reset line active in milliseconds. */
    static const uint32_t RESET_TIME_MS = 50U;

    /** Time to wait between resets to enter bootloader mode in milliseconds. */
    static const uint32_t WAIT_TIME_BOOTLOADER_MODE_MS = 100U;

    /**
     * USB Host driver.
     */
    USBHost m_usbHost;

    /**
     * Simple Timer for reset of device.
     */
    SimpleTimer m_resetTimer;

    /**
     * Simple Timer for waiting to enter bootloader mode.
     */
    SimpleTimer m_waitTimer;

    /**
     * Bootloader mode request flag.
     */
    bool m_bootloaderModeRequest;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* DEVICE_H */
/** @} */
