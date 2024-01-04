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
 * @brief  Abstract device interface
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALInterfaces
 *
 * @{
 */

#ifndef IDEVICE_H
#define IDEVICE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdbool.h>
#include <Stream.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The abstract device interface. */
class IDevice
{
public:
    /**
     * Destroys the interface.
     */
    virtual ~IDevice()
    {
    }

    /**
     * Initialize device driver.
     *
     * @return If successfully initialized, returns true. Otherwise, false.
     */
    virtual bool init() = 0;

    /**
     * Process communication with the device.
     *
     * @return If communication is successful, returns true. Otherwise, false.
     */
    virtual bool process() = 0;

    /**
     * Get comunication Stream.
     *
     * @return Device data Stream.
     */
    virtual Stream& getStream() = 0;

    /**
     * Reset the device.
     */
    virtual void reset() = 0;

    /**
     * Enter Bootloader mode.
     */
    virtual void enterBootloader() = 0;

    /**
     * Is the device in bootloader mode?
     *
     * @return If device is in bootloader mode, it will return true. Otherwise false.
     */
    virtual bool isInBootloaderMode() const = 0;

protected:
    /**
     * Constructs the interface.
     */
    IDevice()
    {
    }

private:
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* IDEVICE_H */
/** @} */
