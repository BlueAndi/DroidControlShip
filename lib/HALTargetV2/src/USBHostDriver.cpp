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
 * @brief  Abstraction and Stream implementation of USB Host
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "USBHostDriver.h"
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
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

USBHost::USBHost() : Stream(), m_rxQueue(nullptr), m_isBootloaderModeActive(false)
{
}

USBHost::~USBHost()
{
}

bool USBHost::init()
{
    bool isSuccess = false;

    /* Initialize RX Queue. */
    m_rxQueue = xQueueCreate(USB_RX_QUEUE_SIZE, sizeof(uint8_t));

    if (nullptr == m_rxQueue)
    {
        LOG_ERROR("Queue creation failed.");
    }
    else
    {
        LOG_DEBUG("USB driver has been successfully initialized.");
        isSuccess = true;
    }

    return isSuccess;
}

void USBHost::process()
{
    /* Not implemented. */
}

size_t USBHost::write(uint8_t value)
{
    /* Not implemented. */
    return 0;
}

size_t USBHost::write(const uint8_t* buffer, size_t length)
{
    /* Not implemented. */
    return 0;
}

int USBHost::available()
{
    int availableBytes = 0;

    if (nullptr != m_rxQueue)
    {
        availableBytes = uxQueueMessagesWaiting(m_rxQueue);
    }

    return availableBytes;
}

int USBHost::read()
{
    /* Not implemented. */
    return -1;
}

int USBHost::peek()
{
    /* Not implemented. */
    return -1;
}

size_t USBHost::readBytes(uint8_t* buffer, size_t length)
{
    size_t count = 0;

    for (count = 0; count < length; count++)
    {
        uint8_t byte;
        if (false == getByte(byte))
        {
            break;
        }
        else
        {
            buffer[count] = byte;
        }
    }

    return count;
}

void USBHost::requestBootloaderMode()
{
    /* Reset flags, ACM and data queue. */
    reset();

    /* Request bootloader mode. */
    m_isBootloaderModeActive = true;
}

void USBHost::reset()
{
    m_isBootloaderModeActive = false;

    if (nullptr != m_rxQueue)
    {
        (void)xQueueReset(m_rxQueue);
    }
}

bool USBHost::isBootloaderModeActive() const
{
    return m_isBootloaderModeActive;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool USBHost::getByte(uint8_t& byte)
{
    if ((nullptr != m_rxQueue) && (0 < available()))
    {
        if (pdTRUE == xQueueReceive(m_rxQueue, &byte, 0))
        {
            return true;
        }
    }
    return false;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
