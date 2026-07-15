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
 * @brief  Abstraction and Stream implementation of the host connection to the
 *         robot via UART.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "USBHostDriver.h"
#include "Pin.h"
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

USBHost::USBHost() : Stream(), m_robotSerial(Serial1), m_isBootloaderModeActive(false)
{
}

USBHost::~USBHost()
{
}

bool USBHost::init()
{
    bool isSuccess = false;

    if (0U == m_robotSerial.setRxBufferSize(UART_RX_BUFFER_SIZE))
    {
        LOG_ERROR("Could not set the UART RX buffer size.");
    }
    else
    {
        m_robotSerial.begin(BAUD_RATE, SERIAL_8N1, Pin::PIN_ROBOT_UART_RX, Pin::PIN_ROBOT_UART_TX);

        LOG_DEBUG("Robot UART driver has been successfully initialized.");
        isSuccess = true;
    }

    return isSuccess;
}

void USBHost::process()
{
    /* Nothing to do. Reception is handled by the UART driver. */
}

size_t USBHost::write(uint8_t value)
{
    return m_robotSerial.write(value);
}

size_t USBHost::write(const uint8_t* buffer, size_t length)
{
    return m_robotSerial.write(buffer, length);
}

int USBHost::available()
{
    return m_robotSerial.available();
}

int USBHost::read()
{
    return m_robotSerial.read();
}

int USBHost::peek()
{
    return m_robotSerial.peek();
}

size_t USBHost::readBytes(uint8_t* buffer, size_t length)
{
    size_t count = 0;

    while ((count < length) && (0 < m_robotSerial.available()))
    {
        int byte = m_robotSerial.read();

        if (0 > byte)
        {
            break;
        }

        buffer[count] = static_cast<uint8_t>(byte);
        ++count;
    }

    return count;
}

void USBHost::requestBootloaderMode()
{
    /* Reset flags and flush the receive buffer. */
    reset();

    /* Request bootloader mode. */
    m_isBootloaderModeActive = true;
}

void USBHost::reset()
{
    m_isBootloaderModeActive = false;

    /* Flush the receive buffer. */
    while (0 < m_robotSerial.available())
    {
        (void)m_robotSerial.read();
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

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
