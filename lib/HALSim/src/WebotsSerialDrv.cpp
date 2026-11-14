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
 * @brief  Webots serial driver
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "WebotsSerialDrv.h"
#include <string.h>

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

WebotsSerialDrv::WebotsSerialDrv(webots::Emitter* emitter, webots::Receiver* receiver) :
    Stream(),
    m_sender(emitter),
    m_receiver(receiver),
    m_peeked(-1)
{
    /* Nothing to do. */
}

WebotsSerialDrv::~WebotsSerialDrv()
{
    /* Nothing to do. */
}

void WebotsSerialDrv::setRxChannel(int32_t channelId)
{
    m_receiver.setChannel(channelId);
}

void WebotsSerialDrv::setTxChannel(int32_t channelId)
{
    /* Shall be positive for inter-robot communication. */
    if (0 <= channelId)
    {
        m_sender.setChannel(channelId);
    }
}

size_t WebotsSerialDrv::write(uint8_t data)
{
    return m_sender.send(&data, sizeof(data));
}

size_t WebotsSerialDrv::write(const uint8_t* buffer, size_t length)
{
    return m_sender.send(buffer, length);
}

int WebotsSerialDrv::available()
{
    int count = m_receiver.available();

    if (0 <= m_peeked)
    {
        ++count;
    }

    return count;
}

int WebotsSerialDrv::read()
{
    int data = m_peeked;

    if (0 <= data)
    {
        m_peeked = -1;
    }
    else
    {
        uint8_t buffer = 0U;

        if (sizeof(buffer) == m_receiver.receive(&buffer, sizeof(buffer)))
        {
            data = buffer;
        }
    }

    return data;
}

int WebotsSerialDrv::peek()
{
    if (0 > m_peeked)
    {
        m_peeked = read();
    }

    return m_peeked;
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
