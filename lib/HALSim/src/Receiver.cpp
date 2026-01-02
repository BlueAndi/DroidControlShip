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
 * @brief  Receiver realization
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Receiver.h"
#include <string.h>

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

void Receiver::setChannel(int32_t channel)
{
    if (nullptr != m_receiver)
    {
        m_receiver->setChannel(channel);
    }
}

size_t Receiver::receive(void* data, size_t size)
{
    size_t read = 0U;

    /* Attention, calling the getData() or getDataSize() is illegal in case the
     * queue is empty!
     */
    if (0 < m_receiver->getQueueLength())
    {
        const uint8_t* message     = static_cast<const uint8_t*>(m_receiver->getData());
        int32_t        messageSize = m_receiver->getDataSize();

        /* If in the last read not all data was read, return the remaining bytes. */
        if (0U < m_dataRemains)
        {
            /* Partial read? */
            if (m_dataRemains > size)
            {
                read = size;
            }
            /* Read all. */
            else
            {
                read = m_dataRemains;
            }

            (void)memcpy(data, &message[messageSize - m_dataRemains], read);
            m_dataRemains -= read;
        }
        else
        {
            /* Partial read? */
            if (messageSize > size)
            {
                read = size;
            }
            /* Read all. */
            else
            {
                read = messageSize;
            }

            (void)memcpy(data, message, read);
            m_dataRemains = messageSize - read;
        }

        /* If the complete head packet is read, it will be thrown away. */
        if (0U == m_dataRemains)
        {
            m_receiver->nextPacket();
        }
    }

    return read;
}

int Receiver::available() const
{
    int32_t dataSize = 0;

    /* If in the last read not all data was read, return the remaining bytes. */
    if (0U < m_dataRemains)
    {
        dataSize = m_dataRemains;
    }
    /* Otherwise return the size of the current head packet.
     * Attention, calling the getDataSize() is illegal in case the queue is
     * empty!
     */
    else if (0 < m_receiver->getQueueLength())
    {
        dataSize = m_receiver->getDataSize();
    }
    else
    {
        /* Nothing to do. */
        ;
    }

    return dataSize;
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
