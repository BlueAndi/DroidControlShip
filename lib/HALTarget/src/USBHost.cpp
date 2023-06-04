/* MIT License
 *
 * Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Abstraction and Stream implementation of USB Host
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "USBHost.h"
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

uint8_t ACMAsyncOper::OnInit(ACM* pacm)
{
    uint8_t ret;

    ret = pacm->SetControlLineState(CONTROL_LINE_STATE);
    if (hrSUCCESS != ret)
    {
        LOG_ERROR("Control Line State failed with error code: %d", ret);
    }
    else
    {
        LINE_CODING lineCoding;
        lineCoding.dwDTERate   = BAUD_RATE;
        lineCoding.bCharFormat = CHAR_FORMAT;
        lineCoding.bParityType = PARITY_TYPE;
        lineCoding.bDataBits   = NUMBER_OF_DATA_BITS;

        ret = pacm->SetLineCoding(&lineCoding);
        if (hrSUCCESS != ret)
        {
            LOG_ERROR("Line coding failed with error code: %d", ret);
        }
    }

    return ret;
}

USBHost::USBHost() : m_usb(), m_asyncOper(), m_acm(&m_usb, &m_asyncOper), m_rxQueue(nullptr)
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
    else if (0 != m_usb.Init())
    {
        LOG_ERROR("Could not initialize USB driver.");
    }
    else
    {
        LOG_DEBUG("USB driver has been successfully initialized.");
        isSuccess = true;
    }

    return isSuccess;
}

bool USBHost::process()
{
    bool isSuccess = true;

    /* Process USB task. */
    m_usb.Task();

    /* USB Device is connected and ready. */
    if (true == m_acm.isReady())
    {
        uint16_t receivedBytes = UINT8_MAX; /* Doubles as number of bytes to be read, after RcvData(). */
        uint8_t  buf[receivedBytes];
        uint8_t  ret = hrSUCCESS;

        /* Receive Data. */
        ret = m_acm.RcvData(&receivedBytes, buf);

        /* Check for errors. NAK can be safely ignored. */
        if ((hrSUCCESS != ret) && (hrNAK != ret))
        {
            LOG_ERROR("Failed to receive from USB Device with error code: %d", ret);
            isSuccess = false;
        }
        else if ((0U < receivedBytes) && (nullptr != m_rxQueue))
        {
            for (uint16_t idx = 0; idx < receivedBytes; idx++)
            {
                /* Push data to RX Queue. */
                if (errQUEUE_FULL == xQueueSend(m_rxQueue, &buf[idx], 0))
                {
                    LOG_WARNING("RX Queue is full. %d incoming bytes are lost.", (receivedBytes - idx));
                    break;
                }
            }
        }
    }

    return isSuccess;
}

size_t USBHost::write(uint8_t value)
{
    /* Not implemented. */
    return 0;
}

size_t USBHost::write(const uint8_t* buffer, size_t length)
{
    return 0;
}

int USBHost::available()
{
    return 0;
}

int USBHost::read()
{
    /* Not implemented. */
    return 0;
}

int USBHost::peek()
{
    /* Not implemented. */
    return 0;
}

size_t USBHost::readBytes(uint8_t* buffer, size_t length)
{
    return 0;
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
