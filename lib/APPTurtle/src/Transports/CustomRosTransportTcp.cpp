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
 * @brief  Custom Micro-ROS transport using TCP over Arduino WifiClient.
 * @author Norbert Schulz <github@schulznorbert.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "CustomRosTransportTcp.h"

#include <Logging.h>
#include <SimpleTimer.hpp>
#include <Board.h>

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

const String CustomRosTransportTcp::m_protocolName("TCP");

const CustomRosTransportTcp::ReadFunc
CustomRosTransportTcp::m_readFunction[InputState::INPUT_STATE_MAX] = 
{
    &CustomRosTransportTcp::readSizePrefix, 
    &CustomRosTransportTcp::readPendingSizePrefix,
    &CustomRosTransportTcp::readPayload, 
    &CustomRosTransportTcp::readFinish
};

/******************************************************************************
 * Public Methods
 *****************************************************************************/

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool CustomRosTransportTcp::open(void)
{
    bool isOpen = false;

    if (0U == m_tcpClient.connect(m_address, m_port))
    {
        LOG_ERROR("TCP connect error");
    }
    else
    {
        isOpen = true;
    }

    return isOpen;
}

bool CustomRosTransportTcp::close()
{
    m_tcpClient.stop();
    return true;
}

size_t CustomRosTransportTcp::write(const uint8_t* buffer, size_t size, uint8_t* errorCode)
{
    size_t sent = 0U;

    if ((nullptr == buffer) || (0U == size) || (nullptr == errorCode))
    {
        LOG_ERROR("One or more parameters are invalid.");
    }
    else
    {
        uint8_t lengthPrefix[2];
        lengthPrefix[0] = static_cast<uint8_t>(size & 0xFFU);
        lengthPrefix[1] = static_cast<uint8_t>((size >> 8U) & 0xFFU);

        if (sizeof(lengthPrefix) != m_tcpClient.write(lengthPrefix, sizeof(lengthPrefix)))
        {
            LOG_ERROR("Write prefix error (size=%zu, sent=%zu)", size, sent);
            *errorCode = 1U;
        }
        else if ((sent = m_tcpClient.write(buffer, size)) != size)
        {
            LOG_ERROR("Write data error (size=%zu, sent=%zu)", size, sent);
            *errorCode = 1U;
        }
        else
        {
            *errorCode = 0U;
        }
    }

    return sent;
}

size_t CustomRosTransportTcp::read(uint8_t* buffer, size_t size, int timeout, uint8_t* errorCode)
{
    size_t readBytes = 0U;

    if ((nullptr == buffer) || (0U == size) || (nullptr == errorCode))
    {
        LOG_ERROR("One or more parameters are invalid.");
    }
    else
    {
        bool loop = true;

        /**
         * Run receive state machine.
         * The read function relationships are:
         *
         * read() -> [state]->read() -> readInternal() -> WifiClient
         */
        while (true == loop)
        {

            if (InputState::INPUT_STATE_MAX > m_inputState)
            {
                ReadFunc stateReadFunc = m_readFunction[m_inputState]; /**< Get state dependend input reader. */

                /* Read state dependend message portion. */
                loop = (this->*stateReadFunc)(timeout, errorCode);  
            }
            else
            {
                LOG_FATAL("Invalid state %d", static_cast<int>(m_inputState));
                *errorCode = 4U;
                loop       = false;
            }
        }

        if (InputState::INPUT_STATE_FINISH == m_inputState)
        {
            if (readBytes > size)
            {
                /* Internal error, request buffer would be overrun. */
                close();
                *errorCode = 5U;
            }
            else
            {
                /* Provide payload to destination buffer. */
                readBytes = m_payloadLen;
                memcpy(buffer, m_inputBuf, readBytes);

                /* Reset state machine*/
                m_inputState = InputState::INPUT_STATE_INIT;
                m_received   = 0U;
                m_payloadLen = 0U;
            }
        }
    }

    return readBytes;
}

size_t CustomRosTransportTcp::readInternal(uint8_t* buffer, size_t size, int timeout, uint8_t* errorCode)
{
    size_t remaining = size;

    SimpleTimer readTimer;
    bool        timeOver = false;

    *errorCode = 0U;

    readTimer.start(timeout);

    while ((0U < remaining) && (false == timeOver))
    {
        int count = m_tcpClient.read(buffer, remaining);
        if (-1 == count)
        {
            *errorCode = 1U;   /* Set error flag    */
            remaining  = size; /* Return 0 on error.*/
            break;
        }

        remaining -= count;
        buffer += count;

#if defined(TARGET_NATIVE)
        /*
         * SimpleTimer requires simulation time to progress
         * otherwise this loop will block until a packet was received.
         * TODO This is a workaround which needs to be discussed further.
         */
        Board::getInstance().stepTime();
#endif
        if (readTimer.isTimeout())
        {
            timeOver = true;
        }
    }

    return size - remaining;
}

bool CustomRosTransportTcp::readSizePrefix(int timeout, uint8_t* errorCode)
{
    bool loop = true;

    uint8_t prefix[2];

    m_received = 0U;

    switch (readInternal(prefix, sizeof(prefix), timeout, errorCode))
    {
    case 0U: /* nothing received */
        if (0U != *errorCode)
        {
            close();
        }
        loop = false;
        break;

    case 1U: /* only low byte out of the 2 length prefix bytes received. */
        m_payloadLen = static_cast<size_t>(prefix[0]);
        m_inputState = InputState::INPUT_STATE_PREFIX_1;
        loop         = true;
        break;

    case 2U: /* 2 byte prefix received */
        m_payloadLen = static_cast<size_t>(prefix[0]) + (static_cast<size_t>(prefix[1]) << 8);
        m_received   = 0U;
        m_inputState = InputState::INPUT_STATE_PLAYLOAD;
        loop         = true;
        break;

    default: /* should never be possible but ...*/
        *errorCode = 2U;
        loop       = false;
        break;
    }

    return loop;
}

bool CustomRosTransportTcp::readPendingSizePrefix(int timeout, uint8_t* errorCode)
{
    bool    loop = true;
    uint8_t prefix[1];

    switch (readInternal(prefix, sizeof(prefix), timeout, errorCode))
    {
    case 0U: /* nothing received */
        if (0U != *errorCode)
        {
            close();
        }
        loop = false;
        break;

    case 1U: /* High byte received. */
        m_payloadLen |= (static_cast<size_t>(prefix[0]) << 8);
        m_received   = 0U;
        m_inputState = InputState::INPUT_STATE_PLAYLOAD;
        loop         = true;
        break;

    default: /* should never be possible but ...*/
        *errorCode = 2U;
        loop       = false;
        break;
    }

    return loop;
}

bool CustomRosTransportTcp::readPayload(int timeout, uint8_t* errorCode)
{
    bool   loop        = true;
    size_t readByteCnt = readInternal(m_inputBuf + m_received, m_payloadLen - m_received, timeout, errorCode);

    if (0U < readByteCnt)
    {
        m_received += readByteCnt;
        if (m_received == m_payloadLen)
        {
            /* record completed */
            m_inputState = InputState::INPUT_STATE_FINISH;
        }
    }
    else
    {
        if (0U != *errorCode)
        {
            close();
        }
        loop = false;
    }

    return loop;
}

bool CustomRosTransportTcp::readFinish(int timeout, uint8_t* errorCode)
{
    (void)timeout;
    (void)errorCode;

    bool loop = false;
    return loop;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
