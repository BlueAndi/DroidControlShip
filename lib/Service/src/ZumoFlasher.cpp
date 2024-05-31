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
 * @brief  Service to flash a firmware to the robot using the CATERINA bootloader.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "ZumoFlasher.h"
#include <string.h>
#include "Logging.h"

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

ZumoFlasher::ZumoFlasher(Stream& deviceStream) :
    m_device(deviceStream),
    m_status(STATUS_IDLE),
    m_flashingState(STOPPED),
    m_waitingForResponse(false),
    m_processStep(0U),
    m_firmwareData(nullptr),
    m_firmwareSize(0U)
{
}

ZumoFlasher::~ZumoFlasher()
{
}

ZumoFlasher::Status ZumoFlasher::process()
{
    switch (m_flashingState)
    {
    case STOPPED:
        break;

    case INITALIZATING:
        processInitializingState();
        break;

    case WRITING:
        processWritingState();
        break;

    case VERIFYING:
        processVerifyingState();
        break;

    case FINISHING:
        processFinishingState();
        break;

    default:
        LOG_ERROR("Unknown flashing state.");
        fatalError();
        break;
    }

    return m_status;
}

bool ZumoFlasher::startFlashing(char* firmwareData, size_t firmwareSize)
{
    bool isSuccessful = false;

    if ((nullptr != firmwareData) && (0U < firmwareSize))
    {
        m_firmwareData = firmwareData;
        m_firmwareSize = firmwareSize;

        m_flashingState      = INITALIZATING;
        m_status             = STATUS_FLASHING;
        m_waitingForResponse = false;
        m_processStep        = 0U;
        isSuccessful         = true;
    }
    else
    {
        LOG_ERROR("Invalid firmware data or size.");
    }

    return isSuccessful;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool ZumoFlasher::sendCommand(const uint8_t* commandData, size_t commandSize)
{
    bool isSuccessful = false;

    if (commandSize == m_device.write(commandData, commandSize))
    {
        isSuccessful = true;
    }
    else
    {
        LOG_ERROR("Failed to send command.");
    }

    return isSuccessful;
}

ZumoFlasher::ResponseState ZumoFlasher::checkReceivedData(COMMAND_ID commandId)
{
    ResponseState responseState         = ResponseState::RESPONSE_WAITING;
    uint8_t       expectedNumberOfBytes = RSP_LENGTHS[commandId];

    if (expectedNumberOfBytes <= m_device.available())
    {
        uint8_t receivedDataBuffer[expectedNumberOfBytes];

        if (expectedNumberOfBytes == m_device.readBytes(receivedDataBuffer, expectedNumberOfBytes))
        {
            if (0 == memcmp(COMMANDS[commandId], receivedDataBuffer, expectedNumberOfBytes))
            {
                responseState = ResponseState::RESPONSE_OK;
            }
            else
            {
                responseState = ResponseState::RESPONSE_ERROR;
            }
        }
    }

    return responseState;
}

void ZumoFlasher::processInitializingState()
{
    if (false == m_waitingForResponse)
    {
        if (true == sendCommand(COMMANDS[m_processStep], CMD_LENGTHS[m_processStep]))
        {
            LOG_DEBUG("Sent command %u", m_processStep);
            m_waitingForResponse = true;
        }
    }
    else
    {
        ResponseState responseState = checkReceivedData(static_cast<COMMAND_ID>(m_processStep));

        switch (responseState)
        {
        case RESPONSE_OK:

            LOG_DEBUG("CMD %u OK", m_processStep);
            m_waitingForResponse = false;

            /* Read extended fuse is the last step of the init state.*/
            if (CMD_ID_READ_EXT_FUSE == m_processStep)
            {
                m_flashingState = WRITING;
            }

            /* Next step. */
            m_processStep++;
            break;

        case RESPONSE_ERROR:
            LOG_ERROR("CMD %u FAILED", m_processStep);
            fatalError();
            break;

        case RESPONSE_WAITING:
            /* Fallthrough. */
        default:
            break;
        }
    }
}

void ZumoFlasher::processWritingState()
{
    LOG_WARNING("Writing not implemented yet.");
    m_flashingState = VERIFYING;
}

void ZumoFlasher::processVerifyingState()
{
    LOG_WARNING("Verifying not implemented yet.");
    m_flashingState = FINISHING;
}

void ZumoFlasher::processFinishingState()
{
    if (false == m_waitingForResponse)
    {
        if (true == sendCommand(COMMANDS[m_processStep], CMD_LENGTHS[m_processStep]))
        {
            LOG_DEBUG("Sent command %u", m_processStep);
            m_waitingForResponse = true;
        }
    }
    else
    {
        ResponseState responseState = checkReceivedData(static_cast<COMMAND_ID>(m_processStep));

        switch (responseState)
        {
        case RESPONSE_OK:

            LOG_DEBUG("CMD %u OK", m_processStep);
            m_waitingForResponse = false;

            if (CMD_ID_READ_EXT_FUSE == m_processStep)
            {
                /* Leave programming mode. */
                m_processStep = CMD_ID_LEAVE_PROG_MODE;
            }
            /* Leave bootloader is the last step of the finishing state.*/
            else if (CMD_ID_LEAVE_BOOTLOADER == m_processStep)
            {
                m_flashingState = STOPPED;
                m_status        = STATUS_FINISHED_OK;
                m_processStep   = 0U;
            }
            else
            {
                /* Next step. */
                m_processStep++;
            }

            break;

        case RESPONSE_ERROR:
            LOG_ERROR("CMD %u FAILED", m_processStep);
            fatalError();
            break;

        case RESPONSE_WAITING:
            /* Fallthrough. */
        default:
            break;
        }
    }
}

void ZumoFlasher::fatalError()
{
    m_status             = STATUS_FINISHED_ERROR;
    m_flashingState      = STOPPED;
    m_waitingForResponse = false;
    m_processStep        = 0U;
    m_firmwareData       = nullptr;
    m_firmwareSize       = 0U;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
