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
 * @brief  BootloaderCom  realization
 *
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "BootloaderCom.h"
#include <Logging.h>
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
/******************************************************************************
 * Public Methods
 *****************************************************************************/

BootloaderCom::BootloaderCom():
m_state(Idle),
m_waitingForResponse(false)
{
    /*Initialize commands and responses.*/
    m_commands[0] = {Zumo32U4Specification::READ_SW_ID, 1U};
    m_commands[1] = {Zumo32U4Specification::READ_SW_VERSION, 1U};
    m_commands[2] = {Zumo32U4Specification::READ_HW_VERSION, 1U};
    m_commands[3] = {Zumo32U4Specification::READ_PROGRAMMER_TYPE, 1U};
    m_commands[4] = {Zumo32U4Specification::CHECK_AUTO_MEM_ADDR_INC_SUPPORT, 1U};
    m_commands[5] = {Zumo32U4Specification::CHECK_BLOCK_FLASH_SUPPORT, 1U};
    m_commands[6] = {Zumo32U4Specification::READ_SUPPORTED_DEVICE_CODE, 1U};

    /*Initialize responses.*/
    m_responses[0] = {Zumo32U4Specification::EXPECTED_SOFTWARE_ID, 7U};
    m_responses[1] = {Zumo32U4Specification::EXPECTED_SW_VERSION, 2U};
    m_responses[2] = {Zumo32U4Specification::EXPECTED_HW_VERSION, 1U};
    m_responses[3] = {Zumo32U4Specification::EXPECTED_PROGRAMMER_TYPE, 1U};
    m_responses[4] = {Zumo32U4Specification::EXPECTED_SUPPORTS_AUTO_MEM_ADDR_INC, 1U};
    m_responses[5] = {Zumo32U4Specification::EXPECTED_BLOCK_BUFFER_SIZE, 3U};
    m_responses[6] = {Zumo32U4Specification::EXPECTED_DEVICE_CODE, 2U};

}

BootloaderCom::~BootloaderCom()
{
}

void BootloaderCom :: enterBootloader()
{
    Board::getInstance().getDevice().enterBootloader();
    LOG_INFO(" bootloader mode is activated");
}

void BootloaderCom::exitBootloader()
{
    LOG_INFO(" bootloader mode is exited");

}


bool BootloaderCom::process()
{
    static uint8_t commandIndex = 0;
    const ResponseInfo &currentResponse = m_responses[commandIndex]; 
    const CommandInfo &currentCommand = m_commands[commandIndex]; 
     size_t newBytes = 0; 
    uint8_t receiveBuffer[currentResponse.responseSize];

    switch(m_state)
    {
        case Idle:
            /*Handle Idle state*/
            m_waitingForResponse = false;
            if(commandIndex < 7)
            {
                if(true == myFlashManager.sendCommand(currentCommand.command,currentCommand.commandsize))
                {
                    m_waitingForResponse = true;
                    m_state = ReadingResponse;   
                }
                else
                {
                    LOG_ERROR("Failed to send command.");
                    m_state = Idle;
                }
                break;
            }
    
            else
            {
                break;
                //return false;
            }
        case ReadingResponse:
            /*Handle Complete state*/
            LOG_DEBUG("Size of currentsize= %d", currentResponse.responseSize);
            newBytes = myFlashManager.readingStream(receiveBuffer, currentResponse.responseSize);
            if(currentResponse.responseSize == newBytes)
            {
                if(0 < compareExpectedAndReceivedResponse(currentCommand.command,receiveBuffer, newBytes, currentResponse.responseSize))
                {
                    m_state = Complete;
                    m_waitingForResponse = false;
                    break;
                }
                else
                {
                    LOG_ERROR("Compare is false");
                    m_state = Complete;
                    m_waitingForResponse = false;   
                    break;
                }
            }
           break;

        case Complete:
            commandIndex++;
            m_state = Idle;
            break;

        default:
            break;
    }
    return true;
}

bool BootloaderCom::compareExpectedAndReceivedResponse(const uint8_t command[], const uint8_t* receivedResponse, size_t readbytes, size_t expectedSize)
{
    if(nullptr == receivedResponse)
    {
        LOG_ERROR("Received Response is a nullptr");
        return false;
    }
    else if (command == Zumo32U4Specification::READ_SW_ID )
    {
        if( 0 == memcmp(receivedResponse , Zumo32U4Specification::EXPECTED_SOFTWARE_ID,expectedSize))
        {
            LOG_INFO("Received software ID is valid!");
            return true; 
        }
        else
        {
            LOG_ERROR("Received software ID is not valid!");
            return false;

        }
    } 

    else if (command == Zumo32U4Specification::READ_SW_VERSION)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::EXPECTED_SW_VERSION, expectedSize))
        {
            LOG_INFO("Received READ_SW_VERSION is  valid!");
            return true;        
        }
        else
        {
            LOG_ERROR("Received software Version is not valid!");
            return false;
        }

    }
    else if (command == Zumo32U4Specification::READ_HW_VERSION)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::EXPECTED_HW_VERSION, expectedSize))
        {
            LOG_INFO("Received READ_HW_VERSION is valid!");
            return true;
        }
        else
        {
            LOG_ERROR("Received READ_HW_VERSION is not valid!");
            return false;
        }
    }
    else if (command == Zumo32U4Specification::READ_PROGRAMMER_TYPE)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::EXPECTED_PROGRAMMER_TYPE, expectedSize))
        {
            LOG_INFO("Received Programmer Type is valid!");
            return true;
        }
        else
        {
            LOG_ERROR("Received Programmer Type is not valid!");
            return false;
        }
    }
    else if (command == Zumo32U4Specification::CHECK_AUTO_MEM_ADDR_INC_SUPPORT)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::EXPECTED_SUPPORTS_AUTO_MEM_ADDR_INC, expectedSize))
        {
            LOG_INFO("Received  MEM_ADDR_INC is valid!");
            return true;
        }
        else
        {
            LOG_ERROR("Received  MEM_ADDR_INC is not valid!");
            return false;
        }
    }

    else if (command == Zumo32U4Specification::CHECK_BLOCK_FLASH_SUPPORT)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::EXPECTED_BLOCK_BUFFER_SIZE,expectedSize))
        {
            LOG_INFO("Received BLOCK_FLASH_SUPPORT is valid!");
            return true;
        }
        else
        {
            LOG_ERROR("Received BLOCK_FLASH_SUPPORT doesn't match!");
            return false;
        }

    }
    else if(command == Zumo32U4Specification::READ_SUPPORTED_DEVICE_CODE)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::EXPECTED_DEVICE_CODE,expectedSize))
        {
            LOG_INFO("Received Device Code is valid!");
            return true;
        }
        else
        {
            LOG_ERROR("Received Device Code doesn't match!");
            return false;
        }
    }
    else if(command == Zumo32U4Specification::ENTER_PROGRAMMING_MODE)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::RET_OK,expectedSize))
        {
            LOG_INFO("Is in Programming Mode!");
            return true;
        }
        else
        {
            LOG_ERROR("Not in Programming Mode!");
            return false;

        }
    }
    else if(command == Zumo32U4Specification::READ_SIGNATURE)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::EXPECTED_SIGNATURE,expectedSize))
        {
            LOG_INFO("Signature is valid!");
            return true;
        }
        else
        {
            LOG_ERROR("Signature is not valid!");
            return false;
        }
    }
    else if(command == Zumo32U4Specification::READ_LSB_FUSE)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::EXPECTED_LSB_FUSE_VALUE,expectedSize))
        {
            LOG_INFO("LSB FUSE VALUE is valid!");
            return true;
        }
        else
        {
            LOG_ERROR("LSB FUSE VALUE is not valid!");
            return false;
        }
    }
    else if(command == Zumo32U4Specification::READ_MSB_FUSE)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::EXPECTED_MSB_FUSE_VALUE,expectedSize))
        {
            LOG_INFO("MSB FUSE VAlue is valid!");
            return true;
        }
        else
        {
            LOG_ERROR("MSB FUSE VALUE is not valid!");
            return false;
        }
    }
    else if(command == Zumo32U4Specification:: READ_EXTENDED_FUSE)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::EXPECTED_EXTENDED_FUSE_VALUE,expectedSize))
        {
            LOG_INFO("EXTENDED FUSE VALUE is valid!");
            return true;
        }
        else
        {
            LOG_ERROR("EXTENDED FUSE VALUE is not valid!");
            return false;
        }
    }
    return true;
}

bool BootloaderCom::enterprogrammingmode()
{
    uint8_t mybuffer[1U];
    Board::getInstance().process();
    myFlashManager.sendCommand(Zumo32U4Specification::ENTER_PROGRAMMING_MODE, 1U);
    myFlashManager.readingStream(mybuffer, 1U);
    compareExpectedAndReceivedResponse(Zumo32U4Specification::ENTER_PROGRAMMING_MODE,mybuffer,1U,1U);
    return true;
}

bool BootloaderCom::verifySignature()
{
    uint8_t mybuffer[1U];
    Board::getInstance().process();
    myFlashManager.sendCommand(Zumo32U4Specification::READ_SIGNATURE, 1U);
    myFlashManager.readingStream(mybuffer, 1U);
    compareExpectedAndReceivedResponse(Zumo32U4Specification::EXPECTED_SIGNATURE,mybuffer,1U,1U);
    return true;
}

bool BootloaderCom::verifyFuses()
{
    uint8_t mybuffer[1U];
    uint8_t thebuffer[1U];
    uint8_t newbuffer[1U];
    Board::getInstance().process();

    myFlashManager.sendCommand(Zumo32U4Specification::READ_LSB_FUSE, 1U);
    myFlashManager.readingStream(mybuffer, 1U);
    compareExpectedAndReceivedResponse(Zumo32U4Specification::EXPECTED_LSB_FUSE_VALUE,mybuffer,1U,1U);

    myFlashManager.sendCommand(Zumo32U4Specification::READ_MSB_FUSE, 1U);
    myFlashManager.readingStream(thebuffer, 1U);
    compareExpectedAndReceivedResponse(Zumo32U4Specification::EXPECTED_MSB_FUSE_VALUE,thebuffer,1U,1U);

    myFlashManager.sendCommand(Zumo32U4Specification::READ_EXTENDED_FUSE, 1U);
    myFlashManager.readingStream(newbuffer, 1U);
    compareExpectedAndReceivedResponse(Zumo32U4Specification::EXPECTED_EXTENDED_FUSE_VALUE,newbuffer,1U,1U);
    return true;
}



























