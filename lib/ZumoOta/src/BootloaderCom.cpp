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
#include <FileHandler.h>
#include "FileManager.h"
#include <cstring>


/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/
/**
 * @class IntProgCmdProvider
 * @brief Implementation of the command provider for internal programming.
 */
class IntProgCmdProvider: public CmdProvider {
public:
    /**
     * @brief Constructor for the IntProgCmdProvider class.
     */
    IntProgCmdProvider() :
        m_index(0)
    {}

    /**
     * @brief Resets the command provider to its initial state.
     * This method resets the internal index that tracks the current command
     * in the command sequence memory.
     */
    virtual void reset() { m_index = 0; }

    /**
     * @brief Retrieves the next command and the next response.
     * This method returns the next command and its associated response
     * from the command sequence memory.
     * @param cmd Reference to a pointer to the next command.
     * @param rsp Reference to a pointer to the next response.
     * @return True if there is a next command and next response, false otherwise.
     */
    virtual bool next(  CommandInfo *& cmd,   ResponseInfo *& rsp)
    {
        
        if (SEQ_LENGHT > m_index)
        {

            cmd = &m_cmds[m_index];
            rsp = &m_responses[m_index];

            ++m_index;
            return true;
        }
        else
        {
            cmd = nullptr;
            rsp = nullptr;
            return false;
        }
    }

private:
    static const uint32_t SEQ_LENGHT = 12; /**< Length of the command sequence */
    uint32_t m_index; /**< current index in sequence */
    static  CommandInfo m_cmds[SEQ_LENGHT]; /**< Array of command information */
    static  ResponseInfo m_responses[SEQ_LENGHT]; /**< Array of response information */
};

/**
 * @class FwProgCmdProvider
 * @brief Implementation of the command provider for firmware programming.
 *
 * This class provides commands for firmware programming, specifically tailored
 * for the Zumo32U4 device.
 */
class FwProgCmdProvider : public CmdProvider {
    public:
        /**
         * @brief Constructor for the FwProgCmdProvider class.
         * @param fs A pointer to the FileHandler object for handling file operations.
         */
         FwProgCmdProvider(const std::string& fileName):
            m_index(0),
            m_fileName(fileName)
    {}

    /**
     * @brief Resets the command provider to its initial state.
     *
     * This method resets the internal index that tracks the current command
     * in the command sequence memory.
     */
    virtual void reset() { m_index = 0;}

    /**
     * @brief Retrieves the next command and the next response.
     *
     * This method returns the next command and its associated response
     * from the command sequence memory.
     *
     * @param cmd Reference to a pointer to the next command.
     * @param rsp Reference to a pointer to the next response.
     * @return True if there is a next command and next response, false otherwise.
     */
    virtual bool next( CommandInfo *& cmd, ResponseInfo *& rsp)
    {
        /*Determine the command and response for the current chunk.*/
        uint32_t commandIndex = m_index % SEQ_LENGHT;
        cmd = &m_cmds[commandIndex];

        if(cmd->command == Zumo32U4Specification::SET_MEMORY_ADDR)
        {
            cmd->commandsize = FinalLength;
        }
        else if(cmd->command == Zumo32U4Specification::WRITE_MEMORY_PAGE)
        {
            size_t bytesRead = m_fileManager.read128Bytes(m_fileName.c_str(), m_buffer);
            /*If no bytes were read, there are no more chunks left.*/
            if (0 == bytesRead)
            {
                cmd = nullptr;
                rsp = nullptr;
                return false;
            }
            else
            {
                cmd->commandsize += bytesRead;
            }
        }
        rsp = &m_responses[commandIndex];

        /* Move to the next chunk.*/
        ++m_index;

        return true;
    }

    FileManager m_fileManager; /**< Instance of Filemanger class.*/
    const size_t FinalLength = 3;/**< Final Size of the updated Command.*/

private:
    uint8_t m_index; /**< current index in sequence.*/
    FileHandler *m_fileSystem; /**< Instance of FileHandler.*/
    static  CommandInfo m_cmds[]; /**< Array of command information.*/
    static  ResponseInfo m_responses[]; /**< Array of response information.*/
    std::string m_fileName = "/zumo_firmware.bin"; /**< Name of the file to be read.*/
    static const uint32_t SEQ_LENGHT = 2U; /**< Length of the command sequence.*/
};

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/
std::string fileName = "/zumo_firmware.bin";
uint8_t* CmdProvider::m_buffer = nullptr;
uint16_t BootloaderCom:: m_currWriteMemAddr = 0x0000; 


 CommandInfo IntProgCmdProvider::m_cmds[] = {
    {Zumo32U4Specification::READ_SW_ID, sizeof(Zumo32U4Specification::READ_SW_ID)},
    {Zumo32U4Specification::READ_SW_VERSION, sizeof(Zumo32U4Specification::READ_SW_VERSION)},
    {Zumo32U4Specification::READ_HW_VERSION, sizeof(Zumo32U4Specification::READ_HW_VERSION)},
    {Zumo32U4Specification::READ_PROGRAMMER_TYPE, sizeof(Zumo32U4Specification::READ_PROGRAMMER_TYPE)},
    {Zumo32U4Specification::CHECK_AUTO_MEM_ADDR_INC_SUPPORT, sizeof(Zumo32U4Specification::CHECK_AUTO_MEM_ADDR_INC_SUPPORT)},
    {Zumo32U4Specification::CHECK_BLOCK_FLASH_SUPPORT, sizeof(Zumo32U4Specification::CHECK_BLOCK_FLASH_SUPPORT)},
    {Zumo32U4Specification::READ_SUPPORTED_DEVICE_CODE, sizeof(Zumo32U4Specification::READ_SUPPORTED_DEVICE_CODE)},
    {Zumo32U4Specification::ENTER_PROGRAMMING_MODE, sizeof(Zumo32U4Specification::ENTER_PROGRAMMING_MODE)},
    {Zumo32U4Specification::READ_SIGNATURE,sizeof(Zumo32U4Specification::READ_SIGNATURE)},
    {Zumo32U4Specification::READ_LSB_FUSE,sizeof(Zumo32U4Specification::READ_LSB_FUSE)},
    {Zumo32U4Specification::READ_MSB_FUSE,sizeof(Zumo32U4Specification::READ_MSB_FUSE)},
    {Zumo32U4Specification::READ_EXTENDED_FUSE,sizeof(Zumo32U4Specification::READ_EXTENDED_FUSE)}
};

 ResponseInfo IntProgCmdProvider::m_responses[] = {
    {Zumo32U4Specification::EXPECTED_SOFTWARE_ID, sizeof(Zumo32U4Specification::EXPECTED_SOFTWARE_ID)},
    {Zumo32U4Specification::EXPECTED_SW_VERSION,sizeof(Zumo32U4Specification::EXPECTED_SW_VERSION) },
    {Zumo32U4Specification::EXPECTED_HW_VERSION, sizeof(Zumo32U4Specification::EXPECTED_HW_VERSION)},
    {Zumo32U4Specification::EXPECTED_PROGRAMMER_TYPE, sizeof(Zumo32U4Specification::EXPECTED_PROGRAMMER_TYPE)},
    {Zumo32U4Specification::EXPECTED_SUPPORTS_AUTO_MEM_ADDR_INC,sizeof(Zumo32U4Specification::EXPECTED_SUPPORTS_AUTO_MEM_ADDR_INC)},
    {Zumo32U4Specification::EXPECTED_BLOCK_BUFFER_SIZE, sizeof(Zumo32U4Specification::EXPECTED_BLOCK_BUFFER_SIZE)},
    {Zumo32U4Specification::EXPECTED_DEVICE_CODE, sizeof(Zumo32U4Specification::EXPECTED_DEVICE_CODE)},
    {Zumo32U4Specification::ENTER_PROGRAMMING_MODE,sizeof(Zumo32U4Specification::ENTER_PROGRAMMING_MODE)},
    {Zumo32U4Specification::EXPECTED_SIGNATURE,sizeof(Zumo32U4Specification::EXPECTED_SIGNATURE)},
    {Zumo32U4Specification::EXPECTED_LSB_FUSE_VALUE,sizeof(Zumo32U4Specification::EXPECTED_LSB_FUSE_VALUE)},
    {Zumo32U4Specification::EXPECTED_MSB_FUSE_VALUE,sizeof(Zumo32U4Specification::EXPECTED_MSB_FUSE_VALUE)},
    {Zumo32U4Specification::EXPECTED_EXTENDED_FUSE_VALUE,sizeof(Zumo32U4Specification::EXPECTED_EXTENDED_FUSE_VALUE)},
};

 CommandInfo FwProgCmdProvider::m_cmds[] = {
            {Zumo32U4Specification::SET_MEMORY_ADDR, sizeof(Zumo32U4Specification::SET_MEMORY_ADDR)},
            {Zumo32U4Specification::WRITE_MEMORY_PAGE, sizeof(Zumo32U4Specification::WRITE_MEMORY_PAGE)}
};

 ResponseInfo FwProgCmdProvider::m_responses[] = {
            {Zumo32U4Specification::RET_OK, sizeof(Zumo32U4Specification::RET_OK)},
            {Zumo32U4Specification::RET_OK, sizeof(Zumo32U4Specification::RET_OK)}
};

/**
 * @brief Static instance of the internal programming command provider.
 * This static variable holds an instance of the IntProgCmdProvider class,
 * which provides commands for internal programming.
 */
static IntProgCmdProvider m_initProvider;

/**
 * @brief Static instance of the internal programming command provider.
 * This static variable holds an instance of the FwProgCmdProvider class,
 * which provides commands for internal programming.
 */
static FwProgCmdProvider m_fwProvider(fileName);

/**
 * @ brief Array of command providers.
 * This static array holds pointers to command providers. It can be used to
 * iterate through different command providers during operation.
 */
static CmdProvider * cmdProviders[] = 
{
    &m_initProvider, /**< Pointer to the internal programming command provider.*/
    &m_fwProvider,
    nullptr  /**< End marker indicating the end of the array.*/
};

/******************************************************************************
 * Public Methods
 *****************************************************************************/

BootloaderCom::BootloaderCom():
m_state(SelectCmdProvider),
m_waitingForResponse(false),
m_cmdProvider(&m_initProvider),
m_currentCommand(nullptr),
m_currentResponse(nullptr),
m_currentProvider(0)
{
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
    size_t newBytes = 0; 
    uint8_t receiveBuffer[8];

    switch(m_state)
    {
        case SelectCmdProvider:
            m_cmdProvider = cmdProviders[m_currentProvider];
            if (nullptr != m_cmdProvider)
            {
                m_cmdProvider->reset();

                ++m_currentProvider;
                m_state = Idle;
            }
            else
            {
                // TODO: We are done, no more provider, signal completion to web interface
                break;
            }
            break;

        case Idle:
            /*Handle Idle state*/
            m_waitingForResponse = false;
            if(m_cmdProvider->next(m_currentCommand, m_currentResponse))
            {
                if(m_currentCommand->command == Zumo32U4Specification::SET_MEMORY_ADDR)
                {
                    size_t newSize = m_currentCommand->commandsize;
                    uint8_t newCommand[newSize];
                    std::memcpy(newCommand, m_currentCommand->command, m_currentCommand->commandsize);
                    // Rufen Sie die Funktion zur Aktualisierung des Befehls auf
                    updateCommand(newCommand);
                    // Senden Sie den aktualisierten Befehl
                    if (true == m_flashManager.sendCommand(newCommand, newSize))
                    {
                        m_waitingForResponse = true;
                        m_state = ReadingResponse;
                    }
                    else
                    {
                        LOG_ERROR("Failed to send command.");
                        m_state = Idle;
                    }
                }
                else if(m_currentCommand->command == Zumo32U4Specification::WRITE_MEMORY_PAGE)
                {
                    size_t newSize = m_currentCommand->commandsize;
                    if (newSize > sizeof(Zumo32U4Specification::WRITE_MEMORY_PAGE))
                    {
                        uint8_t newCommand[newSize];
                        uint8_t* newbuffer = m_cmdProvider->m_buffer;

                        /*Call the function to update the command.*/
                        update2Command(newCommand, newbuffer, newSize);

                        /*Send the updated command.*/
                        if (true == m_flashManager.sendCommand(newCommand, newSize))
                        {
                            m_currentProvider -= 2;
                        }
                        else
                        {
                            LOG_ERROR("Failed to send command.");
                            m_state = Idle;
                        }

                    }
                    else
                    {
                        m_waitingForResponse = true;
                        m_state = ReadingResponse;
                    }

                }
                else if (true == m_flashManager.sendCommand(
                    m_currentCommand->command,
                    m_currentCommand->commandsize))
                    {
                        m_waitingForResponse = true;
                        m_state = ReadingResponse;
                    }

                else
                {
                    LOG_ERROR("Failed to send command.");
                    m_state = Idle;
                }
            }
            else
            {
                m_state = SelectCmdProvider;
            }
            break;

        case ReadingResponse:
            /*Handle Complete state*/
            LOG_DEBUG("Size of currentresponse= %d", m_currentResponse->responseSize);
            newBytes = m_flashManager.readingStream(receiveBuffer, m_currentResponse->responseSize);
            if(m_currentResponse->responseSize == newBytes)
            {
                if(0 < compareExpectedAndReceivedResponse(
                    m_currentCommand->command,receiveBuffer, newBytes, m_currentResponse->responseSize))
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
    else if(command == Zumo32U4Specification::SET_MEMORY_ADDR)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::SET_MEMORY_ADDR,expectedSize))
        {
            LOG_INFO("MEMORY ADRESSE is valid!");
            return true;
        }
        else
        {
            LOG_ERROR("MEMORY ADRESSE is not valid!");
            return false;
        }
    }
    else if(command == Zumo32U4Specification:: WRITE_MEMORY_PAGE)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::SET_MEMORY_ADDR,expectedSize))
        {
            LOG_INFO("Writing Data to Zumo was successfull");
            return true;
        }
        else
        {
            LOG_INFO("Writing Data to Zumo failed!");
            return true;
        }
    }
    return true;
}

 void BootloaderCom:: updateCommand(uint8_t newCommand[])
{
    if (nullptr != newCommand)
    {
        newCommand[0] = 0x41;  /**< The first value is always 0x41.*/
        newCommand[1] = (m_currWriteMemAddr >> 8) & 0xFF; /**<Higher 8 bits of m_currWriteMemAddr.*/
        newCommand[2] = m_currWriteMemAddr & 0xFF; //**<Lower 8 bits of m_currWriteMemAddr.*/

        /*Increase m_currWriteMemAddr by 64 for the next command.*/
        m_currWriteMemAddr += 64U;
    }
    else
    {
        LOG_ERROR("newCommand is a nullptr!");
    }

}

void BootloaderCom:: update2Command(uint8_t newCommand[], uint8_t* buffer, size_t bufferSize)
{
    if (nullptr != newCommand)
    {
        newCommand[0] = 0x42; /**<Command identifier for writing memory pages.*/
        newCommand[1] = 0x00; /**<Command identifier for writing memory pages.*/
        newCommand[2] = 0x80; /**<Command identifier for writing memory pages.*/
        newCommand[3] = 0x46; /**<Command identifier for writing memory pages.*/

        /*Copy data from the buffer into the command starting from index 4.*/
        for (size_t i = 0; i < bufferSize; ++i)
        {
            newCommand[4 + i] = buffer[i];
        }
    }
    else
    {
        LOG_ERROR("newCommand is a nullptr!");
    }
    
}
























