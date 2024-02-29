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
#include <FS.h>
#include <LittleFS.h>
#include <vector>


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
    virtual bool reset() 
    {
        m_index = 0;
        return true;
    }

    /**
     * @brief Retrieves the next command and the next response.
     * This method returns the next command and its associated response
     * from the command sequence memory.
     * @param cmd Reference to a pointer to the next command.
     * @param rsp Reference to a pointer to the next response.
     * @return True if there is a next command and next response, false otherwise.
     */
    virtual bool next(const CommandInfo *& cmd, const ResponseInfo *& rsp)
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
    static const uint32_t SEQ_LENGHT = 12; /**< Number of required commands to initilize the Flash.*/
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
         * @param fileName The name of the file to be used for firmware programming.
         * Initializes a new instance of the FwProgCmdProvider class with the provided file name.
         * @remarks The file specified by fileName should contain the firmware to be programmed.
         */
        FwProgCmdProvider(const char* fileName):
            m_fileName(fileName),
            m_firmwareBytesRead(0),
            m_updatedCmd(),
            m_firmwareFile(),
            m_updatedCmdBuffer{},
            m_count(0U),
            m_currWriteMemAddr(0x0000)
        {}

        /**
         * @brief Resets the command provider to its initial state.
         * This method resets the internal state of the command provider.
         * It checks if the firmware file is open and closes it if necessary.
         * Then, it attempts to open the firmware file with read access.
         * @return True if the firmware file is successfully opened, false otherwise.
         */
        virtual bool reset()
        {
            String tempvar = BootloaderCom::getFirmwareName();
            m_fileName = tempvar.c_str();
            /* Check if the file exists in the file system. */
            LOG_INFO("m_fileName in Reset FwProg ist %s", m_fileName);

            if (!LittleFS.exists(m_fileName))
            {
                /*File does not exist.*/
                return false;
            }

            /* Check if the file is already open. */
            if (m_firmwareFile)
            {
                m_firmwareFile.close();
            }

            /* Open the file in read mode. */
            m_firmwareFile = LittleFS.open(m_fileName, "r");
            if (!m_firmwareFile)
            {
                /*Failed to open the file.*/
                return false;
            }

            /*Calculate the size of the file.*/
            size_t fileSize = m_firmwareFile.size();

            /*Output the file size.*/
            LOG_INFO("File size: %d",fileSize);
            return true;
        }


        /**
         * @brief Retrieves the count of processed firmware blocks. 
         * This function returns the number of firmware blocks that have been processed.
         * The count represents the progress of the firmware programming process.
         * @return The count of processed firmware blocks as a uint8_t.
         */
        uint8_t getCount() const
        {
            return m_count;
        }

        /**
         * @brief Retrieves the next command and the next response.
         * This method returns the next command and its associated response
         * from the command sequence memory.
         * @param cmd Reference to a pointer to the next command.
         * @param rsp Reference to a pointer to the next response.
         * @return True if there is a next command and next response, false otherwise.
         */
        virtual bool next(const CommandInfo*& cmd, const ResponseInfo*& rsp)
        {
            if (m_nextCmd == FLSPRG_COMPLETE) 
            {
                cmd = nullptr;
                rsp = nullptr;

                if (m_firmwareFile)
                {
                    m_firmwareFile.close(); /**<Programming is done you can close the file.*/
                }

                return false;
            }
            else
            {
                rsp = &m_responses[m_nextCmd];
                /*Update the command with required content to write in the flash memory.*/
                if(true == updateFwProgCmd(m_updatedCmd))
                {
                    cmd = &m_updatedCmd;
                    return true;
                }
                else
                {
                    cmd = nullptr;
                    rsp = nullptr;
                    return false;
                }
            }
        }

private:
    /**
     * @brief Length of a flash memory block in bytes.
     * This constant defines the length of a flash memory block in bytes.
     * It is used to specify the size of the blocks when reading or writing data to flash memory.
     */
    static const size_t FLASH_BLOCK_LENGTH = 128U;
    uint8_t m_count;/**<Counter for the number of processed firmware blocks.*/

    /**
     * @brief Length of a write block command in bytes.
     * This constant defines the length of a write block command in bytes.
     * It is used to specify the size of the command for writing a block of data to memory.
     */
    static const size_t WRITE_BLOCK_CMD_LENGTH = 4U;
    static  CommandInfo m_cmds[]; /**< Array of command information.*/
    static  ResponseInfo m_responses[]; /**< Array of response information.*/
    CommandInfo m_updatedCmd; /**<Intermediate storage for updated commands.*/
    const char* m_fileName = (""); /**< Name of the file to be read.*/
    size_t m_firmwareBytesRead; /**< Number of bytes read from the firmware file.*/
    File m_firmwareFile; /**< Handle for the firmware file.*/

    /**
     * @brief Enumeration of programming commands.
     * This enumeration defines the various programming commands used during firmware programming.
     * It includes commands for setting memory addresses and writing data blocks.
     */
    enum ProgCmds
    {
        FLSPRG_SET_ADDR,    /**< Command to set the memory address.*/
        FLSPRG_WRITE_BLOCK, /**< Command to write a data block to memory.*/
        FLSPRG_COMPLETE     /**<Signalize the end of the Flashing.*/
    };

    /** 
     * @brief The next programming command to be executed. 
     * This variable represents the next programming command to be executed in the sequence.
     * It initializes with FLSPRG_SET_ADDR, typically used to set the memory address first.
     */
    ProgCmds m_nextCmd = FLSPRG_SET_ADDR;
    uint8_t m_updatedCmdBuffer[WRITE_BLOCK_CMD_LENGTH + FLASH_BLOCK_LENGTH]; /**<Buffer for updated commands.*/

    /**
     * @brief Current memory address for writing data during firmware programming. 
     * This variable represents the current memory address where data will be written
     * during the firmware programming process. It is used to keep track of the
     * memory address for sequential data writes.
     */
    uint16_t  m_currWriteMemAddr; 

    /**
     * @brief Updates the command.
     * This method updates the provided command based on specific requirements
     * before it is sent for execution. It modifies the command buffer and size
     * to reflect the changes required by the bootloader communication protocol.
     * @param fwProgCmd Pointer to the CommandInfo structure representing the command to be updated.
     * The method modifies the command buffer and size to reflect the changes.
     * @remarks This method is called before sending each command to the bootloader for execution.
     * The update process includes modifying command identifiers and parameters to adhere to the
     * bootloader communication protocol requirements.
     */
    bool updateFwProgCmd(CommandInfo& fwProgCmd)
    {
        if (m_nextCmd == FLSPRG_SET_ADDR)
        {
            /*Perform the required updates.*/
            m_updatedCmdBuffer[0] = 0x41;  /**< The first value is always 0x41.*/
            m_updatedCmdBuffer[1] = (m_currWriteMemAddr >> 8) & 0xFF; /**<Higher 8 bits of m_currWriteMemAddr.*/
            m_updatedCmdBuffer[2] = m_currWriteMemAddr & 0xFF; //**<Lower 8 bits of m_currWriteMemAddr.*/
            /**Increase m_currWriteMemAddr by 64 for the next command.
            *Each address addresses one WORD with 2(!) bytes, this is why one page consists of 64 addresses but 128 bytes memory.
            */
            m_currWriteMemAddr += FLASH_BLOCK_LENGTH/2U;
            /**<Set the Cmd instance to the buffer.*/
            fwProgCmd.command = m_updatedCmdBuffer;
            fwProgCmd.commandsize = 3;
            m_nextCmd = FLSPRG_WRITE_BLOCK;
               return true;
        }
        else if (m_nextCmd == FLSPRG_WRITE_BLOCK)
        {
            /*Update the command identifier bytes for write block*/
            m_updatedCmdBuffer[0] = 0x42;
            m_updatedCmdBuffer[1] = 0x00;
            m_updatedCmdBuffer[2] = 0x80;
            m_updatedCmdBuffer[3] = 0x46;
            /*Update the command for WRITE_MEMORY_PAGE*/
            m_firmwareBytesRead = m_firmwareFile.read(
                &m_updatedCmdBuffer[WRITE_BLOCK_CMD_LENGTH],
                FLASH_BLOCK_LENGTH);
            /*Check if there was an error reading the file.*/
            if (m_firmwareBytesRead < 0)
            {
                /*Error reading the file.*/
                LOG_ERROR("No Bytes available!");
                return false;
            } 
            else if (0 == m_firmwareBytesRead)
            {
                LOG_DEBUG(" last Chunk written = %d", m_count);
                m_nextCmd = FLSPRG_COMPLETE;
                return false;
            }
            /*If the number of bytes read is less than MAX_BYTES_TO_READ,fill the rest of the buffer with a pattern.*/
            size_t gapFill = FLASH_BLOCK_LENGTH - m_firmwareBytesRead;
            if (0U != gapFill)
            {
                m_nextCmd = FLSPRG_COMPLETE;
                while (gapFill > 0U)
                {
                    m_updatedCmdBuffer[WRITE_BLOCK_CMD_LENGTH +m_firmwareBytesRead] = 0xFF;
                    --gapFill;
                    ++m_firmwareBytesRead;
                }
            }
            else
            {
                m_nextCmd = FLSPRG_SET_ADDR;
            }
            fwProgCmd.command = m_updatedCmdBuffer;
            fwProgCmd.commandsize = sizeof(m_updatedCmdBuffer);
            m_count++;
            LOG_DEBUG("Write count = %d", m_count);
            return true;
        }

        return true;
    }
};

/**
 * @class ByteCheckCmdProvider
 * @brief Implementation of a command provider for reading and verifying bytes.
 *
 * This class provides functionality to read bytes and compare them against expected bytes.
 * It is designed to be used in a bootloader communication system to ensure the integrity
 * of data read from a target device.
 */
class ByteCheckCmdProvider : public CmdProvider {
public:
    /**
     * @brief Constructs a new ByteCheckCmdProvider object.
     *
     * @param [in] fileName The name of the file containing the expected bytes.
     * @remarks The file specified by fileName should contain the expected bytes to be compared with.
     */
    ByteCheckCmdProvider(const char* fileName) :
        m_updatedCmd(),
        m_updatedRsp(),
        m_updatedCmdBuffer{},
        m_fileName(fileName),
        m_firmwareFile(),
        m_count(0U),
        m_firmwareSize(0U),
        m_block(0),
        m_currWriteMemAddr(0x0000)
     {}

    /**
     * @brief Resets the firmware file and prepares it for reading.
     * 
     * This function closes the currently opened firmware file, if any,
     * and then attempts to open the firmware file specified by m_fileName
     * in read mode. It retrieves the size of the firmware file and calculates
     * the maximum number of buffers based on the file size divided by 128.
     * @return True if the firmware file is successfully opened, false otherwise.
     */
    virtual bool reset()
    { 
        String tempvar = BootloaderCom::getFirmwareName();
        if (false == tempvar.startsWith("/"))
        {
            tempvar = "/" + tempvar;
        }
        m_fileName = tempvar.c_str();
        /*Check if the file is already open.*/
        if (m_firmwareFile)
        {
            m_firmwareFile.close();
        }

        /*If the file is not yet open, open it.*/
        m_firmwareFile = LittleFS.open(m_fileName, "r");
        /*Retrieves the size of the firmware file and calculates the maximum number of buffers.*/
        m_firmwareSize = m_firmwareFile.size();
        LOG_INFO("Size of the file = %d", m_firmwareSize);
        m_block = m_firmwareFile.size() / FLASH_BLOCK_LENGTH;
        if(m_firmwareFile.size() % FLASH_BLOCK_LENGTH == 0)
        {
            /*m_block doesn't change*/
        }
        else
        {
            /*Incrementing m_block by 1 to accommodate the remainder of the file*/
            m_block += 1;
        }
        LOG_INFO("m_block = %d", m_block);
        return false != m_firmwareFile;
    }

    /**
     * @brief Retrieves the next command and the next response.
     *
     * This method reads bytes from the target device, compares them with
     * the expected bytes, and provides the next command and response accordingly.
     *
     * @param [out] cmd Reference to a pointer to the next command.
     * @param [out] rsp Reference to a pointer to the next response.
     * @return True if there is a next command and response, false otherwise.
     */
    virtual bool next(const CommandInfo*& cmd, const ResponseInfo*& rsp) override
    {
        if(m_count < m_block)
        {
            /*Keep the original command.*/
            if( m_nextCmd == CCMD_SET_ADDR)
            {
                rsp = &m_responses[m_nextCmd];
    
                /*Update the command with required content to write in the flash memory.*/
                if(true == updateSetAdressCmd(m_updatedCmd))
                {
                    cmd = &m_updatedCmd;
                    return true;
                }
            }
            else if (m_nextCmd == CCMD_READ_BLOCK)
            {
                /*Increment the count of processed buffers.*/
                m_count++;
                cmd =&m_cmds[m_nextCmd];

                m_updatedRsp.expectedResponse = nullptr;
                m_updatedRsp.responseSize = FLASH_BLOCK_LENGTH;
                rsp = &m_updatedRsp;

                /*Set m_nextCmd to CCMD_SET_ADDR to prepare for the next command cycle.*/
                m_nextCmd = CCMD_SET_ADDR;
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            m_nextCmd = CCMD_COMPLETE;
            cmd = nullptr;
            rsp = nullptr;
            if (m_firmwareFile)
            {
                m_firmwareFile.close(); /**<Programming is done you can close the file.*/
            }
            return false;
        }
        return true;
    }

 
    /**
     * @brief Enumeration of programming commands.
     * This enumeration defines the various programming commands used during firmware programming.
     * It includes commands for setting memory addresses and writing data blocks.
     */
    enum CheckCmds
    {
        CCMD_SET_ADDR,    /**< Command to set the memory address. */
        CCMD_READ_BLOCK,  /**< Command to write a data block to memory.*/
        CCMD_COMPLETE   /**<Signalize the end of the Flashing.*/
    };
    /**
     * @brief Retrieves the count of processed firmware blocks. 
     * This function returns the number of firmware blocks that have been processed.
     * The count represents the progress of the firmware programming process.
     * @return The count of processed firmware blocks as a uint8_t.
     */
    uint8_t getCount() const
    {
        return m_count;
    }

    /** 
    * @brief The next programming command to be executed. 
    * This variable represents the next programming command to be executed in the sequence.
    * It initializes with FLSPRG_SET_ADDR, typically used to set the memory address first.
    */
    CheckCmds m_nextCmd = CCMD_SET_ADDR;

    /**
     * @brief Length of a flash memory block in bytes.
     * This constant defines the length of a flash memory block in bytes.
     * It is used to specify the size of the blocks when reading or writing data to flash memory.
     */
    static const size_t FLASH_BLOCK_LENGTH = 128U;
    /**
     * @brief Length of a Set adress Updated Command in bytes.
     * It is used to specify the size of the updated SET_ADRESS command when reading or writing data to flash memory.
     */
    static const size_t SET_ADRESS_LENGTH = 3U;


private:
    const char* m_fileName = (""); /**< Name of the file to be read.*/
    static  CommandInfo m_cmds[]; /**< Array of command information.*/
    static ResponseInfo m_responses[]; /**< Array of response information.*/
    CommandInfo m_updatedCmd; /**<Intermediate storage for updated commands.*/
    ResponseInfo m_updatedRsp; /**<Intermediate storage for experted responses.*/
    File m_firmwareFile; /**< Handle for the firmware file.*/
    uint8_t m_updatedCmdBuffer[SET_ADRESS_LENGTH]; /**<Buffer for updated commands.*/
    /**
     * @brief Current memory address for writing data during firmware programming. 
     * This variable represents the current memory address where data will be written
     * during the firmware programming process. It is used to keep track of the
     * memory address for sequential data writes.
     */
    uint16_t  m_currWriteMemAddr; 
    size_t m_block; /**<Maximum number of firmware buffers.*/
    uint8_t m_count; /**<Counter for the number of processed firmware blocks.*/
    size_t m_firmwareSize;/**<Size of the firmware file.*/

    /**
    * @brief Updates the command.
    * This method updates the provided command based on specific requirements
    * before it is sent for execution. It modifies the command buffer and size
    * to reflect the changes required by the bootloader communication protocol.
    * @param cmd Pointer to the CommandInfo structure representing the command to be updated.
    * The method modifies the command buffer and size to reflect the changes.
    * @remarks This method is called before sending each command to the bootloader for execution.
    * The update process includes modifying command identifiers and parameters to adhere to the
    * bootloader communication protocol requirements.
    */
    bool updateSetAdressCmd(CommandInfo& cmd)
    {
        if (m_nextCmd == CCMD_SET_ADDR)
        {
            /*Perform the required updates.*/
            m_updatedCmdBuffer[0] = 0x41;  /**< The first value is always 0x41.*/
            m_updatedCmdBuffer[1] = (m_currWriteMemAddr >> 8) & 0xFF; /**<Higher 8 bits of m_currWriteMemAddr.*/
            m_updatedCmdBuffer[2] = m_currWriteMemAddr & 0xFF; //**<Lower 8 bits of m_currWriteMemAddr.*/

            /**Increase m_currWriteMemAddr by 64 for the next command.
             *Each address addresses one WORD with 2(!) bytes, this is why one page consists of 64 addresses but 128 bytes memory.
             */
            m_currWriteMemAddr += FLASH_BLOCK_LENGTH/2U;

            /**<Set the Cmd instance to the buffer.*/
            cmd.command = m_updatedCmdBuffer;
            cmd.commandsize = SET_ADRESS_LENGTH;
            m_nextCmd = CCMD_READ_BLOCK;
            return true;

        }
        else
        {
            m_nextCmd = CCMD_READ_BLOCK;
            return false;
        }
    }
};

/**
 * @class ReadFusesCmdProvider
 * @brief Implementation of the  command provider for reading Fuses.
 */
class ReadFusesCmdProvider : public CmdProvider {
public:
    /**
     * @brief Constructor for the FinalCmdProvider class.
     */
    ReadFusesCmdProvider() :
        m_index(0)
    {}

    /**
     * @brief Resets the command provider to its initial state.
     * This method resets the internal index that tracks the current command
     * in the command sequence memory.
     */
    virtual bool reset() 
    {
        m_index = 0;
        return true;
    }

    /**
     * @brief Retrieves the next command and the next response.
     * This method returns the next command and its associated response
     * from the command sequence memory.
     * @param cmd Reference to a pointer to the next command.
     * @param rsp Reference to a pointer to the next response.
     * @return True if there is a next command and next response, false otherwise.
     */
    virtual bool next(const CommandInfo *& cmd, const ResponseInfo *& rsp)
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
    static const uint32_t SEQ_LENGHT = 3; /**< Number of required commands to read the Fuses.*/
    uint32_t m_index; /**< current index in sequence */
    static  CommandInfo m_cmds[SEQ_LENGHT]; /**< Array of command information */
    static  ResponseInfo m_responses[SEQ_LENGHT]; /**< Array of response information */
};

/**
 * @class ExitCmdProvider
 * @brief Implementation of the exit command provider for internal programming.
 */
class ExitCmdProvider: public CmdProvider {
public:
    /**
     * @brief Constructor for the ExitCmdProvider class.
     */
    ExitCmdProvider() :
        m_index(0)
    {}

    /**
     * @brief Resets the command provider to its initial state.
     * This method resets the internal index that tracks the current command
     * in the command sequence memory.
     */
    virtual bool reset() 
    {
        m_index = 0;
        return true;
    }

/**
* @brief Retrieves the next command and the next response.
* This method returns the next command and its associated response
* from the command sequence memory.
* @param cmd Reference to a pointer to the next command.
* @param rsp Reference to a pointer to the next response.
* @return True if there is a next command and next response, false otherwise.
*/
virtual bool next(const CommandInfo *& cmd, const ResponseInfo *& rsp)
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
    static const uint32_t SEQ_LENGHT = 2; /**< Number of required commands to initilize the Flash.*/
    uint32_t m_index; /**< current index in sequence */
    static  CommandInfo m_cmds[SEQ_LENGHT]; /**< Array of command information */
    static  ResponseInfo m_responses[SEQ_LENGHT]; /**< Array of response information */
};

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/
/**
 * @brief Name of the firmware file to be used for programming. 
 * This variable holds the name of the firmware file ("/zumo_firmware.bin")
 * that will be used for programming the device.
 * In the finalversion this should be provided by the Webserver
 */
 String fileName = BootloaderCom::getFirmwareName();
 File m_firmwareFile; /**< Handle for the firmware file.*/
 const char* m_fileName = fileName.c_str(); /**< File name for the firmware file.*/

/** 
 * The size of each data chunk used for processing, set to 128 bytes.
 */
static const size_t CHUNK_SIZE = 128U;


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

CommandInfo ByteCheckCmdProvider::m_cmds[] = {
            {Zumo32U4Specification::SET_MEMORY_ADDR, sizeof(Zumo32U4Specification::SET_MEMORY_ADDR)},
            {Zumo32U4Specification::READ_MEMORY_PAGE, sizeof(Zumo32U4Specification::READ_MEMORY_PAGE)}
};

ResponseInfo ByteCheckCmdProvider::m_responses[] = {
            {Zumo32U4Specification::RET_OK, sizeof(Zumo32U4Specification::RET_OK)}
};

CommandInfo ReadFusesCmdProvider::m_cmds[] = {
            {Zumo32U4Specification::READ_LSB_FUSE, sizeof(Zumo32U4Specification::READ_LSB_FUSE)},
            {Zumo32U4Specification::READ_MSB_FUSE, sizeof(Zumo32U4Specification::READ_MSB_FUSE)},
            {Zumo32U4Specification::READ_EXTENDED_FUSE,sizeof(Zumo32U4Specification::READ_EXTENDED_FUSE)}
};

ResponseInfo ReadFusesCmdProvider::m_responses[] = {
            {Zumo32U4Specification::RET_OK, sizeof(Zumo32U4Specification::RET_OK)},
            {Zumo32U4Specification::RET_OK, sizeof(Zumo32U4Specification::RET_OK)},
            {Zumo32U4Specification::RET_OK, sizeof(Zumo32U4Specification::RET_OK)}
};

CommandInfo ExitCmdProvider::m_cmds[] = {
            {Zumo32U4Specification::EXIT_PROGRAMMING_MODE, sizeof(Zumo32U4Specification::EXIT_PROGRAMMING_MODE)},
            {Zumo32U4Specification::EXIT_BOOTLOADER_MODE, sizeof(Zumo32U4Specification::EXIT_BOOTLOADER_MODE)}
};

ResponseInfo ExitCmdProvider::m_responses[] = {
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
static FwProgCmdProvider m_fwProvider(fileName.c_str());

/**
 * @brief Static instance of the internal programming command provider.
 * This static variable holds an instance of the ByteCheckCmdProvider class,
 * which provides commands for internal programming.
 */
static ByteCheckCmdProvider m_bcProvider(fileName.c_str());

/**
 * @brief Static instance of the internal programming command provider.
 * This static variable holds an instance of the IntProgCmdProvider class,
 * which provides commands for internal programming.
 */
static ReadFusesCmdProvider m_ReadFusesCmdProvider;

/**
 * @brief Static instance of the internal programming command provider.
 * This static variable holds an instance of the IntProgCmdProvider class,
 * which provides commands for internal programming.
 */
static ExitCmdProvider m_exitProvider;

/**
 * @ brief Array of command providers.
 * This static array holds pointers to command providers. It can be used to
 * iterate through different command providers during operation.
 */
static CmdProvider * cmdProviders[] = 
{
    &m_initProvider, /**< Pointer to the internal programming command provider.*/
    &m_fwProvider, /**< Pointer to the writing programming command provider.*/
    &m_bcProvider, /**< Pointer to the reading programming command provider.*/
    &m_ReadFusesCmdProvider, /**< Pointer to the reading fuses programming command provider.*/
    &m_exitProvider, /**< Pointer to the exit programming command provider.*/
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

void BootloaderCom::setFirmwareName( const String& firmwareName)
{
    if( false == firmwareName.isEmpty())
    {
        if (false == firmwareName.startsWith("/"))
        {
            m_firmwareName = "/" + firmwareName;
        }
        else
        {
            LOG_DEBUG("m_firmwareName is unchanged!");
            m_firmwareName = firmwareName;

        }
        LOG_DEBUG("Updated Filename is : %s",m_firmwareName.c_str());
    }
    else
    {
        /*Nothing to update!*/
    }

}

String BootloaderCom:: getFirmwareName()
{
    if (false == m_firmwareName.startsWith("/"))
        {
            m_firmwareName = "/" + m_firmwareName;
        }
    return m_firmwareName;
}

void BootloaderCom :: enterBootloader()
{
    Board::getInstance().getDevice().enterBootloader();
    LOG_INFO(" bootloader mode is activated");
}

bool BootloaderCom::process()
{
    size_t newBytes = 0; 
    uint8_t receiveBuffer[128];
    bool success = true;/**<Keeps track of the overall process success status.*/
    switch(m_state)
    {
        case SelectCmdProvider:
            m_cmdProvider = cmdProviders[m_currentProvider];
            if (nullptr != m_cmdProvider)
            {
                if(true == m_cmdProvider->reset() )
                {
                    ++m_currentProvider;
                    m_state = Idle;
                }
                else
                {
                    LOG_ERROR("Fatal error! Process can not start!");
                    success = false;
                    return false;
                }
            }
            else
            {
                /*TODO: We are done, no more provider, signal completion to web interface.*/
                LOG_INFO("no more work!");

                break;
            }
            break;

        case Idle:
            /*Handle Idle state*/
            m_waitingForResponse = false;
            if(m_cmdProvider->next(m_currentCommand, m_currentResponse))
            {
                 if (true == m_flashManager.sendCommand(
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
                    success = false;
                    break;
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
            LOG_INFO(" readingresponse  newbytes = %d", newBytes);
            LOG_INFO(" readingresponse  receivebuffer = %d", sizeof(receiveBuffer));

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
                    success = false;
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
    return success;
}

bool BootloaderCom::compareExpectedAndReceivedResponse(const uint8_t command[], const uint8_t* receivedResponse, size_t readbytes, size_t expectedSize)
{
    /** 
     * Buffer to hold data read from a file, with a size of 128 bytes.
     */
    uint8_t fileReadBuffer[CHUNK_SIZE];

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

    else if(command == Zumo32U4Specification::WRITE_MEMORY_PAGE)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::WRITE_MEMORY_PAGE,expectedSize))
        {
            LOG_INFO("Chunk has been written!");
            return true;
        }
        else
        {
            LOG_ERROR("Chunk has not been written!");
            return false;
        }
    }
    else if(command == Zumo32U4Specification:: READ_MEMORY_PAGE)
    {
        if (m_firmwareFile)
        {
            /*File is already opened.*/
        }
        else
        {
            String testFileName = getFirmwareName();
            if (false == testFileName.startsWith("/"))
            {
                testFileName = "/" + testFileName;
            }
            m_fileName = testFileName.c_str();

            m_firmwareFile = LittleFS.open(m_fileName, "r");
        }

        /*Obtain a ByteCheckCmdProvider pointer from the current command provider.*/
        ByteCheckCmdProvider* temprovider = (ByteCheckCmdProvider*)m_cmdProvider;
        /*Get the count from the ByteCheckCmdProvider instance.*/
        uint8_t idx = temprovider->getCount();

        /*Fill the buffer with Fillvalues 0xFF.*/
            for (int i = 0; i <  CHUNK_SIZE; i++)
            {
                fileReadBuffer[i] = 0xFF;
            }

        /*Read up to 128 bytes from the firmware file into fileReadBuffer.*/
        size_t  m_firmwareBytesRead = m_firmwareFile.read(fileReadBuffer,CHUNK_SIZE);

        /*Check if the number of bytes read is less than 128 (the buffer size).*/
        if (m_firmwareBytesRead < CHUNK_SIZE)
        {
            /*Log the number of bytes read in the last block.*/
            LOG_DEBUG("Last block finalytes read: %d!", m_firmwareBytesRead);
            m_firmwareFile.close();
        }

        /*Print out the content of fileReadBuffer and receivedResponse for comparaison.*/
        for(uint8_t a = 0; a < CHUNK_SIZE; a++)
        {
            Serial.print(fileReadBuffer[a], HEX);
            Serial.print(" vs ");
            Serial.print(receivedResponse[a], HEX);
            Serial.println();
        }
        if (0 == memcmp(fileReadBuffer, receivedResponse, CHUNK_SIZE))
        {
            LOG_INFO("+++++++++++++++++++++++DataChunk %d  read with success", idx);
            return true;
        }
        else
        {
            LOG_INFO("-----------------------Reading of the DataChunk %d failed", idx);
            return false;
        }
    return true;

    }

    else if(command == Zumo32U4Specification:: EXIT_PROGRAMMING_MODE)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::RET_OK,expectedSize))
        {
            LOG_INFO("Exited programming Mode!");
            return true;
        }
        else
        {
            LOG_ERROR("Still in Programming Mode!");
            return false;
        }
    }

    else if(command == Zumo32U4Specification::  EXIT_BOOTLOADER_MODE)
    {
        if (0 == memcmp(receivedResponse,Zumo32U4Specification::RET_OK,expectedSize))
        {
            LOG_INFO("Exited Bootloader Mode!");
            return true;
        }
        else
        {
            LOG_ERROR("Still in Bootloader Mode!");
            return false;
        }
    }
    return true;
}






















