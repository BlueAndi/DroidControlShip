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
 * @brief  ZumoOta application
 * @author Decareme Pauline Ngangnou <ngandeca@yahoo.fr>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef BOOTLOADERCOM_H
#define BOOTLOADERCOM_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "FlashManager.h"
#include <cstdint>
/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/
/** 
 *@brief The structure defines command information for the BootloaderCom class.
 */
struct CommandInfo{
    /**
     *The command buffer.
     */
    const uint8_t* command;

    /**
     *The size of the command buffer.
     */
    size_t commandsize;
};

/**
 *@brief The structure defines response information for the BootloderCom class.
 */
struct ResponseInfo{
    /**
     *@brief The expected response buffer.
     */
    const uint8_t* expectedResponse;

    /**
     *@brief The size of the expected response buffer.
     */
    size_t responseSize;
};

/**
 *@brief BootloaderCom class manages the bootloader protocol.
 */
class BootloaderCom
{
public:
    /**
     * Construct the bootloader  application.
     */
  BootloaderCom();

    /**
     * Destroy the bootloader  application.
     */
    ~BootloaderCom();

    /**
     * @brief enter the bootloader mode.
     */
    void enterBootloader();

   /**
    * @brief exit the bootloader mode.
    */
   void exitBootloader();

   /**
    * @brief Process the flash manager state machine.
    *return bool True if the process was successful, otherwise false.
    */
    bool process();

    /**
     *@brief Compare the received response against the expected response after sending a command.
     *@param command The command to be sent.
     *@param receivedResponse The expected response to compare against.
     *@param readbytes the number of bytes read.
     *@param expectedSize The expected size of the response.
     *@return True if the received response matches the expected response, false otherwise.
     */
     bool compareExpectedAndReceivedResponse(const uint8_t command[], const uint8_t* receivedResponse, size_t readbytes, size_t expectedSize);

    

private:
    /**
     *@brief Enumeration representing different states for the BootloaderCom class.
     */
    enum State
    {
        Idle,
        ReadingResponse,
        Complete
    };
    
    /**
     *@brief Current state of the BootloaderCom class.
     *Initialized to Idle by default.
     */
    State m_state;

    /**
     * Instance of the FlashManager class
     */
     FlashManager myFlashManager;

    /**
     * @brief Flag indicating whether the FlashManager is currently waiting for a response.
     * If true, the system is waiting for a response otherwise, it is not.
     */
     bool m_waitingForResponse;

    /**
     *@brief Array to store the commands.
     */
     CommandInfo m_commands[6];

    /**
     *@brief Array to store expected Responses.
     */
     ResponseInfo m_responses[6]; 
};

#endif /* BOOTLOADERCOM_H */
/** @} */
