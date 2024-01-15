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
 * @brief  FlashManager realization
 *
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "FlashManager.h"
#include <LittleFS.h>
#include <Logging.h>
#include <string.h>
#include <Board.h>
#include <GPIO.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/
const uint8_t USB_RETRY_DELAY_MS = 10U;
const uint16_t USB_TIMEOUT_MS = 2000U;
const uint8_t AWAIT_RESPONSE_DELAY_MS = 50;
const uint8_t NEXT_SERIAL_SEND_DELAY_MS = 10;
/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/
/*Delay before processing the response.
const uint8_t AWAIT_RESPONSE_DELAY_MS = 50U;*/

/** Specifies the number of bytes stored in one Zumo bootloader/flash memory page.*/
static const uint16_t PAGE_SIZE_BYTES = 128U;
/******************************************************************************
 * Public Methods
 *****************************************************************************/

FlashManager::FlashManager():
    m_writtenFirmwareBytes(0),
    m_expectedFirmwareSize(0),
    m_expectedHashValue()
{
}

FlashManager::~FlashManager()
{
}

bool FlashManager::readToRobotFlash(size_t expectedsize)
{
    bool retCode = false;
    Stream& deviceStream   = Board::getInstance().getDevice().getStream();
    int     availableBytes = deviceStream.available();
    if (0 < availableBytes)

    {
        uint8_t buffer[availableBytes];
        size_t  readBytes = deviceStream.readBytes(buffer, availableBytes);
       if(expectedsize == readBytes)
       {
            
            LOG_INFO("Hexadecimal Data:");
            for (size_t idx = 0; idx < readBytes; idx++)
            {
                Serial.print("0x");
                if (buffer[idx] < 0x10) Serial.print("0"); 
                Serial.print(buffer[idx], HEX);
                Serial.print(" ");
            }
            Serial.println(); 

            
            LOG_INFO("Binary Data:");
            for (size_t idx = 0; idx < readBytes; idx++)
            {
                for (int bit = 7; bit >= 0; bit--)
                {
                    Serial.print((buffer[idx] >> bit) & 1);
                }
                Serial.print(" ");
            }
            Serial.println(); 
            retCode = true;
        }
        else
        {
            LOG_INFO("Failure!");
        }  
    }
    return retCode;
    
}

void FlashManager ::sendCommand(const uint8_t command[])
{
    const uint16_t WRITE_BUFFER_SIZE = 256;
    uint8_t writeBuffer[WRITE_BUFFER_SIZE];
    Stream& deviceStream   = Board::getInstance().getDevice().getStream();
    /*Size of array*/
    size_t commandsize = sizeof(command);
     /* Copy the command OpCode into buffer */
    memcpy(writeBuffer, command, commandsize);

    /* Send the OpCode and command data to Zumo robot */
    size_t bytesWritten= deviceStream.write(command, commandsize);

    if(bytesWritten == commandsize)
    {
        /* Await response */
        delay(50);
        readToRobotFlash(commandsize);
    }
    else
    {
        LOG_ERROR("Could not send data packet to Zumo robot. Aborting now");
    }
}

void FlashManager:: enterBootloadermode()
{
    /* enter bootloader mode.*/
    m_bootloader.enterBootloader();
}

void FlashManager::exitBootloader()
{

}




















































































































































































































































































