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
const uint16_t WAIT_TIME_MS = 500U;

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

void FlashManager::readToRobotFlash(size_t expectedsize)
{
    Stream& deviceStream   = Board::getInstance().getDevice().getStream();
    Board::getInstance().getDevice().process();
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
        }
        else
        {
            LOG_INFO("Failure!");
        }  
    }
    
}

void FlashManager ::sendCommand(const uint8_t command[])
{
    Stream& deviceStream   = Board::getInstance().getDevice().getStream();
    size_t commandsize = sizeof(command[0]);
    size_t mysize= deviceStream.write(command, commandsize);
    
    readToRobotFlash(mysize);

}

void FlashManager:: enterBootloadermode()
{
    /* enter bootloader mode.*/
    m_bootloader.enterBootloader();
}

void FlashManager::exitBootloader()
{

}




















































































































































































































































































