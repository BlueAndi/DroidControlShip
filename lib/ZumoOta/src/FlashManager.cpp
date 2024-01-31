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
#include <Board.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/
/*Maximum Buffer Size.*/
const uint8_t MAX_BUFFER_SIZE = 128;

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

FlashManager::FlashManager():
    m_bytesRead(0)
{
}

FlashManager::~FlashManager()
{
}

size_t FlashManager::readingStream(uint8_t* expectedResponse, size_t mybytes)
{
    Stream& deviceStream   = Board::getInstance().getDevice().getStream();
    int     availableBytes = deviceStream.available();
    /*Check if there are bytes available in the input stream.*/
    if ( availableBytes > 0)
    {
        size_t totalReadBytes = 0;
        LOG_INFO("Available size: %d ", availableBytes);
        do
        {
            m_bytesRead = deviceStream.readBytes(expectedResponse, mybytes - m_bytesRead);
            totalReadBytes += m_bytesRead;
        } while (m_bytesRead != 0);

        for (size_t idx = 0; idx < totalReadBytes; idx++)
        {
            /*Handle the Hexadecimal.*/
            Serial.print(expectedResponse[idx], HEX);
        }
        Serial.println();
        LOG_INFO("totalReadBytes ist %d", totalReadBytes);
       
        return totalReadBytes;
    }
    else
    {
        /*Handle the case where there are no bytes available in the input stream.*/
        LOG_ERROR("Failure! No bytes available in the input stream.");
        return 0;
    }
}


bool FlashManager ::sendCommand(const uint8_t* command, size_t commandsize)
{
    {
        Stream& deviceStream   = Board::getInstance().getDevice().getStream();
    
       if(nullptr == command)
       {
          LOG_INFO("command is a nullptr");
          return false;
       }
       else
       {
       
        /* Send the OpCode and command data to Zumo robot. */
        size_t bytesWritten= deviceStream.write(command, commandsize);
      
         LOG_INFO("byteswritten ist %d", bytesWritten);
    
        if(bytesWritten == commandsize)
        {
            LOG_INFO("Send Data packet to Zumo robot");
            return true;
           
        }
        else
        {
            LOG_ERROR("Could not send data packet to Zumo robot FlashManager. Aborting now");
           
            return false;
        }
       }
    }
}





















































































































































































































































































































































































