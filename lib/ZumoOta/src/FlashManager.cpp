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
#include <typeinfo>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/
/*Delay before processing the response.*/
const uint8_t AWAIT_RESPONSE_DELAY_MS = 50;
const uint8_t MAX_BUFFER_SIZE = 128;
const uint16_t USB_TIMEOUT_MS = 2000;

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

size_t FlashManager::readingStream(uint8_t* expectedResponse)
{

    {
        Stream& deviceStream   = Board::getInstance().getDevice().getStream();
        int     availableBytes = deviceStream.available();
        size_t totalReadBytes = 0;
        uint8_t pReadBuffer[availableBytes];
        uint8_t* buffer;


        /*Check if there are bytes available in the input stream.*/
        if ( availableBytes > 0)
        {
            LOG_INFO("Available size: "+ availableBytes);
            buffer = &pReadBuffer[totalReadBytes];
            do
            {
                m_bytesRead = deviceStream.readBytes(buffer, MAX_BUFFER_SIZE);
                totalReadBytes += m_bytesRead;
            } while (m_bytesRead != 0);
            
            
            LOG_INFO("Hexadecimal Data:");
            for (size_t idx = 0; idx < totalReadBytes; idx++)
            {
                Serial.print("0x");
                if (buffer[idx] < 0x10) Serial.print("0"); 
                Serial.print(buffer[idx], HEX);
                Serial.print(" ");
                expectedResponse[idx]= buffer[idx];
            }
            Serial.println(); 
            
            LOG_INFO("Binary Data:");
            for (size_t idx = 0; idx < totalReadBytes; idx++)
            {
                for (int bit = 7; bit >= 0; bit--)
                {
                    Serial.print((buffer[idx] >> bit) & 1);
                    
                }
                Serial.print(" ");
                expectedResponse[idx]= buffer[idx];
            }
            Serial.println(); 
            
            return totalReadBytes;
        }
        else
        {
            /*Handle the case where there are no bytes available in the input stream.*/
            LOG_INFO("Failure! No bytes available in the input stream.");
             /*No valid response received.*/
            return -1;
        }
       
       
    }





}

bool FlashManager ::sendCommand(const uint8_t command[], size_t commandsize)
{
    {
        Stream& deviceStream   = Board::getInstance().getDevice().getStream();
        
    
        unsigned long startTime = 0L;
       
        /* Send the OpCode and command data to Zumo robot. */
        size_t bytesWritten= deviceStream.write(command, commandsize);
         LOG_INFO("byteswritten ist %d", bytesWritten);
    
        if(bytesWritten == commandsize)
        {
            return true;
        }
        else
        {
            LOG_ERROR("Could not send data packet to Zumo robot FlashManager. Aborting now");
           
            return false;
        }
    }
}

bool FlashManager::Check(const uint8_t command[], size_t commandSize, const uint8_t expectedResponse[], size_t expectedResponseSize)
{
    /*Send the Command.*/
    if (true == sendCommand(command, commandSize))
    {
        /*Read the Response.*/
        uint8_t readBuffer[expectedResponseSize];
        size_t readBytes = 0;
        /*Set the actually read bytes.*/
        readBytes = readingStream(readBuffer);
    
        if (0 < readBytes)
        {
            /*Check if the received response matches the expected response.*/ 
            if (readBytes == expectedResponseSize && memcmp(&readBuffer, &expectedResponse, expectedResponseSize) == 0)
            {
                LOG_INFO("Received expected response.");
                return true;
            }
            else
            {
                LOG_ERROR("Received response does not match the expected response.");
            }
        }
        else
        {
            LOG_ERROR("Error reading response.");

        }
        
    }
    else
    {
         LOG_ERROR("Error sending command.");

    }
    return false;
}




















































































































































































































































































































































































