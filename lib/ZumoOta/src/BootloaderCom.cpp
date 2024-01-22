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
#include "BootloaderCom.h"
#include <Logging.h>
#include <Board.h>
#include <GPIO.h>


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
const uint16_t WAIT_TIME_MS = 500U;

/** Specifies the number of bytes stored in one Zumo bootloader/flash memory page.*/
static const uint16_t PAGE_SIZE_BYTES = 128U;
/******************************************************************************
 * Public Methods
 *****************************************************************************/

BootloaderCom::BootloaderCom():
m_state(Idle),
m_waitingForResponse(false)
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
    GpioPins::resetDevicePin.write(HIGH);
    delay(WAIT_TIME_MS);
    GpioPins::resetDevicePin.write(LOW);
    LOG_INFO(" bootloader mode is exited");

}

void BootloaderCom::process()
{
    switch(m_state)
    {
        
        case Idle:
        /*Handle Idle state*/
        m_waitingForResponse = false;
        myFlashManager.sendCommand(Zumo32U4Specification::READ_SW_ID, sizeof(Zumo32U4Specification::READ_SW_ID));
        break;
     
        case Pending:
        /*Handle Pending state*/
        m_state = ReadingResponse;
        m_waitingForResponse = true;
        break;

        case ReadingResponse:
        /*Handle Complete state*/
        uint8_t expectedSW[sizeof(&Zumo32U4Specification::EXPECTED_SOFTWARE_ID)];
        myFlashManager.readingStream(expectedSW);
        m_state = Complete;
        m_waitingForResponse = false;

        if (compareExpectedAndReceivedResponse(Zumo32U4Specification::READ_SW_ID, sizeof(Zumo32U4Specification::READ_SW_ID),Zumo32U4Specification::EXPECTED_SOFTWARE_ID, sizeof(Zumo32U4Specification::EXPECTED_SOFTWARE_ID)))
            {
                LOG_INFO("Received expected response.");
            }
        else
            {
                LOG_ERROR("Received response does not match the expected response.");
            }
        break;

        case Complete:
        m_state = Idle;
        break;

        default:
            break;
 }
}

bool BootloaderCom::compareExpectedAndReceivedResponse(const uint8_t command[], size_t commandSize, const uint8_t expectedResponse[], size_t expectedResponseSize)
{
    
    myFlashManager.Check( command,  commandSize,  expectedResponse,  expectedResponseSize);
    return true;
}
 
 
 
 
 
 
 
 
 
 
 


























