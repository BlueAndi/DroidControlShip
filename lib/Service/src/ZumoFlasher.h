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
 *
 * @addtogroup Service
 *
 * @{
 */
#ifndef ZUMO_FLASHER_H
#define ZUMO_FLASHER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "CaterinaSpecification.h"
#include <Stream.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Class for flashing firmware to the Zumo32U4. */
class ZumoFlasher
{
public:
    /** Status of the ZumoFlasher. */
    enum Status : uint8_t
    {
        STATUS_IDLE = 0U,     /**< Idle. */
        STATUS_FLASHING,      /**< Flashing. */
        STATUS_FINISHED_OK,   /**< Finished OK. */
        STATUS_FINISHED_ERROR /**< Finished with Error. */
    };

    /**
     * ZumoFlasher Constructor.
     *
     * @param[in] deviceStream  Stream to communicate with device.
     */
    ZumoFlasher(Stream& deviceStream);

    /** Default destructor. */
    ~ZumoFlasher();

    /**
     * Process the flashing process.^
     *
     * @return Status of the flashing process.
     */
    Status process();

    /**
     * Start the flashing process.
     * The robot must already be in bootloader mode.
     *
     * @param[in] firmwareData Firmware data
     * @param[in] firmwareSize Firmware size
     *
     * @return If the flashing process was started successfully, it will return true. Otherwise false.
     */
    bool startFlashing(char* firmwareData, size_t firmwareSize);

private:
    /** States of the flashing process. */
    enum FlashState : uint8_t
    {
        STOPPED = 0U,  /**< Flashing process is stopped. */
        INITALIZATING, /**< Flashing process is initializing. */
        WRITING,       /**< Flashing process is writing. */
        VERIFYING,     /**< Flashing process is verifying. */
        FINISHING      /**< Flashing process is being finished. */
    };

    /** Response states. */
    enum ResponseState : uint8_t
    {
        RESPONSE_OK = 0,  /**< Response is OK. */
        RESPONSE_ERROR,   /**< Response is ERROR. */
        RESPONSE_WAITING, /**< Waiting for a response. */
    };

    /** Stream to communicate with device. */
    Stream& m_device;

    /** ZumoFlasher Status. */
    Status m_status;

    /** State of the flashing process. */
    FlashState m_flashingState;

    /** Flag: Waiting for response */
    bool m_waitingForResponse;

    /** Process Step counter. */
    uint8_t m_processStep;

    /** Firmware data */
    char* m_firmwareData;

    /** Firmware size */
    size_t m_firmwareSize;

    /**
     * Send a command to the Zumo32U4.
     *
     * @param[in]   commandData     Command to send.
     * @param[in]   commandSize     Size of the command.
     *
     * @returns true if command was sent, else false.
     */
    bool sendCommand(const uint8_t* commandData, size_t commandSize);

    /**
     * Check the received data against the expected data.
     *
     * @param[in]   commandId       Command ID.
     *
     * @returns state of the response.
     */
    ResponseState checkReceivedData(COMMAND_ID commandId);

    /** Process initializing state. */
    void processInitializingState();

    /** Process writing state. */
    void processWritingState();

    /** Process verifying state. */
    void processVerifyingState();

    /** Process finishing state. */
    void processFinishingState();

    /** Fatal error handler. */
    void fatalError();

    /**
     * Default constructor.
     * Not allowed.
     */
    ZumoFlasher();

    /**
     * Copy constructor
     * Not allowed.
     *
     * @param[in] zumoFlasher Zumo flasher
     */
    ZumoFlasher(const ZumoFlasher& zumoFlasher);

    /**
     * Assignment operator.
     * Not allowed.
     *
     * @param[in] zumoFlasher Zumo flasher
     *
     * @return Zumo flasher
     */
    ZumoFlasher& operator=(const ZumoFlasher& zumoFlasher);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* ZUMO_FLASHER_H */
/** @} */
