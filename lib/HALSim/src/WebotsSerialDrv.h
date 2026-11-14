/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
 * @file
 * @brief  Webots serial driver
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef WEBOTS_SERIAL_DRV_H
#define WEBOTS_SERIAL_DRV_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Stream.h"
#include "Sender.h"
#include "Receiver.h"

#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Webots serial driver.
 */
class WebotsSerialDrv : public Stream
{
public:
    /**
     * Construct a Webots serial driver.
     *
     * @param[in] emitter   Webots emitter used to send data over simulated serial.
     * @param[in] receiver  Webots receiver used to receive data over simulated serial.
     */
    WebotsSerialDrv(webots::Emitter* emitter, webots::Receiver* receiver);

    /**
     * Destruct the Webots serial driver.
     */
    virtual ~WebotsSerialDrv();

    /**
     * Set the serial receive channel id.
     *
     * @param[in] channelId Channel ID, shall be positive for inter-robot communication.
     */
    void setRxChannel(int32_t channelId);

    /**
     * Set the serial sender channel id.
     *
     * @param[in] channelId Channel ID, shall be positive for inter-robot communication.
     */
    void setTxChannel(int32_t channelId);

    /**
     * Write a single byte to serial.
     *
     * @param[in] data Byte to send.
     *
     * @return Number of bytes written.
     */
    size_t write(uint8_t data) final;

    /**
     * Write data buffer to serial.
     *
     * @param[in] buffer Byte buffer to send.
     * @param[in] length Number of bytes to send.
     *
     * @return Number of bytes written.
     */
    size_t write(const uint8_t* buffer, size_t length) final;

    /**
     * Check if any data has been received.
     *
     * @return Number of available bytes.
     */
    int available() final;

    /**
     * Read a single byte from serial.
     *
     * @return The received byte or -1 if no data is available.
     */
    int read() final;

    /**
     * Read a single byte from serial without removing it.
     *
     * @return The received byte or -1 if no data is available.
     */
    int peek() final;

private:
    Sender   m_sender;   /**< The sender to send data over simulated serial. */
    Receiver m_receiver; /**< The receiver to receive data over simulated serial. */
    int      m_peeked;   /**< Byte which was peeked, but not read yet. Its -1 if there is none. */

    /**
     * Default constructor.
     * Not allowed.
     */
    WebotsSerialDrv();

    /**
     * Copy construction of an instance.
     * Not allowed.
     *
     * @param[in] srv Source instance.
     */
    WebotsSerialDrv(const WebotsSerialDrv& srv);

    /**
     * Assignment operation.
     * Not allowed.
     *
     * @param[in] srv Right hand side instance.
     *
     * @returns Reference to WebotsSerialDrv instance.
     */
    WebotsSerialDrv& operator=(const WebotsSerialDrv& srv);
};

#endif /* WEBOTS_SERIAL_DRV_H */
/** @} */
