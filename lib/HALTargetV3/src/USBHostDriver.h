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
 * @brief  Abstraction and Stream implementation of the host connection to the
 *         robot. On the ZumoComSystemV3 the Zumo robot is attached via UART
 *         instead of USB, therefore this driver realizes the host interface
 *         on top of a hardware serial port.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALTargetV3
 *
 * @{
 */

#ifndef USBHOSTDRIVER_H
#define USBHOSTDRIVER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Stream.h>
#include <HardwareSerial.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Class for configuring and managing the host connection to the robot. */
class USBHost : public Stream
{
public:
    /**
     * Default constructor
     */
    USBHost();

    /**
     * Default destructor
     */
    ~USBHost();

    /**
     * Initializes the host connection to the robot.
     *
     * @returns true if initialization is successful. Otherwise, false.
     */
    bool init();

    /**
     * Processes the device connection.
     */
    void process();

    /**
     * Writes a single byte.
     *
     * @param[in] value Byte to send.
     *
     * @returns Number of bytes written
     */
    size_t write(uint8_t value) final;

    /**
     * Writes bytes.
     *
     * @param[in] buffer Byte Array to send.
     * @param[in] length Length of Buffer.
     *
     * @returns Number of bytes written
     */
    size_t write(const uint8_t* buffer, size_t length) final;

    /**
     * Checks if there are available bytes in the Stream.
     *
     * @returns Number of available bytes.
     */
    int available() final;

    /**
     * Reads a byte from the Stream.
     *
     * @returns The first byte of incoming data available (or -1 if no data is available).
     */
    int read() final;

    /**
     * Peeks a byte from the Stream.
     *
     * @returns The first byte of incoming data available (or -1 if no data is available).
     */
    int peek() final;

    /**
     * Reads bytes into a buffer.
     *
     * @param[in] buffer Array to write bytes to.
     * @param[in] length number of bytes to be read.
     *
     * @returns Number of bytes read from Stream.
     */
    size_t readBytes(uint8_t* buffer, size_t length) final;

    /**
     * Requests to enter the bootloader mode.
     */
    void requestBootloaderMode();

    /**
     * Resets the flags and flushes the receive buffer.
     */
    void reset();

    /**
     * Checks if the bootloader mode is active.
     *
     * @return If the bootloader mode is active, returns true. Otherwise, false.
     */
    bool isBootloaderModeActive() const;

private:
    /** Baud rate of the UART connection to the robot. */
    static const uint32_t BAUD_RATE = 115200U;

    /** Size of the UART RX buffer in bytes. */
    static const uint16_t UART_RX_BUFFER_SIZE = 1024U;

    /** UART connection to the robot. */
    HardwareSerial& m_robotSerial;

    /** Bootloader Mode Flag. */
    bool m_isBootloaderModeActive;

private:
    /**
     * Copy construction of an instance.
     * Not allowed.
     *
     * @param[in] usbhost Source instance.
     */
    USBHost(const USBHost& usbhost);

    /**
     * Assignment of an instance.
     * Not allowed.
     *
     * @param[in] usbhost Source instance.
     *
     * @returns Reference to USBHost instance.
     */
    USBHost& operator=(const USBHost& usbhost);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* USBHOSTDRIVER_H */
/** @} */
